import numpy as np
import rospy
import utm
from datapoint import DataPoint
from tools import time_difference
from sensor_msgs.msg import Imu, NavSatFix
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from novatel_msgs.msg import INSPVAX

ekf_localization = PoseWithCovarianceStamped()
ekf_pose = PoseStamped()
ekf_path = Path()
ekf_odom = Odometry()
gps_pose = PoseStamped()
rtk_pose = PoseStamped()


class KalmanFilter:
    """
    A class that predicts the next state of the system given sensor measurements 
    using the Kalman Filter algorithm 
    """

    def __init__(self, n):

        self.n = n
        self.I = np.matrix(np.eye(n))
        self.x = None
        self.P = None
        self.F = None
        self.Q = None

    def start(self, x, P, F, Q):

        self.x = x
        self.P = P
        self.F = F
        self.Q = Q

    def setQ(self, Q):
        self.Q = Q

    def updateF_(self, dt):
        # self.F[0, 2], self.F[1, 3], self.F[4,5]= dt, dt, dt
        self.F[0, 2], self.F[1, 3] = dt, dt
        self.F[0, 4], self.F[1, 5] = (dt*dt)/2, (dt*dt)/2
        self.F[2, 4], self.F[3, 5] = dt, dt

    def updateF(self, dt):
        self.F[0, 3], self.F[1, 4], self.F[2, 5] = dt, dt, dt
        self.F[0, 6], self.F[1, 7], self.F[2, 8] = (dt*dt)/2, (dt*dt)/2, (dt*dt)/2
        self.F[3, 6], self.F[4, 7], self.F[5, 8] = dt, dt, dt


    def getx(self):
        return self.x

    def predict(self):
        ''' Prediction '''    
        self.x = self.F * self.x

        ''' A Priori covariance '''
        self.P = self.F * self.P * self.F.T + self.Q

    def update(self, z, H, Hx, R):

        y = z - Hx
        PHt = self.P * H.T
        S = H * PHt + R  

        '''Kalman Gain'''
        K = PHt * (S.I)

        ''' Measurement update '''
        self.x = self.x + K * y

        ''' A posteriori covariance '''
        self.P = (self.I - K * H) * self.P

class Extended_KF:
    """
    A class that predicts the next state of the system given sensor measurements 
    using an extended Kalman filter algorithm

    The 6 state variables considered in this system are the position (px, py), velocity (vx, vy), 
    yaw angle (yaw) and yaw rate (dyaw).

    This class gets measurements from both lidar and radar sensors
    """

    def __init__(self, d):
        self.initialized = False
        self.timestamp = 0
        self.n = d['number_of_states']
        self.P = d['initial_process_matrix']
        self.F = d['inital_state_transition_matrix']
        self.Q = d['initial_noise_matrix']
        # self.radar_R = d['radar_covariance_matrix']
        self.lidar_R = d['lidar_covariance_matrix']
        self.lidar_H = d['lidar_transition_matrix']
        self.gps_R = d['gps_covariance_matrix']
        self.gps_H = d['gps_transition_matrix']
        self.rtk_R = d['rtk_covariance_matrix']
        self.rtk_H = d['rtk_transition_matrix']
        self.imu_R = d['imu_covariance_matrix']
        self.imu_H = d['imu_transition_matrix']
        self.a = (d['acceleration_noise_x'], d['acceleration_noise_y'], d['acceleration_noise_z'], d['imu_noise'])
        self.kalmanFilter = KalmanFilter(self.n)

    def updateQ(self, dt):

        ''' A function to update process noise '''

        # dt2 = dt * dt
        # dt3 = dt * dt2
        # dt4 = dt * dt3

        dt2 = dt 
        dt3 = dt 
        dt4 = dt 

        ax, ay, az, aq = self.a

        std_ax, std_ay, std_az = ax**2, ay**2, az**2
        std_aqx, std_aqy, std_aqz, std_aqw = aq**2, aq**2, aq**2, aq**2

        r11 = dt4 * std_ax / 4
        r14 = dt3 * std_ax / 2
        r41 = r14
        r17 = dt2 * std_ax
        r71 = r17

        r22 = dt4 * std_ay / 4
        r25 = dt3 * std_ay / 2
        r52 = r25
        r28 = dt2 * std_ay
        r82 = r28

        r33 = dt4 * std_az / 4
        r36 = dt3 * std_az / 2
        r63 = r36
        r39 = dt2 * std_az 
        r93 = r39

        r44 = dt2 * std_ax
        r47 = dt * std_ax
        r74 = r47
        
        r55 = dt2 * std_ay
        r58 = dt * std_ay
        r85 = r58

        r66 = dt2 * std_az
        r69 = dt * std_az
        r96 = r69

        r77 = std_ax
        r88 = std_ay
        r99 = std_az

        rqx = dt4 * std_aqx / 4
        rqy = dt4 * std_aqy / 4
        rqz = dt4 * std_aqz / 4
        rqw = dt4 * std_aqw / 4



        Q = np.eye(self.n)
        Q = np.matrix([[r11,   0,   0, r14,   0,   0, r17,   0,   0,   0,   0,   0,   0],
                       [  0, r22,   0,   0, r25,   0,   0, r28,   0,   0,   0,   0,   0],
                       [  0,   0, r33,   0,   0, r36,   0,   0, r39,   0,   0,   0,   0], 
                       [r41,   0,   0, r44,   0,   0, r47,   0,   0,   0,   0,   0,   0],
                       [  0, r52,   0,   0, r55,   0,   0, r58,   0,   0,   0,   0,   0],
                       [  0,   0, r63,   0,   0, r66,   0,   0, r69,   0,   0,   0,   0],
                       [r71,   0,   0, r74,   0,   0, r77,   0,   0,   0,   0,   0,   0],
                       [  0, r82,   0,   0, r85,   0,   0, r88,   0,   0,   0,   0,   0],
                       [  0,   0, r93,   0,   0, r96,   0,   0, r99,   0,   0,   0,   0],
                       [  0,   0,   0,   0,   0,   0,   0,   0,   0, rqx,   0,   0,   0],
                       [  0,   0,   0,   0,   0,   0,   0,   0,   0,   0, rqy,   0,   0],
                       [  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, rqz,   0],
                       [  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, rqw]])
        

        self.kalmanFilter.setQ(Q)
        print(dt)


    def update(self, data):

        dt = time_difference(self.timestamp, data.get_timestamp())
        self.timestamp = data.get_timestamp()

        self.kalmanFilter.updateF(dt)
        self.updateQ(dt)
        self.kalmanFilter.predict()

        z = np.matrix(data.get_raw()).T
        x = self.kalmanFilter.getx()
        # print(x)

        '''
        if data.get_name() == 'radar':        

            px, py,  vx, vy, yaw, dyaw = x[0, 0], x[1, 0], x[2, 0], x[3, 0], x[4,0], x[5,0]
            rho, phi, drho = cartesian_to_polar(px, py, vx, vy, yaw, dyaw)
            H = calculate_jacobian(px, py, vx, vy, yaw, dyaw)
            Hx = (np.matrix([[rho, phi, drho]])).T
            R = self.radar_R 
        '''

        if data.get_name() == 'imu':        

            H = self.imu_H
            Hx = self.imu_H * x
            R = self.imu_R

        elif data.get_name() == 'lidar':

            H = self.lidar_H
            Hx = self.lidar_H * x
            R = self.lidar_R

            # print('lidar: ', x[0], x[1])
        
        elif data.get_name() == 'gps':

            H = self.gps_H
            Hx = self.gps_H * x
            R = self.gps_R

            # print('gps: ', x[0], x[1])

        elif data.get_name() == 'rtk':

            H = self.rtk_H
            Hx = self.rtk_H * x
            R = self.rtk_R

            # print('rtk: ', x[0], x[1])

        self.kalmanFilter.update(z, H, Hx, R)

    def start(self, data):

        self.timestamp = data.get_timestamp()
        x = np.zeros([self.n,1])
        self.kalmanFilter.start(x, self.P, self.F, self.Q)
        self.initialized = True

    def process(self, data):

        if self.initialized: 
            self.update(data)
        else:
            self.start(data)

    def get(self):
        return self.kalmanFilter.getx()

def state_estimate(ekf, data):
    """
    Calculates all state estimations given a FusionEKF instance() and sensor measurements

    Args:
    EKF - an instance of a FusionEKF() class 
    sensor_data - a list of sensor measurements as a DataPoint() instance

    Returns:
    state_estimations 
      - a list of all state estimations as predicted by the EKF instance
      - each state estimation is wrapped in  DataPoint() instance
    """

    ekf.process(data)

    x = ekf.get()

    px, py, pz = x[0, 0], x[1, 0], x[2, 0]
    vx, vy, vz = x[3, 0], x[4, 0], x[5, 0]
    ax, ay, az = x[6, 0], x[7, 0], x[8, 0]
    qx, qy, qz, qw = x[9, 0], x[10, 0], x[11, 0], x[12,0]


    # ekf_localization.header.stamp = rospy.Time.now()
    # ekf_localization.header.frame_id = '/camera_init'
    # ekf_localization.pose.pose.position.x = px
    # ekf_localization.pose.pose.position.y = py
    # ekf_localization.pose.pose.position.z = pz
    # ekf_localization.pose.pose.orientation.x = qx
    # ekf_localization.pose.pose.orientation.y = qy
    # ekf_localization.pose.pose.orientation.z = qz
    # ekf_localization.pose.pose.orientation.w = qw

    ekf_odom.header.stamp = rospy.Time.now()
    ekf_odom.header.frame_id = '/camera_init'
    ekf_odom.child_frame_id = '/base_link'
    ekf_odom.pose.pose.position.x = px
    ekf_odom.pose.pose.position.y = py
    ekf_odom.pose.pose.position.z = pz
    ekf_odom.pose.pose.orientation.x = qx
    ekf_odom.pose.pose.orientation.y = qy
    ekf_odom.pose.pose.orientation.z = qz
    ekf_odom.pose.pose.orientation.w = qw
    ekf_odom.twist.twist.linear.x = vx
    ekf_odom.twist.twist.linear.y = vy
    ekf_odom.twist.twist.linear.z = vz

    # ekf_pose.header.stamp = rospy.Time.now()
    # ekf_path.header.frame_id = '/camera_init'
    # ekf_pose.header.frame_id = '/camera_init'
    # ekf_pose.pose.position.x = px
    # ekf_pose.pose.position.y = py
    # ekf_pose.pose.position.z = pz
    # ekf_pose.pose.orientation.x = qx
    # ekf_pose.pose.orientation.y = qy
    # ekf_pose.pose.orientation.z = qz
    # ekf_pose.pose.orientation.w = qw
    # ekf_path.poses.append(ekf_pose)
    

    print('{:15s}'.format("PREDICTION:"))
    print('{:5s} {:8.3f} | {:5s} {:8.3f} '.format("X:", px, "Y:", py))
    print('{:5s} {:8.3f} | {:5s} {:8.3f} '.format("VX:", vx, "VY:", vy))
    print('{:5s} {:8.3f} | {:5s} {:8.3f} '.format("AX:", ax, "AY", ay))
    print('{:5s} {:8.3f} | {:5s} {:8.3f} | {:5s} {:8.3f} | {:5s} {:8.3f}'.format("QX", qx, "QY", qy, "QZ", qz, "QW", qw ))

def imu_data(data):

    imu_time = rospy.get_time()

    d = {
    'timestamp' : imu_time,
    'name' : 'imu',
    'ax' : data.linear_acceleration.x,
    'ay' : data.linear_acceleration.y,
    'az' : data.linear_acceleration.z,
    'qx' : data.orientation.x,
    'qy' : data.orientation.y,
    'qz' : data.orientation.z,
    'qw' : data.orientation.w
    }

    d_imu = DataPoint(d)     

    state_estimate(EKF, d_imu)


def lidar_data(data):

    lidar_time = rospy.get_time()

    d ={'timestamp' : lidar_time,
        'name' : 'lidar',
        'x' : data.pose.pose.position.x,
        'y' : data.pose.pose.position.y,
        'z' : data.pose.pose.position.z,
        'qx' : data.pose.pose.orientation.x,
        'qy' : data.pose.pose.orientation.y,
        'qz' : data.pose.pose.orientation.z,
        'qw' : data.pose.pose.orientation.w}

    d_lidar = DataPoint(d)     

    state_estimate(EKF, d_lidar)

def gps_data(data):

    x = data.latitude
    y = data.longitude

    co_or = utm.from_latlon(x,y)

    px = co_or[0] - utmOffset[0]
    py = co_or[1] - utmOffset[1]

    gps_time = rospy.get_time()
    
    p_measured = np.array([px, py, 0])
    d ={'timestamp' : gps_time,
        'name' : 'gps',
        'x' : px,
        'y' : py}

    d_gps = DataPoint(d)     

    gps_pose.header.stamp = rospy.Time.now()
    gps_pose.header.frame_id = '/camera_init'
    gps_pose.pose.position.x = px
    gps_pose.pose.position.y = py

    state_estimate(EKF, d_gps)

def rtk_data(data):

    x = data.latitude
    y = data.longitude
    z = data.altitude

    co_or = utm.from_latlon(x,y)

    px = co_or[0] - utmOffset[0]
    py = co_or[1] - utmOffset[1]

    rtk_time = rospy.get_time()
    
    p_measured = np.array([px, py, 0])
    d ={'timestamp' : rtk_time,
        'name' : 'rtk',
        'x' : px,
        'y' : py}

    d_rtk = DataPoint(d)     

    rtk_pose.header.stamp = rospy.Time.now()
    rtk_pose.header.frame_id = '/camera_init'
    rtk_pose.pose.position.x = px
    rtk_pose.pose.position.y = py

    state_estimate(EKF, d_rtk)


def locator():

    global ekf_localization, ekf_odom, ekf_path

    rospy.init_node('sensor_fusion_urbannav', anonymous = True)

    ekf_localization_pub = rospy.Publisher('/ekf_localization', PoseWithCovarianceStamped, queue_size = 1)
    ekf_odometry_pub = rospy.Publisher('/ekf_odometry', Odometry, queue_size = 10)
    ekf_path_pub = rospy.Publisher('/ekf_path', Path, queue_size = 1)
    gps_utm = rospy.Publisher('/gps_utm', PoseStamped, queue_size = 1)
    rtk_utm = rospy.Publisher('/span_utm', PoseStamped, queue_size = 1)
   
    print('SENSOR FUSION INITIATED! \n (Waiting for data ...)')


    rospy.Subscriber('/imu/data', Imu, imu_data)
    rospy.Subscriber('/aft_mapped_to_init', Odometry, lidar_data)
    rospy.Subscriber('/navsat/fix', NavSatFix, gps_data)
    rospy.Subscriber('/novatel_data/inspvax', INSPVAX, rtk_data)
    
    publish_rate = 50
    rate = rospy.Rate(publish_rate)

    while not rospy.is_shutdown():

        # ekf_localization_pub.publish(ekf_localization)
        # ekf_path_pub.publish(ekf_path)
        ekf_odometry_pub.publish(ekf_odom)
        gps_utm.publish(gps_pose)
        rtk_utm.publish(rtk_pose)
        rate.sleep()
      
    rospy.spin()
    

if __name__ == '__main__':
    
    global utmOffset

    ''' Initial LATLON position of the vehicle for 2020-03-14-16-45-35.bag '''
    latlonOffset = [22.3304856504,114.179539603, 12.3115594089]  
    
    utmOffset = utm.from_latlon(latlonOffset[0], latlonOffset[1]) # Gives offset to subtract away from gps


    std_lidar = 1000      # standard deviation of lidar position measurement, applies for x and y [m]
    std_gps = 1000        # standard deviation of gps measurement, applies for x and y [m]
    std_rtk = 1000
    # std_radar_r = 0.3     # standard deviation of radius radar measurement [m]
    # std_radar_phi = 0.1   # standard deviation of angle radar measurement [rad]
    # std_radar_rdot = 0.2  # standard deviation of angle rate radar measurement [rad/time]
    std_imu = 1000

    states = 13
    P = np.matrix(np.eye(states))
    F = np.matrix(np.eye(states))
    Q = np.matrix(np.zeros([states,states]))
    G = np.matrix(np.zeros([states,1]))

    ''' Lidar sensor covariance matrix '''
    
    gps_R = np.matrix([[std_gps, 0], 
                       [0, std_gps]])
    rtk_R = np.matrix([[std_rtk, 0], 
                       [0, std_rtk]])

    lidar_R = std_lidar * np.matrix(np.eye(7))
    imu_R = std_imu * np.matrix(np.eye(7))

    lidar_H = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                         [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                         [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

    gps_H = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

    rtk_H = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                       [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

    imu_H = np.matrix([[0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

    d = {
    'number_of_states' : states,
    'initial_process_matrix' : P,
    'imu_covariance_matrix': imu_R,
    'imu_transition_matrix': imu_H,
    'lidar_covariance_matrix': lidar_R, 
    'lidar_transition_matrix': lidar_H,
    'gps_covariance_matrix': gps_R,
    'gps_transition_matrix': gps_H,
    'rtk_covariance_matrix': rtk_R,
    'rtk_transition_matrix': rtk_H,
    'inital_state_transition_matrix': F,
    'initial_noise_matrix': Q, 
    'acceleration_noise_x': 10**5,
    'acceleration_noise_y': 10**5,
    'acceleration_noise_z': 10**5,
    'imu_noise': 10**6
    }
    
    global EKF
    EKF = Extended_KF(d)

    try:
        locator()
    except rospy.ROSInterruptException:
        pass


