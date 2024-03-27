import rospy
import serial
from ublox_gps import UbloxGps
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class GPSPublisher():

    def __init__(self):
        self.publisher_ = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
        rospy.init_node('gps_publisher')
        # windows serial
        #   self.port = serial.Serial('/dev/serial0', baudrate=38400, timeout=1)
        # linux serial
        self.port = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=1)
        self.gps = UbloxGps(self.port)

    def publish(self):
        try: 
            print("Listening for UBX Messages")
            while not rospy.is_shutdown():
                try:
                    geo = self.gps.geo_coords()
                    cov = self.gps.geo_cov()
                    msg = NavSatFix()
                    msg.longitude = geo.lon
                    msg.latitude = geo.lat
                    msg.altitude = 0.001 * float(geo.height)
                    NN = cov.posCovNN
                    NE = cov.posCovNE
                    ND = cov.posCovND
                    EE = cov.posCovEE
                    ED = cov.posCovED
                    DD = cov.posCovDD
                    # Following the convention here: https://www.ros.org/reps/rep-0105.html
                    msg.position_covariance = [EE, NE, ED, NE, NN, ND, ED, ND, DD]
                    msg.position_covariance_type = 3
                    self.publisher_.publish(msg)
                    # rate.sleep()
                except (ValueError, IOError) as err:
                    print(err)
        finally:
            self.port.close()


def main(args=None):
    # rclpy.init(args=args)

    gps_publisher = GPSPublisher()
    gps_publisher.publish()

if __name__ == '__main__':
    main()

