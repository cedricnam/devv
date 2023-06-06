import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32, Float32MultiArray, Bool, String

import math
from geographiclib.geodesic import Geodesic

X = 0.46152
Y = 0.51576
Z = 0.12784

# Laser Range Finder
from geolocation.rangefinder import LaserRangeFinder

# Database
import sys
import os
sys.path.append('./devv/src/devv/reta/reta')
from database import GeolocationDatabase

class Geolocation(Node):
    def __init__(self):
        super().__init__('Geolocation')

        ### Subscribers
        self.subs_gimbal_orientation = self.create_subscription(
                Float32MultiArray,
                '/gimbal/orientation',
                self.callback_gimbal_orientation,
                10)
        self.subs_signal = self.create_subscription(
                Bool,
                '/geolocation/signal',
                self.callback_signal_geolocation,
                1)
        self.subs_global_position = self.create_subscription(
                Float32MultiArray,
                '/autopilot/gps',
                self.callback_global_position,
                10)
        self.subs_drone_heading = self.create_subscription(
                Int16,
                '/autopilot/heading',
                self.callback_drone_heading,
                10)
        self.sub_restart = self.create_subscription(
			String,
			'/restart',
			self.restart_subcallback,
			10
		)

        ### Publisher
        self.obj_info_pub = self.create_publisher(
                Float32MultiArray,
                '/geolocation/object_info', 
                10)

        ### Attributes
        self.signal = False
        self.msg_object_gps = Float32MultiArray()
        self.global_position = (0,0,0) 
        self.gimbal_orientation = (45,30,0) # (TILT, PAN, ROLL)
        self.geod = Geodesic.WGS84
        self.drone_heading = 0

        # Load params from database
        self.db = GeolocationDatabase()
        self._connection_type = self.db.connection_type
        self._ip_laser = self.db.ip_laser
        self._tcp_port = self.db.tcp_port
        self._serial_port = self.db.serial_port
        self._serial_baud = self.db.serial_baud

        self.laser = LaserRangeFinder(self._connection_type, self._ip_laser, self._tcp_port, self._serial_port, self._serial_baud)
        self.abs_distance = -1.0
        self.vertical_distance = self.global_position[2]
        timer_period_laser = 0.7  # seconds
        self.timer_laser = self.create_timer(timer_period_laser, self.timer_laser_callback)

        print('Geolocation init sucsscessfully!')

        timer_period_pub = 1/30  # seconds
        self.timer_pub = self.create_timer(timer_period_pub, self.timer_pub_callback)

    def callback_signal_geolocation(self, msg):
        self.signal = msg.data

    def timer_laser_callback(self):
        if self.signal:
            try:
                self.abs_distance = self.laser.get_distance()
                if self.abs_distance != -1:
                    self.vertical_distance = math.sin(math.radians(abs(self.gimbal_orientation[0]))) * self.abs_distance
                else:
                    self.vertical_distance = 0
            except:
                pass

    def timer_pub_callback(self):
        if self.signal:
            self.compute_object_info()

    def compute_object_info(self):
        azi = self.azimuth(
                heading=self.drone_heading, 
                pan=-self.gimbal_orientation[1])

        if self.gimbal_orientation[0] == 90:
            latObj = self.global_position[0]
            lonObj = self.global_position[1]
            absolute_distance = self.abs_distance
            horizontal_distance = 0.0
        else:
            try:
                if self.gimbal_orientation[0] != 0:
                    horizontal_distance = self.vertical_distance/(math.tan(math.radians(abs(self.gimbal_orientation[0]))))
                    latObj, lonObj = self.gps_geographiclib(lat=self.global_position[0],
                                                            lon=self.global_position[1],
                                                            azi=azi,
                                                            distance=horizontal_distance)
                    absolute_distance = self.abs_distance
                elif self.abs_distance != -1:
                    horizontal_distance = self.abs_distance
                    latObj, lonObj = self.gps_geographiclib(lat=self.global_position[0],
                                                            lon=self.global_position[1],
                                                            azi=azi,
                                                            distance=horizontal_distance)
                    absolute_distance = self.abs_distance
                else:
                    latObj, lonObj, absolute_distance, horizontal_distance = -1.0, -1.0, -1.0, -1.0
            except:
                latObj, lonObj, absolute_distance, horizontal_distance = -1.0, -1.0, -1.0, -1.0

        self.msg_object_gps.data = [latObj, lonObj, absolute_distance, horizontal_distance]
        self.obj_info_pub.publish(self.msg_object_gps)

    def azimuth(self, heading, pan):
        if heading + pan > 360:
            azi = heading - (360 - pan)
        else:  
            azi = heading + pan
        return azi

    def callback_gimbal_orientation(self, msg):
        self.gimbal_orientation = msg.data

    def callback_global_position(self, msg):
        self.global_position = msg.data

    def callback_drone_heading(self, msg):
        self.drone_heading = msg.data

    def gps_geographiclib(self, lat, lon, azi, distance):
        dir = self.geod.Direct(lat, lon, azi, distance)
        lat, lon = dir['lat2'], dir['lon2']    
        return lat, lon

    def restart_subcallback(self, msg):
        if msg.data == "geolocation":
            sys.exit()


def main(args=None):
    rclpy.init(args=args)

    geolocation = Geolocation()

    rclpy.spin(geolocation)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    geolocation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()