#!/bin/bash
import os
import sys

os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'

import time
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool, Int32MultiArray, Float32
from rcl_interfaces.msg import SetParametersResult

from res.res_function import Gimbal
sys.path.append('./devv/src/devv/reta/reta')
from database import GimbalDatabase
print(os.getcwd())
class Kiringimbal(Node):
	def __init__(self):
		super().__init__('Gimbal')

		db = GimbalDatabase()
		self._serial_port = db.serial_port
		self._gimbal_type = db.gimbal_type
		self._source_system = db.source_system
		self._source_component = db.source_component
		self._baud = db.baud
		self._autoreconnect = db.autoreconnect

		self.gimbal = Gimbal(self._gimbal_type, self._autoreconnect, self._serial_port, self._source_system, self._source_component, self._baud)

		self.sub_SOT_bbox = self.create_subscription(
			Float32MultiArray,
			'/SOT/bbox',
			self.SOT_bbox_callback,
			10)

		self.sub_gimbal_lock = self.create_subscription(
			Bool,
			'/gimbal/lock',
			self.gimbal_lock_callback,
			10)
		self.sub_gimbal_manual = self.create_subscription(
			String,
			'/gimbal/manual',
			self.gimbal_manual_callback,
			10)	
		self.sub_gimbal_point = self.create_subscription(
			Float32MultiArray,
			'/gimbal/point',
			self.gimbal_point_callback,
			10)
		self.sub_gimbal_home = self.create_subscription(
			Bool,
			'/gimbal/home',
			self.gimbal_home_callback,
			10)
		self.sub_gimbal_angle = self.create_subscription(
			Int32MultiArray,
			'/gimbal/angle',
			self.gimbal_angle_callback,
			10)
		self.zoom_ratio_sub = self.create_subscription(
			Float32,
			'/camera/frame_zoom_ratio',
			self.zoomlevel_subcallback,
			10)
		self.sub_signal_SOT = self.create_subscription(
			Bool,
			'/signal/SOT',
			self.signal_SOT_callback,
			10)
		self.sub_camera_mode = self.create_subscription(
			String,
			'/mode_camera',
			self.camera_mode_subcallback,
			10)

		self.sub_restart = self.create_subscription(
			String,
			'/restart',
			self.restart_subcallback,
			10
		)

		self.pub_gimbal_orientation = self.create_publisher(
			Float32MultiArray,
			'/gimbal/orientation',
			10
		)
		timer_period = 0.1
		self.timer = self.create_timer(timer_period, self.timer_publisher)
		self.orientation = Float32MultiArray()

		self.master = None

		self.tilt_move = 0
		self.pan_move = 0
		self.signal_lock = False
		self.signal_manual = None
		self.signal_angle = None
		self.signal_point = None
		self.signal_home = False
		self.signal_SOT = False
		self.signal_tracking = False
		self.zoom_ratio = 1
		self.resolution = self.gimbal.resolution
		self.resolution_thermal = self.gimbal.resolution_thermal
		self.pre_bbox = [0, 0, int(self.resolution[0]), int(self.resolution[1])]
		self.mode_camera = 'rgb'
		self.i = 0
		self.j = 0
		self.orientation.data = [0.0, 0.0, 0.0]

	def timer_publisher(self):
		self.pub_gimbal_orientation.publish(self.orientation)

	def parameters_callback(self, params):
		# do some actions, validate parameters, update class attributes, etc.        
		return SetParametersResult(successful=True)

	def filter_gimbal(self, tilt, pan):
		gimbal_limits = {
			'pixyu': (40, 300, -90, -300),
			't3v3': (45, 180, -90, -180)
		}
		if self._gimbal_type in gimbal_limits:
			tilt_limit, pan_limit, tilt_lower, pan_lower = gimbal_limits[self._gimbal_type]
			tilt = min(max(tilt, tilt_lower), tilt_limit)
			pan = min(max(pan, pan_lower), pan_limit)

			tilt = round(tilt,2)
			pan = round(pan,2)
			self.gimbal.tilt_point_pre = tilt
			self.gimbal.pan_point_pre = pan

		return tilt, pan

	def SOT_bbox_callback(self, msg):
		rgb_x1, rgb_y1, rgb_x2, rgb_y2, ther_x1, ther_y1, ther_x2, ther_y2, conf = msg.data
		switcher = {
			'rgb': (rgb_x1, rgb_y1, rgb_x2 - rgb_x1, rgb_y2 - rgb_y1),
			'thermal': (ther_x1, ther_y1, ther_x2 - ther_x1,ther_y2 - ther_y1)
		}
		x, y, w, h = switcher.get(self.mode_camera, (0, 0, 0, 0))
		self.bbox = (x, y, w, h, conf)
		self.signal_tracking = True
		if not self.signal_lock and self.signal_SOT:
			if self.bbox[4] > 0.5:
				if self.i == 0:
					self.gimbal.tilt_point_pre =  - self.gimbal.tilt_point_pre
				switcher = {
					"rgb": self.gimbal.cal_gimbal_pos_pid,
					"thermal": self.gimbal.cal_gimbal_pos_pid_thermal
				}
				func = switcher.get(self.mode_camera)
				pan_signal, tilt_signal, pan_rate, tilt_rate = func(self.bbox, self.pre_bbox, self.zoom_ratio)
				self.pan_move, self.tilt_move = self.gimbal.cal_point_to_move(pan_signal, tilt_signal, pan_rate, tilt_rate, tracking=True)
				self.tilt_move, self.pan_move = self.filter_gimbal(self.tilt_move, self.pan_move)
				self.gimbal.control_gimbal(self.master, 154, self.tilt_move, self.pan_move, 0)
				self.orientation.data = [float(self.tilt_move), float(self.pan_move), 0.0]
				self.i += 1
				self.pre_bbox = self.bbox
			else:
				pass
			self.j = 0

	def gimbal_lock_callback(self, msg):
		self.signal_lock = msg.data

	def gimbal_manual_callback(self, msg):
		self.signal_manual = msg.data
		manual_dict = {'up': (0.62/self.zoom_ratio,0), 
					'down':(-0.62/self.zoom_ratio, 0),
					'left':(0, 0.62/self.zoom_ratio), 
					'right':(0,-0.62/self.zoom_ratio)}
		if not self.signal_lock and not self.signal_SOT and self.signal_manual is not None:
			if self.signal_tracking:
				self.tilt_move = -self.tilt_move
			deltatilt, deltapan = manual_dict[self.signal_manual]
			self.tilt_move += deltatilt
			self.pan_move += deltapan

			self.tilt_move, self.pan_move = self.filter_gimbal(self.tilt_move, self.pan_move)
			self.gimbal.control_gimbal(self.master, 154, -self.tilt_move, self.pan_move, 0)
			self.gimbal.pan_point_pre, self.gimbal.tilt_point_pre = self.pan_move, self.tilt_move
			self.orientation.data = [float(-self.tilt_move), float(self.pan_move), 0.0]
			self.signal_tracking = False

	def gimbal_point_callback(self, msg):
		self.signal_point = msg.data
		self.signal_tracking = False
		if not self.signal_lock and not self.signal_SOT:
			pan_signal, tilt_signal, pan_point, tilt_point = self.gimbal.get_point_to_move(int(self.signal_point[0]), int(self.signal_point[1]), self.zoom_ratio)
			self.pan_move, self.tilt_move = self.gimbal.cal_point_to_move(pan_signal, tilt_signal, pan_point, tilt_point)
			self.tilt_move, self.pan_move = self.filter_gimbal(self.tilt_move, self.pan_move)
			self.gimbal.control_gimbal(self.master, 154, -self.tilt_move, self.pan_move, 0)
			self.orientation.data = [float(-self.tilt_move), float(self.pan_move), 0.0]
			self.gimbal.pan_point_pre, self.gimbal.tilt_point_pre = self.pan_move, self.tilt_move

	def gimbal_home_callback(self, msg):
		self.signal_home = msg.data
		if not self.signal_lock and not self.signal_SOT and self.signal_home:
			self.gimbal.control_gimbal(self.master, 154, 0, 0, 0)
			self.orientation.data = [0.0, 0.0, 0.0]
			self.gimbal.pan_point_pre, self.gimbal.tilt_point_pre = 0, 0
			self.tilt_move, self.pan_move = 0, 0
			self.signal_tracking = False

	def gimbal_angle_callback(self, msg):
		self.signal_angle = msg.data
		self.signal_tracking = False
		if not self.signal_lock and not self.signal_SOT:
			self.tilt_move, self.pan_move = self.signal_angle[0], self.signal_angle[1]
			self.gimbal.control_gimbal(self.master, 154, -self.tilt_move, self.pan_move, self.signal_angle[2])
			self.orientation.data = [float(-self.tilt_move), float(self.pan_move), 0.0]
			self.gimbal.pan_point_pre, self.gimbal.tilt_point_pre = self.signal_angle[1], -self.signal_angle[0]

	def signal_SOT_callback(self, msg):
		self.signal_SOT = msg.data
		if not self.signal_SOT:
			self.i = 0
			if self.j == 0:
				self.gimbal.tilt_point_pre = -self.gimbal.tilt_point_pre
			self.j += 1
			self.bbox = None
			if self.mode_camera == "rgb":
				resolution = self.resolution
			elif self.mode_camera == "thermal":
				resolution = self.resolution_thermal
			self.pre_bbox = [0, 0, int(resolution[0]), int(resolution[1])]
			self.gimbal.pre_bbox = [0, 0, int(resolution[0]), int(resolution[1])]

	def zoomlevel_subcallback(self, msg):
		self.zoom_ratio = msg.data
		self.gimbal.zoom_ratio = self.zoom_ratio

	def camera_mode_subcallback(self, msg):
		self.mode_camera = msg.data
		self.gimbal.mode_camera = self.mode_camera

	def restart_subcallback(self, msg):
		if msg.data == "gimbal":
			sys.exit()

def main(args=None):
	try:
		rclpy.init()
		rosgimbal = Kiringimbal()
		time_start = time.time()
		while True:
			rosgimbal.gimbal.control_gimbal(rosgimbal.gimbal.master_gimbal, 154, 0, 0, 0)
			time.sleep(0.5)
			if (time.time() - time_start) > 3:
				print('Gimbal is ready!')
				break
		rosgimbal.master = rosgimbal.gimbal.master_gimbal
		rclpy.spin(rosgimbal)
		rosgimbal.destroy_node()
		rclpy.shutdown()
	except KeyboardInterrupt:
		sys.exit()

if __name__ == "__main__":
	main()
