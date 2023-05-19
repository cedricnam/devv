#!/bin/bash
import os
os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'

import time
import operator
import sys
from threading import Thread
from pymavlink import mavutil

sys.path.append('./devv/src/devv/reta/reta')
from database import GimbalDatabase

class Gimbal():
	def __init__(self, gimbal_type, autoreconnect, device, source_system, source_component, baud):

		self.master_gimbal = self.init_mav_connector(gimbal_type, autoreconnect, device, source_system, source_component, baud)
		self.Thread_shake_hands = Thread(target=self.shake_hands, args=[self.master_gimbal], daemon=True)
		self.Thread_shake_hands.start()

		self.resolution = [1280,720]
		self.resolution_thermal = [1280,720]
		self.window_size = 30
		self.tilt_move = 0
		self.pan_move = 0
		self.time_step=1/30 # 1/fps

		self.pan_point_pre = 0
		self.tilt_point_pre = 0
		self.origin_center_x = int(self.resolution[0]/2)
		self.origin_center_y = int(self.resolution[1]/2)
		self.mode_camera = 'rgb'
		db = GimbalDatabase()

		self.pan_rgb1 = db.pan_rgb1
		self.pan_rgb2 = db.pan_rgb2
		self.tilt_rgb1 = db.tilt_rgb1
		self.tilt_rgb2 = db.tilt_rgb2

		self.pan_thermal1 = db.pan_thermal1
		self.pan_thermal2 = db.pan_thermal2
		self.tilt_thermal1 = db.tilt_thermal1
		self.tilt_thermal2 = db.tilt_thermal2

		self.pid_pan_rgb = db.pan_pid_rgb
		self.pid_tilt_rgb = db.tilt_pid_rgb
		self.pid_pan_thermal = db.pan_pid_thermal
		self.pid_tilt_thermal = db.tilt_pid_thermal
		
	def init_mav_connector(self, gimbal_type, autoreconnect, device, source_system, source_component, baud=115200, timeout=0):
		print('[ACTION]: Mavlink is connecting to', device)

		master = mavutil.mavlink_connection(device, baud, source_system, source_component, autoreconnect=True)

		msg = None
		while not msg:
			master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GIMBAL, mavutil.mavlink.MAV_TYPE_GENERIC, 0, 0, 0)
			msg = master.recv_match()
			time.sleep(0.1)
		master.wait_heartbeat()

		try:
			master.mav.request_data_stream_send(master.target_system, 
												master.target_component, 
												mavutil.mavlink.MAV_DATA_STREAM_ALL, 
												stream_rate=20, 
												min_interval=1)
		except:
			pass

		print('[MESSAGE]: Successful Connection to {} ({}, {})'.format(device, master.target_system, master.target_component))

		return master

	def shake_hands(self, master):
		while True:
			master.mav.heartbeat_send(2,0,0,0,0,3)
			master.wait_heartbeat()
			time.sleep(0.5)

	def control_gimbal(self, master, gimbal_comp_id, tilt, pan, roll):
		"""
		Func for controling til, roll, pan of Gremsy Gimbal:

		- master: mavlink master
		- gimbal_comp_id (MAV_COMP_ID_GIMBAL): [int] -> https://mavlink.io/en/messages/common.html#MAV_COMPONENT
		- tilt (pitch): [float] -150 to +150 degree
		- roll (roll): [float] +80 to -264 degree
		- pan (pan): [float] -345 to +345 degree
		
		Maximum Controlled Rotation Speed = 180 degree/sec
		"""
		master.mav.command_long_send(
				master.target_system,
				gimbal_comp_id,
				mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
				1,
				tilt,
				roll,
				pan,
				0, 0, 0,
				mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)

	def cal_gimbal_pos_pid(self, bbox, pre_bbox, zoom_ratio=1):
		"""
		Func for calculating tilt, pan of Gremsy Gimbal from boudingbox and pre_boudingbox
		"""
		
		integral = [0, 0]
		pan_rate = 0
		tilt_rate = 0
		pan_signal = 'stop'
		tilt_signal = 'stop'

		self.origin_center_x = int(self.resolution[0]/2)
		self.origin_center_y = int(self.resolution[1]/2)

		box_center_x = int(bbox[0] + bbox[2]/2)
		box_center_y = int(bbox[1] + bbox[3]/2)
		pre_box_center_x = int(pre_bbox[0] + pre_bbox[2]/2)
		pre_box_center_y = int(pre_bbox[1] + pre_bbox[3]/2)

		dx = 0
		dy = 0
		pre_dx=0
		pre_dy=0

		dx = self.origin_center_x - box_center_x
		dy = self.origin_center_y - box_center_y

		pre_dx= self.origin_center_x - pre_box_center_x
		pre_dy= self.origin_center_y - pre_box_center_y

		if abs(dx) < self.window_size: dx = 0
		if abs(dy) < self.window_size: dy = 0

		pan_signal = 'stop' if dx == 0 else ('left' if dx > 0 else 'right')
		tilt_signal = 'stop' if dy == 0 else ('down' if dy > 0 else 'up')

		if zoom_ratio > 1:
			dx /= zoom_ratio
			dy /= zoom_ratio
			pre_dx /= zoom_ratio
			pre_dy /= zoom_ratio

		dx /= self.origin_center_x
		dy /= self.origin_center_y
		pre_dx /= self.origin_center_x
		pre_dy /= self.origin_center_y
		
		integral[0] += dx*self.time_step
		integral[1] += dy*self.time_step

		pan_rate = abs(self.pid_pan_rgb[0] * dx + self.pid_pan_rgb[1] * integral[0] + self.pid_pan_rgb[2] * (dx - pre_dx) / self.time_step)
		tilt_rate = abs(self.pid_tilt_rgb[0] * dy + self.pid_tilt_rgb[1] * integral[1] + self.pid_tilt_rgb[2] * (dy - pre_dy) / self.time_step)

		return pan_signal, tilt_signal, pan_rate, tilt_rate

	def cal_gimbal_pos_pid_thermal(self, bbox, pre_bbox):
		"""
		Func for calculating tilt, pan of Gremsy Gimbal from boudingbox and pre_boudingbox
		"""

		integral = [0,0]
		pan_rate = 0
		tilt_rate = 0
		pan_signal = 'stop'
		tilt_signal = 'stop'

		self.origin_center_x = int(self.resolution_thermal[0]/2)
		self.origin_center_y = int(self.resolution_thermal[1]/2)

		box_center_x = int(bbox[0] + bbox[2]/2)
		box_center_y = int(bbox[1] + bbox[3]/2)
		pre_box_center_x = int(pre_bbox[0] + pre_bbox[2]/2)
		pre_box_center_y = int(pre_bbox[1] + pre_bbox[3]/2)

		dx = 0
		dy = 0
		pre_dx=0
		pre_dy=0

		dx = self.origin_center_x - box_center_x
		dy = self.origin_center_y - box_center_y

		pre_dx= self.origin_center_x - pre_box_center_x
		pre_dy= self.origin_center_y - pre_box_center_y

		if abs(dx) < self.window_size: dx = 0
		if abs(dy) < self.window_size: dy = 0

		pan_signal = 'stop' if dx == 0 else ('left' if dx > 0 else 'right')
		tilt_signal = 'stop' if dy == 0 else ('down' if dy > 0 else 'up')
		
		dx /= self.origin_center_x
		dy /= self.origin_center_y
		pre_dx /= self.origin_center_x
		pre_dy /= self.origin_center_y

		integral[0] += dx*self.time_step
		integral[1] += dy*self.time_step
	
		pan_rate = abs(self.pid_pan_thermal[0] * dx + self.pid_pan_thermal[1] * integral[0] + self.pid_pan_thermal[2] * (dx - pre_dx) / self.time_step)
		tilt_rate = abs(self.pid_tilt_thermal[0] * dy + self.pid_tilt_thermal[1] * integral[1] + self.pid_tilt_thermal[2] * (dy - pre_dy) / self.time_step)

		return pan_signal, tilt_signal, pan_rate, tilt_rate

	def get_point_to_move(self, x_percent, y_percent, zoom_ratio=1):
		"""
		Func for calculating tilt, pan of Gremsy Gimbal at move Point to move
		"""

		pan_point = (self.resolution[0]*x_percent)/100
		tilt_point = (self.resolution[1]*y_percent)/100

		if (x_percent != 0) and (y_percent != 0): 
			pan_signal = 'right' if x_percent > 0 else 'left'
			tilt_signal = 'down' if y_percent > 0 else 'up'
		else: 
			pan_signal = 'stop'
			tilt_signal = 'stop'
		
		if self.mode_camera == "rgb":
			if 0 < abs(x_percent) < 50:
				pan_point = pan_point * self.pan_rgb1 + self.pan_rgb2 * (-1 if x_percent < 0 else 1)
			if 0 < abs(y_percent) < 50:
				tilt_point = tilt_point * self.tilt_rgb1 + self.tilt_rgb2 * (-1 if y_percent < 0 else 1)

		elif self.mode_camera == 'thermal':
			if 0 < abs(y_percent) < 50:
				tilt_point = tilt_point * self.pan_thermal1 + self.pan_thermal2 * (-1 if y_percent < 0 else 1)
			if 0 < abs(x_percent) < 50:
				pan_point = pan_point * self.pan_thermal1 + self.pan_thermal2 * (-1 if x_percent < 0 else 1)
			zoom_ratio = 1

		pan_point = -pan_point/zoom_ratio
		tilt_point = -tilt_point/zoom_ratio

		return pan_signal, tilt_signal, pan_point, tilt_point

	def cal_point_to_move(self, pan_signal='stop', tilt_signal='stop', pan_rate = 0, tilt_rate = 0, tracking=False):
		"""
		Move gimbal to given position

		Args:
		tilt (float): tilt angle is degrees (0 is forward) (tilt)
		roll (float,optional): roll angle is degrees (0 is forward) 
		pan (float,optional): pan angle is degrees (0 is forcal_point_to_moveward) (pan)
		"""
		# Define a dictionary that maps signals to functions
		if tracking:
			pan_dict = {'right': operator.sub, 'left': operator.add, 'stop': lambda x,y: x}
			tilt_dict = {'up': operator.add, 'down': operator.sub, 'stop': lambda x,y: x}
		else:
			pan_dict = {'right': operator.add, 'left': operator.add, 'stop': lambda x,y: x}
			tilt_dict = {'up': operator.add, 'down': operator.add, 'stop': lambda x,y: x}

		# Use the dictionaries to update the pan and tilt movements
		self.pan_move = pan_dict[pan_signal](self.pan_point_pre, pan_rate)
		self.pan_point_pre = self.pan_move
		self.tilt_move = tilt_dict[tilt_signal](self.tilt_point_pre, tilt_rate)
		self.tilt_point_pre = self.tilt_move

		return self.pan_move, self.tilt_move