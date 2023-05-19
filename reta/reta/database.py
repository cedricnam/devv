from pymongo import MongoClient

class CommunicationDatabase:
	def __init__(self, ip_address='192.168.0.215', port=27017):
		self.ip_address = ip_address
		self.port = port
		self.client = MongoClient(f'mongodb://{ip_address}:{port}/')
		self.database = self.client['kirins']
		self.collection = self.database['Communication']
		self.load_data()

	def load_data(self):
		data = self.collection.find()
		self._ip = data[0]['ip']

	@property
	def ip(self):
		return self._ip

	@ip.setter
	def ip(self, value):
		self._ip = value
		self.collection.update_one({}, {'$set': {'ip': value}})

class GimbalDatabase:
	def __init__(self, ip_address='192.168.0.215', port=27017):
		self.ip_address = ip_address
		self.port = port
		self.client = MongoClient(f'mongodb://{ip_address}:{port}/')
		self.database = self.client['kirins']
		self.gimbal_collection = self.database['Gimbal']
		self.advance_gb_collection = self.database['Gimbal Advance']
		self.load_data()

	def load_data(self):
		data = self.gimbal_collection.find()
		self._gimbal_type = data[0]['gimbal_type']
		gimbal_data = self.advance_gb_collection.find({'type': f'{self._gimbal_type}'})[0]
		rgb_data = gimbal_data['rgb']
		thermal_data = gimbal_data['thermal']
		pid_rgb = gimbal_data['pid_rgb']
		pid_thermal = gimbal_data['pid_thermal']

		self._autoreconnect = data[0]['autoreconnect']
		self._serial_port = data[0]['serial_port']
		self._source_system = data[0]['source_system']
		self._source_component = data[0]['source_component']
		self._baud = data[0]['baud']

		self._pan_rgb1 = rgb_data['pan_rgb1']
		self._pan_rgb2 = rgb_data['pan_rgb2']
		self._tilt_rgb1 = rgb_data['tilt_rgb1']
		self._tilt_rgb2 = rgb_data['tilt_rgb2']

		self._pan_thermal1 = thermal_data['pan_thermal1']
		self._pan_thermal2 = thermal_data['pan_thermal2']
		self._tilt_thermal1 = thermal_data['tilt_thermal1']
		self._tilt_thermal2 = thermal_data['tilt_thermal2']

		self._pan_pid_rgb = pid_rgb['pan']['pid']
		self._pan_pid_thermal = pid_rgb['tilt']['pid']
		self._tilt_pid_rgb = pid_thermal['pan']['pid']
		self._tilt_pid_thermal = pid_thermal['tilt']['pid']

	@property
	def gimbal_type(self):
		return self._gimbal_type

	@gimbal_type.setter
	def gimbal_type(self, value):
		allowed_values = list(self.advance_gb_collection.distinct('type'))
		if value not in allowed_values:
			value = 'pixyu'
		self._gimbal_type = value
		self.gimbal_collection.update_one({}, {'$set': {'gimbal_type': value}})

	@property
	def autoreconnect(self):
		return self._autoreconnect
	
	@autoreconnect.setter
	def autoreconnect(self, value):
		allowed_values = [True, False]
		if value not in allowed_values:
			value = True
		self._autoreconnect = value
		self.gimbal_collection.update_one({}, {'$set': {'autoreconnect': value}})

	@property
	def serial_port(self):
		return self._serial_port
	
	@serial_port.setter
	def serial_port(self, value):
		self._serial_port = value
		self.gimbal_collection.update_one({}, {'$set': {'serial_port': value}})

	@property
	def source_system(self):
		return self._source_system
	
	@source_system.setter
	def source_system(self, value):
		self._source_system = value
		self.gimbal_collection.update_one({}, {'$set': {'source_system': value}})

	@property
	def source_component(self):
		return self._source_component

	@source_component.setter
	def source_component(self, value):
		self._source_component = value
		self.gimbal_collection.update_one({}, {'$set': {'source_component': value}})

	@property
	def baud(self):
		return self._baud
	
	@baud.setter
	def baud(self, value):
		self._baud = value
		self.gimbal_collection.update_one({}, {'$set': {'baud': value}})
	
	@property
	def pan_rgb1(self):
		return self._pan_rgb1

	@pan_rgb1.setter
	def pan_rgb1(self, value):
		self._pan_rgb1 = value
		self.gimbal_collection.update_one({}, {'$set': {'pan_rgb1': value}})

	@property
	def pan_rgb2(self):
		return self._pan_rgb2

	@pan_rgb2.setter
	def pan_rgb2(self, value):
		self._pan_rgb2 = value
		self.gimbal_collection.update_one({}, {'$set': {'pan_rgb2': value}})

	@property
	def tilt_rgb1(self):
		return self._tilt_rgb1

	@tilt_rgb1.setter
	def tilt_rgb1(self, value):
		self._tilt_rgb1 = value
		self.gimbal_collection.update_one({}, {'$set': {'tilt_rgb1': value}})
	
	@property
	def tilt_rgb2(self):
		return self._tilt_rgb2

	@tilt_rgb2.setter
	def tilt_rgb2(self, value):
		self._tilt_rgb2 = value
		self.gimbal_collection.update_one({}, {'$set': {'tilt_rgb2': value}})

	@property
	def pan_thermal1(self):
		return self._pan_thermal1

	@pan_thermal1.setter
	def pan_thermal1(self, value):
		self._pan_thermal1 = value
		self.gimbal_collection.update_one({}, {'$set': {'pan_thermal1': value}})

	@property
	def pan_thermal2(self):
		return self._pan_thermal2

	@pan_thermal2.setter
	def pan_thermal2(self, value):
		self._pan_thermal2 = value
		self.gimbal_collection.update_one({}, {'$set': {'pan_thermal2': value}})
	
	@property
	def tilt_thermal1(self):
		return self._tilt_thermal1

	@tilt_thermal1.setter
	def tilt_thermal1(self, value):
		self._tilt_thermal1 = value
		self.gimbal_collection.update_one({}, {'$set': {'tilt_thermal1': value}})

	@property
	def tilt_thermal2(self):
		return self._tilt_thermal2

	@tilt_thermal2.setter
	def tilt_thermal2(self, value):
		self._tilt_thermal2 = value
		self.gimbal_collection.update_one({}, {'$set': {'tilt_thermal2': value}})

	@property
	def pan_pid_rgb(self):
		return self._pan_pid_rgb

	@pan_pid_rgb.setter
	def pan_pid_rgb(self, value):
		self._pan_pid_rgb = value
		self.gimbal_collection.update_one({}, {'$set': {'pan_pid_rgb': value}})

	@property
	def pan_pid_thermal(self):
		return self._pan_pid_thermal

	@pan_pid_thermal.setter
	def pan_pid_thermal(self, value):
		self._pan_pid_thermal = value
		self.gimbal_collection.update_one({}, {'$set': {'pan_pid_thermal': value}})

	@property
	def tilt_pid_rgb(self):
		return self._tilt_pid_rgb

	@tilt_pid_rgb.setter
	def tilt_pid_rgb(self, value):
		self._tilt_pid_rgb = value
		self.gimbal_collection.update_one({}, {'$set': {'tilt_pid_rgb': value}})

	@property
	def tilt_pid_thermal(self):
		return self._tilt_pid_thermal

	@tilt_pid_thermal.setter
	def tilt_pid_thermal(self, value):
		self._tilt_pid_thermal = value
		self.gimbal_collection.update_one({}, {'$set': {'tilt_pid_thermal': value}})
		
class AutopilotDatabase:
	def __init__(self, ip_address='192.168.0.215', port=27017):
		self.ip_address = ip_address
		self.port = port
		self.client = MongoClient(f'mongodb://{ip_address}:{port}/')
		self.database = self.client['kirins']
		self.collection = self.database['Autopilot']
		self.load_data()

	def load_data(self):
		data = self.collection.find()
		self._autopilot_ip = data[0]['autopilot_ip']
		self._source_system = data[0]['source_system']
		self._source_component = data[0]['source_component']
		self._baud = data[0]['baud']

	@property
	def autopilot_ip(self):
		return self._autopilot_ip

	@autopilot_ip.setter
	def autopilot_ip(self, value):
		self._autopilot_ip = value
		self.collection.update_one({}, {'$set': {'autopilot_ip': value}})

	@property
	def source_system(self):
		return self._source_system

	@source_system.setter
	def source_system(self, value):
		self._source_system = value
		self.collection.update_one({}, {'$set': {'source_system': value}})

	@property
	def source_component(self):
		return self._source_component

	@source_component.setter
	def source_component(self, value):
		self._source_component = value
		self.collection.update_one({}, {'$set': {'source_component': value}})

	@property
	def baud(self):
		return self._baud

	@baud.setter
	def baud(self, value):
		self._baud = value
		self.collection.update_one({}, {'$set': {'baud': value}})
	
class CameraDatabase:
	def __init__(self, ip_address='192.168.0.215', port=27017):
		self.ip_address = ip_address
		self.port = port
		self.client = MongoClient(f'mongodb://{ip_address}:{port}/')
		self.database = self.client['kirins']
		self.collection = self.database['Camera']
		self.load_data()

	def load_data(self):
		data = self.collection.find()
		self._cam_type = data[0]['cam_type']
		self._connection_type = data[0]['connection_type']
		self._ip_camera = data[0]['ip_camera']
		self._tcp_port = data[0]['tcp_port']
		self._serial_port = data[0]['serial_port']
		self._serial_baud = data[0]['serial_baud']

	@property
	def cam_type(self):
		return self._cam_type

	@cam_type.setter
	def cam_type(self, value):
		self._cam_type = value
		self.collection.update_one({}, {'$set': {'cam_type': value}})

	@property
	def connection_type(self):
		return self._connection_type
	
	@connection_type.setter
	def connection_type(self, value):
		self._connection_type = value
		self.collection.update_one({}, {'$set': {'connection_type': value}})

	@property
	def ip_camera(self):
		return self._ip_camera

	@ip_camera.setter
	def ip_camera(self, value):
		self._ip_camera = value
		self.collection.update_one({}, {'$set': {'ip_camera': value}})

	@property
	def tcp_port(self):
		return self._tcp_port

	@tcp_port.setter
	def tcp_port(self, value):
		self._tcp_port = value
		self.collection.update_one({}, {'$set': {'tcp_port': value}})

	@property
	def serial_port(self):
		return self._serial_port

	@serial_port.setter
	def serial_port(self, value):
		self._serial_port = value
		self.collection.update_one({}, {'$set': {'serial_port': value}})

	@property
	def serial_baud(self):
		return self._serial_baud

	@serial_baud.setter
	def serial_baud(self, value):
		self._serial_baud = value
		self.collection.update_one({}, {'$set': {'serial_baud': value}})
	
class GeolocationDatabase:
	def __init__(self, ip_address='192.168.0.215', port=27017):
		self.ip_address = ip_address
		self.port = port
		self.client = MongoClient(f'mongodb://{ip_address}:{port}/')
		self.database = self.client['kirins']
		self.collection = self.database['Geolocation']
		self.load_data()

	def load_data(self):
		data = self.collection.find()
		self._availability = data[0]['availability']
		self._connection_type = data[0]['connection_type']
		self._ip_laser = data[0]['ip_laser']
		self._tcp_port = data[0]['tcp_port']
		self._serial_port = data[0]['serial_port']
		self._serial_baud = data[0]['serial_baud']

	@property
	def availability(self):
		return self._availability
	
	@availability.setter
	def availability(self, value):
		self._availability = value
		self.collection.update_one({}, {'$set': {'availability': value}})
	
	@property
	def connection_type(self):
		return self._connection_type
	
	@connection_type.setter
	def connection_type(self, value):
		self._connection_type = value
		self.collection.update_one({}, {'$set': {'connection_type': value}})

	@property
	def ip_laser(self):
		return self._ip_laser
	
	@ip_laser.setter
	def ip_laser(self, value):
		self._ip_laser = value
		self.collection.update_one({}, {'$set': {'ip_laser': value}})

	@property
	def tcp_port(self):
		return self._tcp_port
	
	@tcp_port.setter
	def laser_tcp_port(self, value):
		self._laser_tcp_port = value
		self.collection.update_one({}, {'$set': {'laser_tcp_port': value}})

	@property
	def serial_port(self):
		return self._serial_port
	
	@serial_port.setter
	def serial_port(self, value):
		self._serial_port = value
		self.collection.update_one({}, {'$set': {'serial_port': value}})

	@property
	def serial_baud(self):
		return self._serial_baud
	
	@serial_baud.setter
	def serial_baud(self, value):
		self._serial_baud = value
		self.collection.update_one({}, {'$set': {'serial_baud': value}})
	
class DeepStreamDatabase:
	def __init__(self, ip_address='192.168.0.215', port=27017):
		self.ip_address = ip_address
		self.port = port
		self.client = MongoClient(f'mongodb://{ip_address}:{port}/')
		self.database = self.client['kirins']
		self.collection = self.database['DeepStream']
		self.load_data()

	def load_data(self):
		data = self.collection.find()
		common = data[0]['Common']
		livestream = data[0]['Livestream']
		snapshot = data[0]['Snapshot']
		recording = data[0]['Recording']
		download = data[0]['Download']

		self._input_rgb = common['input_rgb']
		self._input_thermal = common['input_thermal']
		self._ip = common['ip']

		self._mount_point = livestream['mount_point']
		self._port_livestream = livestream['port']
		self._bitrate = livestream['bitrate']
		self._frame_rate = livestream['frame_rate']
		self._GOP_size = livestream['GOP_size']
		self._codec = livestream['codec']
		self._control_rate = livestream['control_rate']
		self._resolution = livestream['resolution']

		self._dir_path_snapshot = snapshot['dir_path']
		self._format_snapshot = snapshot['format']
		self._duration_snapshot = snapshot['duration']
		self._resolution_snapshot = snapshot['resolution']

		self._dir_path_recording = recording['dir_path']
		self._format_recording = recording['format']
		self._bitrate_recording = recording['bitrate']
		self._frame_rate_recording = recording['frame_rate']
		self._GOP_size_recording = recording['GOP_size']
		self._codec_recording = recording['codec']
		self._resolution_recording = recording['resolution']

		self._user = download['user']
		self._address = download['address']
		self._password = download['password']
		self._images_path = download['images_path']
		self._videos_path = download['videos_path']
		self._auto_download = download['auto_download']

	@property
	def input_rgb(self):
		return self._input_rgb

	@input_rgb.setter
	def input_rgb(self, value):
		self._input_rgb = value
		self.collection.update_one({}, {'$set': {'Common.input_rgb': value}})

	@property
	def input_thermal(self):
		return self._input_thermal

	@input_thermal.setter
	def input_thermal(self, value):
		self._input_thermal = value
		self.collection.update_one({}, {'$set': {'Common.input_thermal': value}})

	@property
	def ip(self):
		return self._ip

	@ip.setter
	def ip(self, value):
		self._ip = value
		self.collection.update_one({}, {'$set': {'Common.ip': value}})

	@property
	def mount_point(self):
		return self._mount_point

	@mount_point.setter
	def mount_point(self, value):
		self._mount_point = value
		self.collection.update_one({}, {'$set': {'Livestream.mount_point': value}})

	@property
	def port_livestream(self):
		return self._port_livestream

	@port_livestream.setter
	def port_livestream(self, value):
		self._port_livestream = value
		self.collection.update_one({}, {'$set': {'Livestream.port': value}})

	@property
	def bitrate(self):
		return self._bitrate

	@bitrate.setter
	def bitrate(self, value):
		self._bitrate = value
		self.collection.update_one({}, {'$set': {'Livestream.bitrate': value}})

	@property
	def frame_rate(self):
		return self._frame_rate

	@frame_rate.setter
	def frame_rate(self, value):
		self._frame_rate = value
		self.collection.update_one({}, {'$set': {'Livestream.frame_rate': value}})

	@property
	def GOP_size(self):
		return self._GOP_size

	@GOP_size.setter
	def GOP_size(self, value):
		self._GOP_size = value
		self.collection.update_one({}, {'$set': {'Livestream.GOP_size': value}})

	@property
	def codec(self):
		return self._codec

	@codec.setter
	def codec(self, value):
		self._codec = value
		self.collection.update_one({}, {'$set': {'Livestream.codec': value}})

	@property
	def control_rate(self):
		return self._control_rate

	@control_rate.setter
	def control_rate(self, value):
		self._control_rate = value
		self.collection.update_one({}, {'$set': {'Livestream.control_rate': value}})

	@property
	def resolution(self):
		return self._resolution

	@resolution.setter
	def resolution(self, value):
		self._resolution = value
		self.collection.update_one({}, {'$set': {'Livestream.resolution': value}})

	@property
	def dir_path_snapshot(self):
		return self._dir_path_snapshot

	@dir_path_snapshot.setter
	def dir_path_snapshot(self, value):
		self._dir_path_snapshot = value
		self.collection.update_one({}, {'$set': {'Snapshot.dir_path': value}})

	@property
	def format_snapshot(self):
		return self._format_snapshot

	@format_snapshot.setter
	def format_snapshot(self, value):
		self._format_snapshot = value
		self.collection.update_one({}, {'$set': {'Snapshot.format': value}})

	@property
	def duration_snapshot(self):
		return self._duration_snapshot

	@duration_snapshot.setter
	def duration_snapshot(self, value):
		self._duration_snapshot = value
		self.collection.update_one({}, {'$set': {'Snapshot.duration': value}})

	@property
	def resolution_snapshot(self):
		return self._resolution_snapshot

	@resolution_snapshot.setter
	def resolution_snapshot(self, value):
		self._resolution_snapshot = value
		self.collection.update_one({}, {'$set': {'Snapshot.resolution': value}})

	@property
	def dir_path_recording(self):
		return self._dir_path_recording

	@dir_path_recording.setter
	def dir_path_recording(self, value):
		self._dir_path_recording = value
		self.collection.update_one({}, {'$set': {'Recording.dir_path': value}})

	@property
	def format_recording(self):
		return self._format_recording

	@format_recording.setter
	def format_recording(self, value):
		self._format_recording = value
		self.collection.update_one({}, {'$set': {'Recording.format': value}})

	@property
	def bitrate_recording(self):
		return self._bitrate_recording

	@bitrate_recording.setter
	def bitrate_recording(self, value):
		self._bitrate_recording = value
		self.collection.update_one({}, {'$set': {'Recording.bitrate': value}})

	@property
	def frame_rate_recording(self):
		return self._frame_rate_recording

	@frame_rate_recording.setter
	def frame_rate_recording(self, value):
		self._frame_rate_recording = value
		self.collection.update_one({}, {'$set': {'Recording.frame_rate': value}})

	@property
	def GOP_size_recording(self):
		return self._GOP_size_recording

	@GOP_size_recording.setter
	def GOP_size_recording(self, value):
		self._GOP_size_recording = value
		self.collection.update_one({}, {'$set': {'Recording.GOP_size': value}})

	@property
	def codec_recording(self):
		return self._codec_recording

	@codec_recording.setter
	def codec_recording(self, value):
		self._codec_recording = value
		self.collection.update_one({}, {'$set': {'Recording.codec': value}})

	@property
	def resolution_recording(self):
		return self._resolution_recording

	@resolution_recording.setter
	def resolution_recording(self, value):
		self._resolution_recording = value
		self.collection.update_one({}, {'$set': {'Recording.resolution': value}})

	@property
	def user(self):
		return self._user
	
	@user.setter
	def user(self, value):
		self._user = value
		self.collection.update_one({}, {'$set': {'Download.user': value}})

	@property
	def address(self):
		return self._address
	
	@address.setter
	def address(self, value):
		self._address = value
		self.collection.update_one({}, {'$set': {'Download.address': value}})
	
	@property
	def password(self):
		return self._password
	
	@password.setter
	def password(self, value):
		self._password = value
		self.collection.update_one({}, {'$set': {'Download.password': value}})

	@property
	def images_path(self):
		return self._images_path
	
	@images_path.setter
	def images_path(self, value):
		self._images_path = value
		self.collection.update_one({}, {'$set': {'Download.images_path': value}})
	
	@property
	def videos_path(self):
		return self._videos_path
	
	@videos_path.setter
	def videos_path(self, value):
		self._videos_path = value
		self.collection.update_one({}, {'$set': {'Download.videos_path': value}})

	@property
	def auto_download(self):
		return self._auto_download
	
	@auto_download.setter
	def auto_download(self, value):
		self._auto_download = value
		self.collection.update_one({}, {'$set': {'Download.auto_download': value}})
