from serial import Serial
import socket
import time

''' RANGEFINDER PAYLOAD LEN '''
RANGEFINDER_PAYLOAD_0  = 3
RANGEFINDER_PAYLOAD_1  = 4
RANGEFINDER_PAYLOAD_2  = 5
RANGEFINDER_PAYLOAD_3  = 6
RANGEFINDER_PAYLOAD_4  = 7
RANGEFINDER_PAYLOAD_5  = 8

''' RANGEFINDER COMMAND ID'''
RANGEFINDER_STATUS = 0x80
RANGEFINDER_RESULT = 0x81
RANGEFINDER_START  = 0x83
RANGEFINDER_STOP   = 0x84
RANGEFINDER_COM_PARAM = 0x9E
RANGEFINDER_READ_COM = 0xBE
RANGEFINDER_TEST = 0xC0
RANGEFINDER_LASER = 0x8F

''' RANGEFINDER RESPOND ID AND IT LENGTH'''
RANGEFINDER_RESPOND = 0x60
RANGEFINDER_RESPOND_LEN = RANGEFINDER_PAYLOAD_0
RANGEFINDER_RANGE   = 0x01
RANGEFINDER_RANGE_LEN = RANGEFINDER_PAYLOAD_4
RANGEFINDER_STATUS_FRAME = 0x20
RANGEFINDER_STATUS_FRAME_LEN = RANGEFINDER_PAYLOAD_3
RANGEFINDER_COM_FRAME  = 0x21
RANGEFINDER_COM_FRAME_LEN = RANGEFINDER_PAYLOAD_3
RANGEFINDER_ERROR  = 0xE0
RANGEFINDER_ERROR_LEN = RANGEFINDER_PAYLOAD_1

''' RANGEFINDER DATA EXPORT METHOD'''
RANGEFINDER_QUERY_MODE = 0
RANGEFINDER_AUTO_MODE  = 1

''' RANGEFINDER MEASUREMENT MODE'''
RANGEFINDER_RANGE_MODE  = 0
RANGEFINDER_SPEED_MODE  = 1
RANGEFINDER_SMALL_MODE  = 2

''' RANGEFINDER BAUDRATE SETTING'''
RANGEFINDER_BAUD_1200  = 0
RANGEFINDER_BAUD_2800  = 1
RANGEFINDER_BAUD_9600  = 2
RANGEFINDER_BAUD_19200 = 4

'''RANGEFINDER MEASUREMENT METHOD'''
RANGEFINDER_CON_RANGE  = 1
RANGEFINDER_DISCON_RANGE  = 0

''' RANGEFINDER MEASUREMENT UNIT '''
RANGEFINDER_M_UNIT  = 0
RANGEFINDER_YARD_UNIT  = 1

''' RANGEFINDER FOG MODE '''
RANGEFINDER_FOG_MODE_EN  = 1
RANGEFINDER_FOG_MODE_DI  = 0

class LaserRangeFinder():
    def __init__(self, connection_type, ip_laser, tcp_port, serial_port, serial_baud):
        if connection_type == 'serial':
            self.lrf_path = serial_port
            self.baudrate = serial_baud
            timeout = 5

            # Create serial port to the device
            while True:
                try:
                    self.serialport = Serial(port=self.lrf_path, baudrate=self.baudrate, timeout=timeout)
                    self.serialport.reset_input_buffer()
                    break
                except:
                    time.sleep(1)
                
        elif connection_type == 'tcp':
            while True:
                try:
                    self.ip_address = ip_laser
                    self.tcp_port = tcp_port    
                    self.tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.tcp.connect((self.ip_address, tcp_port))
                    self.tcp.settimeout(1)
                    break
                except:
                    time.sleep(1)

        self.connection = connection_type

        # Initialize variables
        self.distance = 0.0
        self.angle = 0.0

        # Default device address
        self.header = 0x10 

        # Default settings
        self.range_mode = RANGEFINDER_RANGE_MODE
        self.uint = RANGEFINDER_M_UNIT
        self.fog_mode = RANGEFINDER_FOG_MODE_DI
        self.one_shot = RANGEFINDER_DISCON_RANGE

        # Initialize payload to send to device
        self.payload = self.init_payload()

        # Initialize parameters
        self.signal = False
        self.drone_gps = []
        self.drone_orientation = []
        self.drone_heading = 0
        self.gimbal_orientation = []
        self.bearing = 0.0

        # Earth's mean radius
        self.Radius = 6371009 # meters

    def get_distance_tcp(self):
        self.tcp.send(self.payload)
        _ = self.tcp.recv(4096)
        recv_data = self.tcp.recv(4096)
        self.distance = float(((recv_data[3] << 8 | recv_data[4])/10))
        # Reset if LRF can't find range
        if self.distance == 3276.8:
            self.distance = -1.0

    def get_distance_serial(self):
        # Write payload to serial device
        self.serialport.write(self.payload)
        self.wait_for_response()
        self.wait_for_response()

    def get_distance(self):
        if self.connection == 'serial':
            self.get_distance_serial()
        elif self.connection == 'tcp':
            self.get_distance_tcp()

        return self.distance

    def init_payload(self):
        '''Initialize payload to send to device
        '''

        payload = [None]*RANGEFINDER_PAYLOAD_1
        payload[0] = self.header
        payload[1] = RANGEFINDER_START
        payload[2] = self.range_mode & 0xF | ((self.uint & 0x1) << 4) | \
                    ((self.fog_mode & 0x1) << 5) | ((self.one_shot & 0x1) << 6)
        payload[3] = self.checksum(payload, RANGEFINDER_PAYLOAD_1)

        raw = [payload[i].to_bytes(1,'little') for i in range(0, RANGEFINDER_PAYLOAD_1)]
        raw = b''.join(raw)
        return raw

    def twos_comp(self, val):
        '''Compute two's complement number for checksum
        '''

        if (val & (1 << (8 - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            val = val - (1 << 8)        # compute negative value
        else:
            val = 0x100 - (val & 0xFF)
        return abs(val)                 # return positive value

    def checksum(self, payload, payload_len):
        '''Compute transmit checksum 
        '''

        sum_payload = sum(payload[1:payload_len-1])
        return self.twos_comp(sum_payload)

    def reset_result(self):
        '''Reset range and angle if LRF can't find range
        '''
        
        self.distance = -1.0
        self.angle = -1.0

    def wait_for_response(self):
        '''Wait for response from device after payload is sent
        '''

        recv_byte = [b'']*10
        count_fail = 0
        recv_len = 0
        expect_len = 0
        while (recv_byte[0] != self.header):
            if (count_fail > 7):
                return -1
            try:
                recv_byte[0] = ord(self.serialport.read(1)) # Wait for message header
            except:
                return -1
            recv_len = 1
            count_fail += 1

        recv_byte[1] = ord(self.serialport.read(1))
        recv_len += 1

        if (recv_byte[1] == RANGEFINDER_RESPOND):
            expect_len = RANGEFINDER_RESPOND_LEN
        elif (recv_byte[1] == RANGEFINDER_RANGE):
            expect_len = RANGEFINDER_RANGE_LEN

            recv = list(self.serialport.read(expect_len - recv_len))
            recv_len += len(recv)
            recv_byte[2:] = recv

            self.distance = float(((recv_byte[3] << 8 | recv_byte[4])/10))
            self.angle = int(recv_byte[5])
            if self.angle < -180:
                self.angle = 256 - self.angle

        # Reset if LRF can't find range
        if self.distance == 3276.8:
            self.reset_result()