import socket
import time
from parameter import *
from constant import *

class KvHostLink:
    def __init__(self):
        self.socket = None
        self.error = True
        self.init_degrees_right = None
        self.degrees_old_right = 0
        self.init_degrees_left = None
        self.degrees_old_left = 0

    def __str__(self):
        return f'Socket info: {self.socket}'

    def connect_plc(self):
        if self.socket is not None:
            self.close_plc()
        start_time = time.time()
        while (time.time() - start_time) < WAIT_CONNECTING_TIME:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket.settimeout(TIMEOUT)
                self.socket.connect((IP_ADDRESS, PORT))
                print(f'Connect! {self}')
            except socket.error as e:
                print(f'Connection failure. {e}')
            else:
                self.error = False
                time.sleep(0.5)
                break
        return self.error

    def close_plc(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None
            self.error = True
            print("Close!")

    def read_plc(self, target):
        if self.error:
            return self.error, None
        message = 'RD DM{}.{}\r'.format(target['dm'], target['format'])
        try:
            self.socket.send(message.encode('ascii'))
            response_message = self.socket.recv(PACKET_SIZE).decode('utf-8')
        except Exception as e:
            self.error = True
            response_message = None
            print(f'{e}: {self}')
        else:
            if response_message in ERROR:
                print(f'{ERROR[response_message]}: {self}')
                self.error = True
                response_message = None
            else:
                response_message = int(response_message)
        if target == ENCODER_RIGHT:
            response_message = self.convert_angle_right(response_message)
        if target == ENCODER_LEFT:
            response_message = self.convert_angle_left(response_message)
        return self.error, response_message

    def write_plc(self, target, value):
        if self.error:
            return self.error
        message = 'WR DM{}.{} {}\r'.format(target['dm'], target['format'], int(round(value, 0)))
        print(f'{message}')
        try:
            self.socket.send(message.encode('ascii'))
            response_message = self.socket.recv(PACKET_SIZE).decode('utf-8')
        except Exception as e:
            self.error = True
            print(f'{e}: {self}')
        else:
            if response_message != NOT_ERROR:
                print(f'{ERROR[response_message]}: {self}')
                self.error = True
        return self.error

    def convert_angle_left(self, read_angle):
        if self.init_degrees_left is None:
            self.init_degrees_left = read_angle
            return_angle = 0
        else:
            if self.degrees_old_left < -2145960848 and read_angle > 2145960847:
                temp_diff_1 = abs(MAX_MINUS - self.init_degrees_left)
                temp_diff_2 = abs(MAX_PLUS - read_angle)
                return_angle = ((temp_diff_1 + temp_diff_2 + 1)% 36000)
                self.init_degrees_left = return_angle
            elif self.degrees_old_left > 2145960847 and read_angle < -2145960848:
                temp_diff_1 = abs(MAX_PLUS - self.init_degrees_left)
                temp_diff_2 = abs(MAX_MINUS - read_angle)
                return_angle = ((temp_diff_1 + temp_diff_2 + 1)% 36000)
                self.init_degrees_left = return_angle
            else:
                diff = abs(read_angle - self.init_degrees_left)
                return_angle = diff % 36000
        return return_angle

    def convert_angle_right(self, read_angle):
        if self.init_degrees_right is None:
            self.init_degrees_right = read_angle
            return_angle = 0
        else:
            if self.degrees_old_right < -2145960848 and read_angle > 2145960847:
                temp_diff_1 = abs(MAX_MINUS - self.init_degrees_right)
                temp_diff_2 = abs(MAX_PLUS - read_angle)
                return_angle = ((temp_diff_1 + temp_diff_2 + 1)% 36000)
                self.init_degrees_right = return_angle
            elif self.degrees_old_right > 2145960847 and read_angle < -2145960848:
                temp_diff_1 = abs(MAX_PLUS - self.init_degrees_right)
                temp_diff_2 = abs(MAX_MINUS - read_angle)
                return_angle = ((temp_diff_1 + temp_diff_2 + 1)% 36000)
                self.init_degrees_right = return_angle
            else:
                diff = abs(read_angle - self.init_degrees_right)
                return_angle = diff % 36000
        return return_angle


# if __name__ == '__main__':
#     kv = KvHostLink()
#     if not kv.connect_plc():
#         start_time = time.time()
#         while time.time() - start_time < 3:
#             error = kv.write_plc(VELOCITY_RIGHT, 100.5*10)
#             if not error:
#                 error, res = kv.read_plc(ENCODER_RIGHT)
#             else:
#                 break
#             if not error:
#                 encoder_data = res*100
#             else:
#                 break
#             time.sleep(0.1)
#         kv.close_plc()

