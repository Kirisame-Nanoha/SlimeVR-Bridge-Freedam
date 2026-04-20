import asyncio
import struct
import socket
import math
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import AsyncIOOSCUDPServer

# ==========================================
# SlimeVR Server 送信先
# ==========================================
SLIME_IP = "127.0.0.1"
SLIME_PORT = 6969
OSC_RECEIVE_PORT = 3333  # OSC受信するポート

# ==========================================
# SlimeVRパケット生成関数（元コード維持）
# ==========================================
def build_rotation_packet(qw, qx, qy, qz, packet_counter, sensor_id=0):
    buffer = b'\x00\x00\x00\x11'  # packet type = 17
    buffer += struct.pack('>Q', packet_counter)
    buffer += struct.pack('B', sensor_id)
    buffer += struct.pack('B', 1)
    buffer += struct.pack('>ffff', -qx, qz, qy, qw)  # 座標変換: x→-x, y→z, z→y
    buffer += struct.pack('B', 0)
    return buffer

# ==========================================
# クォータニオンをX軸+90度回転 + YZ入替　必要に応じて変更する
# ==========================================
def rotate_quaternion(quat, axis, angle_degrees):
    angle_radians = math.radians(angle_degrees)
    sin_half = math.sin(angle_radians / 2)
    cos_half = math.cos(angle_radians / 2)

    w2 = cos_half
    x2 = axis[0] * sin_half
    y2 = axis[1] * sin_half
    z2 = axis[2] * sin_half

    w1, x1, y1, z1 = quat

    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )

def convert_and_transform_quaternion(qw, qx, qy, qz):
    original_quat = (qw, qx, -qy, qz)  
    rotated_quat = rotate_quaternion(original_quat, (1, 0, 0), 90)  # X軸+90°
    final_quat = (
        rotated_quat[0],  # w
        rotated_quat[1],  # x
        rotated_quat[3],  # z -> y
        rotated_quat[2],  # y -> z
    )
    return final_quat


def build_handshake(mac, fw, packet_counter):
    fw_string = f"MoSlime/OSC - Puck Version:{fw}"
    buffer = b'\x00\x00\x00\x03'                   # packet type = 3
    buffer += struct.pack('>Q', packet_counter)    # packet counter
    buffer += struct.pack('>I', 10)                # board type
    buffer += struct.pack('>I', 8)                 # IMU type
    buffer += struct.pack('>I', 7)                 # MCU type
    buffer += struct.pack('>III', 0, 0, 0)         # IMU info
    buffer += struct.pack('>I', int(fw.replace('.', '')))  # FW version as int
    buffer += struct.pack('B', len(fw_string))     # string length
    buffer += fw_string.encode('utf-8')            # firmware string
    mac_bytes = bytes.fromhex(mac.replace(':', ''))
    buffer += struct.pack('6s', mac_bytes)         # MAC
    buffer += struct.pack('B', 255)                # padding
    return buffer


def build_sensor_info(packet_counter, sensor_id=0):
    buffer = b'\x00\x00\x00\x0f'                   # packet type = 15
    buffer += struct.pack('>Q', packet_counter)    # packet counter
    buffer += struct.pack('B', sensor_id)          # tracker (sensor) ID
    buffer += struct.pack('B', 0)                  # sensor status
    buffer += struct.pack('B', 8)                  # sensor type (BMI160相当)
    return buffer

# ==========================================
# SlimeVR送信クラス
# ==========================================
class SlimeVRSender:
    def __init__(self, ip, port, mac="3C:38:F4:B4:95:01", fw="1.0.0", sensor_id=0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (ip, port)
        self.packet_counter = 0
        self.handshake_sent = False
        self.mac = mac
        self.fw = fw
        self.sensor_id = sensor_id

    def send(self, data: bytes):
        self.sock.sendto(data, self.addr)

    def send_handshake(self):
        self.packet_counter += 1
        packet = build_handshake(self.mac, self.fw, self.packet_counter)
        self.send(packet)

        # 少し後にセンサ情報も送信
        self.packet_counter += 1
        sensor_info = build_sensor_info(self.packet_counter, sensor_id=self.sensor_id)
        self.send(sensor_info)

    def send_quaternion(self, quat):
        if not self.handshake_sent:
            self.send_handshake()
            self.handshake_sent = True

        self.packet_counter += 1
        packet = build_rotation_packet(*quat, self.packet_counter, sensor_id=self.sensor_id)
        self.send(packet)



# ==========================================
# OSCハンドラ
# ==========================================

def imu_rotation_handler(address, *args):
    if len(args) != 4:
        print("Invalid OSC quaternion length")
        return

    qw, qx, qy, qz = args
    transformed = convert_and_transform_quaternion(qw, qx, qy, qz)
    sender.send_quaternion(transformed)
    print(f"[OSC] Received Quaternion -> W:{qw:.2f} X:{qx:.2f} Y:{qy:.2f} Z:{qz:.2f}")

# ==========================================
# メインループ
# ==========================================
async def main():
    dispatcher = Dispatcher()
    dispatcher.map("/imu/quat", imu_rotation_handler)# 必要に応じてOSCメッセージを変更する

    server = AsyncIOOSCUDPServer(("0.0.0.0", OSC_RECEIVE_PORT), dispatcher, asyncio.get_event_loop())
    transport, protocol = await server.create_serve_endpoint()

    print(f"Listening for OSC on port {OSC_RECEIVE_PORT}...")

    try:
        while True:
            await asyncio.sleep(1)
    finally:
        transport.close()

# ==========================================
# 実行
# ==========================================
if __name__ == "__main__":
    sender = SlimeVRSender(SLIME_IP, SLIME_PORT)
    asyncio.run(main())
