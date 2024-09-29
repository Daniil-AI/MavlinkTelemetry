from pymavlink import mavutil
from threading import Thread

import time
import datetime
import math


class FlightController:
    def __init__(self, port: str = "com4", baudrate: int = 57600):
        self.__port = port
        self.baudrate = baudrate
        self.master = mavutil.mavlink_connection(self.__port, self.baudrate)
        self.__start_thread: bool = False

        # GPS
        self.gps_lat: float = 0.0
        self.gps_lon: float = 0.0
        self.gps_alt: float = 0.0
        self.gps_home_alt: float = 0.0
        self.gps_x_speed: float = 0.0
        self.gps_y_speed: float = 0.0

        # IMU
        self.imu_x_acc: float = 0.0
        self.imu_y_acc: float = 0.0
        self.imu_z_acc: float = 0.0

        # Attitude
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.yaw: float = 0.0

        # Battery
        self.voltage_battery: float = 0.0
        self.current_battery: float = 0.0
        self.battery_remaining: float = 0.0

        # Other
        self.satellite: int = 0
        self.temp: float = 0

        # Time
        self.local_time: datetime.timedelta = datetime.timedelta(milliseconds=0)

    def receive_messages(self) -> None:
        while self.__start_thread:
            msg = self.master.recv_msg()
            if msg is not None:
                self.process_message(msg)

    def process_message(self, msg) -> None:
        if msg.get_type() == "GLOBAL_POSITION_INT":
            self.gps_lat = msg.lat / 1e7
            self.gps_lon = msg.lon / 1e7
            self.gps_alt = msg.alt
            self.gps_home_alt = msg.relative_alt

            self.gps_x_speed = msg.vx
            self.gps_y_speed = msg.vy

        elif msg.get_type() == "SYS_STATUS":
            self.voltage_battery = msg.voltage_battery / 1000
            self.current_battery = msg.current_battery / 1000
            self.battery_remaining = msg.battery_remaining

        elif msg.get_type() == "GPS_STATUS":
            self.satellite = msg.satellites_visible

        elif msg.get_type() == "SYSTEM_TIME":
            self.local_time = datetime.timedelta(milliseconds=msg.time_boot_ms)

        elif msg.get_type() == "SCALED_IMU":
            # Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
            self.temp = msg.temperature / 100

        elif msg.get_type() == "ATTITUDE":
            self.roll = math.degrees(msg.roll)
            self.pitch = math.degrees(msg.pitch)
            self.yaw = math.degrees(msg.yaw)

        elif msg.get_type() == "RAW_IMU":
            self.imu_x_acc = msg.xacc
            self.imu_y_acc = msg.yacc
            self.imu_z_acc = msg.zacc

        # elif msg.get_type() == "VFR_HUD":
        #     self.alt = msg.alt

    def get_current_telemetry(self) -> None:

        print()
        print(f"GPS: Latitude - {self.gps_lat}, Longitude - {self.gps_lon}")
        print(f"Высота GPS: {self.gps_alt} м")
        print(
            f"Акселерометр: X - {self.imu_x_acc}, Y - {self.imu_y_acc}, Z - {self.imu_z_acc}"
        )
        print(f"Положение: roll: {self.roll}, pitch: {self.pitch}, yaw: {self.yaw}")
        print(
            f"Батарейка: voltage: {self.voltage_battery}, сurrent: {self.current_battery}, procent: {self.battery_remaining}"
        )
        print(f"Время {self.local_time}")
        print(f"Видим спутников {self.satellite}")
        print(f"Температура {self.satellite}")

    def translate_info(self) -> None:
        freq = 5

        while self.__start_thread:
            self.get_current_telemetry()
            time.sleep(1 / freq)

    def start(self) -> None:
        self.__start_thread = True
        self.thread = Thread(target=self.receive_messages)
        self.see_info_thread = Thread(target=self.translate_info)

        self.thread.start()
        self.see_info_thread.start()

    def stop(self) -> None:
        self.__start_thread = False
        if self.thread is not None:
            self.thread.join()

        if self.see_info_thread is not None:
            self.see_info_thread.join()


if __name__ == "__main__":
    fc = FlightController("com4", 57600)
    fc.start()
    time.sleep(4)
    fc.stop()
