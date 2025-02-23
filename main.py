from pymavlink import mavutil
from threading import Thread

import time
import datetime
import math


class FlightController:
    def __init__(self, port: str = "com0", baudrate: int = 57600):

        # Drone status
        self.arm: bool | None = None
        self.drone_mode: str = "not known"

        # GPS
        self.gps_lat: float = 0.0
        self.gps_lon: float = 0.0
        self.gps_alt: float = 0.0
        self.gps_home_alt: float = 0.0
        self.gps_x_speed: float = 0.0
        self.gps_y_speed: float = 0.0
        self.gps_z_speed: float = 0.0
        self.gps_speed: float = 0.0

        # Home point
        self.home_gps_lat: float = 0.0
        self.home_gps_lon: float = 0.0
        self.home_gps_altitude: float = 0.0
        self.distance_to_home: int = 0

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
        self.energy_consumed: float = 0.0
        self.current_consumed: float = 0.0
        self.battery_remaining: float = 0.0
        self.battery_time_remaining: datetime.timedelta = datetime.timedelta(
            milliseconds=0
        )

        # Other
        self.satellite: int = 0
        self.temp: int = 0

        # Time
        self.takeoff_time_point: datetime.timedelta = datetime.timedelta(milliseconds=0)
        self.time_in_air: datetime.timedelta = datetime.timedelta(milliseconds=0)

        self.see_info_thread = None

        self.__port = port
        self.baudrate = baudrate
        self.__start_thread: bool = False

        try:
            self.master = mavutil.mavlink_connection(self.__port, self.baudrate)
            self.status = 'OK'
        except Exception as e:
            self.status = e

    def receive_messages(self) -> None:
        while self.__start_thread:
            msg = self.master.recv_msg()
            if msg is not None:
                try:
                    self.process_message(msg)
                except Exception as e:
                    pass

    def process_message(self, msg) -> None:
        if msg.get_type() == "GLOBAL_POSITION_INT":
            self.gps_lat = msg.lat / 1e7
            self.gps_lon = msg.lon / 1e7
            self.gps_alt = msg.alt / 1e3
            self.gps_home_alt = msg.relative_alt / 1e3

            self.gps_x_speed = msg.vx
            self.gps_y_speed = msg.vy
            self.gps_z_speed = msg.vz / 1e3
            self.gps_speed = (self.gps_x_speed ** 2 + self.gps_y_speed ** 2) ** (1/2) / 1e3

        elif msg.get_type() == "HEARTBEAT":
            self.drone_mode = mavutil.mode_string_v10(msg)
            self.arm = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        elif msg.get_type() == "SYS_STATUS":
            self.voltage_battery = msg.voltage_battery / 1000
            self.current_battery = msg.current_battery / 1000
            self.battery_remaining = msg.battery_remaining

        elif msg.get_type() == "BATTERY_STATUS":
            self.energy_consumed = msg.energy_consumed  # hJ
            self.current_consumed = msg.current_consumed  # mAh
#            self.battery_time_remaining = datetime.timedelta(
#                seconds=msg.time_remaining
#            )  # seconds
#            voltages = msg.voltages  # / 1e3  # mV

        elif msg.get_type() == "HOME_POSITION":
            self.home_gps_lat = msg.latitude / 1e7
            self.home_gps_lon = msg.longitude / 1e7
            self.home_gps_altitude = msg.altitude / 1e6  # mm
            self.distance_to_home = self.calculate_distance(
                (self.home_gps_lat, self.home_gps_lon),
                (self.gps_lat, self.gps_lon),
            )

        elif msg.get_type() == "GPS_RAW_INT":
            self.satellite = msg.satellites_visible

        elif msg.get_type() == "SYSTEM_TIME":
            global_datatime = datetime.timedelta(milliseconds=msg.time_boot_ms)
            if len(str(global_datatime)) > 7:
                self.local_time = str(global_datatime)[:-7]


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

        elif msg.get_type() == "FLIGHT_INFORMATION":
            self.takeoff_time_point = datetime.timedelta(
                microseconds=msg.takeoff_time_utc
            )  # us
            self.time_in_air = self.calculate_time_in_air(
                self.takeoff_time_point, self.local_time
            )

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

    def start(self, see_info=False) -> None:
        if self.status == 'OK':
            self.__start_thread = True
            self.thread = Thread(target=self.receive_messages)
            self.thread.start()

            if see_info:
                self.see_info_thread = Thread(target=self.translate_info)
                self.see_info_thread.start()
        else:
            self.thread = None

    def stop(self) -> None:
        self.__start_thread = False
        if self.thread is not None:
            self.thread.join()

        if self.see_info_thread is not None:
            self.see_info_thread.join()


if __name__ == "__main__":
    fc = FlightController("com0", 57600)
    fc.start(see_info=True)
    time.sleep(4)

    fc.get_current_telemetry()
    time.sleep(4)
    fc.get_current_telemetry()
    fc.stop()
