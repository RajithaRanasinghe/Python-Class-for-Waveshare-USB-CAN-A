from typing import Dict, Union
from usb_can_adapter_v1 import UsbCanAdapter
import platform
class IMUIf99xB20nterpreter:
    # Dictionary to map PGNs to their respective data descriptions
    pgn_data_map = {
        int('0xFF00', 16): ("Acceleration", ["Acc X", "Acc Y", "Acc Z", "App Flag Acc"]),
        int('0xFF01', 16): ("Angular Rate", ["Gyro X", "Gyro Y", "Gyro Z", "App Flag Gyro"]),
        int('0xFF02', 16): ("Rotational Acceleration", ["RotAcc X", "RotAcc Y", "RotAcc Z"]),
        int('0xFF03', 16): ("Gravity Vector", ["Grav X", "Grav Y", "Grav Z"]),
        int('0xFF04', 16): ("Linear Acceleration", ["LinAcc X", "LinAcc Y", "LinAcc Z"]),
        int('0xFF05', 16): ("Angle", ["Roll", "Pitch", "Yaw", "Flag"]),
        int('0xFF06', 16): ("Quaternion", ["Quat X", "Quat Y", "Quat Z", "Quat W"]),
        int('0xFF07', 16): ("Mics", ["Temp Sens", "Temp Main"])
    }

    # Factor and offset definitions for each field
    factors = {
        "Acceleration": 1000,
        "Angular Rate": 100,
        "Rotational Acceleration": 1,
        "Gravity Vector": 1000,
        "Linear Acceleration" : 1000,
        "Angle": 100,
        "Quaternion": 1000,
        "Mics": 10
    }

    TWO_COMP_MAX = 2**15
    TWO_COMP_MOD = 2**16


    @staticmethod
    def get_port():
        """Returns the port where CH340 is connected."""

        if platform.system() == "Linux":
            # On Linux, the CH340 is typically connected to /dev/ttyUSB0
            return "/dev/ttyUSB0"
        elif platform.system() == "Windows":
            # On Windows, the CH340 is typically connected to COM4
            return "COM4"
        else:
            # Raise an exception if the platform is not supported
            raise Exception("Unsupported platform: {}".format(platform.system()))

    @staticmethod
    def twos_complement(value: int) -> int:
        """Convert 2's complement to signed integer"""
        return value - IMUIf99xB20nterpreter.TWO_COMP_MOD if value >= IMUIf99xB20nterpreter.TWO_COMP_MAX else value

    def interpret_frame(self, data_dict: Dict[str, Union[bytearray, str]]) -> Dict[str, float]:
        imu_data = {}
        pgn = int(data_dict["pgn"], 16)  # Convert hex string to integer
        byte_data = data_dict["data"]

        # Extracting PGN and Source Address
        pdo_name, fields = IMUIf99xB20nterpreter.pgn_data_map.get(pgn, ("Unknown", []))
        factor = IMUIf99xB20nterpreter.factors.get(pdo_name, 1)

        try:
            byte_index = 0
            for field in fields:
                # Assuming each field requires 2 bytes
                int_val = (byte_data[byte_index] << 8) | byte_data[byte_index + 1]
                int_val = self.twos_complement(int_val)  # Convert 2's complement hex to signed integer

                # Store the converted and scaled value in the data dictionary
                imu_data[field] = int_val / factor

                byte_index += 2
        except IndexError as e:
            error = f"IndexError: {e}"
            print(error)
            return {}  # Return an empty dictionary on error

        return imu_data


if __name__ == "__main__":
    uca = UsbCanAdapter()
    imu = IMUIf99xB20nterpreter()


    uca.set_port(imu.get_port())
    uca.adapter_init()
    uca.command_settings(speed=250000)

    while True:
        frame_len = uca.frame_receive()

        if frame_len == -1:
            print("Frame receive error!")
            break
        else:
            data = imu.interpret_frame(uca.extract_data(uca.frame))

        try:
            roll, pitch, yaw = data["Roll"], data["Pitch"], data["Yaw"]
            print(f"Roll: {roll} Pitch: {pitch} Yaw: {yaw}")
        except KeyError as e:
            #error = f"KeyError: {e}"
            #print(error)
            pass # Sometimes the frame does not have data