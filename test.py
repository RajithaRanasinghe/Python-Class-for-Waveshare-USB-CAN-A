from usb_can_adapter_v1 import UsbCanAdapter
from imu_f99xB20 import IMUIf99xB20nterpreter

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
            pass # Sometimes the frame does not have data