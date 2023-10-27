# USB CAN Adapter Python Interface
This repository contains a Python script to interface with a [USB-CAN-A  by Waveshare](https://www.waveshare.com/wiki/USB-CAN-A). The script is a convertion and modification of the C code provided by waveshare.
The UsbCanAdapter python class is used for communicating with CAN-enabled devices including the Pepperl Fuchs Inertial measurement unit [IMUF99PL-SC3600-0KB20V1501](https://www.pepperl-fuchs.com/global/en/classid_6422.htm?view=productdetails&prodid=102520) sensor.

## Dependencies
Python 3.6+
pyserial

## Usage
Run the script usb_can_adapter_v1.py with the necessary command-line arguments to operate the USB CAN adapter.

```bash
python usb_can_adapter_v1.py <options>
```

## Options:
    -h, --help - Display help and exit.
    -t - Print TTY/serial traffic debugging info.
    -d DEVICE - Use TTY DEVICE.
    -s SPEED - Set CAN SPEED in bps.
    -b BAUDRATE - Set TTY/serial BAUDRATE (default: 2000000).
    -i ID - Inject using ID (specified as a hex string).
    -j DATA - CAN DATA to inject (specified as a hex string).
    -n COUNT - Terminate after COUNT frames (default: infinite).
    -g MS - Inject sleep gap in MS milliseconds (default: 200 ms).
    -m MODE - Inject payload MODE (0 = random, 1 = incremental, 2 = fixed).

## Classes and Enumerations:
    * CANUSB_SPEED - Enumeration for CAN bus speeds.

    * CANUSB_MODE - Enumeration for CAN adapter modes (NORMAL, LOOPBACK, SILENT, LOOPBACK_SILENT).

    * CANUSB_FRAME - Enumeration for CAN frame formats (STANDARD, EXTENDED).

    * CANUSB_PAYLOAD_MODE - Enumeration for payload injection modes (RANDOM, INCREMENTAL, FIXED).

    * UsbCanAdapter - Main class for interfacing with the USB CAN adapter.

    * IMUIf99xB20nterpreter - Class for interpreting data from Pepperl Fuchs IMUF99PL-SC3600-0KB20V1501 sensor.

## UsbCanAdapter Class:
The UsbCanAdapter class encapsulates the functionality for interacting with the USB CAN adapter. It provides methods to send and receive CAN frames, configure the adapter settings, and inject data frames into the CAN bus. Below are some of the key methods provided by this class:

* adapter_init(device_port: str = None, baudrate: int = None) -> serial.
Serial - Initializes the adapter with the specified device port and baudrate.
* adapter_close() -> None - Closes the serial port connection.

* command_settings(speed: int = 0, mode: CANUSB_MODE = CANUSB_MODE.NORMAL, frame: CANUSB_FRAME = CANUSB_FRAME.STANDARD) -> int - Configures the CAN to serial adapter settings.

* frame_send(frame: bytearray) -> int - Sends a frame to the USB-CAN-ADAPTER device.

* frame_receive(frame_len_max: int = 20) -> int - Receives a frame from the USB-CAN-ADAPTER device.

* inject_data_frame(hex_id: str, hex_data: str) -> int - Injects a data frame with the specified hexadecimal ID and data.

* dump_data_frames(print_flag: bool) -> int - Dumps data frames to the console or log.

* main() -> None - Main function to parse arguments and run the program.

## IMUIf99xB20nterpreter Class:
The IMUIf99xB20nterpreter class is used for interpreting data from the Pepperl Fuchs IMUF99PL-SC3600-0KB20V1501 sensor. It contains methods to parse the CAN frames and extract useful information such as acceleration, angular rate, and orientation angles.

## Running the Script:
The script can be run directly from the command line, with options for specifying the TTY device, CAN speed, baud rate, and other parameters.

```bash
python usb_can_adapter_v1.py -d "COM4" -s 250000
```
In this example, the script will use the COM4 device with a CAN speed of 250,000 bps.

![Terminal output](/results/TerminalOut.png)

# Test with Pepperl Fuchs IMUF99PL-SC3600-0KB20V1501 Sensor
The following code snippet demonstrates how to use the **UsbCanAdapter** and **IMUIf99xB20nterpreter** classes to read data from the Pepperl Fuchs IMUF99PL-SC3600-0KB20V1501 sensor.

```bash
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

```

![Test output](/results/Test_IMUF99PL.png)