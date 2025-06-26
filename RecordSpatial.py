import datetime
import sys
import math

import an_devices.spatial_device as spatial_device
from anpp_packets.an_packet_protocol import ANPacket
from anpp_packets.an_packets import PacketID


if __name__ == '__main__':
    # Checks enough arguments in command for serial communications. Otherwise prompts user on use.
    if len(sys.argv) != 3:
        print(
            f"Usage: program com_port baud_rate\n"
            f"Windows Example: python spatial_example.py COM1 115200\n"
            f"Linux Example: python spatial_example.py /dev/ttyUSB0 115200"
        )
        exit()
    comport = str(sys.argv[1])
    baudrate = sys.argv[2]

    spatial = spatial_device.Spatial(comport, baudrate)
    spatial.start()

    # Checks serial port connection is open
    if spatial.is_open:
        print(f"Connected to port:{spatial.port} with baud:{spatial.baud}")
        spatial.flush()

        # Creates log file for received binary data from device
        now = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        logFile = open(f"SpatialLog_{now}.anpp", 'xb')

        an_packet = ANPacket()

        # Sets sensor ranges
        spatial.set_sensor_ranges(True,
                                  spatial_device.AccelerometerRange.accelerometer_range_4g,
                                  spatial_device.GyroscopeRange.gyroscope_range_500dps,
                                  spatial_device.MagnetometerRange.magnetometer_range_8g)

        spatial.get_device_and_configuration_information()

        while spatial.is_open:
            if spatial.in_waiting() > 0:
                # Get bytes in serial buffer
                bytes_in_buffer = spatial.in_waiting()
                data_bytes = spatial.read(bytes_in_buffer)

                # Record in log file the raw binary of ANPP packets
                logFile.write(data_bytes)

                print(datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + " Record Bytes")


    else:
        print(f"No connection.")

    spatial.close()