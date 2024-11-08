

# Wren
Wren is a compact altimeter capable of fitting in a standard 18mm model rocket. It is designed to be assembled by JLCPCB for as low of a price as possible. The altitude, velocity and acceleration are sampled and saved at 50Hz. The battery should last for tens of hours while waiting for a launch and is charged through USB-C

![](docs/scale.jpg)


## Hardware
A 12mm x 34mm 4 layer PCB, designed to only use components from JLCPCB's stock. Single sided assembly with only a debug connector and battery pads (and branding of course ;) ) on the back.

1. **MCU:** An nRF52833, small package, USB PHY and incredibly low power.

1. **Accelerometer:** A LIS2DH rotated 45° allowing it to measure accelerations up to 22.5G instead of the typical 16G limit.

1. **Barometer:** A BMP388 for altitude measurements, better options available but limited by availability for PCB assembly

1. **Flash Memory:** 4Mbit of onboard flash. Should log ~4 flights at 50Hz

1. **Battery:** LiPo charging at 30mA

![](docs/assembled.jpg)


## Software
The onboard firmware is being developed using Rust with the Embassy framework. The dashboard is written in Python