import struct
from collections import defaultdict
import serial
import serial.tools.list_ports
import json

IdVoltage = 1
IdTelemetry = 2
IdState = 3
IdPowerOn = 4
IdDebug = 5
IdTime = 6

class TimeSeries:
    def __init__(self):
        self.x = []
        self.y = []

class WrenManager():
    def __init__(self):
        self.splits = []
        self.usb_is_open = False

        self.altitude = 0
        self.voltage = 0
        self.battery_voltage = 0
        self.battery_percent = 0
        self.acceleration = 0
        self.flash_used = 100
        self.time = 0

    def lipo_charge_percentage(self, voltage):
        voltage_to_percent = [
            (3.0, 0),     # 3.0V -> 0% (fully discharged)
            (3.2, 10),    # 3.2V -> 10%
            (3.4, 20),    # 3.4V -> 20%
            (3.6, 40),    # 3.6V -> 40%
            (3.8, 60),    # 3.8V -> 60%
            (4.0, 80),    # 4.0V -> 80%
            (4.2, 100),   # 4.2V -> 100% (fully charged)
        ]

        if voltage <= 3.0:
            return 0.0
        if voltage >= 4.2:
            return 100.0

        for i in range(len(voltage_to_percent) - 1):
            v1, p1 = voltage_to_percent[i]
            v2, p2 = voltage_to_percent[i + 1]
            if v1 <= voltage <= v2:
                percentage = p1 + (p2 - p1) * (voltage - v1) / (v2 - v1)
                return round(percentage, 1)

        # Default fallback (shouldn't be reached)
        return 0.0

    def open_file(self, path):
        f = open(path, "rb")
        buf = f.read()
        if path.endswith(".csv"):
            self.parse_csv(buf)
        else:
            self.parse_binary(buf)

    def open_usb(self):
        ports = serial.tools.list_ports.comports()

        for port in ports:
            try:
                # Open serial connection
                ser = serial.Serial(port.device, timeout=20)  # Timeout of 1 second
                ser.read_all()
                ser.write(b"wren?\n")  # Send query to device

                # Read response
                response = ser.readline().strip().decode("utf-8")
                if response == "wren!":
                    self.ser = ser  # Save the serial connection for future use
                    self.usb_is_open = True
                    print("found usb port")
                    return
                else:
                    print(response)
                ser.close()  # Close the port if the response is incorrect
            except (serial.SerialException, UnicodeDecodeError) as e:
                print(f"Error checking port {port.device}: {e}")
        self.usb_open = False
        print("did not find")

    def fetch_status(self):
        self.ser.write(b"get_status\n")
        response = self.ser.readline().strip().decode("utf-8")

        parsed = json.loads(response)
        self.flash_used = parsed["flash_used"] / parsed["flash_size"]
        self.battery_voltage = parsed["volt"]
        self.battery_percent = self.lipo_charge_percentage(self.battery_voltage)
        self.time = parsed["time"] / 1000
        self.acceleration = parsed["acceleration"]
        self.altitude = parsed["altitude"]

    def read_flash(self):
        self.ser.write(b"read_flash\n")
        response = self.ser.readline().strip().decode("utf-8")
        parsed = json.loads(response)
        binary_length = parsed["length"]
        print(binary_length)
        buf = self.ser.read(binary_length)
        print(len(buf))
        if len(buf) == binary_length:
            return buf
        else:
            print("bad binary file")

    def clear_flash(self):
        self.ser.write(b"clear_flash\n")

    def parse_binary(self, buf):
        current_time = 0
        offset_time = 0
        just_powered_on = True # set this to true to not initialize
        last_raw_time = 0
        index = 0
        split_index = 0
        current_dict = defaultdict(lambda: TimeSeries())
        while True:
            if index == len(buf):
                if "boost" in current_dict["state"].y:
                    self.splits.append(current_dict)
                break
            id = buf[index]
            index += 1
            if id == IdPowerOn: # power on
                if just_powered_on == False:
                    if "boost" in current_dict["state"].y:
                        self.splits.append(current_dict)
                    current_dict = defaultdict(lambda: TimeSeries())
                    just_powered_on = True
                continue

            just_powered_on = False
            if id == IdTelemetry:
                (alt_raw, vel_raw, acc_raw) = struct.unpack_from("<HHH", buf, index)
                index += struct.calcsize("<HHH")
                alt = alt_raw / 4
                vel = vel_raw / 100 - 100
                acc = acc_raw / 200 - 200

                current_dict["altitude"].x.append(current_time)
                current_dict["altitude"].y.append(alt)

                current_dict["velocity"].x.append(current_time)
                current_dict["velocity"].y.append(vel)

                current_dict["acceleration"].x.append(current_time)
                current_dict["acceleration"].y.append(acc)


            elif id == IdState:
                (state,) = struct.unpack_from("<B", buf, index)
                index += struct.calcsize("<B")

                state_to_name = {
                    0: "idle",
                    1: "maybe_launched",
                    2: "boost",
                    3: "coast",
                    4: "descent",
                }

                name = state_to_name[state]
                current_dict["state"].x.append(current_time)
                current_dict["state"].y.append(name)

            elif id == IdVoltage:
                (volt_raw,) = struct.unpack_from("<H", buf, index)
                index += struct.calcsize("<H")
                volt = volt_raw / 1000
                current_dict["volt"].x.append(current_time)
                current_dict["volt"].y.append(volt)

            elif id == IdDebug:
                index += struct.calcsize("<fffdI")

            elif id == IdTime:
                (time,) = struct.unpack_from("<I", buf, index)
                index += struct.calcsize("<I")
                current_time = time / 1000.0

            else:
                print("invalid id", id)

if __name__ == "__main__":
    wren = WrenManager()
    wren.open_file("/tmp/wren_flash.bin")
    wren.open_usb()
    wren.fetch_status()
    wren.read_flash()