import struct
from collections import defaultdict

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

class Parser():
    def __init__(self):
        self.splits = []

    def open_file(self, path):
        f = open(path, "rb")
        buf = f.read()
        if path.endswith(".csv"):
            self.parse_csv(buf)
        else:
            self.parse_binary(buf)

    def open_serial(self, port):
        pass

    def get_status_text(self):
        pass

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
                print(current_time)

            else:
                print("invalid id", id)

if __name__ == "__main__":
    parser = Parser()
    parser.open_file("/tmp/wren_flash.bin")