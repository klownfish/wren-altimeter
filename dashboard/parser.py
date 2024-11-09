import struct
from collections import defaultdict

IdMetadata = 1,
IdTelemetry = 2,
IdState = 3,
IdPowerOn = 4

class TimeSeries:
    def __init__(x, y):
        self.x = []
        self.y = []

class Parser():
    def __init__():
        self.coom = 0
        self.data = {}

    def open_file(path):
        f = open(path, "rb")
        buf = f.readall()
        if path.endswith(".csv"):
            self.data = self.parse_csv(buf)
        else:
            self.data = self.parse_binary(buf)

    def open_serial(self, port):
        pass

    def get_status_text(self,):
        pass

    def parse_binary(self, buf):
        current_time = 0
        offset_time = 0
        just_powered_on = False
        last_raw_time = 0
        index = 0
        self.data["everything"] = defaultdict(lambda: TimeSeries())
        while True:
            id = buf[index]
            index += 1

            if id == PowerOn: # power on
                just_powered_on = True

            if id == IdTelemetry:
                current_time += 50
                (alt_raw, vel_raw, acc_raw) = struct.unpack_from("<HHH", buf, index)
                index += struct.calcsize("<HHH", buf, index)
                alt = alt_raw / 4
                vel = vel_raw / 100
                acc = acc_raw / 200
                self.data["everything"]["altitude"].x.append(current_time)
                self.data["everything"]["altitude"].y.append(alt)

                self.data["everything"]["velocity"].x.append(current_time)
                self.data["everything"]["velocity"].y.append(vel)

                self.data["everything"]["acceleration"].x.append(current_time)
                self.data["everything"]["acceleration"].y.append(acc)


            if id == IdState:
                (state) = struct.unpack_from("<B", buf, index)
                index += struct.calcsize("<B", buf, index)

                state_to_name = {
                    0: "idle",
                    1: "maybe_launched",
                    2: "boost",
                    3: "coast",
                    4: "descent",
                }

                name = state_to_name[state]
                if name != "maybe_launched":
                    self.data["everything"]["events"].x.append(current_time)
                    self.data["everything"]["events"].y.append(name)

            if id == IdMetadata:
                (raw_time, volt_raw) = struct.unpack_from("<IH", buf, index)
                volt = volt_raw / 1000

                if just_powered_on:
                    offset_time = current_time
                    just_powered_on = False
                    current_time = (raw_time / 1000) + offset_time
                    self.data["everything"]["events"].x.append(current_time)
                    # self.data["everything"]["events"].y.append("power_on")
                else:
                    current_time = raw_time + offset_time

                self.data["everything"]["volt"].x.append(current_time)
                self.data["everything"]["volt"].y.append(volt)
