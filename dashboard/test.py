import serial
import json
from parser import Parser

def send_command_and_read_response(port, command):
    with serial.Serial(port, timeout=10) as ser:
        ser.write(command.encode('utf-8'))

        json_response = b""
        open_braces = 0
        while True:
            char = ser.read(1)
            if not char:
                raise ValueError("Timeout while reading JSON response")

            json_response += char

            if char == b"{":
                open_braces += 1
            elif char == b"}":
                open_braces -= 1
                if open_braces == 0:
                    break

        parsed_json = json.loads(json_response.decode('utf-8'))

        binary_length = parsed_json.get("length")
        if binary_length is None:
            raise ValueError("got no length")
        print(f"Expected Binary Length: {binary_length}")

        binary_stream = ser.read(binary_length)
        print(binary_stream)
        if len(binary_stream) != binary_length:
            raise ValueError("Incomplete binary data received")

        return parsed_json, binary_stream

# Usage
if __name__ == "__main__":
    port = "/dev/ttyACM0"
    command = "read_flash\n"  # Replace with the exact command format

    json_data, binary_data = send_command_and_read_response(port, command)
    print("JSON:", json_data)

    parser = Parser()
    parser.parse
    parser.parse_binary(binary_data)

