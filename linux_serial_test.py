import serial


class Ser:
    def __init__(self):
        self.port = serial.Serial(port='ttyUSB0', baudrate=115200, bytesize=8, parity='E', stopbits=1, timeout=2)

    def send_cmd(self, cmd):
        self.port.write(cmd)
        response = self.port.read()
        response = self.convert_hex(response)
        return response

    def convert_hex(self, string):
        res = []
        result = []
        for item in string:
            res.append(item)
        for i in res:
            result.append(hex(i))
        return result


if __name__ == '__main__':
    s = Ser()
