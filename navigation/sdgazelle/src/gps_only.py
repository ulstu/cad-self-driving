import serial
import sys
import struct
import binascii

dict = { 'gps_state_status' : 0,
         'gps_int_longitude' : 8,
         'gps_int_latitude' : 4,
         'gps_altitude' : 12
}

s = serial.Serial('/dev/ttyUSB0', 921600)
status = 0
latitude = 0
longitude = 0
altitude = 0
position_covariance = 0
position_covariance_type = 0


def bytes_to_int(offset):
    r = 0
    for i in range(4):
        d = i * 8
        r += int.from_bytes(paket[offset + i], 'little') << d
    return r


while True:

    paket = [b'\x00']
    print()
    paket[0] = s.read()

    while paket[0] != b'\xff':
        paket[0] = s.read()

    for byte in range(3):
        paket.append(s.read())

    for byte in range(int.from_bytes(paket[3], byteorder=sys.byteorder) + 4):
        paket.append(s.read())

    longitude = bytes_to_int(dict['gps_int_longitude'] + 4) * 360 / 4294967296
    latitude = bytes_to_int(dict['gps_int_latitude'] + 4) * 360 / 4294967296
    status = bool(bytes_to_int(dict['gps_state_status'] + 4) & (2 ** 16)) - 1
    altitude = paket[16] + paket[17] + paket[18] + paket[19]
    altitude = struct.unpack('f', altitude)
    print(status, latitude, longitude, altitude[0])

    counter = 1
    for j in paket:
        print(f"  {counter:04}   | ", end="")
        counter += 1
    print()
    for j in paket:
        binary = str(bin(int.from_bytes(j, byteorder=sys.byteorder)))[2:]
        print('{:08}'.format(int(binary)), end=" | ")
        #print(int.from_bytes(j, "little", signed=True), end=" | ")
        # print(int(j, 16), end=" | ")

