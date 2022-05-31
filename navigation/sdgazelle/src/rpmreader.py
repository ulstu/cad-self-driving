import obd
import time

try:
    connection = obd.OBD()  # auto-connects to USB or RF port
    while True:
        speed = obd.commands.SPEED  # select an OBD command (sensor)
        rspeed = connection.query(speed)  # send the command, and parse the response
        rpm = obd.commands.RPM  # select an OBD command (sensor)
        rrpm = connection.query(rpm)  # send the command, and parse the response
        throttle = obd.commands.THROTTLE_POS # select an OBD command (sensor)
        rthrottle = connection.query(throttle)  # send the command, and parse the response

        str = "speed: {}; rpm: {}; throttlepos: {}".format(rspeed, rrpm, rthrottle)
        print(str)  # returns unit-bearing values thanks to Pint
        time.sleep(0.2)
except KeyboardInterrupt:
    pass
