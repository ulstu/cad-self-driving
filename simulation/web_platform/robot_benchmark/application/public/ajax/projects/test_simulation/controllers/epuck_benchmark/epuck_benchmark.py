from controller import Supervisor
from math import sqrt
import sys
import os

try:
    includePath = os.environ.get("WEBOTS_HOME") + "/projects/samples/robotbenchmark/include"
    includePath.replace('/', os.sep)
    sys.path.append(includePath)
    from robotbenchmark import robotbenchmarkRecord
except ImportError:
    sys.stderr.write("Warning: 'robotbenchmark' module not found.\n")
    sys.exit(0)


TIME_STEP = 64

supervisor = Supervisor()

targetDist = 0.1989

robot_node = supervisor.getFromDef("MY_ROBOT")
if robot_node is None:
    sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
    sys.exit(1)
trans_field = robot_node.getField("translation")

while supervisor.step(TIME_STEP) != -1:
    values = trans_field.getSFVec3f()
    dist = sqrt(values[0] * values[0] + values[2] * values[2])
    percent = 1 - abs(dist - targetDist) / targetDist
    message = "percent:" + str(percent)
    supervisor.wwiSendText(message)
    print("Distance: {}".format(dist))
    print("Percent: {}".format(percent))