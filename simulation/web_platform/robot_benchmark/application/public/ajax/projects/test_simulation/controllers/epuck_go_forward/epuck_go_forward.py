from controller import Robot, Motor

TIME_STEP = 64

# create the Robot instance.
robot = Robot()

#Вам необходимо изменить эту переменную
position = 20.0

# get the motor devices
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
# set the target position of the motors
leftMotor.setPosition(position)
rightMotor.setPosition(position)

while robot.step(TIME_STEP) != -1:
   pass