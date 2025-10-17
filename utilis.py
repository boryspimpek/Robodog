import time
import math
from st3215 import ST3215, ContinuousServoController


servo = ST3215('COM3')

###################################################### SETTING SECTION #################

######################## List servos
# print(servos := servo.ListServos())

######################## Ping servo
# print(alive := servo.PingServo(1))

######################## Change ID os servo
# servo.ChangeId(2, 1)
# time.sleep(1)
# print(servos := servo.ListServos())

####################### Define middle point
# servo.DefineMiddle(2)
# time.sleep(2)
# print(position := servo.ReadPosition(2))



