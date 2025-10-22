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
# servo.ChangeId(1, 4)
# time.sleep(3)
# print(servos := servo.ListServos())

####################### Define middle point
# servo.DefineMiddle(1)
# time.sleep(5)
# print(position := servo.ReadPosition(1))


# servo.MoveTo(1, 0, 2400, 0, False)
# time.sleep(0.5)
# servo.MoveTo(2, 3072, 2400, 0, False)


# servo_positions = {1: 3757, 2: 2903}  
# servo.SyncMoveTo(servo_positions, max_speed=800, acc=30, wait=False)

# servo_positions = {1: 3239, 2: 2385}  
# servo.SyncMoveTo(servo_positions, max_speed=800, acc=30, wait=False)


