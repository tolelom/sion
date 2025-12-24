from jetbot import Robot
import time

print("Robot init...")
r = Robot()
print("Robot init OK, moving...")

r.set_motors(0.3, 0.3)
time.sleep(1.0)
r.stop()
print("Done")
