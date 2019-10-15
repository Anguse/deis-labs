import lcm
import time

from exlcm import detectmsg_t

lc = lcm.LCM()

msg = detectmsg_t()
msg.timestamp = int(time.time() * 1000000)
msg.type = "alert"
lc.publish("EXAMPLE_int", msg.encode())

