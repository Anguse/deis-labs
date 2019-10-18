import lcm
import time

from exlcm import extmsg_t

lc = lcm.LCM()

id = -1
mode = 0

msg = extmsg_t()
msg.id = id
msg.timestamp = int(time.time())
msg.mode = mode
msg.type = "alert"
lc.publish("EXAMPLE_ext", msg.encode())

