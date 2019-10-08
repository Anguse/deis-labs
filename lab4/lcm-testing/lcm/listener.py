import lcm
import time
from exlcm import detectmsg_t

oldtime = time.time()

def my_handler(channel, data):
    msg = detectmsg_t.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   timestamp   = %s" % str(msg.timestamp))
    print("   type   = %s" % str(msg.type))
    print("")
    print("time difference is: %s" % str(time.time()-msg.timestamp))
    if(msg.type == "heartbeat"):
	newtime = int(msg.timestamp)
	print("time since last heartbeat is: %s" % str(newtime-oldtime))
	oldtime = newtime
    

lc = lcm.LCM()
subscription = lc.subscribe("EXAMPLE", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass

lc.unsubscribe(subscription)

