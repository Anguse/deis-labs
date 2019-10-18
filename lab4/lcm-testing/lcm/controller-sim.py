import lcm
import time
import sys
import threading
from exlcm import extmsg_t, detectmsg_t

print("")
print("###########################################")
print("This is a simulation of a robot controller.")
print("Its purpose is to represent a robot running")
print("in a convoy")
print("###########################################")
print("")

mode = '2'
id = -1
oldtime = 0
newtime = 0

if(len(sys.argv) > 2):
	print("too many input arguments")
	quit()
elif(len(sys.argv) == 2):
	id = int(sys.argv[1])
else:
	id = int(input("Enter id: "))

def ext_handler(channel, data):
	global oldtime, newtime
	msg = extmsg_t.decode(data)
	if(int(msg.id) != id-1):
		return
	print("")
	print("Received message on channel \"%s\"" % channel)
	print("   timestamp   = %s" % str(msg.timestamp))
	print("   id   = %s" % str(msg.id))
	print("   type   = %s" % str(msg.type))
	print("   mode   = %s" % str(msg.mode))
	print("   time diff: %s" % str(time.time()-msg.timestamp))
	print("")
	if(msg.type == 'alert'):
		print('break, broadcast to convoy')
		msg.id = id
		msg.timestamp = int(time.time())
		msg.mode = int(mode)
		lc.publish("EXAMPLE_ext", msg.encode())
	if(msg.type == 'heartbeat'):
		newtime = msg.timestamp
		print("newtime: %s" %newtime)
		print("oldtime: %s" %oldtime)
		print("    jitter between two heartbeat signals: %s" % str(1-(float(newtime)-float(oldtime))))
		oldtime = newtime

def int_handler(channel, data):
	msg = detectmsg_t.decode(data)
	print("")
	print("Received message on channel \"%s\"" % channel)
	print("   timestamp   = %s" % str(msg.timestamp))
	print("   type   = %s" % str(msg.type))
	print("   time diff: %s" % str(time.time()-msg.timestamp))
	print("")
	if(mode == '0'):
		print("break")
		lc.publish("EXAMPLE_break", msg.encode())
	elif(mode == '1'):
		print("break, broadcast to convoy")
		lc.publish("EXAMPLE_break", msg.encode())
		msg = extmsg_t()
		msg.id = id
		msg.timestamp = int(time.time())
		msg.mode = int(mode)
		msg.type = 'alert'
		lc.publish("EXAMPLE_ext", msg.encode())

def send_heartbeat():
	msg = extmsg_t()
	msg.timestamp = int(time.time())
	msg.type = "heartbeat"
	msg.id = id
	msg.mode = int(mode)
	lc.publish("EXAMPLE_ext", msg.encode())
	threading.Timer(1, send_heartbeat).start()

oldtime = time.time()
lc = lcm.LCM()
send_heartbeat()

print("Starting in follow convoy mode")
ext_subscription = lc.subscribe("EXAMPLE_ext", ext_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass

lc.unsubscribe(ext_subscription)
lc.unsubscribe(int_subscription)

