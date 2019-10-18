import lcm
import time
import threading
from exlcm import extmsg_t, detectmsg_t

print("")
print("###############################################")
print(" This is the real controller of a robot.       ")
print(" Its purpose is to log information about       ")
print(" received messages and break when either       ")
print(" a external message from a robot ahead is      ")
print(" received or a moving object is detected.      ")
print("")
print(" Available modes are:                          ")
print(" 0 - Single vehicle mode                       ")
print(" 1 - Head convoy mode                          ")
print(" 2 - Follow convoy mode                        ")
print("###############################################")
print("")

modes = ['0', '1', '2']
mode = ''
id = -1
oldtime = 0.0
newtime = 0.0

while(mode not in modes):
	mode = input("Enter mode: ")
	if(mode not in modes):
		print("Invalid mode, chose 0, 1 or 2")
	else: 
		break

id = int(input("Enter id: "))

def ext_handler(channel, data):
	msg = extmsg_t.decode(data)
	if(msg.id >= id):
		return
	print("")
	print("Received message on channel \"%s\"" % channel)
	print("   timestamp   = %s" % str(msg.timestamp))
	print("   id   = %s" % str(msg.id))
	print("   type   = %s" % str(msg.type))
	print("   mode   = %s" % str(msg.mode))
	print("   time diff: %s" % str(time.time()-msg.timestamp))
	print("")
	if(mode == '2' and msg.type == 'alert'):
		print('break, broadcast to convoy')
		break_msg = detectmsg_t()
		break_msg.timestamp = int(time.time())
		break_msg.type = 'alert'
		lc.publish("EXAMPLE_break", break_msg.encode())
		msg.id = id
		msg.timestamp = int(time.time())
		msg.mode = int(mode)
		lc.publish("EXAMPLE_ext", msg.encode())

def int_handler(channel, data):
	global oldtime, newtime
	msg = detectmsg_t.decode(data)
	print("")
	print("Received message on channel \"%s\"" % channel)
	print("   timestamp   = %s" % str(msg.timestamp))
	print("   type   = %s" % str(msg.type))
	print("   time diff: %s" % str(time.time()-msg.timestamp))
	print("")
	if(msg.type == 'heartbeat'):
		newtime = msg.timestamp
		print("")
		print("    jitter between two heartbeat signals: %s" % str(1-(float(newtime)-float(oldtime))))
		oldtime = newtime
	if(mode == '0' and msg.type == 'alert'):
		print("break")
		lc.publish("EXAMPLE_break", msg.encode())
	elif(mode == '1' and msg.type == 'alert'):
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


# contruct lcm for broadcasting notifications about movement
lc = lcm.LCM()
# start sending heartbeat messages
send_heartbeat()
oldtime = time.time()

if(mode == '0'):
	print("Starting in single vehicle mode")
	int_subscription = lc.subscribe("EXAMPLE_int", int_handler)	
elif(mode == '1'):
	print("Starting in head convoy mode")
	int_subscription = lc.subscribe("EXAMPLE_int", int_handler)
elif(mode == '2'):
	print("Starting in follow convoy mode")
	ext_subscription = lc.subscribe("EXAMPLE_ext", ext_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass

lc.unsubscribe(ext_subscription)
lc.unsubscribe(int_subscription)

