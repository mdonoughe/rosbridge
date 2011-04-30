#!/usr/bin/python
from ROSProxy import ROSProxy
from terribleWSS import Session, serveForever
import sys, traceback
import json
from threading import Lock, Thread
from time import time, sleep
from urllib2 import urlopen
import re

class ServiceCaller(Thread):
	def __init__(self, ros, session, service, arguments):
		Thread.__init__(self)
		self.daemon = True
		self.ros = ros
		self.session = session
		self.service = service
		self.arguments = arguments

	def run(self):
		def handleResponse(rsp):
			call = {'receiver':self.service,'msg':self.ros.generalize(rsp)}
			self.session.sock.send(encode(call))
		self.ros.callService(self.service,self.arguments,handleResponse)

def encode(obj):
	return '\x00' + json.dumps(obj) + '\xff'

def handleFrameHelper(frame, session, handleMsgFactory, sub, keyurl, ros):
	now = time()
	if (not 'rosjsExtStarted' in dir(session)):
		session.rosjsExtStarted = 0
		session.rosjsExtDuration = 0

	try:
		call = ''
		try:
			call = json.loads(frame)
		except:
			call = json.loads(frame[1:])
		receiver = call["receiver"]
		msg = call["msg"]


		if 'type' in call.keys():
			#print "Publish Topic!"
			if (not keyurl or (now - session.rosjsExtStarted <= session.rosjsExtDuration)):
				typ = call["type"]

				if not receiver in session.data.keys():
					session.data[receiver] = rospy.Publisher(receiver.encode('ascii','ignore'), ros.msgClassFromTypeString(typ))

				session.data[receiver].publish(ros.specify(typ,msg))

		else:
			#print "Service Call!"
			if (not keyurl or (now - session.rosjsExtStarted <= session.rosjsExtDuration) or receiver.find('/rosjs/authorize') == 0):
				if receiver.find('/rosjs') == 0:
					if (receiver == "/rosjs/topics"):
						call = {'receiver':'/rosjs/topics','msg':ros.topics}
						session.sock.send(encode(call))
					elif (receiver == "/rosjs/services"):
						call = {'receiver':'/rosjs/services','msg':ros.services}
						session.sock.send(encode(call))
					elif (receiver == "/rosjs/subscribe"):
						topic = msg[0].encode('ascii','ignore')
						delay = msg[1]
						sub(topic, handleMsgFactory(topic))
						if len(msg) > 2:
							session.data[topic] = {'delay':delay,'lastEmission':0,'lastSeq':0,'encoding':msg[2],'width':msg[3],'height':msg[4],'quality':msg[5]}
						else:
							session.data[topic] = {'delay':delay,'lastEmission':0,'lastSeq':0}
						call = {'receiver':'/rosjs/subscribe','msg':'OK'}
						session.sock.send(encode(call))
					elif (receiver == "/rosjs/log"):
						filename = msg[0].encode('ascii','ignore')
						filename = ''.join(c for c in filename if c.isalnum()) + '.log'
						obj = msg[1];

						success = True
						try:
							log = open(filename, 'w')
							log.write(obj)
							log.close()
						except:
							success = False

						call = {'receiver':'/rosjs/log','msg':'OK'}
						if (not success):
							call = {'receiver':'/rosjs/log','msg':'ERROR'}
						session.sock.send(encode(call))
					elif (receiver == "/rosjs/authorize"):
						key = msg[0].encode('ascii','ignore')
						duration = 0
						if (keyurl):
							req = urlopen(keyurl + '?jsonp=invoke&key=' + key)
							req = req.read()
							duration = int(re.sub('[^0-9]','',req))
						session.rosjsExtStarted = now
						session.rosjsExtDuration = duration
						print "authorizing %s for %s seconds" % (key,duration)
						call = {'receiver':'/rosjs/authorize','msg':'OK'}
						session.sock.send(encode(call))
					elif (receiver == "/rosjs/typeStringFromTopic"):
						topic = msg[0].encode('ascii','ignore')
						call = {'receiver':'/rosjs/typeStringFromTopic','msg':ros.typeStringFromTopic(topic)}
						session.sock.send(encode(call))
					elif (receiver == "/rosjs/typeStringFromService"):
						service = msg[0].encode('ascii','ignore');
						call = {'receiver':'/rosjs/typeStringFromService','msg':ros.typeStringFromService(service)}
						session.sock.send(encode(call))
					elif (receiver == "/rosjs/msgClassFromTypeString"):
						typStr = msg[0].encode('ascii','ignore');
						call = {'receiver':'/rosjs/msgClassFromTypeString','msg':ros.generalize(ros.msgClassFromTypeString(typStr)())}
						session.sock.send(encode(call))
					elif (receiver == "/rosjs/reqClassFromTypeString"):
						typStr = msg[0].encode('ascii','ignore');
						call = {'receiver':'/rosjs/reqClassFromTypeString','msg':ros.generalize(ros.srvClassFromTypeString(typStr)._request_class())}
						session.sock.send(encode(call))
					elif (receiver == "/rosjs/rspClassFromTypeString"):
						typStr = msg[0].encode('ascii','ignore');
						call = {'receiver':'/rosjs/rspClassFromTypeString','msg':ros.generalize(ros.srvClassFromTypeString(typStr)._response_class())}
						session.sock.send(encode(call))
					elif (receiver == "/rosjs/classFromTopic"):
						topic = msg[0].encode('ascii','ignore')
						call = {'receiver':'/rosjs/classFromTopic','msg':ros.generalize(ros.classFromTopic(topic)())}
						session.sock.send(encode(call))
					elif (receiver == "/rosjs/classesFromService"):
						service = msg[0].encode('ascii','ignore')
						call = {'receiver':'/rosjs/classesFromService','msg':{'req':ros.generalize(ros.classFromService(service)._request_class()),'rsp':ros.generalize(ros.classFromService(service)._response_class())}}
						session.sock.send(encode(call))
				else:
					idx = receiver.find('protected')
					if idx >= 0 and idx <=1:
						print "ignoring call to protected service"
						call = {'receiver':'nop','msg':{}}
						session.sock.send(encode(call))
					else:
						cls = ros.classFromService(receiver)
						serviceCaller = ServiceCaller(ros,session,receiver,ros.specify(cls._request_class._slot_types,msg))
						serviceCaller.start()

	except:
		print "Problem " * 10
		traceback.print_exc()
		print "Problem " * 10

	if (not keyurl or (now - session.rosjsExtStarted <= session.rosjsExtDuration)):
		session.transition(Session.ready)
	else:
		session.transition(Session.closed)

def handleFrameFactory(handleMsgFactory, sub, keyurl, ros):
	def handleFrame(frame, session):
		return handleFrameHelper(frame, session, handleMsgFactory, sub, keyurl, ros)
	return handleFrame

if __name__ == "__main__":
	ros = ROSProxy()
	import rospy
	rospy.init_node('rosjs')

	latest = {}
	latestLock = Lock()

	keyurl = rospy.get_param('/brown/rosjs/keyurl','')
	if "--keyurl" in sys.argv:
		idx = sys.argv.index("--keyurl")+1
		if idx < len(sys.argv):
			keyurl = sys.argv[idx]
			print "Only users with keys from %s will be accepted." % (keyurl,)
		else:
			print "Please provide the URL of the keyserver."
			sys.exit(-1)

	host = rospy.get_param('/brown/rosjs/host','')
	if "--host" in sys.argv:
		idx = sys.argv.index("--host")+1
		if idx < len(sys.argv):
			host = sys.argv[idx]
			print "rosjs is bound to %s." % (host,)
		else:
			print "Please provide a hostname."
			sys.exit(-1)

	port = rospy.get_param('/brown/rosjs/port',9090)
	if "--port" in sys.argv:
		idx = sys.argv.index("--port")+1
		if idx < len(sys.argv):
			port = int(sys.argv[idx])
			print "rosjs is will use port %s." % (port,)
		else:
			print "Please provide a port number."
			sys.exit(-1)

	def sub(topic, handler):
		idx = topic.find('protected')
		if idx >= 0 and idx <=1:
			print "ignoring request to listen to protected topic"
			return
		latestLock.acquire()
		repeat = False
		if topic in latest.keys():
			repeat = True
		latestLock.release()

		if not repeat:
			print "subscribing to: %s" % (topic,)
			cls = ros.classFromTopic(topic)
			rospy.Subscriber(topic, cls, handler, queue_size=1)

	def handleMsgFactory(topic):
		def handleMsg(msg):
			latestLock.acquire()

			seq = 0
			if topic in latest.keys():
				seq = latest[topic]['seq']
			latest[topic] = {'seq':seq + 1,'msg':msg}

			latestLock.release()

		return handleMsg

	def handleOutput(session):
		if (keyurl):
			if ('rosjsExtStarted' in dir(session) and time() - session.rosjsExtStarted > session.rosjsExtDuration):
				session.transition(Session.closed)
				return

		for topic in session.data.keys():
			if hasattr(session.data[topic],'publish'):
				continue
			delay = session.data[topic]['delay']
			lastEmission = session.data[topic]['lastEmission']
			lastSeq = session.data[topic]['lastSeq']
			data = session.data[topic]

			latestLock.acquire()

			if topic in latest.keys():
				current = latest[topic]

				#release as soon as possible
				latestLock.release()

				if current['seq'] > lastSeq:
					elapsed = (time() - lastEmission)*1000
					if elapsed > delay:
						if 'encoding' in data:
							call = {'receiver':topic, 'msg': ros.generalize(current['msg'],data['encoding'],data['width'],data['height'],data['quality'])}
						else:
							call = {'receiver':topic, 'msg': ros.generalize(current['msg'])}
						session.sock.send(encode(call))
						if 'encoding' in data: 
							session.data[topic] = {'delay':delay,'lastEmission':time(),'lastSeq':current['seq'],'encoding':data['encoding'],'width':data['width'],'height':data['height'],'quality':data['quality']}
						else:
							session.data[topic] = {'delay':delay,'lastEmission':time(),'lastSeq':current['seq']}
			else:
				latestLock.release()

	def loop():
		return not rospy.is_shutdown()

	restart = True
	while loop():
		if restart:
			restart = False
			try:
				serveForever(handleFrameFactory(handleMsgFactory, sub, keyurl, ros), handleOutput, loop, host, port)
			except:
				restart = True
				print "Problem " * 10
				traceback.print_exc()
				print "Problem " * 10
		sleep(1)
