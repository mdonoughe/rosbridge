#!/usr/bin/python
from ROSProxy import ROSProxy
import rospy; rospy.init_node('rosjs')
import sys, traceback
import json
from threading import Thread
from twisted.internet import reactor
from twisted.web.static import File
from websocket import WebSocketHandler, WebSocketSite
from time import time
from urllib2 import urlopen
import re

def encode(obj):
	return json.dumps(obj)

if __name__ == "__main__":
	ros = ROSProxy()
	rospy.init_node('rosjs')

	latest = {}
	openSockets = []

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

	class Handler(WebSocketHandler):
		def connectionMade(self):
			print('Connection from %s' % self.transport.getPeer())
			self.publishers = {}
			self.subscribers = {}
			self.authed = not self.keyurl
			self.openSockets.append(self)

		def sub(self, topic, handler):
			idx = topic.find('protected')
			if idx >= 0 and idx <=1:
				print "ignoring request to listen to protected topic"
				return
			repeat = False
			if topic in latest.keys():
				repeat = True

			if not repeat:
				print "subscribing to: %s" % (topic,)
				cls = self.ros.classFromTopic(topic)
				rospy.Subscriber(topic, cls, handler, queue_size=1)

		def frameReceived(self, frame):
			print(frame)
			call = json.loads(frame)
			receiver = call["receiver"]
			msg = call["msg"]

			if 'type' in call.keys():
				#print "Publish Topic!"
				if self.authed:
					typ = call["type"]

					if not receiver in self.publishers:
						self.publishers[receiver] = rospy.Publisher(receiver.encode('ascii','ignore'), ros.msgClassFromTypeString(typ))

					self.publishers[receiver].publish(ros.specify(typ,msg))
			else:
				#print "Service Call!"
				callback = call["callback"]
				if self.authed or receiver.startswith('/rosjs/authorize'):
					if receiver.startswith('/rosjs'):
						call = {'callback':callback}
						if (receiver == "/rosjs/topics"):
							call['msg'] = ros.topics
							self.transport.write(encode(call))
						elif (receiver == "/rosjs/services"):
							call['msg'] = ros.services
							self.transport.write(encode(call))
						elif (receiver == "/rosjs/subscribe"):
							topic = msg[0].encode('ascii','ignore')
							delay = msg[1]
							self.sub(topic, handleMsgFactory(topic))
							if len(msg) > 2:
								self.subscribers[topic] = {'delay':delay,'lastEmission':0,'lastSeq':0,'encoding':msg[2],'width':msg[3],'height':msg[4],'quality':msg[5]}
							else:
								self.subscribers[topic] = {'delay':delay,'lastEmission':0,'lastSeq':0}
							call['msg'] = 'OK'
							self.transport.write(encode(call))
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

							call['msg'] = 'OK'
							if (not success):
								call['msg'] = 'ERROR'
							self.transport.write(encode(call))
						elif (receiver == "/rosjs/authorize"):
							key = msg[0].encode('ascii','ignore')
							if (self.keyurl):
								reactor.callInThread(self.doAuth, self, key, callback)
							else:
								call['msg'] = 'OK'
								self.transport.write(encode(call))
						elif (receiver == "/rosjs/typeStringFromTopic"):
							topic = msg[0].encode('ascii','ignore')
							call['msg'] = ros.typeStringFromTopic(topic)
							self.transport.write(encode(call))
						elif (receiver == "/rosjs/typeStringFromService"):
							service = msg[0].encode('ascii','ignore');
							call['msg'] = ros.typeStringFromService(service)
							self.transport.write(encode(call))
						elif (receiver == "/rosjs/msgClassFromTypeString"):
							typStr = msg[0].encode('ascii','ignore');
							call['msg'] = ros.generalize(ros.msgClassFromTypeString(typStr)())
							self.transport.write(encode(call))
						elif (receiver == "/rosjs/reqClassFromTypeString"):
							typStr = msg[0].encode('ascii','ignore');
							call['msg'] = ros.generalize(ros.srvClassFromTypeString(typStr)._request_class())
							self.transport.write(encode(call))
						elif (receiver == "/rosjs/rspClassFromTypeString"):
							typStr = msg[0].encode('ascii','ignore');
							call['msg'] = ros.generalize(ros.srvClassFromTypeString(typStr)._response_class())
							self.transport.write(encode(call))
						elif (receiver == "/rosjs/classFromTopic"):
							topic = msg[0].encode('ascii','ignore')
							call['msg'] = ros.generalize(ros.classFromTopic(topic)())
							self.transport.write(encode(call))
						elif (receiver == "/rosjs/classesFromService"):
							service = msg[0].encode('ascii','ignore')
							call['msg'] = {'req':ros.generalize(ros.classFromService(service)._request_class()),'rsp':ros.generalize(ros.classFromService(service)._response_class())}
							self.transport.write(encode(call))
					else:
						idx = receiver.find('protected')
						if idx >= 0 and idx <=1:
							print "ignoring call to protected service"
							# is this really the right thing to do?
							call = {'callback':callback,'msg':{}}
							self.transport.write(encode(call))
						else:
							cls = ros.classFromService(receiver)
							def handleResponse(rsp):
								def completeResponse():
									call = {'callback':callback,'msg':self.ros.generalize(rsp)}
									self.transport.write(encode(call))
								reactor.callFromThread(completeResponse)
							reactor.callInThread(self.ros.callService, self.ros, receiver, ros.specify(cls._request_class._slot_types,msg), handleResponse)

			if not self.authed:
				self.transport.loseConnection()

		# runs in a background thread
		def doAuth(self, key, callback):
			duration = 0
			try:
				req = urlopen(self.keyurl + '?jsonp=invoke&key=' + key)
				req = req.read()
				duration = int(re.sub('[^0-9]','',req))
			except e:
				print('auth error: %s' % e)
			reactor.callFromThread(self.finishAuth, self, duration)

		def finishAuth(self, duration, callback):
			if duration > 0:
				self.authed = True
				if self.deauth and self.deauth.active():
					self.deauth.reset(duration)
				else:
					self.deauth = reactor.callLater(duration, self.doDeauth, self)
				self.transport.write(encode({'callback':callback,'msg':'OK'}))
			else:
				self.transport.write(encode({'callback':callback,'msg':'ERROR'}))

		def doDeauth(self):
			print('deauth')
			self.authed = False

		def sendLatest(self, topic):
			if not self.authed:
				self.transport.loseConnection()
				return

			if topic in self.subscribers and topic in latest:
				delay = self.subscribers[topic]['delay']
				lastEmission = self.subscribers[topic]['lastEmission']
				lastSeq = self.subscribers[topic]['lastSeq']
				data = self.subscribers[topic]

				current = latest[topic]

				if 'encoding' in data:
					call = {'receiver':topic, 'msg': ros.generalize(current['msg'],data['encoding'],data['width'],data['height'],data['quality'])}
				else:
					call = {'receiver':topic, 'msg': ros.generalize(current['msg'])}
				self.transport.write(encode(call))
				data['lastEmission'] = time()
				data['lastSeq'] = current['seq']

		def connectionLost(self, reason):
			print('closed %s: %s' % (self.transport.getPeer(), reason))
			# don't do this because it can break rosproxy
			#for topic in self.publishers:
				#publisher = self.publishers[topic]
				#publisher.unregister()
			self.publishers = {}
			for topic in self.subscribers:
				subscriber = self.subscribers[topic]
				if 'futureSend' in subscriber and subscriber['futureSend'].active():
					subscriber['futureSend'].cancel()
			self.subscribers = {}
			self.openSockets.remove(self)

	def wsHandlerFactory(transport):
		handler = Handler(transport)
		handler.keyurl = keyurl
		handler.ros = ros
		handler.openSockets = openSockets
		handler.latest = latest
		return handler

	def handleMsgFactory(topic):
		# this is called by the ROS thread
		def handleMsg(msg):
			# call in the reactor thread
			def handle(msg):
				seq = 0
				if topic in latest.keys():
					seq = latest[topic]['seq']
				latest[topic] = {'seq':seq + 1,'msg':msg}
				for socket in openSockets:
					if topic in socket.subscribers:
						sub = socket.subscribers[topic]
						elapsed = time() - sub['lastEmission']
						if elapsed * 1000 > sub['delay']:
							socket.sendLatest(topic)
						else:
							if not ('futureSend' in sub and sub['futureSend'].active()):
								sub['futureSend'] = reactor.callLater(sub['delay'] / 1000.0 - elapsed, socket.sendLatest, topic)
			reactor.callFromThread(handle, msg)
		return handleMsg

	def run_ros_thread():
		def wait_for_exit():
			rospy.spin()
			reactor.stop()
		# don't use reactor.callInThread or we might clog the thread pool
		thread = Thread(target = wait_for_exit)
		# the ros thread won't stop on exit
		thread.setDaemon(True)
		thread.start()
	reactor.callWhenRunning(run_ros_thread)

	#TODO: this path should be read from a preference with a different default
	root = File('.')
	#TODO: this should probably support exporting the script on top of a user specified file system
	site = WebSocketSite(root)
	#TODO: this path should be read from a preference
	site.addHandler('/ws', wsHandlerFactory)
	# 50 is the default backlog size
	reactor.listenTCP(port, site, 50, host)
	reactor.run()
