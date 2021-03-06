#!/usr/bin/env python
from ROSProxy import ROSProxy
import roslib
import rospy; rospy.init_node('rosjs')
import sys, traceback
import json
from threading import Thread
from twisted.internet import reactor
from twisted.web.static import File
from twisted.web.resource import Resource
from twisted.web.client import getPage
from websocket import WebSocketHandler, WebSocketSite
from time import time
import re

def encode(obj):
	return json.dumps(obj)

if __name__ == "__main__":
	ros = ROSProxy()
	rospy.init_node('rosjs')

	latest = {}
	openSockets = []

	keyurl = rospy.get_param('~keyurl','')
	host = rospy.get_param('~host','')
	port = rospy.get_param('~port',9090)
	wspath = rospy.get_param('~wspath','/')
	jsfile = rospy.get_param('~jsfile','ros.js')
	docroot = rospy.get_param('~docroot','')

	class Handler(WebSocketHandler):
		def connectionMade(self):
			print('Connection from %s' % self.transport.getPeer())
			self.publishers = {}
			self.subscribers = {}
			self.authed = not self.keyurl
			self.deauth = None
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
				rospy.Subscriber(topic, self.classFromTopic(topic), handler, queue_size=1)

		def typeStringFromTopic(self, topic):
			typStr = self.ros.typeStringFromTopic(topic)
			if typStr is None:
				raise NameError('Topic does not exist')
			return typStr

		def typeStringFromService(self, topic):
			typStr = self.ros.typeStringFromService(topic)
			if typStr is None:
				raise NameError('Service does not exist')
			return typStr

		def msgClassFromTypeString(self, typStr):
			cls = self.ros.msgClassFromTypeString(typStr)
			if cls is None:
				raise NameError('Class does not exist')
			return cls

		def srvClassFromTypeString(self, typStr):
			cls = self.ros.srvClassFromTypeString(typStr)
			if cls is None:
				raise NameError('Class does not exist')
			return cls

		def classFromTopic(self, topic):
			return self.msgClassFromTypeString(self.typeStringFromTopic(topic))

		def classFromService(self, topic):
			return self.srvClassFromTypeString(self.typeStringFromService(topic))

		def callService(self, receiver, msg, callback):
			call = {'callback':callback}
			try:
				# rosproxy automatically waits for the service
				rsp = self.ros.callService(receiver, ros.specify(self.classFromService(receiver)._request_class.slot_types, msg))
				call['msg'] = self.ros.generalize(rsp)
			except Exception as e:
				call['error'] = str(e)
			reactor.callFromThread(self.transport.write, encode(call))

		def frameReceived(self, frame):
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
						try:
							if (receiver == "/rosjs/topics"):
								call['msg'] = self.ros.topics
								self.transport.write(encode(call))
							elif (receiver == "/rosjs/services"):
								call['msg'] = self.ros.services
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

								with open(filename, 'w') as log:
									log.write(obj)
								call['msg'] = 'OK'

								self.transport.write(encode(call))
							elif (receiver == "/rosjs/authorize"):
								key = msg[0].encode('ascii','ignore')
								if (self.keyurl):
									get = getPage(self.keyurl + '?jsonp=invoke&key=' + key)
									get.addCallback(self.doAuth, callback)
									get.addErrback(self.failAuth, callback)
								else:
									call['msg'] = 'OK'
									self.transport.write(encode(call))
							elif (receiver == "/rosjs/typeStringFromTopic"):
								topic = msg[0].encode('ascii','ignore')
								call['msg'] = self.typeStringFromTopic(topic)
								self.transport.write(encode(call))
							elif (receiver == "/rosjs/typeStringFromService"):
								service = msg[0].encode('ascii','ignore');
								call['msg'] = self.typeStringFromService(service)
								self.transport.write(encode(call))
							elif (receiver == "/rosjs/msgClassFromTypeString"):
								typStr = msg[0].encode('ascii','ignore');
								call['msg'] = ros.generalize(self.msgClassFromTypeString(typStr)())
								self.transport.write(encode(call))
							elif (receiver == "/rosjs/reqClassFromTypeString"):
								typStr = msg[0].encode('ascii','ignore');
								call['msg'] = ros.generalize(self.srvClassFromTypeString(typStr)._request_class())
								self.transport.write(encode(call))
							elif (receiver == "/rosjs/rspClassFromTypeString"):
								typStr = msg[0].encode('ascii','ignore');
								call['msg'] = ros.generalize(self.srvClassFromTypeString(typStr)._response_class())
								self.transport.write(encode(call))
							elif (receiver == "/rosjs/classFromTopic"):
								topic = msg[0].encode('ascii','ignore')
								call['msg'] = ros.generalize(self.classFromTopic(topic)())
								self.transport.write(encode(call))
							elif (receiver == "/rosjs/classesFromService"):
								service = msg[0].encode('ascii','ignore')
								cls = self.classFromService(service)
								call['msg'] = {'req':ros.generalize(cls._request_class()),'rsp':ros.generalize(cls._response_class())}
								self.transport.write(encode(call))
						except Exception as e:
							call['error'] = str(e)
							self.transport.write(encode(call))
					else:
						idx = receiver.find('protected')
						if idx >= 0 and idx <=1:
							print "ignoring call to protected service"
							call = {'callback':callback,'error':'Forbidden'}
							self.transport.write(encode(call))
						else:
							reactor.callInThread(self.callService, receiver, msg, callback)

			if not self.authed:
				self.transport.loseConnection()

		def doAuth(self, body, callback):
			try:
				duration = int(re.sub('[^0-9]','',body))
			except e:
				self.failAuth(e, callback)
				return
			if duration < 1:
				self.failAuth('duration=0', callback)
				return
			self.authed = True
			if self.deauth and self.deauth.active():
				self.deauth.reset(duration)
			else:
				self.deauth = reactor.callLater(duration, self.doDeauth)
			self.transport.write(encode({'callback':callback,'msg':'OK'}))

		def failAuth(self, error, callback):
			print('auth error: %s' % error)
			self.transport.write(encode({'callback':callback,'msg':'ERROR'}))

		def doDeauth(self):
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

	if docroot:
		root = File(docroot)
	else:
		root = Resource()
	if jsfile:
		root.putChild(jsfile, File(roslib.packages.resource_file('rosbridge', '', 'ros.js')))
	site = WebSocketSite(root)
	site.addHandler(wspath, wsHandlerFactory)
	# 50 is the default backlog size
	reactor.listenTCP(port, site, 50, host)
	reactor.run()
