"""This module provides a class that allows for (slightly) easier dynamic access to ROS"""

import roslib; roslib.load_manifest('rospy')
import rospy
roslib.load_manifest('rosservice')
import roslib.rostime
import rosservice
import re 

roslib.load_manifest('cv_bridge')
from cv_bridge import CvBridge, CvBridgeError
import cv
from PIL import Image
import StringIO
from base64 import standard_b64encode

class ROSProxy(object):
	def __init__(self):
		self.mans = {}
		self.mods = {}
		self.bridge = CvBridge()
		self.imgCls = self.msgClassFromTypeString('sensor_msgs/Image')

	def __GetTopics(self):
		return [x[0] for x in rospy.get_published_topics()]
	topics = property(fget=__GetTopics)

	def __GetServices(self):
		return rosservice.get_service_list()
	services = property(fget=__GetServices)

	def typeStringFromTopic(self, topic):
		try:
			return [x[1] for x in rospy.get_published_topics() if x[0] == topic][0]
		except:
			print "Can't find topic %s" % (topic,)
			return None

	def typeStringFromService(self, service):
		try:
			return rosservice.get_service_type(service)
		except:
			print "Can't find service %s" % (service,)
			return None

	def __classFromTypeString(self, typeString, subname):
		basemodule, itype = typeString.split('/')

		if not (basemodule in self.mans):
			try:
				roslib.load_manifest(basemodule)	
				self.mans[basemodule] = True;
			except:
				print "Can't find class for %s" % (typeString,)
				return None

		modname = basemodule + '.'+ subname + '._' + itype
		if not (modname in self.mods):
			try:
				mod = __import__(modname)
				self.mods['modname'] = mod
			except:
				return None

		mod = self.mods['modname']

		return getattr(getattr(getattr(mod,subname),'_' + itype), itype)

	def msgClassFromTypeString(self, typeString):
		return self.__classFromTypeString(typeString, 'msg')

	def srvClassFromTypeString(self, typeString):
		return self.__classFromTypeString(typeString, 'srv')

	def classFromTopic(self, topic):
		return self.msgClassFromTypeString(self.typeStringFromTopic(topic))

	def classFromService(self, service):
		return self.srvClassFromTypeString(self.typeStringFromService(service))

	def callService(self, service, arguments, callback=False, wait=True):
		def defCallback(x):
			pass

		if callback == False:
			callback = defCallback

		if wait:
			try:
				rospy.wait_for_service(service)
			except:
				callback(None)
				raise
		try:
			function = rospy.ServiceProxy(service, self.classFromService(service))
			response = apply(function, arguments)
			callback(response)
		except:
			callback(None)
			raise

	def generalize(self, inst, encoding='jpg', width=160, height=120, qual=30):
		if isinstance(inst, self.imgCls):
			if (len(inst.data) == 0):
				return {}
			cvImg = self.bridge.imgmsg_to_cv(inst,"rgb8")
			size = cv.GetSize(cvImg)
			img = Image.fromstring("RGB", size, cvImg.tostring())
			if width != size[0] or height != size[1]:
				img = img.resize((width,height),Image.NEAREST)
			buf = StringIO.StringIO()
			if encoding == 'png':
				img.save(buf, 'PNG', optimize=1)
			else:
				img.save(buf, 'JPEG', quality=qual)
			data = buf.getvalue()
			buf.close()
			return {'uri':'data:image/' + encoding + ';base64,' + standard_b64encode(data)}
		elif hasattr(inst,'__slots__'):
			obj = {}
			for i in xrange(len(inst.__slots__)):
				field = inst.__slots__[i]
				if (hasattr(inst,'_slot_types') and inst._slot_types[i] == 'uint8[]'):
					obj[field] = self.generalize(standard_b64encode(getattr(inst,field)), encoding, width, height, qual)
				else:
					obj[field] = self.generalize(getattr(inst,field), encoding, width, height, qual)
			return obj
		elif isinstance(inst,tuple) or isinstance(inst,list):
			return [self.generalize(x, encoding, width, height, qual) for x in inst]
		else:
			return inst

	braces = re.compile(r'\[[^\]]*\]') 
	atomics = ['bool', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'float32', 'float64', 'string']

	def specify(self, typeStr, obj):
		if isinstance(typeStr,list):
			lst = []
			for i in xrange(len(typeStr)):
				lst.append(self.specify(typeStr[i],obj[i]))
			return lst
		elif typeStr != self.braces.sub('',typeStr):
			return [self.specify(self.braces.sub('',typeStr), x) for x in obj]
		elif typeStr in self.atomics:
			if typeStr == 'string':
				return obj.encode('ascii','ignore')
			return obj
		elif typeStr == 'time' or typeStr == 'duration':
			inst = None
			if typeStr == 'time':
				inst = roslib.rostime.Time()
			else:
				inst = roslib.rostime.Duration()
			inst.nsecs = obj['nsecs']
			inst.secs = obj['secs']
			return inst
		else:
			if typeStr == 'Header':
				typeStr = 'roslib/Header'

			cls = self.msgClassFromTypeString(typeStr)
			inst = cls()
			for i in xrange(len(cls._slot_types)):
				field = cls.__slots__[i]
				typ = cls._slot_types[i]
				if field in obj:
					setattr(inst,field,self.specify(typ,obj[field]))
			return inst
