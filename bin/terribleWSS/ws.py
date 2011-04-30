#!/usr/bin/python
import socket
from select import select
from threading import Lock, Thread
from time import sleep
from struct import pack, unpack
import sys
import re
import hashlib

class Sender(Thread):
	def __init__(self,proxy):
		Thread.__init__(self)
		self.proxy = proxy

	def run(self):
		msg = self.proxy.msg
		args = self.proxy.args
		sent = 0
		toSend = len(msg)
		while sent < toSend:
			justSent = self.proxy.sock.send(msg[sent:],*args)
			if justSent == 0:
				raise RuntimeError("connection closed prematurely")
			sent = sent + justSent
		oldLock = self.proxy.lock
		Thread.__init__(self) 
		oldLock.release()

class SockProxy(object):
	def __init__(self,sock):
		self.sock = sock
		self.lock = Lock()
		thisSelf = self
		self.msg = ''
		self.args = []
		self.sender = Sender(thisSelf)

	def send(self,msg,*args):
		if self.lock.acquire(False):
			self.msg = msg
			self.args = args
			self.sender.start()

	def close(self):
		self.sock.close();

class Session(object):
	handshake = 0
	determineResponse = 1
	ready = 2
	receiveKey = 3
	sentinel = 4
	closed = 5
	def __init__(self,sock):
		self.sock = SockProxy(sock)
		self.buffer = []
		self.state = Session.handshake
		self.count = 0
		self.data = {}

	def transition(self,state,flush=False):
		self.count = 0
		self.state = state
		if flush:
			self.buffer = []

def googleHandshake(buffer):
	resp = ["HTTP/1.1 101 Web Socket Protocol Handshake\r\n"]
	resp.append("Upgrade: WebSocket\r\n")
	resp.append("Connection: Upgrade\r\n")

	resp.append("WebSocket-Location: ws://")
	resp.append(re.compile(r'.*Host:\s*([^\r\n]*).*',re.DOTALL).match(buffer).group(1))
	resp.append("/\r\n")

	resp.append("WebSocket-Origin: ")
	resp.append(re.compile(r'.*Origin:\s*([^\r\n]*).*',re.DOTALL).match(buffer).group(1))
	resp.append("\r\n")

	resp.append("\r\n")

	return ''.join(resp)

def actualHandshake(buffer):
	try:
		keyNumber1 = re.compile(r'.*Sec-WebSocket-Key1: ([^\r\n]*)\s*.*',re.DOTALL).match(buffer).group(1)
		spaces1 = len(re.findall(' ',keyNumber1))
		keyNumber1 = re.sub(r'[^0123456789]','',keyNumber1)
		keyNumber1 = int(keyNumber1)
		keyNumber2 = re.compile(r'.*Sec-WebSocket-Key2: ([^\r\n]*)\s*.*',re.DOTALL).match(buffer).group(1)
		spaces2 = len(re.findall(' ',keyNumber2))
		keyNumber2 = re.sub(r'[^0123456789]','',keyNumber2)
		keyNumber2 = int(keyNumber2)


		if (spaces1 == 0 or spaces2 == 0):
			return None

		if (keyNumber1 % spaces1 != 0 or keyNumber2 % spaces2 != 0):
			return None

		part1 = keyNumber1 / spaces1
		part2 = keyNumber2 / spaces2

		challenge = pack('!ii',part1,part2) + buffer[-8:]

		checker = hashlib.md5()
		checker.update(challenge)

		response = checker.digest()

		resp = ["HTTP/1.1 101 Web Socket Protocol Handshake\r\n"]
		resp.append("Upgrade: WebSocket\r\n")
		resp.append("Connection: Upgrade\r\n")

		resp.append("Sec-WebSocket-Location: ws://")
		resp.append(re.compile(r'.*Host:\s*([^\r\n]*).*',re.DOTALL).match(buffer).group(1))
		resp.append("/\r\n")

		resp.append("Sec-WebSocket-Origin: ")
		resp.append(re.compile(r'.*Origin:\s*([^\r\n]*).*',re.DOTALL).match(buffer).group(1))
		resp.append("\r\n")

		resp.append("\r\n")

		resp.append(response)

		return ''.join(resp)

	except:
		return None

def defaultHandleFrame(frame, session):
	session.transition(Session.ready)

def defaultHandleOutput(session):
	pass

def defaultLoop():
	pass

def serveForever(handleFrame = defaultHandleFrame, handleOutput=defaultHandleOutput, loop=defaultLoop, host='', port=9090):
	incoming = []
	try:
		serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		serverSocket.bind((host,port))
		serverSocket.listen(5)

		incoming = [serverSocket]
		outgoing = []
		sessions = {}

		def closeSocket(sock):
			print "closed %s" % (sock.fileno(),)
			del sessions[sock.fileno()]
			incoming.remove(sock)
			outgoing.remove(sock)
			sock.shutdown(socket.SHUT_RDWR)
			sock.close()

		def closeSockets():
			for i in sessions.keys():
				session = sessions[i]
				if session.state == Session.closed:
					closeSocket(session.sock.sock)

		while loop():
			inputReady,outputReady,errors = select(incoming,outgoing,[],0.005)

			for input in inputReady:
				if (input == serverSocket):
					#new connection
					connection, address = serverSocket.accept()
					print "Connection from %s:%s" % address
					sessions[connection.fileno()] = Session(connection)
					incoming.append(connection)
					outgoing.append(connection)

					print "%s concurrent connections.\n" % (len(incoming),)

				else:
					session = sessions[input.fileno()]
					data = input.recv(1)

					if len(data) == 0:
						closeSocket(input)
						continue

					session.buffer.append(data)

					if (session.state == Session.handshake):
						if "\r\n\r\n" ==  ''.join(session.buffer[-4:]):
							session.transition(Session.determineResponse)

					if (session.state == Session.determineResponse):
						call = ''.join(session.buffer[:-1])

						print "------"
						print call
						print "------"

						if re.compile(r'.*Sec-WebSocket-Key2',re.DOTALL).match(call) != None:
							print "actual handshake!"
							session.transition(Session.receiveKey)

						elif re.compile(r'raw').match(call) != None:
							print "raw socket"
							session.transition(Session.ready)

						else:
							#google handshake
							print "Google handshake"
							input.send(googleHandshake(call))
							session.transition(Session.ready)
						print "------\n"

					if (session.state == Session.receiveKey):
						if (session.count >= 8):
							resp = actualHandshake(''.join(session.buffer))
							if resp != None:
								input.send(resp)
								session.transition(Session.ready)
								continue
							else:
								closeSocket(input)
						else:	
							session.count = session.count + 1

					if (session.state == Session.ready):
						data = unpack('!b',data)[0]
						if (data >> 7) == 0:
							session.transition(Session.sentinel, flush=True)
							continue
						else:
							print "Binary data frames are unsupported"
							closeSocket(input)

					if (session.state == Session.sentinel):
						if data == '\xff':
							frame =  ''.join(session.buffer[:-1])
							handleFrame(frame, session)

			inputReady,outputReady,errors = select(incoming,outgoing,[],0.005)

			for output in outputReady:
				try:	
					handleOutput(sessions[output.fileno()])
				except socket.error, e:
					pass

			closeSockets()

	except:
		raise

	finally:
		socks = incoming + filter(lambda x:x not in incoming,outgoing)
		for sock in socks:
			try:
				print "closing sock: %s" % (sock.fileno(),)
				sock.shutdown(1)
				sock.close()
			except:
				print "failed to close sock: %s" % (sock.fileno(),)
