__package__ = 'messages'

import sys
import json

class ROSLinkHeader(object):

	def __init__(self, roslink_version=None, ros_version=None, system_id=None, owner_id=None, message_id=None, sequence_number=None, key=None):
		self.roslink_version = roslink_version
		self.ros_version = ros_version
		self.system_id = system_id
		self.message_id = message_id
		self.sequence_number = sequence_number
		self.key = key
		self.owner_id =owner_id
		pass
		
	def from_json(self, message):
		self.__dict__ = json.loads(message)
		self.roslink_version = self.__dict__['header']['roslink_version']
		self.ros_version = self.__dict__['header']['ros_version']
		self.system_id = self.__dict__['header']['system_id']
		self.message_id = self.__dict__['header']['message_id']
		self.sequence_number = self.__dict__['header']['sequence_number']
		self.key = self.__dict__['header']['key']
		self.owner_id = self.__dict__['header']['owner_id']
		
	def printMessage(self):
		print '--------------------------'
		print 'roslink_version: ', self.roslink_version
		print 'ros_version    : ', self.ros_version
		print 'system_id      : ', self.system_id
		print 'message_id     : ', self.message_id
		print 'sequence_number: ', self.sequence_number
		print 'key            : ', self.key
		print '--------------------------'
		
