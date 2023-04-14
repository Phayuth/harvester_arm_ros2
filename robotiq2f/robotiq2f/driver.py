import comModbusRtu

class gg:
	def __init__(self, device_id=0, comport='/dev/ttyUSB0',baud=115200):

		self.client = comModbusRtu.communication()
		
		connected = self.client.connectToDevice(device = comport)
		if not connected:
			raise Exception("Communication with gripper %d on serial port: %s and baud rate: %d not achieved" % (device_id, comport, baud))
		
		self.device_id = device_id+9
		self.initialize_communication_variables()

		self.message = []       
	
	def _update_cmd(self):

		#Initiate command as an empty list
		self.message = []

		#Build the command with each output variable
		self.message.append(self.rACT + (self.rGTO << 3) + (self.rATR << 4))
		self.message.append(0)
		self.message.append(0)
		self.message.append(self.rPR)
		self.message.append(self.rSP)
		self.message.append(self.rFR)
		print(self.message)

	def sendCommand(self):
		"""Send the command to the Gripper."""
		print("SEND COMMAND")
		return self.client.sendCommand(self.message)


	def getStatus(self):
		"""Request the status from the gripper"""

		#Acquire status from the Gripper
		status = self.client.getStatus(6)

		# Check if read was successful
		if( status is None ):
			return False

		#Assign the values to their respective variables
		self.gACT = (status[0] >> 0) & 0x01
		self.gGTO = (status[0] >> 3) & 0x01
		self.gSTA = (status[0] >> 4) & 0x03
		self.gOBJ = (status[0] >> 6) & 0x03
		self.gFLT =  status[2]
		self.gPR  =  status[3]
		self.gPO  =  status[4]
		self.gCU  =  status[5]

		stat = [
		self.gACT, # 0 Reset - 1 Activation
		self.gGTO, # 0 Stopped (performing activate / auto release) - 1 Goto Pos Request
		self.gSTA, # 0 Reset - 1 Act in progress - 2 Not used - 3 Act Completed
		self.gOBJ, # 0 No Obj Detected - 1 Detected on Open - 2 Detected on Close - 3 Finger arrived on req pos
		self.gFLT, # 0 No fault - 5:15 Fault  
		self.gPO,  # Get req Pos, 0 Fully Open - 255 Fully Closed
		self.gPR,  # Get current Pos , 0 Fully Open - 255 Fully Closed
		self.gCU]  # Get electric current
		
		for i in range(len(stat)):
			print(stat[i])

		return True

	def reqStatus(self):
		stat = [
		self.rACT, # 0 Reset - 1 Activation
		self.rPR,  # Req Pos , 0 Fully Open - 255 Fully Closed
		self.rSP,  # Req speed 0:255
		self.rFR,  # Req force 0:255
		self.rARD, #
		self.rATR,
		self.rGTO]
		
		for i in range(len(stat)):
			print(stat[i])

	def initialize_communication_variables(self):
		# Out
		self.rPR = 0
		self.rSP = 0
		self.rFR = 0
		self.rARD = 0
		self.rATR = 0
		self.rGTO = 0
		self.rACT = 0
		# In
		self.gSTA = 0
		self.gACT = 0
		self.gGTO = 0
		self.gOBJ = 0
		self.gFLT = 0
		self.gPO = 0
		self.gPR = 0
		self.gCU = 0

		self._update_cmd()
		
	def shutdown(self):
		self.client.disconnectFromDevice()
	
	def activate(self):
		self.rACT = 1
		self.rPR = 0
		self.rSP = 255
		self.rFR = 150
		self._update_cmd()
	
	def deactivate(self):
		self.rACT = 0
		self._update_cmd()
		
	def activate_emergency_release(self,open_gripper):
		self.rATR = 1
		if (open_gripper == True):
			self.rARD=0
		else:
			self.rARD = 1
		self._update_cmd()

		print("Power off the Gripper and Power it on again, then activate gripper")
				
	def deactivate_emergency_release(self):
		self.rATR = 0
		self._update_cmd()

	def goto(self, pos, vel, force):
		self.rACT = 1
		self.rGTO = 1
		self.rPR = pos      # 0 Fully Open - 255 Fully Close
		self.rSP = vel      # 0 Min Speed  - 255 Max Speed
		self.rFR = force    # 0 Min Force  - 255 Max Force
		self._update_cmd()
		

	def stop(self):
		self.rACT = 1
		self.rGTO = 0
		self._update_cmd()
				
	def is_ready(self):
		return self.gSTA == 3 and self.gACT == 1

	def is_reset(self):
		return self.gSTA == 0 or self.gACT == 0

	def is_moving(self):
		return self.gGTO == 1 and self.gOBJ == 0

	def is_stopped(self):
		return self.gOBJ != 0

	def object_detected(self):
		return self.gOBJ == 1 or self.gOBJ == 2

	def get_fault_status(self):
		return self.gFLT

	def get_pos(self):
		po = float(self.gPO) # 0 Fully Open - 255 Fully Close
		return po

	def get_req_pos(self):
		pr = float(self.gPR) # 0 Fully Open - 255 Fully Close
		return pr

	def get_current(self):
		return self.gCU * 0.1