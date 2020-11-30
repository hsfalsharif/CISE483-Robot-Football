from position import Position


class CMD:
	def __init__(self):
		self.id = 0
		self.des = ""
		self.point = Position(0, 0)
		self.next_point = Position(0, 0)
		self.kickspeedx = 0
		self.kickspeedz = 0
		self.veltangent = 0
		self.velnormal = 0
		self.velangular = 0
		self.spinner = 0
		self.wheelsspeed = 0
		self.wheel1 = 0
		self.wheel2 = 0
		self.wheel3 = 0
		self.wheel4 = 0
		self.timeout = 0
		self.isteamyellow = True

	def velocity(self, x, y, angle):
		self.veltangent = x
		self.velnormal = y
		self.velangular = angle

	def getx(self):
		return self.point.x + 0

	def gety(self):
		return self.point.y + 0

	def to_string(self):
		return "des :{0} ({1},{2}) ".format(self.des, self.point.x, self.point.y)

	# self.wheel1 =  (1.0 / 0.027) * (( (0.0793 * vw) - (vx * math.sin(60)) + (vy * math.cos(60))) )
	# self.wheel3 =  (1.0 / 0.027) * (( (0.0793 * vw) - (vx * math.sin(225)) + (vy *math.cos(225))) )
	# self.wheel2 =  (1.0 / 0.027) * (( (0.0793 * vw) - (vx * math.sin(135)) + (vy *math.cos(135))) )
	# self.wheel4 =  (1.0 / 0.027) * (( (0.0793 * vw) - (vx * math.sin(300)) + (vy *math.cos(300))) )
