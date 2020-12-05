class Position:
	def __init__(self, x, y):
		self.x = x
		self.y = y

	def to_string(self):
		return "Position :({0} , {1})".format(self.x, self.y)
