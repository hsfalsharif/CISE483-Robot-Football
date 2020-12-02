class InfoRobot:
    """docstring for InfoRobot"""

    def __init__(self, robot_id, confidence, x, y, orientation):
        self.id = robot_id
        self.confidence = confidence
        self.x = x
        self.y = y
        self.orientation = orientation

    def to_string(self):
        return "Robot(id:{0}) : ({1} , {2}) ~ {3} orientation : {4}".format(self.id, self.x, self.y, self.confidence,
                                                                            self.orientation)


class InfoBall:
    """docstring for InfoBall"""

    def __init__(self, confidence, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.confidence = confidence

    def to_string(self):
        return "Ball : ({0} , {1} , {2}) ~ {3} ".format(self.z, self.y, self.x, self.confidence)
