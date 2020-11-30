import pythonProto.messages_robocup_ssl_wrapper_pb2 as wrapper

from info import InfoBall, InfoRobot


class Packet:
    """docstring for packet"""
    info_ball = InfoBall(0, 0, 0, 0)
    frame_number = -1

    def parse(self, binary):
        parser = wrapper.SSL_WrapperPacket()
        parser.ParseFromString(binary)
        if parser.detection.IsInitialized():
            self.frame_number = parser.detection.frame_number
            self.t_capture = parser.detection.t_capture
            self.t_sent = parser.detection.t_sent
            self.camera_id = parser.detection.camera_id
            for i in parser.detection.balls:
                self.info_ball = InfoBall(i.confidence, i.x, i.y, i.z)
            for i in parser.detection.robots_yellow:
                self.info_robotY.append(InfoRobot(i.robot_id, i.confidence, i.x, i.y, i.orientation))
            for i in parser.detection.robots_blue:
                self.info_robotB.append(InfoRobot(i.robot_id, i.confidence, i.x, i.y, i.orientation))

            pass
        if parser.geometry.IsInitialized():
            pass

    def __init__(self, binary):
        self.info_robotY = []
        self.info_robotB = []
        self.geometry = None

        self.parse(binary)

    # print(self.frame_number)

    # the simulation program sends only one team in each packet so the get the information of both teams
    # we need to receive two packet and combine what is inside but the ball will be according the last packet
    def add(self, binary):
        self.parse(binary)

    def to_string(self):
        str = "frame_number : {0}\n".format(self.frame_number)
        str += self.info_ball.to_string()
        str += "\nTeam Yellow : "
        for i in self.info_robotY:
            str += "\n\t" + i.to_string()

        str += "\nTeam Blue : "
        for i in self.info_robotB:
            str += "\n\t" + i.to_string()

        return str


class SPacket:
    timestamp = 0
    is_team_yellow = 0
    robots_commands = []

    def __init__(self, timestamp, is_team_yellow, robots_array):
        self.timestamp = timestamp
        self.is_team_yellow = is_team_yellow
        self.robots_commands = robots_array

    def to_string(self):
        str = "Timestamp : {0} \nisYellow : {1}\n".format(self.timestamp, self.is_team_yellow)
        for i in self.robots_commands:
            str += "id:{0} , \n\tkick_speedX:{1} , \n\tkick_speedY:{2} , \n\tvel_tangent:{3} , \n\tvel_normal:{4} , " \
                   "\n\tvel_angular:{5}, \n\tspinner:{6} , \n\twheels_speed:{7} , \n\twheel1:{8} , \n\twheel2:{9} , " \
                   "\n\twheel3:{10} , \n\twheel4:{11}\n\n".format(i.robotID, i.kick_speedX, i.kick_speedZ,
                                                                  i.vel_tangent, i.vel_normal, i.vel_angular, i.spinner,
                                                                  i.wheels_speed, i.wheel1, i.wheel2, i.wheel3,
                                                                  i.wheel4)
        return str
