from Commands import CMD
from position import Position


class Robot:
    robotID = 0
    position = Position(0, 0)
    orientation = 0
    kick_speedX = 0
    kick_speedZ = 0
    vel_tangent = 0
    vel_normal = 0
    vel_angular = 0
    spinner = 0
    wheels_speed = 0
    wheel1 = 0
    wheel2 = 0
    wheel3 = 0
    wheel4 = 0
    commands = []

    def set_parameter(self, robot_id, position):
        self.robotID = robot_id
        self.position = position

    def __init__(self, robot_id, position):
        self.robotID = robot_id
        self.position = position
        self.commands = CMD()

    def set_cmd(self, cmd):
        self.commands = cmd

    def do_now(self, cmd):
        self.do_command(cmd)

    def update(self):
        self.do_command(self.commands)

    def do_command(self, cmd):
        self.wheels_speed = cmd.wheels_speed
        self.wheel1 = cmd.wheel1
        self.wheel2 = cmd.wheel2
        self.wheel3 = cmd.wheel3
        self.wheel4 = cmd.wheel4
        self.vel_tangent = cmd.vel_tangent
        self.vel_normal = cmd.vel_normal
        self.vel_angular = cmd.vel_angular

    def to_string(self):
        return "id:{0} , \n\tkick_speedX:{1} , \n\tkick_speedY:{2} , \n\tvel_tangent:{3} , \n\tvel_normal:{4} , " \
               "\n\tvel_angular:{5}, \n\tspinner:{6} , \n\twheels_speed:{7} , \n\twheel1:{8} , \n\twheel2:{9} , " \
               "\n\twheel3:{10} , \n\twheel4:{11}\n\n".format(self.robotID, self.kick_speedX, self.kick_speedZ,
                                                              self.vel_tangent, self.vel_normal, self.vel_angular,
                                                              self.spinner, self.wheels_speed, self.wheel1, self.wheel2,
                                                              self.wheel3, self.wheel4)
