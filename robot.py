import random
import time
from math import sqrt, cos, sin, copysign

import numpy as np

from Commands import CMD
from Node import Node
from position import Position


class Robot:
    robotID = 0
    position = Position(0, 0)
    orientation = 0
    kickspeedx = 0
    kickspeedz = 0
    veltangent = 0
    velnormal = 0
    velangular = 0
    spinner = 0
    wheelsspeed = 0
    wheel1 = 0
    wheel2 = 0
    wheel3 = 0
    wheel4 = 0
    ball = Position(0, 0)
    commands = []
    obstacles = []
    planned_path = []
    max_velocity = 0.2
    current_path_segment = []
<<<<<<< HEAD
    target_position = Position(0, 0)
    target_velocity = [0, 0]
=======
    target_position = Position(None, 0)
    target_velocity = [None, 0]
>>>>>>> main
    current_state = None
    current_sub_state = None
    next_state = None
    next_sub_state = None
    R0 = None
    R1 = None
    R2 = None
<<<<<<< HEAD
    global_velocity = [0, 0]
    window = None
    impl = None
=======
>>>>>>> main

    def set_parameter(self, robot_id, position):
        self.robotID = robot_id
        self.position = position

    def __init__(self, robot_id, position):
        self.robotID = robot_id
        self.position = position
        self.commands = CMD()
        # imgui.create_context()
        # self.window = self.impl_glfw_init()
        # self.impl = GlfwRenderer(self.window)

    def set_cmd(self, cmd):
        self.commands = cmd

    def do_now(self, cmd):
        self.do_command(cmd)

    def update(self, Playground):
        # is robot on path ??
<<<<<<< HEAD
        self.target_position.x = self.ball.x
        self.target_position.y = self.ball.y
        if self.robotID == 2 and self.target_position.x is not None:
            self.move_to()# move_to or move_to_RRT
            # print(self.position.to_string())
=======
        # self.target_position.x = self.ball.x
        # self.target_position.y = self.ball.y
        if self.target_position.x is not None and self.target_velocity[0] is not None:
            self.move_to()
>>>>>>> main
        # re-adjust velocities
        # self.do_command(self.commands)

    def do_command(self, cmd):
        self.wheelsspeed = cmd.wheelsspeed
        self.wheel1 = cmd.wheel1
        self.wheel2 = cmd.wheel2
        self.wheel3 = cmd.wheel3
        self.wheel4 = cmd.wheel4
        self.veltangent = cmd.veltangent
        self.velnormal = cmd.velnormal
        self.velangular = cmd.velangular

    def to_string(self):
        return "id:{0} , \n\tkickspeedx:{1} , \n\tkick_speedY:{2} , \n\tveltangent:{3} , \n\tvelnormal:{4} , " \
               "\n\tvelangular:{5}, \n\tspinner:{6} , \n\twheelsspeed:{7} , \n\twheel1:{8} , \n\twheel2:{9} , " \
               "\n\twheel3:{10} , \n\twheel4:{11}\n\n".format(self.robotID, self.kickspeedx, self.kickspeedz,
                                                              self.veltangent, self.velnormal, self.velangular,
                                                              self.spinner, self.wheelsspeed, self.wheel1, self.wheel2,
                                                              self.wheel3, self.wheel4)

<<<<<<< HEAD
    def move_to_RRT(self, Playground):  # start, goal and obstacles are all positions
        start = Position(self.position.x + self.global_velocity[0] * 0.018,
                         self.position.y + self.global_velocity[1] * 0.018)
        goal = Position(self.target_position.x, self.target_position.y)

        path = self.RRT(start, goal, Playground)
        # print(len(path))
        # self.planned_path = path
        #print(path)
        first_segment = path[0]
        # print(first_segment)
        vx = first_segment[1][0] - first_segment[0][0]
        vy = first_segment[1][1] - first_segment[0][1]
        #print("Simulator: ({0}, {1}) -> ({2}, {3})".format(first_segment[0][0], first_segment[0][1],first_segment[1][1], first_segment[0][1]))
        magnitude = self.vector_mag([vx, vy])
        vx = self.safe_division(vx, magnitude)
        vy = self.safe_division(vy, magnitude)
        self.global_velocity = [vx, vy]
        lv = self.world_to_local([vx, vy])

        d = self.distance_to_point([goal.x, goal.y])

        vfx = lv[0] * d
        vfy = lv[1] * d

        self.veltangent = self.clamp(vfx, self.max_velocity)
        self.velnormal = self.clamp(vfy, self.max_velocity)

    def RRT(self, start, goal, Playground):  # how to check for obstacles here?
        tree = [Node(start, None)]
        # print([random_position.x,random_position.y])
        distance_to_goal = np.inf
        # building tree by traversing environment
        while distance_to_goal >= 100:
            distance_to_goal = self.extend(tree, goal, Playground)
            # print(distance_to_goal)
        # return path by traversing tree
        # for n in tree:
        #    print(n.to_string())
        last_node = tree[len(tree) - 1]  # indicates the final position in the path
        path = [[[last_node.position.x - 0.00001, last_node.position.y - 0.00001],
                 [last_node.position.x, last_node.position.y]]]
        current_node = last_node
        while current_node is not None:
            if current_node.parent is not None:
                path.append([[current_node.position.x, current_node.position.y], [current_node.parent.position.x,
                                                                                  current_node.parent.position.y]])
            current_node = current_node.parent
        path.reverse()
        return path
        #  traverse all paths until the final node is reached
        #  if paths lead to dead ends, eliminate them
        #  keep the path that leads us to the goal

    def extend(self, tree, goal, Playground):  # do we need to account for obstacles here? if so, how?
        random.seed(time.time())
        u = (-1) ** (random.randint(1, 2)) * random.random()
        random_position = Position(u * 12, u * 10)
        # print(random_position.to_string())
        node_index = 0
        smallest_distance = np.inf
        nearest_node = Node(Position(np.inf, np.inf), None)
        for i in range(len(tree)):
            # print(tree[i])
            distance = sqrt(
                (tree[i].position.x - goal.x) ** 2 + (tree[i].position.y - goal.y) ** 2)
            if distance < smallest_distance:
                smallest_distance = distance
                # print(smallest_distance)
                nearest_node = tree[i]
                node_index = i

        vector = [random_position.x - nearest_node.position.x, random_position.y - nearest_node.position.y]
        # print(vector)
        vector_mag = sqrt(vector[0] ** 2 + vector[1] ** 2)
        unit_vector = [vector[0] / vector_mag, vector[1] / vector_mag]
        new_node = Node(Position(nearest_node.position.x + 0.01 * unit_vector[0],
                                 nearest_node.position.y + 0.01 * unit_vector[1]),
                        nearest_node)  # 0.3 is incrementation margin
        tree.append(new_node)  # adding new node to the tree
        self.planned_path.append([[nearest_node.position.x, nearest_node.position.y],
                                  [new_node.position.x, new_node.position.y]])
        #print(self.planned_path)
        #Playground.update_gui()
        distance_to_goal = sqrt((new_node.position.x - goal.x) ** 2 +
                                (new_node.position.y - goal.y) ** 2)
        return distance_to_goal
        # need to check if we have reached the goal or not
        # then if we have reached the goal, I need to return a path from the start node to the goal node by traversing
        # the tree
        # also need to assess the real-time operation of this RRT algorithm and whether we need to do it on each from
        # or not

    def move_to(self):
        origin = [self.position.x + self.global_velocity[0] * 0.018, self.position.y + self.global_velocity[1] * 0.018]
        goal = [self.target_position.x, self.target_position.y]
        path = self.planner(origin, goal, self.obstacles, 0)
        guipath = []
        for i in range(len(path)-1):
            guipath.append([path[i],path[i+1]])

        
        self.planned_path = guipath
        first_segment = [path[0], path[1]]
        vx = first_segment[1][0] - first_segment[0][0]
        vy = first_segment[1][1] - first_segment[0][1]

        magnitude = self.vector_mag([vx, vy])
        vx = self.safe_division(vx, magnitude)
        vy = self.safe_division(vy, magnitude)
        self.global_velocity = [vx, vy]
        lv = self.world_to_local([vx, vy])

        d = self.distance_to_point(goal)

        vfx = lv[0] * d
        vfy = lv[1] * d

        self.veltangent = self.clamp(vfx, self.max_velocity)
        self.velnormal = self.clamp(vfy, self.max_velocity)

    def clamp(self, x, mx):
        if abs(x) > mx:
            return copysign(mx, x)
        return x

=======
    def move_to(self):
        origin = [self.position.x, self.position.y]
        goal = [self.target_position.x, self.target_position.y]
        path = self.planner(origin, goal, self.obstacles, 0)
        first_segment = [path[0], path[1]]
        vx = first_segment[1][0] - first_segment[0][0]
        vy = first_segment[1][1] - first_segment[0][1]
        lv = self.world_to_local([vx, vy])
        final_v = self.world_to_local(self.target_velocity)
        magnitude = self.vector_mag(lv)
        lvyu = self.safe_division(lv[1], magnitude)
        lvxu = self.safe_division(lv[0], magnitude)
        d = self.distance_to_point(goal)
        print("path :{0}\nWorld Velocity:{1} Local Velocity:{2}".format(first_segment, [vx, vy], [lvxu * d, lvyu * d]))
        print("Computed path : {0}".format(path))
        self.veltangent = final_v[0] + lvxu * d
        self.velnormal = final_v[0] + lvyu * d

>>>>>>> main
    def local_to_world(self, vec):
        o = self.orientation
        vx = vec[0] * cos(o) - vec[1] * sin(o)
        vy = vec[0] * sin(o) + vec[1] * cos(o)
        return [vx, vy]

    def world_to_local(self, vec):
        o = self.orientation
        vx = vec[0] * cos(o) + vec[1] * sin(o)
        vy = -vec[0] * sin(o) + vec[1] * cos(o)
        return [vx, vy]

    def distance_to_point(self, endp):
        return sqrt((self.position.x - endp[0]) ** 2 + (self.position.y - endp[1]) ** 2)

    def distance_from_path(self):
        x0 = self.position.x
        y0 = self.position.y
        current_path_segment = []
        x1 = current_path_segment[0][0]
        y1 = current_path_segment[0][1]
        x2 = current_path_segment[1][0]
        y2 = current_path_segment[1][1]

        distance = abs((y1 - y2) * x0 + (x2 - x1) * y0 + x1 * y2 - x2 * y1) / sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return distance

    def vector_mag(self, vec):
        return sqrt(vec[0] ** 2 + vec[1] ** 2)

    def overlap(self, l, c):
        x0 = c[0]
        y0 = c[1]
        r = c[2]
        x1 = l[0][0]
        y1 = l[0][1]
        x2 = l[1][0]
        y2 = l[1][1]
<<<<<<< HEAD
=======

>>>>>>> main
        min_x = min(x1, x2)
        max_x = max(x1, x2)
        min_y = min(y1, y2)
        max_y = max(y1, y2)
        if (x0 < min_x or x0 > max_x) or (y0 < min_y or y0 > max_y):
            return False

        distance = abs((y1 - y2) * x0 + (x2 - x1) * y0 + x1 * y2 - x2 * y1) / sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        if distance < r:
            return True

        return False

    def safe_division(self, n, d):
        return n / d if d else 0

    def get_sub_goal(self, l, o, margin):
        x0 = l[0][0]
        y0 = l[0][1]
        x1 = l[1][0]
        y1 = l[1][1]
        x3 = o[0]
        y3 = o[1]
        # line equation
        m = (y0 - y1) / (x0 - x1)
        b = -m * x0 + y0
        # perpendicular line with obstacle equation
        pm = -1 / m
        pb = -pm * x3 + y3
        # point of intersection
        xi = (pb - b) / (m - pm)
        yi = m * xi + b
        # print("line : {0}x+{1} \nperp: {2}x+{3}".format(m,b,pm,pb))
        # unit vector parallel to the perpendicular line 
        # vx = xi-x3
        # if vx == 0:
        vx = -xi
        # vy = y1-y3
        # if vy == 0:
        vy = pb - yi
<<<<<<< HEAD
        if vy == 0:
            vy = (xi + 10) * pm

        mag = sqrt(vx ** 2 + vy ** 2)
        vx /= mag
        vy /= mag

=======
        mag = sqrt(vx ** 2 + vy ** 2)
        vx /= mag
        vy /= mag
>>>>>>> main
        # print("point of intersection is {0} ".format([xi,yi]))

        xsg = xi + margin * vx
        ysg = yi + margin * vy

        return [xsg, ysg]

    def get_obstacles(self, l, env):
        for o in env:
            if o[0] == self.position.x and o[1] == self.position.y:
                continue
            if self.overlap(l, o):
<<<<<<< HEAD
                # print("get_obstacle : line ({0},{1}),({2},{3}) overlap with ({4},{5})".format(l[0][0],l[0][1],
                # l[1][0],l[1][1],o[0],o[1]))
=======
                print("get_obstacle : line ({0},{1}),({2},{3}) overlap with ({4},{5})".format(l[0][0], l[0][1], l[1][0],
                                                                                              l[1][1], o[0], o[1]))
>>>>>>> main
                return o
        else:
            return None

    def planner(self, s, g, env, i):
        line = [[s[0], s[1]], [g[0], g[1]]]
        obstacle = self.get_obstacles(line, env)
        if obstacle and i < 50:
<<<<<<< HEAD
            sub_goal = self.get_sub_goal(line, obstacle, 0.3)
=======
            sub_goal = self.get_sub_goal(line, obstacle, 0.5)
            # print("Create sub-goal at {0} to avoid {1}".format(sub_goal,obstacle))
>>>>>>> main
            # add_sub_goal(sub_goal)
            path_part1 = self.planner(s, sub_goal, env, i + 1)

            path_part2 = self.planner(sub_goal, g, env, i + 1)

            for p in path_part2:
                if p not in path_part1:
                    path_part1.append(p)

            return path_part1
<<<<<<< HEAD
        else:
            path = [s, g]
            return path
=======
        path = [s, g]
        return path
>>>>>>> main
