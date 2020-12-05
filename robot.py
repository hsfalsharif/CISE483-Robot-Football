import glfw
import OpenGL.GL as gl

import imgui
from imgui.integrations.glfw import GlfwRenderer

from Commands import CMD
from Node import Node
from position import Position
from math import sqrt, cos, sin, copysign
import random
import numpy as np


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
    target_position = Position(0, 0)
    target_velocity = [0, 0]
    current_state = None
    current_sub_state = None
    next_state = None
    next_sub_state = None
    R0 = None
    R1 = None
    R2 = None
    global_velocity = [0, 0]
    # random.seed(42)
    window = None
    impl = None

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

    def update(self):
        # is robot on path ??
        self.target_position.x = self.ball.x
        self.target_position.y = self.ball.y
        if self.robotID == 2 and self.target_position.x is not None:
            self.move_to_RRT()  # move_to or move_to_RRT
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

    def move_to_RRT(self):  # start, goal and obstacles are all positions
        start = Position(self.position.x + self.global_velocity[0] * 0.018,
                         self.position.y + self.global_velocity[1] * 0.018)
        goal = Position(self.target_position.x, self.target_position.y)
        tree = [Node(start, neighbours=[])]
        node_path = self.RRT(tree, goal)
        path = []
        for node in node_path:
            path.append([node.position.x, node.position.y])
        self.planned_path = path
        first_segment = [path[0], path[1]]
        vx = first_segment[1][0] - first_segment[0][0]
        vy = first_segment[1][1] - first_segment[0][1]

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

    def RRT(self, tree, goal):  # how to check for obstacles here?
        u = random.random()
        random_position = Position(u * 12, u * 10)
        distance_to_goal = np.inf
        # building tree by traversing environment
        while distance_to_goal > 0.3:
            distance_to_goal = self.extend(tree, random_position, goal)
            print(distance_to_goal)
        # return path by traversing tree
        path = []
        last_node = tree[len(tree) - 1]  # indicates the final position in the path
        current_node = tree[0]
        while current_node != last_node:  # while the last node has not been found
            if current_node.visited is not True:  # if we have not visited this node
                path.append(current_node)  # add it to the path
            for i in range(len(current_node.neighbours)):  # otherwise, move on to its neighbours
                neighbour = tree[current_node.neighbours[i]]
                # if the neighbour has not been visited and it has no neighbours and it is not the goal,
                # then it is a dead end, so dont add it to the path and set it as visited
                if neighbour.visited is not True and len(neighbour.neighbours) == 0 and neighbour.position != goal:
                    neighbour.visited = True
                # else if it is not visited and it has no neighbours and it is the goal, add it to path and return path
                elif neighbour.visited is not True and len(neighbour.neighbours) == 0 and neighbour.position == goal:
                    path.append(neighbour)
                    return path
                # else if we have not visited it and it has neighbours, then assign it to current node
                elif neighbour.visited is not True and len(neighbour.neighbours) != 0:
                    current_node = neighbour  # do we need to set it to visited here?
                    break  # now test this current node
                # else if we are on the last neighbour and it is visited, then this node is a dead end so remove it from
                # the path and set it to visited, also set current node to last element in the path
                elif neighbour.visited and i == len(current_node.neighbours) - 1:
                    current_node.visited = True
                    path.pop()
                    current_node = path[len(path) - 1]
                elif neighbour.visited:
                    pass
        #  traverse all paths until the final node is reached
        #  if paths lead to dead ends, eliminate them
        #  keep the path that leads us to the goal

    def extend(self, tree, random_position, goal):  # do we need to account for obstacles here? if so, how?
        node_index = 0
        smallest_distance = np.inf
        nearest_node = Node(Position(np.inf, np.inf), neighbours=[])
        for i in range(len(tree)):
            distance = sqrt(
                (tree[i].position.x - random_position.x) ** 2 + (tree[i].position.y - random_position.y) ** 2)
            if distance < smallest_distance:
                smallest_distance = distance
                nearest_node = tree[i]
                node_index = i
        vector = [random_position.x - nearest_node.position.x, random_position.y - nearest_node.position.y]
        vector_mag = sqrt(vector[0] ** 2 + vector[1] ** 2)
        unit_vector = [vector[0] / vector_mag, vector[1] / vector_mag]
        new_node = Node(Position(nearest_node.position.x + 0.1 * unit_vector[0],
                                 nearest_node.position.y + 0.1 * unit_vector[1]),
                        neighbours=[])  # 0.3 is incrementation margin
        tree[node_index].neighbours.append(len(tree))  # adding new node to neighbours of the nearest node
        tree.append(new_node)  # adding new node to the tree
        self.planned_path.append([[nearest_node.position.x, nearest_node.position.y],
                                 [new_node.position.x, new_node.position.y]])
        distance_to_goal = sqrt((new_node.position.x - goal.x) ** 2 +
                                (new_node.position.y - goal.y) ** 2)
        # self.render_gui()
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
        # self.planned_path = path
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
        if vy == 0:
            vy = (xi + 10) * pm

        mag = sqrt(vx ** 2 + vy ** 2)
        vx /= mag
        vy /= mag

        # print("point of intersection is {0} ".format([xi,yi]))

        xsg = xi + margin * vx
        ysg = yi + margin * vy

        return [xsg, ysg]

    def get_obstacles(self, l, env):
        for o in env:
            if o[0] == self.position.x and o[1] == self.position.y:
                continue
            if self.overlap(l, o):
                # print("get_obstacle : line ({0},{1}),({2},{3}) overlap with ({4},{5})".format(l[0][0],l[0][1],
                # l[1][0],l[1][1],o[0],o[1]))
                return o
        else:
            return None

    def planner(self, s, g, env, i):
        line = [[s[0], s[1]], [g[0], g[1]]]
        obstacle = self.get_obstacles(line, env)
        if obstacle and i < 50:
            sub_goal = self.get_sub_goal(line, obstacle, 0.3)
            # add_sub_goal(sub_goal)
            path_part1 = self.planner(s, sub_goal, env, i + 1)

            path_part2 = self.planner(sub_goal, g, env, i + 1)

            for p in path_part2:
                if p not in path_part1:
                    path_part1.append(p)

            return path_part1
        else:
            path = [s, g]
            return path

    def render_gui(self):
        if glfw.window_should_close(self.window):
            self.impl.shutdown()
            glfw.terminate()

        glfw.poll_events()
        self.impl.process_inputs()
        imgui.new_frame()
        imgui.begin("string 5aleeha")
        imgui.text("ay kalam")
        imgui.end()
        # self.render_map()

        gl.glClearColor(1., 1., 1., 1)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        imgui.render()
        self.impl.render(imgui.get_draw_data())
        glfw.swap_buffers(self.window)

    def render_map(self):
        # draw the field
        length = 12
        width = 9
        radius = 0.5
        line_thickness = 0.01
        penalty_w = 2.4
        penalty_d = 1.2
        margin = 0.3
        referee_margin = 0.4
        goal_w = 1.2
        goal_d = 0.2
        widget_margin = 25
        pixel_to_meter = 50
        field_widget_size = [(width + (margin + referee_margin) * 2) * pixel_to_meter + widget_margin * 2,
                             (length + (margin + referee_margin) * 2) * pixel_to_meter + widget_margin * 2]
        imgui.set_next_window_size(field_widget_size[0], field_widget_size[1])

        imgui.begin("Field Widget")
        draw_list = imgui.get_window_draw_list()

        # draw walls
        lw = line_thickness * pixel_to_meter
        xo, yo = imgui.get_window_position()
        x0, y0 = xo + widget_margin, yo + widget_margin
        x1, y1 = x0 + ((margin + referee_margin) * 2 + width) * pixel_to_meter, y0
        x2, y2 = x0, y0 + ((margin + referee_margin) * 2 + length) * pixel_to_meter
        x3, y3 = x1, y2

        draw_list.add_line(x0, y0, x1, y1, imgui.get_color_u32_rgba(1, 0, 0, 1), lw)
        draw_list.add_line(x0, y0, x2, y2, imgui.get_color_u32_rgba(1, 0, 0, 1), lw)
        draw_list.add_line(x1, y1, x3, y3, imgui.get_color_u32_rgba(1, 0, 0, 1), lw)
        draw_list.add_line(x2, y2, x3, y3, imgui.get_color_u32_rgba(1, 0, 0, 1), lw)

        # draw field

        x4, y4 = x0 + pixel_to_meter * (referee_margin + margin), y0 + pixel_to_meter * (referee_margin + margin)
        x5, y5 = x1 - pixel_to_meter * (referee_margin + margin), y1 + pixel_to_meter * (referee_margin + margin)
        x6, y6 = x2 + pixel_to_meter * (referee_margin + margin), y2 - pixel_to_meter * (referee_margin + margin)
        x7, y7 = x3 - pixel_to_meter * (referee_margin + margin), y3 - pixel_to_meter * (referee_margin + margin)

        draw_list.add_line(x4, y4, x5, y5, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_line(x4, y4, x6, y6, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_line(x5, y5, x7, y7, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_line(x6, y6, x7, y7, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)

        # draw the circle and center line
        xc, yc = x4 + (x5 - x4) / 2, y4 + (y6 - y4) / 2
        r = radius * pixel_to_meter

        draw_list.add_line(xc, y4, xc, y6, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_line(x4, yc, x5, yc, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_circle(xc, yc, r, imgui.get_color_u32_rgba(1, 1, 1, 1), thickness=lw)

        # draw goal
        x8, y8 = xc - (penalty_w / 2) * pixel_to_meter, y4
        x9, y9 = xc + (penalty_w / 2) * pixel_to_meter, y4
        x10, y10 = x8, y4 + penalty_d * pixel_to_meter
        x11, y11 = x9, y10
        x12, y12 = x8, y6
        x13, y13 = x9, y6
        x14, y14 = x10, y6 - penalty_d * pixel_to_meter
        x15, y15 = x11, y14

        draw_list.add_line(x8, y8, x9, y9, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_line(x8, y8, x10, y10, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_line(x9, y9, x11, y11, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_line(x10, y10, x11, y11, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)

        draw_list.add_line(x12, y12, x13, y13, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_line(x12, y12, x14, y14, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_line(x13, y13, x15, y15, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_line(x14, y14, x15, y15, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)

        # draw goals

        x16, y16 = xc - (goal_w / 2) * pixel_to_meter, y4
        x17, y17 = xc + (goal_w / 2) * pixel_to_meter, y4
        x18, y18 = x16, y4 - goal_d * pixel_to_meter
        x19, y19 = x17, y18
        x20, y20 = x16, y6
        x21, y21 = x17, y6
        x22, y22 = x18, y6 + goal_d * pixel_to_meter
        x23, y23 = x19, y22

        draw_list.add_line(x16, y16, x17, y17, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_line(x16, y16, x18, y18, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_line(x17, y17, x19, y19, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_line(x18, y18, x19, y19, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)

        draw_list.add_line(x20, y20, x21, y21, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_line(x20, y20, x22, y22, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_line(x21, y21, x23, y23, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)
        draw_list.add_line(x22, y22, x23, y23, imgui.get_color_u32_rgba(1, 1, 1, 1), lw)

        # draw ball
        bx, by = yc + self.ball.x * pixel_to_meter, xc + self.ball.y * pixel_to_meter
        r = 0.0215000000 * pixel_to_meter
        draw_list.add_circle_filled(by, bx, r, imgui.get_color_u32_rgba(1, 0, 0, 1))

        # draw robots

        for l in self.planned_path:
            x0, y0 = yc + l[0][0] * pixel_to_meter, xc + l[0][1] * pixel_to_meter
            x1, y1 = yc + l[1][0] * pixel_to_meter, xc + l[1][1] * pixel_to_meter
            draw_list.add_line(y0, x0, y1, x1, imgui.get_color_u32_rgba(1, 1, 0, 1), 2)

        if imgui.is_mouse_hovering_rect(xo, yo, xo + imgui.get_window_width(), yo + imgui.get_window_height()):
            imgui.set_tooltip("({0},{1})".format((imgui.get_mouse_pos()[0] - xc) / pixel_to_meter,
                                                 (imgui.get_mouse_pos()[1] - yc) / pixel_to_meter))

        imgui.end()

    def impl_glfw_init(self):
        self.width, self.height = 440, 840
        window_name = "minimal ImGui/GLFW3 example"

        if not glfw.init():
            print("Could not initialize OpenGL context")
            exit(1)

        # OS X supports only forward-compatible core profiles from 3.2
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)

        glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, gl.GL_TRUE)

        # Create a windowed mode window and its OpenGL context
        window = glfw.create_window(
            int(self.width), int(self.height), window_name, None, None
        )
        glfw.make_context_current(window)

        if not window:
            glfw.terminate()
            print("Could not initialize Window")
            exit(1)

        return window
