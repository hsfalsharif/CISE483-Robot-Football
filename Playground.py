from ball import Ball
from Network import Network
from robot import Robot
from position import Position
from Packet import SPacket
from Commands import CMD
import glfw
import OpenGL.GL as gl

import imgui
from imgui.integrations.glfw import GlfwRenderer


class Playground:
    ball = Ball(4)
    net = Network()
    robotYellow = []
    robotBlue = []
    obstacles = []
    coachYellow = "null"
    coachBlue = "null"
    window = None
    impl = None
    width = 0
    height = 0

    def __init__(self):
        imgui.create_context()
        self.window = self.impl_glfw_init()
        self.impl = GlfwRenderer(self.window)

        for i in range(8):
            self.robotBlue.append(Robot(i, Position(0, 0)))

        for i in range(8):
            self.robotYellow.append(Robot(i, Position(0, 0)))

    def update(self):
        # receive new packet
        self.net.update()
        p = self.net.get_packet()
        # print(p.to_string())
        # update ball position
        if p.info_ball.confidence > 0.5:
            self.ball.x = p.info_ball.x
            self.ball.y = p.info_ball.y
            self.ball.z = p.info_ball.z

        # update Yellow team position and angle
        for i in p.info_robotY:
            if i.confidence > 0.5:
                self.robotYellow[i.id].position.x = i.x
                self.robotYellow[i.id].position.y = i.y
                self.robotYellow[i.id].orientation = i.orientation
                self.robotYellow[i.id].ball = self.ball

        # update Blue team position and angle
        for i in p.info_robotB:
            if i.confidence > 0.5:
                self.robotBlue[i.id].position.x = i.x
                self.robotBlue[i.id].position.y = i.y
                self.robotBlue[i.id].orientation = i.orientation
                self.robotBlue[i.id].ball = self.ball

        self.obstacles = []
        for i in self.robotYellow:
            self.obstacles.append([i.position.x, i.position.y, 0.0875 * 2])
        for i in self.robotBlue:
            self.obstacles.append([i.position.x, i.position.y, 0.0875 * 2])
        for r in self.robotYellow:
            r.obstacles = self.obstacles
        for r in self.robotBlue:
            r.obstacles = self.obstacles

        ####
        # test sending command to robots
        # new command

        # print(p.frame_number)
        cmd = CMD()
        # command description for debugging
        cmd.des = "point1"
        cmd.wheel_speed = False
        cmd.velocity(1, 0, 0)
        self.robotYellow[0].set_cmd(cmd)

        for i in self.robotYellow:
            i.update()

        for i in self.robotBlue:
            i.update()

        # we send each team commands to the simulator so the robots will move after this block
        # according to the protocol we send each team command packet alone
        next_command_y = SPacket(0, 1, self.robotYellow)
        # print(len(next_command_Y.robots_commands))
        # print("##############################")
        # print(next_command_Y.to_string())
        # print("##############################")
        self.net.send_packet(next_command_y)
        # print(next_command_Y.to_string())
        next_command_b = SPacket(0, 0, self.robotBlue)
        self.net.send_packet(next_command_b)

        self.render_gui()

    def render_gui(self):
        if glfw.window_should_close(self.window):
            self.impl.shutdown()
            glfw.terminate()

        glfw.poll_events()
        self.impl.process_inputs()
        imgui.new_frame()

        self.render_map()
        self.render_info()

        gl.glClearColor(1., 1., 1., 1)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        imgui.render()
        self.impl.render(imgui.get_draw_data())
        glfw.swap_buffers(self.window)

    def render_info(self):

        # imgui.set_next_window_position(field_widget_position[0],field_widget_position[1])
        # imgui.set_next_window_size(field_widget_size[0],field_widget_size[1])
        imgui.begin("Info Widget")
        imgui.text("Yellow Team")

        for i in self.robotYellow:
            if imgui.tree_node("Robot{0}".format(i.robotID)):
                imgui.text("position :({0},{1})".format(i.position.x, i.position.y))
                imgui.text("orientation :{0}".format(i.orientation))
                imgui.text("kickspeedx  :{0}".format(i.kickspeedx))
                imgui.text("kickspeedz  :{0}".format(i.kickspeedz))
                imgui.text("veltangent  :{0}".format(i.veltangent))
                imgui.text("velnormal  :{0}".format(i.velnormal))
                imgui.text("velangular  :{0}".format(i.velangular))
                imgui.text("spinner  :{0}".format(i.spinner))
                imgui.text("wheelsspeed :{0}".format(i.wheelsspeed))
                imgui.text("wheel1 :{0}".format(i.wheel1))
                imgui.text("wheel2 :{0}".format(i.wheel2))
                imgui.text("wheel3 :{0}".format(i.wheel3))
                imgui.text("wheel4 :{0}".format(i.wheel4))
                if imgui.tree_node("Planned path"):
                    for j in i.planned_path:
                        imgui.bullet_text("{0}".format(j))
                    imgui.tree_pop()
                imgui.text("max_velocity  :{0}".format(i.max_velocity))
                imgui.text("current_path_segment :{0}".format(i.current_path_segment))
                imgui.text("target_position  :({0},{1})".format(i.target_position.x, i.target_position.y))
                imgui.text("target_velocity :{0}".format(i.target_velocity))
                imgui.text("current_state  :{0}".format(i.current_state))
                imgui.text("next_state = :{0}".format(i.next_state))
                imgui.text("current_sub_state :{0}".format(i.current_sub_state))
                imgui.text("next_sub_state  :{0}".format(i.next_sub_state))
                imgui.text("global_velocity :{0}".format(i.global_velocity))
                imgui.tree_pop()

        imgui.text("Blue Team")

        for i in self.robotYellow:
            if imgui.tree_node("Robot{0} ".format(i.robotID)):
                imgui.text("position :({0},{1})".format(i.position.x, i.position.y))
                imgui.text("orientation :{0}".format(i.orientation))
                imgui.text("kickspeedx  :{0}".format(i.kickspeedx))
                imgui.text("kickspeedz  :{0}".format(i.kickspeedz))
                imgui.text("veltangent  :{0}".format(i.veltangent))
                imgui.text("velnormal  :{0}".format(i.velnormal))
                imgui.text("velangular  :{0}".format(i.velangular))
                imgui.text("spinner  :{0}".format(i.spinner))
                imgui.text("wheelsspeed :{0}".format(i.wheelsspeed))
                imgui.text("wheel1 :{0}".format(i.wheel1))
                imgui.text("wheel2 :{0}".format(i.wheel2))
                imgui.text("wheel3 :{0}".format(i.wheel3))
                imgui.text("wheel4 :{0}".format(i.wheel4))

                if imgui.tree_node("Planned path"):
                    for j in i.planned_path:
                        imgui.bullet_text("{0}".format(j))
                    imgui.tree_pop()

                if imgui.tree_node("obstacles"):
                    for j in i.obstacles:
                        imgui.bullet_text("{0}".format(j))
                    imgui.tree_pop()
                imgui.text("max_velocity  :{0}".format(i.max_velocity))
                imgui.text("current_path_segment :{0}".format(i.current_path_segment))
                imgui.text("target_position  :({0},{1})".format(i.target_position.x, i.target_position.y))
                imgui.text("target_velocity :{0}".format(i.target_velocity))
                imgui.text("current_state  :{0}".format(i.current_state))
                imgui.text("next_state = :{0}".format(i.next_state))
                imgui.text("current_sub_state :{0}".format(i.current_sub_state))
                imgui.text("next_sub_state  :{0}".format(i.next_sub_state))
                imgui.text("global_velocity :{0}".format(i.global_velocity))
                imgui.tree_pop()

        imgui.end()

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
        for i in self.robotYellow:
            x, y = yc + i.position.x * pixel_to_meter, xc + i.position.y * pixel_to_meter
            r = 0.0793 * pixel_to_meter
            draw_list.add_circle_filled(y, x, r, imgui.get_color_u32_rgba(1, 1, 0, 1))

            for l in range(len(i.planned_path) - 1):
                x0, y0 = yc + i.planned_path[l][0] * pixel_to_meter, xc + i.planned_path[l][1] * pixel_to_meter
                x1, y1 = yc + i.planned_path[l + 1][0] * pixel_to_meter, xc + i.planned_path[l + 1][1] * pixel_to_meter
                draw_list.add_line(y0, x0, y1, x1, imgui.get_color_u32_rgba(1, 1, 0, 1), 2)

            vx = x + i.global_velocity[0] * pixel_to_meter / 10
            vy = y + i.global_velocity[1] * pixel_to_meter / 10
            draw_list.add_line(y, x, vy, vx, imgui.get_color_u32_rgba(1, 1, 1, 1), 2)

        for i in self.robotBlue:
            x, y = yc + i.position.x * pixel_to_meter, xc + i.position.y * pixel_to_meter
            r = 0.0793 * pixel_to_meter
            draw_list.add_circle_filled(y, x, r, imgui.get_color_u32_rgba(0, 0, 1, 1))

            for l in range(len(i.planned_path) - 1):
                x0, y0 = yc + i.planned_path[l][0] * pixel_to_meter, xc + i.planned_path[l][1] * pixel_to_meter
                x1, y1 = yc + i.planned_path[l + 1][0] * pixel_to_meter, xc + i.planned_path[l + 1][1] * pixel_to_meter
                draw_list.add_line(y0, x0, y1, x1, imgui.get_color_u32_rgba(0, 0, 1, 1), 2)

            vx = x + i.global_velocity[0] * pixel_to_meter / 10
            vy = y + i.global_velocity[1] * pixel_to_meter / 10
            draw_list.add_line(y, x, vy, vx, imgui.get_color_u32_rgba(1, 1, 1, 1), 2)

        if imgui.is_mouse_hovering_rect(xo, yo, xo + imgui.get_window_width(), yo + imgui.get_window_height()):
            imgui.set_tooltip("({0},{1})".format((imgui.get_mouse_pos()[0] - xc) / pixel_to_meter,
                                                 (imgui.get_mouse_pos()[1] - yc) / pixel_to_meter))

        imgui.end()

    def gui_field_projection(self, start_x, start_y, x, y):
        return [start_y + 6 - y, start_x + 8 - x]

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
