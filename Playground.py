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
	ball = Ball(4, 4)
	net = Network()
	robotYellow = []
	robotBlue = []
	obsticals = []
	coachYellow = "null"
	coachBlue = "null"
	window =None
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
		print(p.to_string())
		# update ball position
		if p.info_ball.confidence > 0.5:
			self.ball.x = p.info_ball.x
			self.ball.y = p.info_ball.y
			self.ball.z = p.info_ball.z

		# update Yellow team position and angle
		self.obsticals = []
		for i in p.info_robotY:
			self.robotYellow[i.id].position.x = i.x
			self.robotYellow[i.id].position.y = i.y
			self.robotYellow[i.id].orientation = i.orientation
			self.robotYellow[i.id].ball = self.ball

			self.obsticals.append([i.x,i.y,0.0875])

		# update Blue team position and angle
		for i in p.info_robotB:
			self.robotBlue[i.id].position.x = i.x
			self.robotBlue[i.id].position.y = i.y
			self.robotBlue[i.id].orientation = i.orientation
			self.obsticals.append([i.x,i.y,0.1])
			self.robotBlue[i.id].ball = self.ball

		
		for r in self.robotYellow :
			r.obsticals = self.obsticals
		
		for r in self.robotBlue :
			r.obsticals = self.obsticals


		####
		# test sending command to robots
		# new command

		print(p.frame_number)
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

		if  glfw.window_should_close(self.window):
			self.impl.shutdown()
			glfw.terminate()
		glfw.poll_events()
		self.impl.process_inputs()

		imgui.new_frame()

		if imgui.begin_main_menu_bar():
			if imgui.begin_menu("File", True):

				clicked_quit, selected_quit = imgui.menu_item(
					"Quit", 'Cmd+Q', False, True
				)

				if clicked_quit:
					exit(1)

				imgui.end_menu()
			imgui.end_main_menu_bar()


		imgui.begin("Filled circle example",)
		draw_list = imgui.get_window_draw_list()
		for i in self.robotYellow:
			dx = 5.5 - i.position.x
			dy = 7 - i.position.y
			draw_list.add_circle_filled( dx*600/11.0,dy*800/14 ,4, imgui.get_color_u32_rgba(1,1,0,1))
			for l in range(len(i.planned_path)-1):
				x0 = 5.5 - i.planned_path[l][0]
				y0 = 7 - i.planned_path[l][1]
				x1 = 5.5 - i.planned_path[l+1][0]
				y1 = 7 - i.planned_path[l+1][1]
				print("path : {0} {1}".format([x0,y0],[x1,y1]))
				draw_list.add_line(x0*600/11.0, y0*800/14 , x1*600/11.0, y1*800/14 , imgui.get_color_u32_rgba(1,1,0,1), 2)
		
		for i in self.robotBlue:
			dx = 5.5 - i.position.x
			dy = 7 - i.position.y
			draw_list.add_circle_filled( dx*600/11.0,dy*800/14 ,4, imgui.get_color_u32_rgba(0,0,1,1))
			for l in range(len(i.planned_path)-1):
				x0 = 5.5 - i.planned_path[l][0]
				y0 = 7 - i.planned_path[l][1]
				x1 = 5.5 - i.planned_path[l+1][0]
				y1 = 7 - i.planned_path[l+1][1]
				print("path : {0} {1}".format([x0,y0],[x1,y1]))
				draw_list.add_line(x0*600/11.0, y0*800/14 , x1*600/11.0, y1*800/14 , imgui.get_color_u32_rgba(0,0,1,1), 1)
		imgui.end()




		gl.glClearColor(1., 1., 1., 1)
		gl.glClear(gl.GL_COLOR_BUFFER_BIT)

		imgui.render()
		self.impl.render(imgui.get_draw_data())
		glfw.swap_buffers(self.window)






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
