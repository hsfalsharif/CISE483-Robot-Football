from Commands import CMD
from position import Position
from math import  sqrt,cos,sin

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
    ball = Position(0,0)
    commands = []
    obsticals = []
    planned_path = []
    current_path_segment = []
    target_postion = Position(0, 0)
    target_velocity = [0.5,0]
    current_state = None
    next_state = None
    next_sub_state = None
    R0 = None
    R1 = None
    R2 = None


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
        # is robot on path ??
        self.target_postion.x = self.ball.x
        self.target_postion.y = self.ball.y
        if self.robotID == 2:
            self.move_to()
        # re-adjust velocities
        #self.do_command(self.commands)

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


    def move_to(self):
        origin = [self.position.x,self.position.y]
        goal = [self.target_postion.x,self.target_postion.y]
        path = self.planner(origin,goal,self.obsticals,0)
        first_segment = [path[0],path[1]]
        vx = first_segment[1][0] - first_segment[0][0]
        vy = first_segment[1][1] - first_segment[0][1]
        lv = self.world_to_local([vx,vy])
        final_v = self.world_to_local(self.target_velocity)
        magnitude = self.vector_mag(lv)
        lvxu = lv[0]/magnitude
        lvyu = lv[1]/magnitude
        d = self.distance_to_point(goal)
        print("path :{0}\nWorld Velocity:{1} Local Velocity:{2}".format(first_segment,[vx,vy],[lvxu * d,lvyu * d]))

        self.veltangent = final_v[0] + lvxu * d
        self.velnormal = final_v[0]  + lvyu * d


    def local_to_world(self,vec):
        o = self.orientation
        vx = vec[0]*cos(o) - vec[1]*sin(o)
        vy = vec[0]*sin(o) + vec[1]*cos(o)
        return [vx,vy]
    
    def world_to_local(self,vec):
        o = self.orientation
        vx = vec[0]*cos(o) + vec[1]*sin(o)
        vy = -vec[0]*sin(o) + vec[1]*cos(o)
        return [vx,vy]

    def distance_to_point(self,endp):
        return sqrt((self.position.x-endp[0])**2+(self.position.y - endp[1])**2)

    def distance_from_path(self):
        x0 = self.position.x
        y0 = self.position.y
        current_path_segment = []
        x1 = current_path_segment[0][0]
        y1 = current_path_segment[0][1]
        x2 = current_path_segment[1][0]
        y2 = current_path_segment[1][1]

        distance = abs((y1-y2)*x0+(x2-x1)*y0+x1*y2-x2*y1)/sqrt((x2-x1)**2+(y2-y1)**2)
        return distance

    def vector_mag(self,vec):
        return sqrt(vec[0]**2+vec[1]**2)

    def overlap(self,l , c):
        x0 = c[0]
        y0 = c[1]
        r  = c[2]
        x1 = l[0][0]
        y1 = l[0][1]
        x2 = l[1][0]
        y2 = l[1][1]
        
        min_x = min(x1,x2)
        max_x = max(x1,x2)
        min_y = min(y1,y2)
        max_y = max(y1,y2)
        if (x0 < min_x or x0 > max_x) or (y0 < min_y or y0 > max_y) :
            return False
        
        
        distance = abs((y1-y2)*x0+(x2-x1)*y0+x1*y2-x2*y1)/sqrt((x2-x1)**2+(y2-y1)**2)
        if distance < r :
            return True
        
        
        return False


    def get_sub_goal(self,l,o,margin):
        x0 = l[0][0]
        y0 = l[0][1]
        x1 = l[1][0]
        y1 = l[1][1]
        x3 = o[0]
        y3 = o[1]
        # line equation
        m = (y0-y1)/(x0-x1)
        b = -m*x0 + y0
        # perpendicular line with obstical equation
        pm = -1/m
        pb = -pm*x3 + y3
        # point of intersection
        xi = (pb-b)/(m-pm)
        yi = m*xi + b
        #print("line : {0}x+{1} \nperp: {2}x+{3}".format(m,b,pm,pb))
        # unit vector parallel to the perpendicular line 
        #vx = xi-x3
        #if vx == 0:
        vx = -xi
        #vy = y1-y3
        #if vy == 0:
        vy = pb - yi
        mag = sqrt(vx**2 + vy**2)
        vx /= mag
        vy /= mag
        #print("point of intersection is {0} ".format([xi,yi]))
        
        xsg = xi + margin * vx
        ysg = yi + margin * vy
        
        return [xsg,ysg]

    def get_obsticals(self,l,env):
        for o in env:
            if o[0] == self.position.x and o[1] == self.position.y:
                continue
            if self.overlap(l,o):
                print("get_obstical : line ({0},{1}),({2},{3}) overlap with ({4},{5})".format(l[0][0],l[0][1],l[1][0],l[1][1],o[0],o[1]))
                return o
        else: return None
        

    def planner(self,s,g,env,i):
        path = []
        line = [[s[0],s[1]],[g[0],g[1]]]
        obstical = self.get_obsticals(line,env) 
        if obstical and i < 10 :
            sub_goal = self.get_sub_goal(line,obstical,3)
            #print("Create sub-goal at {0} to avoid {1}".format(sub_goal,obstical))
        # add_sub_goal(sub_goal)
            path_part1 = self.planner(s,sub_goal,env,i+1)
            
            path_part2 =self.planner(sub_goal,g,env,i+1)
            
            for p in path_part2:
                if p not in path_part1:
                    path_part1.append(p)
                
            return path_part1
        path = [s,g]
        return path
        
