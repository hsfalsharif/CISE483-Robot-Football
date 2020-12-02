# -*- coding: utf-8 -*-
"""
Created on Wed Dec  2 03:18:43 2020

@author: oxygen
"""
from math import  sqrt
def overlap(l , c):
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


def get_sub_goal(l,o,margin):
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
    print("line : {0}x+{1} \nperp: {2}x+{3}".format(m,b,pm,pb))
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
    print("point of intersection is {0} ".format([xi,yi]))
    
    xsg = xi + margin * vx
    ysg = yi + margin * vy
    
    return [xsg,ysg]

def get_obsticals(l,env):
    for o in env:
        if overlap(l,o):
            print("get_obstical : line ({0},{1}),({2},{3}) overlap with ({4},{5})".format(l[0][0],l[0][1],l[1][0],l[1][1],o[0],o[1]))
            return o
    else: return None
    

def planner(s,g,env,i):
    path = []
    line = [[s[0],s[1]],[g[0],g[1]]]
    print(i)
    obstical = get_obsticals(line,env) 
    if obstical and i < 10 :
        sub_goal = get_sub_goal(line,obstical,3)
        print("Create sub-goal at {0} to avoid {1}".format(sub_goal,obstical))
       # add_sub_goal(sub_goal)
        path_part1 = planner(s,sub_goal,env,i+1)
        
        path_part2 = planner(sub_goal,g,env,i+1)
        
        for p in path_part2:
            if p not in path_part1:
                path_part1.append(p)
            
        return path_part1
    path = [s,g]
    return path
    
