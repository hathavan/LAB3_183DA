import numpy as np
import matplotlib.pyplot as plt
import random as rand

L = 92
H = 61
obstacle_array = []

# each obstacle is lower left, length, height
rock = [(25, 25), 30, 25]
rock2 = [(85, 25), 8, 25]
obstacle_array.append(rock)
obstacle_array.append(rock2)

robot_radius = 10 # cm
initial = (20, 10, 0)

robot = initial
end = (70, 40, 0)

# velocity step size cm/s
v = 10

V = [initial]
E = []
path = []

# arbitrary step size velocity

# Part 2.2a : Making environment with grid, obstacles, and init + goal states
def segment(point1, point2, color):
    x = [point1[0], point2[0]]
    y = [point1[1], point2[1]]
    plt.plot(x, y, color = color)
    return 0
def rectangle(obs):
    point0 = obs[0]
    x = point0[0]
    y = point0[1]

    point1 = (x, y + obs[2])
    point2 = (x + obs[1], y)
    point3 = (x + obs[1], y + obs[2])
    segment(point0, point1, 'r')
    segment(point1, point3, 'r')
    segment(point2, point3, 'r')
    segment(point0, point2, 'r')
    return 0
def create_environment():
    plt.plot(initial[0], initial[1], 'o', color='c')
    plt.plot(end[0], end[1], 'o', color='m')
    plt.plot(robot[0], robot[1], 'o', color='g')
    for i in range(len(obstacle_array)):
        rectangle(obstacle_array[i])
    return 0
def show_robot():
    robot_circle = plt.Circle(robot, robot_radius, color='g', fill=False)
    ax = plt.gca()
    ax.add_artist(robot_circle)
    return 0

# Part 2.2b : We change create_environment to take in an array of lower left corner, length, and height
# (similar to an obstacle)
# We display a rectangle instead of a circle on the grid.

# Part 2.2c : Generate smooth trajectory, to update robot position.
# The control input is the velocity of the trajectory during this second.
def find_theta(point1, point2):
    if (point2[0] - point1[0]) != 0:
        deltay = (point2[1] - point1[1])
        deltax = (point2[0] - point1[0])
        slope= float(deltay) / float(deltax)
        theta = np.arctan(slope)
    else:
        if point2[1] > point1[1]:
            theta = np.pi / 2
        else:
            theta = -np.pi / 2
    return theta
def trajectory(start, target, display):
    theta = find_theta(start, target)
    old_theta = start[2]
    if theta < old_theta-np.deg2rad(75):
        new_point = (start[0] + v * np.cos(np.deg2rad(15)), start[1] + v*np.sin(np.deg2rad(15)), start[2] - np.deg2rad(75))
    else:
        if theta > old_theta + np.deg2rad(75):
            new_point = (start[0] - v * np.cos(np.deg2rad(15)), start[1] + v*np.sin(np.deg2rad(15)), start[2] + np.deg2rad(75))
        else:
            new_point = (start[0] + v * np.cos(theta), start[1] + v * np.sin(theta), start[2] + theta)

    if display is True: segment(start, new_point, 'k')
    return new_point

# Part 2.2d : see if the end point of a trajectory (circle) is in an obstacle (rectangle)
def collision(point, obstacle):
    testX = point[0]
    testY = point[1]
    origin = obstacle[0]
    rx = origin[0]
    ry = origin[1]
    l = obstacle[1]
    h = obstacle[2]

    if point[0] < rx: testX = rx
    else:
        if point[0] > (rx + l): testX = rx + l

    if point[1] < ry: testY = ry
    else:
        if point[1] > (ry + h): testY = ry + h

    distX = point[0] - testX
    distY = point[1] - testY
    distance = np.sqrt( distX **2 + distY **2)

    if distance <= robot_radius:
        return True

    return False
def circle_collision(point):
    for obs in obstacle_array:
        if (True and collision(point, obs)) is True:
            return True
    return False
def line_collision(point1, point2, point3, point4):
    x1 = point1[0]
    x2 = point2[0]
    x3 = point3[0]
    x4 = point4[0]
    y1 = point1[1]
    y2 = point2[1]
    y3 = point3[1]
    y4 = point4[1]
    bottom = ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1))
    if bottom == 0:
        a = np.inf
        b = np.inf
    else:
        a = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / bottom
        b = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / bottom

    if a >= 0 and a <= 1 and b >=0 and b <= 1:
        return True
    return False
def rectangle_collision(point1, point2, obs):
    origin = obs[0]
    x = origin[0]
    y = origin[1]
    left = line_collision(point1, point2, origin, (x, y + obs[2]))
    right = line_collision(point1, point2, (x + obs[1], y) , (x + obs[1], y+ obs[2]))
    top = line_collision(point1, point2, (x, y + obs[2]), (x + obs[1], y+ obs[2]))
    bottom = line_collision(point1, point2, origin, (x + obs[1], y))

    if (left or right or top or bottom) is True:
        return True
    else:
        return False
def valid_trajectory(start, end):
    bad_end = circle_collision(end)
    if bad_end is True:
        return False
    R = robot_radius
    theta = np.pi/2 - find_theta(start, end)
    x1 = start[0]
    y1 = start[1]
    x2 = end[0]
    y2 = end[1]

    start1 = (x1 + R * np.cos(theta), y1 - R * np.sin(theta))
    start2 = (x1 - R * np.cos(theta), y1 - R * np.sin(theta))
    end1 = (x2 + R * np.cos(theta), y2 - R * np.sin(theta))
    end2 = (x2 - R * np.cos(theta), y2 - R * np.sin(theta))

    for i in obstacle_array:
        origin2origin = rectangle_collision(start, end, i)
        bound1 = rectangle_collision(start1, end1, i)
        bound2 = rectangle_collision(start2, end2, i)
        detection = origin2origin or bound1 or bound2
        if detection is True:
            return False
    return True


# Part 2.2e : finding shortest distance point in V to target
def distance_to_pt(point1, point2):
    distX = point1[0]-point2[0]
    distY = point1[1]-point2[1]
    return  np.sqrt( distX **2 + distY **2)
def closest(point_list, target):
    current_min = np.inf
    current_pt = target

    for i in point_list:
        dist = distance_to_pt(i, target)
        if dist <= current_min:
            if valid_trajectory(target, i) is True:
                current_min = dist
                current_pt = i

    return current_pt

# Part 2.2f: Visualizing trajectories from initial to goal
def last_segment(point):
    theta_difference = end[2] - point[2]
    if theta_difference < np.deg2rad(75) or theta_difference > np.deg2rad(-75):
        E.append((point, end))
        path.append((point, end))
        segment(point, end, 'm')
        return 0
    else:
        if theta_difference > 75: #rotate 75, repeat
            last_segment((point[0], point[1], point[2] + np.deg2rad(75)))
        else:
            last_segment((point[0], point[1], point[2] - np.deg2rad(75)))

def graph_build():
    found = False
    #loop
    while True:
        xrand = rand.randint(0, L)
        yrand = rand.randint(0, H)
        pointrand = (xrand, yrand, 0)
        pointnearest = closest(V, pointrand)

        if pointnearest == pointrand:
            continue
        test = find_theta(pointnearest, pointrand)
        if (test < pointnearest[2] - np.deg2rad(75)) or (test > pointnearest[2] + np.deg2rad(75)):
            continue

        pointnew = trajectory(pointnearest, pointrand, False)

        if valid_trajectory(pointnearest, pointnew) is True:
            V.append(pointnew)
            E.append((pointnearest, pointnew))
            if distance_to_pt(pointnew, end) < 10:
                # E.append((pointnew, end))
                # path.append((pointnew, end))
                # found = True
                # print('yay!')
                # segment(pointnew, end, 'm')
                last_segment(pointnew)
                found = True
                print ('yay')
            segment(pointnearest, pointnew, 'k')
        if found is True:
            break
    return 0

# Part 2.2g: Constructing the definite path from initial to goal.
def path_construct():
    # find the edge where the second point is the first point of current
    i = 0
    edge = path[i]
    end_path = False

    for i in range(100):
        search_point = edge[0]
        for seg in E:
            if seg[1] == search_point:
                path.append(seg)
                i += 1
                edge = path[i]
                if seg[0] == initial:
                    end_path = True
        if end_path is True:
            break
    for edge in path:
        segment(edge[0], edge[1], 'm')
    return 0


graph_build()
path_construct()
create_environment()
path_stf = path[::-1]
for i in path_stf:
    t = i[0]
    print t[2]

plt.xlim(0, L)
plt.ylim(0, H)
plt.axis('equal')
plt.xlabel('X-distance (cm)')
plt.ylabel('Y-distance (cm)')
plt.show()
