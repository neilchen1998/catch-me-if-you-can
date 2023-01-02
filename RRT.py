import numpy as np
import math
from numpy import loadtxt
import matplotlib.pyplot as plt
import time
import random
import sys
plt.ion()

# functions to time how long planning takes  
def tic():
  return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))

class CatchingGame:

    def __init__(self, map) -> None:

        if map == '0':
            robotstart = np.array([0, 2])
            targetstart = np.array([5, 3])
            self.mapfile = r'C:\Users\neilp\Documents\VSCode\ece276b\Project2\maps\map0.txt'
            self.R = 1.4
            self.beta = 20

        elif map == '1':
            robotstart = np.array([699, 799])
            targetstart = np.array([699, 1699])
            self.mapfile = r'C:\Users\neilp\Documents\VSCode\ece276b\Project2\maps\map1.txt'
            self.R = 15
            self.beta = 20

        elif map == '2':
            robotstart = np.array([0, 2])
            targetstart = np.array([7, 9])
            self.mapfile = r'C:\Users\neilp\Documents\VSCode\ece276b\Project2\maps\map2.txt'
            self.R = 1.1
            self.beta = 20

        elif map == '3':
            robotstart = np.array([249, 249])
            targetstart = np.array([399, 399])
            self.mapfile = r'C:\Users\neilp\Documents\VSCode\ece276b\Project2\maps\map3.txt'
            self.R = 20
            self.beta = 20

        elif map == '4':
            robotstart = np.array([0, 0])
            targetstart = np.array([5, 6])
            self.mapfile = r'C:\Users\neilp\Documents\VSCode\ece276b\Project2\maps\map4.txt'
            self.R = 2
            self.beta = 20

        elif map == '5':
            robotstart = np.array([0, 0])
            targetstart = np.array([29, 59])
            self.mapfile = r'C:\Users\neilp\Documents\VSCode\ece276b\Project2\maps\map5.txt'
            self.R = 2
            self.beta = 20

        elif map == '6':
            robotstart = np.array([0, 0])
            targetstart = np.array([29, 36])
            self.mapfile = r'C:\Users\neilp\Documents\VSCode\ece276b\Project2\maps\map6.txt'
            self.R = 5
            self.beta = 20

        elif map == '7':
            robotstart = np.array([0, 0])
            targetstart = np.array([4998, 4998])
            self.mapfile = r'C:\Users\neilp\Documents\VSCode\ece276b\Project2\maps\map7.txt'
            self.R = 300
            self.beta = 10

        elif map == '1b':
            robotstart = np.array([249, 1199])
            targetstart = np.array([1649, 1899])
            self.mapfile = r'C:\Users\neilp\Documents\VSCode\ece276b\Project2\maps\map1.txt'
            self.R = 15
            self.beta = 20

        elif map == '3b':
            robotstart = np.array([74, 249])
            targetstart = np.array([399, 399])
            self.mapfile = r'C:\Users\neilp\Documents\VSCode\ece276b\Project2\maps\map3.txt'
            self.R = 20
            self.beta = 20

        elif map == '3c':
            robotstart = np.array([4, 399])
            targetstart = np.array([399, 399])
            self.mapfile = r'C:\Users\neilp\Documents\VSCode\ece276b\Project2\maps\map3.txt'
            self.R = 20
            self.beta = 20

        self.envmap = loadtxt(self.mapfile)
        self.robotpos = robotstart
        self.targetpos = targetstart
        self.numofmoves = 0
        self.caught = False

        # RRT
        (x, y) = robotstart[0], robotstart[1]
        self.start = robotstart
        self.goal = targetstart
        self.goalFlag = False   # whether the robot has reached to the goal position or not
        
        # the implementation of nodes
        self.Maph, self.Mapw = self.envmap.shape   # the height and the width of the map
        self.x = [] # stores all the nodes' x coordinates
        self.y = [] # stores all the nodes' y coordinates
        self.parents = []    # stores all the nodes' parents

        # set up the path parameters
        self.goalState = None
        self.path = []  # stores the path from the start to goal
        
        # initialize the tree
        self.x.append(x)
        self.y.append(y)
        self.parents.append(0)

        self.path_commands = []
        self.temporalGoal = targetstart

    # def RobotPlannerGreedy(self):
    #     # all possible directions of the robot
    #     numofdirs = 8
    #     dX = [-1, -1, -1, 0, 0, 1, 1, 1]
    #     dY = [-1,  0,  1, -1, 1, -1, 0, 1]
        
    #     # use the old position if we fail to find an acceptable move
    #     newrobotpos = np.copy(self.robotpos)
        
    #     # for now greedily move towards the target 
    #     # but this is the gateway function for your planner 
    #     mindisttotarget = 1000000
    #     for dd in range(numofdirs):
    #         newx = self.robotpos[0] + dX[dd]
    #         newy = self.robotpos[1] + dY[dd]
        
    #         if (newx >= 0 and newx < self.envmap.shape[0] and newy >= 0 and newy < self.envmap.shape[1]):
    #             if(self.envmap[newx, newy] == 0):
    #                 disttotarget = math.sqrt((newx-self.targetpos[0])**2 + (newy-self.targetpos[1])**2)
    #                 if(disttotarget < mindisttotarget):
    #                     mindisttotarget = disttotarget
    #                     newrobotpos[0] = newx
    #                     newrobotpos[1] = newy
        
    #     # # print the new position
    #     # print("Robot's position: {}\t".format(newrobotpos), end="")

    #     return newrobotpos

    
    def add_node(self, n, x, y):
        '''
        Add a new node to the list
        '''
        self.x.insert(n, x) # insert(idex, element)
        self.y.append(y)

    def remove_node(self, n):
        '''
        Remove a node on the list
        '''
        self.x.pop(n)   # pop(index)
        self.y.pop(n)

    def add_edge(self, parent, child):
        '''
        Add a new edge to the parent list
        '''
        self.parents.insert(child, parent)    # insert(parent, child)

    def remove_edge(self, child):
        '''
        Remove an edge on the list
        '''
        self.parents.pop(child)

    def number_of_nodes(self):
        '''
        Return the number of the nodes
        '''
        return len(self.x)

    def distance(self, idx1, idx2):
        '''
        Calculate the distance (Euclidean distance) between two node
        '''
        a = (self.x[idx1], self.y[idx1])    # the coordinates of index1
        b = (self.x[idx2], self.y[idx2])    # the coordinates of index2

        return math.dist(a, b)

    def sample_envir(self):
        '''
        Randomly generate a new node on the map
        '''
        x = int(random.uniform(0, self.envmap.shape[0]))
        y = int(random.uniform(0, self.envmap.shape[1]))
        return x, y

    def nearest(self, n):
        '''
        Calculate the nearest node
        '''
        minD = math.inf
        ret = -1

        # loop through the all nodes
        for i in range(n):

            # if tempD is smaller than the current, then update minD
            if self.distance(i, n) < minD:
                minD = self.distance(i, n)
                ret = i
                
        return ret

    def isFree(self):
        '''
        Check if the newly added node collides with any obstacle
        '''
        idx_new = self.number_of_nodes() - 1    # the newly added node is located at the end of the list
        (x_new, y_new) = (self.x[idx_new], self.y[idx_new]) # grab the newly added node's coordinates

        if x_new < 0 or x_new >= self.envmap.shape[0] or y_new < 0 or y_new >= self.envmap.shape[1]:
            self.remove_node(idx_new)
            return False

        elif self.envmap[x_new, y_new] != 0:
            self.remove_node(idx_new)
            return False

        # return true if it passes all the obstacles
        return True

    def crossObstacle(self, x1, x2, y1, y2):
        '''
        Check if the path collides with any obstacles along the path
        '''
        if x2 < 0 or y2 < 0 or x2 >= self.envmap.shape[0] or y2 >= self.envmap.shape[1]:
            return True
        elif self.envmap[x2, y2] != 0:
            return True

        return False

    def connect(self, parent, child):
        '''
        Connect two nodes
        '''
        (x1, y1) = (self.x[parent], self.y[parent])
        (x2, y2) = (self.x[child], self.y[child])
        if self.crossObstacle(x1, x2, y1, y2):
            self.remove_node(child)
            return False
        else:
            self.add_edge(parent, child)
            return True

    def step(self, idx_old, idx_new, maxR=1.4, goalD = 1.4):
        '''
        Step function
        '''
        # check if the distance is within the radius
        if self.distance(idx_old, idx_new) > maxR:

            # grab the coordinates of the old node and the new node
            (x_old, y_old) = self.x[idx_old], self.y[idx_old]
            (x_new, y_new) = self.x[idx_new], self.y[idx_new]

            # # calculate the theta
            # theta = math.atan2((y_new - y_old), (x_new - x_old))    # the signs of x and y are considered
            
            # # calculate the furthest the robot can travel (lookahead distance)
            # x_rim = x_old + maxR * math.cos(theta)
            # y_rim = y_old + maxR * math.sin(theta)

            center = (x_old, y_old)
            candidates = np.array([[center[0] + 1, center[1]], \
                [center[0] + 1, center[1] + 1], \
                [center[0] , center[1] + 1], \
                [center[0] - 1, center[1] + 1], \
                [center[0] - 1, center[1]], \
                [center[0] - 1, center[1] - 1], \
                [center[0], center[1] - 1], \
                [center[0] + 1, center[1] - 1], \
                    ])
            minVal = math.inf
            for candidate in candidates:
                temp = math.dist(candidate, (x_new, y_new))
                if temp < minVal:
                    minVal = temp
                    (x_rim, y_rim) = candidate

            # remove the new node
            self.remove_node(idx_new)

            # check if it reaches the goal
            if (abs(x_rim - self.goal[0]) + abs(y_rim - self.goal[1])) < goalD:
                self.add_node(idx_new, x_rim, y_rim)
                self.add_node(idx_new + 1, self.goal[0], self.goal[1])
                self.goalState = idx_new
                self.goalFlag = True
            else:
                self.add_node(idx_new, x_rim, y_rim)

    def step2(self, idx_old, idx_new, minR=1.4, goalR = 1.4):
        '''
        Step function version 2
        '''
        # check if the distance is within the radius
        if self.distance(idx_old, idx_new) > minR:

            # grab the coordinates of the old node and the new node
            (oldX, oldY) = self.x[idx_old], self.y[idx_old]
            (newX, newY) = self.x[idx_new], self.y[idx_new]

            # remove the new node
            self.remove_node(idx_new)

            # add 
            candidatesX = []
            candidatesY = []
            curX = oldX
            curY = oldY
            itr = 0

            if random.randint(0, 1):

                # move along the x axis
                if curX < newX:
                    while curX != newX and itr < self.R:
                        candidatesX.append(curX + 1)
                        candidatesY.append(curY)
                        curX = curX + 1
                        itr = itr + 1
                else:
                    while curX != newX and itr < self.R:
                        candidatesX.append(curX - 1)
                        candidatesY.append(curY)
                        curX = curX - 1
                        itr = itr + 1
                
                # move along the y axis
                if curY < newY:
                    while curY != newY and itr < self.R:
                        candidatesX.append(curX)
                        candidatesY.append(curY + 1)
                        curY = curY + 1
                        itr = itr + 1
                else:
                    while curY != newY and itr < self.R:
                        candidatesX.append(curX)
                        candidatesY.append(curY - 1)
                        curY = curY - 1
                        itr = itr + 1
            else:
                
                # move along the y axis
                if curY < newY:
                    while curY != newY and itr < self.R:
                        candidatesX.append(curX)
                        candidatesY.append(curY + 1)
                        curY = curY + 1
                        itr = itr + 1
                else:
                    while curY != newY and itr < self.R:
                        candidatesX.append(curX)
                        candidatesY.append(curY - 1)
                        curY = curY - 1
                        itr = itr + 1

                # move along the x axis
                if curX < newX:
                    while curX != newX and itr < self.R:
                        candidatesX.append(curX + 1)
                        candidatesY.append(curY)
                        curX = curX + 1
                        itr = itr + 1
                else:
                    while curX != newX and itr < self.R:
                        candidatesX.append(curX - 1)
                        candidatesY.append(curY)
                        curX = curX - 1
                        itr = itr + 1

            # make sure there is no obstacles along the path
            idx = 0
            curX = oldX
            curY = oldY
            for _ in range(len(candidatesX) - 1):
                if self.crossObstacle(curX, candidatesX[idx], curY, candidatesY[idx]):
                    break
                else:
                    self.add_node(idx_new, candidatesX[idx], candidatesY[idx])
                    self.connect(idx_old, idx_new)
                    curX = candidatesX[idx]
                    curY = candidatesY[idx]
                    idx_old = idx_new
                    idx_new = idx_new + 1
                    idx = idx + 1

            # check if it reaches the goal
            if (abs(curX - self.goal[0]) + abs(curY - self.goal[1])) < goalR and\
                not self.crossObstacle(curX, self.goal[0], curY, self.goal[1]):
                self.add_node(idx_new, curX, curY)
                self.connect(idx_old, idx_new)
                idx_old = idx_new
                idx_new = idx_new + 1
                self.add_node(idx_new, self.goal[0], self.goal[1])
                self.connect(idx_old, idx_new)
                self.goalState = idx_new
                self.goalFlag = True
                if len(self.x) != len(self.parents):
                        breakpoint()
                        print("here")
        else:
            self.remove_node(idx_new)
            if len(self.x) != len(self.parents):
                breakpoint()
                print("here")

    def bias(self, ngoal):
        '''
        Steer toward the goal
        '''
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)

        return self.x, self.y, self.parents

    def bias2(self, ngoal):
        '''
        Steer toward the goal
        '''
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step2(nnear, n)

        return self.x, self.y, self.parents

    def expand(self):
        '''
        Randomly explore the map
        '''
        n = self.number_of_nodes()
        x, y = self.sample_envir()
        self.add_node(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.x, self.y, self.parents

    def expand2(self):
        '''
        Randomly explore the map
        '''
        n = self.number_of_nodes()
        x, y = self.sample_envir()
        self.add_node(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step2(xnearest, n)
            if len(self.x) != len(self.parents):
                breakpoint()
                print("here")
        return self.x, self.y, self.parents

    def path_to_goal(self):
        '''
        Check if we have founded a path to the goal
        '''
        # only if we have reached the goal
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalState)

            new_post = self.parents[self.goalState]

            # keep going backwards until we reach the start point
            while new_post != 0:
                self.path.append(new_post)
                new_post = self.parents[new_post]

            return self.goalFlag

        else:
            return self.goalFlag

    def get_path_coords(self):
        '''
        Get the coordinates of each node along the path
        '''
        # create a list of coordinates
        path_coordinates_reverse = []
        path_coordinates = []

        # loop through the entire path and extract the coordinates
        for node in self.path:
            (x, y) = self.x[node], self.y[node]
            path_coordinates_reverse.append((x, y))

        self.temporalGoal = path_coordinates_reverse[0]

        for i in range(len(path_coordinates_reverse) - 1, -1, -1):
            path_coordinates.append(path_coordinates_reverse[i])

        return path_coordinates

    def RobotPlannerRRT(self):
        
        # use the old position if we fail to find an acceptable move
        newrobotpos = np.copy(self.robotpos)
        
        if self.path_commands:
            new_command = self.path_commands.pop(0)
            newrobotpos[0] = new_command[0]
            newrobotpos[1] = new_command[1]

        elif self.goalFlag == False:
            iteration = 0
            while not self.path_to_goal():
                if iteration % 20 == 0:
                    _, _, _ = self.bias(self.targetpos)
                else:
                    _, _, _ = self.expand()

                # increment the iteration by one
                iteration = iteration + 1

            print("Number of iterations: {}".format(iteration))
            self.path_commands = self.get_path_coords()
            new_command = self.path_commands.pop(0)
            newrobotpos[0] = new_command[0]
            newrobotpos[1] = new_command[1]
        else:
            # greedy algorithm
            
            # all possible directions of the robot
            numofdirs = 8
            dX = [-1, -1, -1, 0, 0, 1, 1, 1]
            dY = [-1,  0,  1, -1, 1, -1, 0, 1]
            
            # use the old position if we fail to find an acceptable move
            newrobotpos = np.copy(self.robotpos)
            
            # for now greedily move towards the target 
            # but this is the gateway function for your planner 
            mindisttotarget = 1000000
            for dd in range(numofdirs):
                newx = self.robotpos[0] + dX[dd]
                newy = self.robotpos[1] + dY[dd]
            
                if (newx >= 0 and newx < self.envmap.shape[0] and newy >= 0 and newy < self.envmap.shape[1]):
                    if(self.envmap[newx, newy] == 0):
                        disttotarget = math.sqrt((newx-self.targetpos[0])**2 + (newy-self.targetpos[1])**2)
                        if(disttotarget < mindisttotarget):
                            mindisttotarget = disttotarget
                            newrobotpos[0] = newx
                            newrobotpos[1] = newy

        return newrobotpos

    def RobotPlannerRRT2(self):
        
        # use the old position if we fail to find an acceptable move
        newrobotpos = np.copy(self.robotpos)
        
        if self.path_commands:
            new_command = self.path_commands.pop(0)
            newrobotpos[0] = new_command[0]
            newrobotpos[1] = new_command[1]

        elif self.goalFlag == False:
            iteration = 0
            while not self.path_to_goal():
                # if iteration > 10000:
                #     breakpoint()
                #     print("takes too long!")
                if iteration % self.beta == 0:
                    self.bias2(self.targetpos)
                else:
                    self.expand2()

                # increment the iteration by one
                iteration = iteration + 1
            print("Number of iterations: {}".format(iteration))
            self.path_commands = self.get_path_coords()
            new_command = self.path_commands.pop(0)
            newrobotpos[0] = new_command[0]
            newrobotpos[1] = new_command[1]
        else:
            # greedy algorithm

            # all possible directions of the robot
            numofdirs = 8
            dX = [-1, -1, -1, 0, 0, 1, 1, 1]
            dY = [-1,  0,  1, -1, 1, -1, 0, 1]
            
            # use the old position if we fail to find an acceptable move
            newrobotpos = np.copy(self.robotpos)
            
            # for now greedily move towards the target 
            # but this is the gateway function for your planner 
            mindisttotarget = 1000000
            for dd in range(numofdirs):
                newx = self.robotpos[0] + dX[dd]
                newy = self.robotpos[1] + dY[dd]
            
                if (newx >= 0 and newx < self.envmap.shape[0] and newy >= 0 and newy < self.envmap.shape[1]):
                    if(self.envmap[newx, newy] == 0):
                        disttotarget = math.sqrt((newx-self.targetpos[0])**2 + (newy-self.targetpos[1])**2)
                        if(disttotarget < mindisttotarget):
                            mindisttotarget = disttotarget
                            newrobotpos[0] = newx
                            newrobotpos[1] = newy

        return newrobotpos

    def targetplanner(self, movetime):
        # all possible directions of the target
        numofdirs = 4
        dX = [-1, 0, 0, 1]
        dY = [ 0, -1, 1, 0]

        # all possible directions of the robot
        numofrobodirs = 8
        robot_dX = [-1, -1, -1, 0, 0, 1, 1, 1]
        robot_dY = [-1, 0, 1, -1, 1, -1, 0, 1]

        newtargetpos = np.copy(self.targetpos)
        
        for _ in range(movetime):
            end_pos_list = []
            start_pos = newtargetpos

            # all possible actions and next positions of the target
            for dd in range(numofdirs):
                newx = start_pos[0] + dX[dd]
                newy = start_pos[1] + dY[dd]
            
                if (newx >= 0 and newx < self.envmap.shape[0] and newy >= 0 and newy < self.envmap.shape[1]):
                    if(self.envmap[newx, newy] == 0):
                        end_pos_list.append(np.array([newx, newy]))
            
            min_dist_arr = []
            for end_pos in end_pos_list:
                min_dist = np.linalg.norm(end_pos - self.robotpos)

                # for each target position, compute the minimal distance from robot
                for robo_dd in range(numofrobodirs):
                    robot_newx = self.robotpos[0] + robot_dX[robo_dd]
                    robot_newy = self.robotpos[1] + robot_dY[robo_dd]
                    if (robot_newx >= 0 and robot_newx < self.envmap.shape[0] and robot_newy >= 0 and robot_newy < self.envmap.shape[1]):
                        if(self.envmap[robot_newx, robot_newy] == 0):
                            dist = np.linalg.norm(end_pos - np.array([robot_newx, robot_newy]))
                            if dist < min_dist:
                                min_dist = dist
                min_dist_arr.append(min_dist)
            
            # choose the action that achieves the maximal minimal distance
            min_dist_arr = np.array(min_dist_arr)
            minimax_idx = np.argmax(min_dist_arr)
            newtargetpos = end_pos_list[minimax_idx]

            # # print the new position
            # print("Target's position: {}".format(newtargetpos))
        
        return newtargetpos

    def runtest(self):
            
        # # draw the environment
        # # transpose because imshow places the first dimension on the y-axis
        # f, ax = plt.subplots()
        # ax.imshow(self.envmap.T, interpolation="none", cmap='gray_r', origin='lower', \
        #             extent=(-0.5, self.envmap.shape[0]-0.5, -0.5, self.envmap.shape[1]-0.5) )
        # ax.axis([-0.5, self.envmap.shape[0]-0.5, -0.5, self.envmap.shape[1]-0.5])
        # ax.set_xlabel('x')
        # ax.set_ylabel('y')  
        # hr = ax.plot(self.robotpos[0], self.robotpos[1], 'bs')
        # ht = ax.plot(self.targetpos[0],self.targetpos[1], 'rs')
        # f.canvas.flush_events()
        # plt.show()

        # now comes the main loop
        
        for _ in range(20000):

            # # print the number of moves
            # print("Number of moves: {}".format(numofmoves + 1))

            # call robot planner
            t0 = tic()
            newrobotpos = self.RobotPlannerRRT()
            # compute move time for the target, if it is greater than 2 sec, the target will move multiple steps
            movetime = max(1, math.ceil((tic()-t0)/2.0))
            
            # check that the new commanded position is valid
            if (newrobotpos[0] < 0 or newrobotpos[0] >= self.envmap.shape[0] or \
                newrobotpos[1] < 0 or newrobotpos[1] >= self.envmap.shape[1] ):
                print('ERROR: out-of-map robot position commanded\n')
                break
            elif (self.envmap[newrobotpos[0], newrobotpos[1]] != 0 ):
                print('ERROR: invalid robot position commanded\n')
                break
            elif (abs(newrobotpos[0]-self.robotpos[0]) > 1 or abs(newrobotpos[1]-self.robotpos[1]) > 1):
                print('ERROR: invalid robot move commanded\n')
                break

            # call target planner to see how the target moves within the robot planning time
            newtargetpos = self.targetplanner(movetime)
            
            # make the moves
            self.robotpos = newrobotpos
            self.targetpos = newtargetpos
            self.numofmoves += 1
            
            # # draw positions
            # hr[0].set_xdata(self.robotpos[0])
            # hr[0].set_ydata(self.robotpos[1])
            # ht[0].set_xdata(self.targetpos[0])
            # ht[0].set_ydata(self.targetpos[1])
            # f.canvas.flush_events()
            # plt.show()

            # check if target is caught
            if (abs(self.robotpos[0]-self.targetpos[0]) <= 1 and abs(self.robotpos[1]-self.targetpos[1]) <= 1):
                print('robotpos = (%d,%d)' %(self.robotpos[0], self.robotpos[1]))
                print('targetpos = (%d,%d)' %(self.targetpos[0], self.targetpos[1]))
                self.caught = True
                break

        return self.caught, self.numofmoves

    def runtest2(self):
        '''
        Uses RRT2
        '''
        # draw the environment
        # transpose because imshow places the first dimension on the y-axis
        # f, ax = plt.subplots()
        # ax.imshow(self.envmap.T, interpolation="none", cmap='gray_r', origin='lower', \
        #             extent=(-0.5, self.envmap.shape[0]-0.5, -0.5, self.envmap.shape[1]-0.5) )
        # ax.axis([-0.5, self.envmap.shape[0]-0.5, -0.5, self.envmap.shape[1]-0.5])
        # ax.set_xlabel('x')
        # ax.set_ylabel('y')  
        # hr = ax.plot(self.robotpos[0], self.robotpos[1], 'bs')
        # ht = ax.plot(self.targetpos[0],self.targetpos[1], 'rs')
        # f.canvas.flush_events()
        # plt.show()

        # now comes the main loop
        
        for _ in range(20000):

            # # print the number of moves
            # print("Number of moves: {}".format(numofmoves + 1))

            # call robot planner
            t0 = tic()
            newrobotpos = self.RobotPlannerRRT2()
            # compute move time for the target, if it is greater than 2 sec, the target will move multiple steps
            movetime = max(1, math.ceil((tic()-t0)/2.0))
            
            # check that the new commanded position is valid
            if (newrobotpos[0] < 0 or newrobotpos[0] >= self.envmap.shape[0] or \
                newrobotpos[1] < 0 or newrobotpos[1] >= self.envmap.shape[1] ):
                print('ERROR: out-of-map robot position commanded\n')
                break
            elif (self.envmap[newrobotpos[0], newrobotpos[1]] != 0 ):
                print('ERROR: invalid robot position commanded\n')
                break
            elif (abs(newrobotpos[0]-self.robotpos[0]) > 1 or abs(newrobotpos[1]-self.robotpos[1]) > 1):
                print('ERROR: invalid robot move commanded\n')
                break

            # call target planner to see how the target moves within the robot planning time
            newtargetpos = self.targetplanner(movetime)
            
            # make the moves
            self.robotpos = newrobotpos
            self.targetpos = newtargetpos
            self.numofmoves += 1
            
            # # draw positions
            # hr[0].set_xdata(self.robotpos[0])
            # hr[0].set_ydata(self.robotpos[1])
            # ht[0].set_xdata(self.targetpos[0])
            # ht[0].set_ydata(self.targetpos[1])
            # f.canvas.flush_events()
            # plt.show()

            # check if target is caught
            if (abs(self.robotpos[0]-self.targetpos[0]) <= 1 and abs(self.robotpos[1]-self.targetpos[1]) <= 1):
                print('robotpos = (%d,%d)' %(self.robotpos[0], self.robotpos[1]))
                print('targetpos = (%d,%d)' %(self.targetpos[0], self.targetpos[1]))
                self.caught = True
                break

        return self.caught, self.numofmoves

    def print_result(self):
        print('Number of moves made: {}; Target caught: {}.\n'.format(self.numofmoves, self.caught))


if __name__ == "__main__":

    mapNo = "7"
    print("Map #{}".format(mapNo))
    
    # test 1
    t1 = CatchingGame(mapNo)
    print("Trial #1 (RRT)")
    timeStart = tic()
    t1.runtest()
    timeEnd = toc(timeStart)
    t1.print_result()

    # test 1
    t2 = CatchingGame(mapNo)
    print("Trial #2 (RRT)")
    timeStart = tic()
    t2.runtest()
    timeEnd = toc(timeStart)
    t2.print_result()

    # test 1
    t3 = CatchingGame(mapNo)
    print("Trial #3 (RRT)")
    timeStart = tic()
    t3.runtest()
    timeEnd = toc(timeStart)
    t3.print_result()