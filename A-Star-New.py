import numpy as np
import math
from numpy import loadtxt
import matplotlib.pyplot as plt
import time
import random
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
            self.R = 50
            self.beta = 10

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
            self.R = 200
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

        # A* algorithm
        self.idxStart = self.c2i(robotstart)
        self.idxGoal = self.c2i(targetstart)
        self.Fs = self.fill_Inf(self.idxStart)
        self.Gs = self.fill_Inf(self.idxStart)
        self.parents = np.ones((len(self.envmap) * len(self.envmap[0]), 1))
        self.open = []
        self.open.append((0.0, self.idxStart))
        self.closed = []
        self.foundA = False
        self.path_commands = []


    def RobotPlannerGreedy(self):
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

        # # print the new position
        # print("Robot's position: {}\t".format(newrobotpos), end="")

        return newrobotpos

    def c2i(self, coordinates) -> int:
        '''
        Translate from row-col system to index system
        '''
        width = len(self.envmap[0])
        row, col = coordinates
        return row * width + col

    def i2c(self, index):
        '''
        Translate from index system to coordinate system
        '''
        width = len(self.envmap[0])
        row = int(index / width)
        col = index - row * width
        return (row, col)

    def fill_Inf(self, idxStart):
        '''
        Generate an array of infinite except the start point
        '''
        width = len(self.envmap[0])
        height = len(self.envmap)
        ret = np.zeros((height * width, 1))
        for i in range(len(ret)):
            ret[i] = math.inf
        ret[idxStart] = 0
        return ret

    def within_bound(self, c):
        '''
        Check if the point is within the boundary
        '''
        row, col = c
        if  row < 0 or row >= len(self.envmap) or col < 0 or col >= len(self.envmap[0]):
            return False
        else:
            return True

    def is_valid(self, idx) -> bool:
        '''
        Check if the move is valid or not
        '''
        row, col = self.i2c(idx)
        if row < 0 or row >= len(self.envmap) or col < 0 or col >= len(self.envmap[0]):
            return False
        elif self.envmap[row][col] == 1:
            return False
        else:
            return True

    def priority(self, e):
        '''
        A function that returns the value of the key
        '''
        (p, _) = e
        return p

    def h_value(self, idxA):
        '''
        Calculate the heuristic value
        '''
        coordinateB = self.targetpos
        rowA, colA = self.i2c(idxA)
        idxB = self.c2i(coordinateB)
        rowB, colB = self.i2c(idxB)

        return abs(rowA - rowB) + abs(colA - colB)

    def get_path_coords(self):
        idxCur = self.idxGoal
        paths_reversed = []
        paths = []
        commands = []
        paths_reversed.append(idxCur)

        while idxCur != self.idxStart:
            idxTemp = int(self.parents[idxCur][0])
            paths_reversed.append(idxTemp)
            idxCur = idxTemp

        for p in reversed(paths_reversed):
            paths.append(p)

        for p in paths:
            row, col = self.i2c(p)
            commands.append((row, col))

        return commands

    def RobotPlannerAStar(self):

        # use the old position if we fail to find an acceptable move
        newrobotpos = np.copy(self.robotpos)

        if self.path_commands:
            new_command = self.path_commands.pop(0)
            newrobotpos[0] = new_command[0]
            newrobotpos[1] = new_command[1]

        elif self.foundA == False:
            while self.open and self.Fs[self.idxGoal] == math.inf:

                _, idxCur = self.open.pop(0)

                if not idxCur in self.closed:

                    self.closed.append(idxCur)
                    valG = self.Gs[idxCur][0]

                # c0
                i0 = idxCur + 1
                c0 = self.i2c(i0)
                if self.within_bound(c0):
                    if not i0 in self.closed:
                        if self.is_valid(i0):
                            if (valG + 1) < self.Gs[i0][0]:
                                self.Gs[i0][0] = (valG + 1)
                                self.parents[i0][0] = idxCur
                                self.Fs[i0][0] = self.Gs[i0][0] + self.h_value(i0)
                        self.open.append((self.Fs[i0][0], i0))
                        self.open.sort(key=self.priority)

                # c1
                i1 = idxCur - len(self.envmap[0]) + 1
                c1 = self.i2c(i1)
                if self.within_bound(c1):
                    if not i1 in self.closed:
                        if self.is_valid(i1):
                            if (valG + 1) < self.Gs[i1][0]:
                                self.Gs[i1][0] = (valG + 1.4)
                                self.parents[i1][0] = idxCur
                                self.Fs[i1][0] = self.Gs[i1][0] + self.h_value(i1)
                        self.open.append((self.Fs[i1][0], i1))
                        self.open.sort(key=self.priority)

                # c2
                i2 = idxCur - len(self.envmap[0])
                c2 = self.i2c(i2)
                if self.within_bound(c2):
                    if not i2 in self.closed:
                        if self.is_valid(i2):
                            if (valG + 1) < self.Gs[i2][0]:
                                self.Gs[i2][0] = (valG + 1)
                                self.parents[i2][0] = idxCur
                                self.Fs[i2][0] = self.Gs[i2][0] + self.h_value(i2)
                        self.open.append((self.Fs[i2][0], i2))
                        self.open.sort(key=self.priority)

                # c3
                i3 = idxCur - len(self.envmap[0]) - 1
                c3 = self.i2c(i3)
                if self.within_bound(c3):
                    if not i3 in self.closed:
                        if self.is_valid(i3):
                            if (valG + 1) < self.Gs[i3][0]:
                                self.Gs[i3][0] = (valG + 1.4)
                                self.parents[i3][0] = idxCur
                                self.Fs[i3][0] = self.Gs[i3][0] + self.h_value(i3)
                        self.open.append((self.Fs[i3][0], i3))
                        self.open.sort(key=self.priority)

                # c4
                i4 = idxCur - 1
                c4 = self.i2c(i4)
                if self.within_bound(c4):
                    if not i4 in self.closed:
                        if self.is_valid(i4):
                            if (valG + 1) < self.Gs[i4][0]:
                                self.Gs[i4][0] = (valG + 1)
                                self.parents[i4][0] = idxCur
                                self.Fs[i4][0] = self.Gs[i4][0] + self.h_value(i4)
                        self.open.append((self.Fs[i4][0], i4))
                        self.open.sort(key=self.priority)

                # c5
                i5 = idxCur + len(self.envmap[0]) - 1
                c5 = self.i2c(i5)
                if self.within_bound(c5):
                    if not i5 in self.closed:
                        if self.is_valid(i5):
                            if (valG + 1) < self.Gs[i5][0]:
                                self.Gs[i5][0] = (valG + 1.4)
                                self.parents[i5][0] = idxCur
                                self.Fs[i5][0] = self.Gs[i5][0] + self.h_value(i5)
                        self.open.append((self.Fs[i5][0], i5))
                        self.open.sort(key=self.priority)

                # c6
                i6 = idxCur + len(self.envmap[0])
                c6 = self.i2c(i6)
                if self.within_bound(c6):
                    if not i6 in self.closed:
                        if self.is_valid(i6):
                            if (valG + 1) < self.Gs[i6][0]:
                                self.Gs[i6][0] = (valG + 1)
                                self.parents[i6][0] = idxCur
                                self.Fs[i6][0] = self.Gs[i6][0] + self.h_value(i6)
                        self.open.append((self.Fs[i6][0], i6))
                        self.open.sort(key=self.priority)

                # c7
                i7 = idxCur + len(self.envmap[0]) + 1
                c7 = self.i2c(i7)
                if self.within_bound(c7):
                    if not i7 in self.closed:
                        if self.is_valid(i7):
                            if (valG + 1) < self.Gs[i7][0]:
                                self.Gs[i7][0] = (valG + 1.4)
                                self.parents[i7][0] = idxCur
                                self.Fs[i7][0] = self.Gs[i7][0] + self.h_value(i7)
                        self.open.append((self.Fs[i7][0], i7))
                        self.open.sort(key=self.priority)

            self.foundA = True

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

        # draw the environment
        # transpose because imshow places the first dimension on the y-axis
        f, ax = plt.subplots()
        ax.imshow(self.envmap.T, interpolation="none", cmap='gray_r', origin='lower', \
                    extent=(-0.5, self.envmap.shape[0]-0.5, -0.5, self.envmap.shape[1]-0.5) )
        ax.axis([-0.5, self.envmap.shape[0]-0.5, -0.5, self.envmap.shape[1]-0.5])
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        hr = ax.plot(self.robotpos[0], self.robotpos[1], 'bs')
        ht = ax.plot(self.targetpos[0],self.targetpos[1], 'rs')
        f.canvas.flush_events()
        plt.show()

        # now comes the main loop

        for _ in range(20000):

            # # print the number of moves
            # print("Number of moves: {}".format(numofmoves + 1))

            # call robot planner
            t0 = tic()
            newrobotpos = self.RobotPlannerAStar()
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

            # draw positions
            hr[0].set_xdata(self.robotpos[0])
            hr[0].set_ydata(self.robotpos[1])
            ht[0].set_xdata(self.targetpos[0])
            ht[0].set_ydata(self.targetpos[1])
            f.canvas.flush_events()
            plt.show()

            # check if target is caught
            if (abs(self.robotpos[0]-self.targetpos[0]) <= 1 and abs(self.robotpos[1]-self.targetpos[1]) <= 1):
                print('robotpos = (%d,%d)' %(self.robotpos[0], self.robotpos[1]))
                print('targetpos = (%d,%d)' %(self.targetpos[0], self.targetpos[1]))
                self.caught = True
                break

        return self.caught, self.numofmoves

    def runtest3(self):
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
            newrobotpos = self.RobotPlannerAStar()
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

    #
    t1 = CatchingGame("5")
    t1.runtest3()
    t1.print_result()
