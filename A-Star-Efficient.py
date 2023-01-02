import math
import numpy as np
import sys
import time

height = 6
width = 6
start = (0, 2)
goal = (5, 3)

maze = np.zeros((width, height))
maze[2][3] = 1

def c2i(coordinates, width=width) -> int:
    '''
    Translate from row-col system to index system
    '''
    row, col = coordinates
    return row * width + col

def i2c(index, width=width):
    '''
    Translate from index system to coordinate system
    '''
    row = int(index / width)
    col = index - row * width
    return (row, col)

def fill_Inf(idxStart, height=height, width=width):
    '''
    Generate an array of infinite except the start point
    '''
    ret = np.zeros((height * width, 1))
    for i in range(len(ret)):
        ret[i] = math.inf
    ret[idxStart] = 0
    return ret

def within_bound(idx, maze=maze):
    '''
    Check if the point is within the boundary
    '''
    row, col = i2c(idx)
    if  row < 0 or row >= len(maze) or col < 0 or col >= len(maze[0]):
        return False
    else:
        return True

def is_valid(idx, maze=maze) -> bool:
    '''
    Check if the move is valid or not
    '''
    row, col = i2c(idx)
    if row < 0 or row >= len(maze) or col < 0 or col >= len(maze[0]):
        return False
    elif maze[row][col] == 1:
        return False
    else:
        return True


def priority(e):
    '''
    A function that returns the value of the key
    '''
    (p, _) = e
    return p

def h_value(idxA, coordinateB=goal):
    '''
    Calculate the heuristic value
    '''
    rowA, colA = i2c(idxA)
    idxB = c2i(coordinateB)
    rowB, colB = i2c(idxB)

    return abs(rowA - rowB) + abs(colA - colB)

idxStart = c2i(start)
idxGoal = c2i(goal)
Fs = fill_Inf(idxStart)
Gs = fill_Inf(idxStart)
parents = np.zeros((height * width, 1))

open = []
open.append((0.0, idxStart))
closed = []

while open and Fs[idxGoal] == math.inf:

    _, idxCur = open.pop(0)

    if not idxCur in closed:

        closed.append(idxCur)
        valG = Gs[idxCur][0]

        # c0
        c0 = idxCur + 1
        if within_bound(c0):
            if not c0 in closed:
                if is_valid(c0):
                    if (valG + 1) < Gs[c0][0]:
                        Gs[c0][0] = (valG + 1)
                        parents[c0][0] = idxCur
                        Fs[c0][0] = Gs[c0][0] + h_value(c0)
                open.append((Fs[c0][0], c0))
                open.sort(key=priority)

        # c1
        c1 = idxCur + width + 1
        if within_bound(c1):
            if not c1 in closed:
                if is_valid(c1):
                    if (valG + 1) < Gs[c1][0]:
                        Gs[c1][0] = (valG + 1.4)
                        parents[c1][0] = idxCur
                        Fs[c1][0] = Gs[c1][0] + h_value(c1)
                open.append((Fs[c1][0], c1))
                open.sort(key=priority)

        # c2
        c2 = idxCur - width
        if within_bound(c2):
            if not c2 in closed:
                if is_valid(c2):
                    if (valG + 1) < Gs[c2][0]:
                        Gs[c2][0] = (valG + 1)
                        parents[c2][0] = idxCur
                        Fs[c2][0] = Gs[c2][0] + h_value(c2)
                open.append((Fs[c2][0], c2))
                open.sort(key=priority)

        # c3
        c3 = idxCur - width - 1
        if within_bound(c3):
            if not c3 in closed:
                if is_valid(c3):
                    if (valG + 1) < Gs[c3][0]:
                        Gs[c3][0] = (valG + 1.4)
                        parents[c3][0] = idxCur
                        Fs[c3][0] = Gs[c3][0] + h_value(c3)
                open.append((Fs[c3][0], c3))
                open.sort(key=priority)

        # c4
        c4 = idxCur - 1
        if within_bound(c4):
            if not c4 in closed:
                if is_valid(c4):
                    if (valG + 1) < Gs[c4][0]:
                        Gs[c4][0] = (valG + 1)
                        parents[c4][0] = idxCur
                        Fs[c4][0] = Gs[c4][0] + h_value(c4)
                open.append((Fs[c4][0], c4))
                open.sort(key=priority)

        # c5
        c5 = idxCur + width - 1
        if within_bound(c5):
            if not c5 in closed:
                if is_valid(c5):
                    if (valG + 1) < Gs[c5][0]:
                        Gs[c5][0] = (valG + 1.4)
                        parents[c5][0] = idxCur
                        Fs[c5][0] = Gs[c5][0] + h_value(c5)
                open.append((Fs[c5][0], c5))
                open.sort(key=priority)

        # c6
        c6 = idxCur + width
        if within_bound(c6):
            if not c6 in closed:
                if is_valid(c6):
                    if (valG + 1) < Gs[c6][0]:
                        Gs[c6][0] = (valG + 1)
                        parents[c6][0] = idxCur
                        Fs[c6][0] = Gs[c6][0] + h_value(c6)
                open.append((Fs[c6][0], c6))
                open.sort(key=priority)

        # c7
        c7 = idxCur + width + 1
        if within_bound(c7):
            if not c7 in closed:
                if is_valid(c7):
                    if (valG + 1) < Gs[c7][0]:
                        Gs[c7][0] = (valG + 1.4)
                        parents[c7][0] = idxCur
                        Fs[c7][0] = Gs[c7][0] + h_value(c7)
                open.append((Fs[c7][0], c7))
                open.sort(key=priority)

print(Fs[idxGoal][0])

idxCur = idxGoal
paths_reversed = []
paths = []
commands = []
paths_reversed.append(idxCur)
while idxCur != idxStart:
    idxTemp = int(parents[idxCur][0])
    paths_reversed.append(idxTemp)
    idxCur = idxTemp

for p in reversed(paths_reversed):
    paths.append(p)

for p in paths:
    row, col = i2c(p)
    commands.append((row, col))
print(commands)

sum = sys.getsizeof(Fs) + sys.getsizeof(Gs) + \
    sys.getsizeof(parents) + sys.getsizeof(open) + \
        sys.getsizeof(closed) + sys.getsizeof(paths_reversed) +\
            sys.getsizeof(paths) + sys.getsizeof(commands)
print("Memory used: {} bytes".format(sum))
