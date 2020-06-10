from copy import deepcopy
from numpy import absolute
from a_star_template import AStar
import numpy as np
import matplotlib.pyplot as plt

class State:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Node:
    def __init__(self, state, cost):
        self.state = state
        self.cost = cost

def comparefunc(node1, node2):
    if node1.state == node2.state:
        return True
    return False

def extendfunc(node_in):
    node_out1 = deepcopy(node_in)
    node_out1.state.x = node_out1.state.x+1
    node_out1.total_cost = node_out1.total_cost + 1
    node_out2 = deepcopy(node_in)
    node_out2.state.y = node_out2.state.y+1
    node_out2.total_cost = node_out2.total_cost + 1
    node_out = [node_out1, node_out2]
    return node_out

def heuristicfunc(goalx, goaly):
    def hfunc(node):
        return absolute(node.state.x - goalx) + absolute(node.state.y - goaly)
    return hfunc


def checkgoal(goalx, goaly):
    def isgoal(node):
        if node.state.x == goalx and node.state.y == goaly:
            return True
        return False
    return isgoal


def drawfunc(closedlist, openlist):
    plt.clf()
    for n in closedlist:
        plt.plot(n.state.x, n.state.y, "xk")
    for n in openlist:
        plt.plot(n.state.x, n.state.y, "ok")
    plt.ylim(top=20, bottom=0)
    plt.xlim(right=20, left=0)
    plt.pause(0.1)

s0 = State(0, 0)
n0 = Node(s0, 0)

goalx = 10
goaly = 12
problem = AStar(n0, extendfunc, heuristicfunc(goalx, goaly), comparefunc, checkgoal(goalx, goaly), drawfunc)
problem.run(25)