"""
AStar class provides the skeleton of the A* algorithm. It is initialized with an initial node and the required
functions for A*. (see init)
It can then be called through it's run function to run the algorithm for the requested number of iterations
or until a solution is found.
Author: Pouria Tajvar
"""
class AStar:
    class Node:
        def __init__(self, state, hvalue, cost=0, isgoal=False):
            """
            :param state: state of the node :D
            :param hvalue: heuristic value of the node
            :param cost: cost of reaching the node
            :param isgoal: whether the state has reached the goal
            """
            self.state = state
            self.hvalue = hvalue
            self.cost = cost
            self.isgoal = isgoal
            self.totcost = cost+hvalue
            self.parent = None

    def __init__(self, s0=None, extendfunc=None, heuristicfunc=None, comparefunc=None, checkgoal=None, drawfunc=None):
        """
        :param s0: initial state object - should have two attributes: state and cost
        :param extendfunc: computes successor states for a state
        :param heuristicfunc: estimates the distance of a state from the goal
        :param comparefunc: returns True if two states are considered the same (e.g. too close)
        :param checkgoal: checks if a node satisfies the goal condition.
        :param drawfunc: draws the search process steps
        """
        h0 = heuristicfunc(s0)
        isgoal0=checkgoal(s0)
        node = self.Node(state=s0.state, cost=s0.total_cost, hvalue=h0, isgoal=isgoal0)
        self.openlist = [node]
        self.closedlist = []
        self.extendfunc = extendfunc
        self.heuristicfunc = heuristicfunc
        self.comparefunc = comparefunc
        self.checkgoal = checkgoal
        self.drawfunc = drawfunc
        "success becomes true when a goal state enters the closed nodes"
        self.success = False
        "solution is the goal state in the closed nodes"
        self.solution = None
        self.totaliteration = 0

    def run(self, maxiter, draw='always'):
        """
        runs the astar algorithm up to a maximum number of iterations.
        :param maxiter: maximum number of iterations
        """
        for itern in range(maxiter):
            "check if the goal is already reached"
            if self.success:
                print('solution found after {0} iterations (total: {1}).'.format(itern, self.totaliteration))
                if draw == 'end':
                    self.drawfunc(self.closedlist, self.openlist)
                return
            self.totaliteration = self.totaliteration + 1
            "move the first state from the openlist (sorted based on hvalue+cost) to the closedlist"
            current_node = self.openlist.pop(0)
            self.closedlist.append(current_node)
            "If the state is a goal state, the algorithm has found the optimal solution (given an admissible heuristic)"
            if current_node.isgoal:
                self.solution = current_node
                self.success = True
                continue
            "The successor states are put in the open list"
            next_states = self.extendfunc(current_node)
            for s in next_states:
                "checking if the state is not already visited"
                if not self.isunique(s):
                    continue
                "Adding the astar elements"
                hval = self.heuristicfunc(s)
                isgoal = self.checkgoal(s)
                node = self.Node(state=s.state, cost=s.total_cost, hvalue=hval, isgoal=isgoal)
                node.parent = len(self.closedlist)-1
                # print('{0}'.format(node.totcost))
                self.sortedinsert(node)
            if self.drawfunc is not None and draw == 'always':
                self.drawfunc(self.closedlist, self.openlist)
        print('solution not found after {0} iterations (total: {1}).'.format(maxiter, self.totaliteration))
    def getsolution(self):
        solution = [self.solution]
        while solution[-1].parent is not None:
            solution.append(self.closedlist[solution[-1].parent])
        return solution
    def isunique(self, newnode):
        """
        Checking if the state is not already visited.
        :param node: the node to be checked
        """
        for s in self.closedlist:
            if self.comparefunc(s, newnode):
                return False
        return True

    def sortedinsert(self, newnode):
        """
        Adds a node to the right place in the sorted open list (suboptimal O(n) instead of O(logn))
        :param newnode: the new element to be added
        :return: the list with added element in the right place
        """
        index = len(self.openlist)
        for i in range(len(self.openlist)):
            if self.openlist[i].totcost >= newnode.totcost:
                index = i
                break
        self.openlist = self.openlist[:index] + [newnode] + self.openlist[index:]