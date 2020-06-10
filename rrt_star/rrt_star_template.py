"""
RRTStar provides the skeleton of the RRT* algorithm. It an initial state (root) and the functions that
are required for the algorithm. see __init__.
Author: Pouria Tajvar
Inspired by code from Atsushi Sakai(@Atsushi_twi): https://github.com/AtsushiSakai/PythonRobotics
"""

import numpy as np

class RRTStar:
    class TreeNode:

        def __init__(self, state, node_id=-1, parent=-1, path=None, path_cost=0, total_cost=0):
            """
            :param state: User defined variable that holds information about the node (e.g. position in the workspace)
            :param node_id: node id
            """
            self.state = state
            self.id = node_id
            self.path = path  # path to the parent node
            self.parent = parent  # the parent node id, -1 means no parent (root node).
            self.path_cost = path_cost  # cost from parent
            self.total_cost = total_cost  # cost from root

    def __init__(self, sample_func, steer_func, dist_func, root_state=None, draw_func=None, max_iter=1000):
        """
        :param root_state: state of the tree root
        :param sample_func: No input. Samples random state from the state space
        :param steer_func: Input: state A, state B. Output: state B', trajectory between A and B', cost of the
        trajectory. Tries to steer A toward B, will return None for every output if it fails (no valid/collision free
        trajectory)
        :param dist_func: Input: state A, state B. Output: distance between the two states.
        :param draw_func: Input: List of trajectories. Draws the tree.
        :param max_iter: maximum number of RRT* iterations.
        """
        self.root = self.TreeNode(state=root_state, node_id=0)
        self.sample_func = sample_func
        self.steer_func = steer_func
        self.dist_func = dist_func
        self.draw_func = draw_func
        self.max_iter = max_iter
        self.node_list = [self.root]
        self.node_count = 1

    def run(self, max_iter=0):
        """
        Building the tree
        :param max_iter:  Maximum number of iterations to build the tree
        """
        if max_iter == 0:
            max_iter = self.max_iter
        for i in range(max_iter):
            rand_state = self.sample_func()
            nearest_node = self.get_nearest_node(rand_state)
            new_node_state, new_node_path, new_node_path_cost = self.steer_func(nearest_node.state, rand_state)
            if not (new_node_state is None):
                new_node_total_cost = new_node_path_cost + nearest_node.total_cost
                new_node = self.TreeNode(state=new_node_state, parent=nearest_node.id, path=new_node_path,
                                         path_cost=new_node_path_cost, total_cost=new_node_total_cost)
                self.add_node(new_node)
                if not (self.draw_func is None):
                    self.draw_func([node.path for node in self.node_list])
                self.rewire(new_node)

    def add_node(self, node: TreeNode):
        """
        Add a new node to the tree and assign an id.
        @Note: the way we add id now is using node_count so things get messy if we delete any nodes. Maybe we need a
        separate counter for unique nodes.
        """
        self.node_count = self.node_count + 1
        node.id = self.node_count - 1  # node id starts from 0
        self.node_list.append(node)

    def get_near_nodes(self, state):
        """
        :return: 1- a list that contains the node indices sorted based on their distance from the input 'state' and
        2- the list of distances from each node to the state.
        """
        distance_list = [self.dist_func(node.state, state) for node in self.node_list]
        distance_list = np.array(distance_list)
        sorted_index = np.argsort(distance_list).tolist()
        return sorted_index, distance_list

    def get_nearest_node(self, state) -> TreeNode:
        distance_list = [self.dist_func(node.state, state) for node in self.node_list]
        min_index = distance_list.index(min(distance_list))
        return self.node_list[min_index]

    def rewire(self, node: TreeNode, max_distance=np.infty):
        sorted_near_nodes_index, distances = self.get_near_nodes(node.state)
        near_count = min(sum(distances <= max_distance), 100)
        sorted_near_nodes_index = sorted_near_nodes_index[0:near_count]
        for snn in sorted_near_nodes_index:
            new_node_state, new_node_path, new_node_path_cost = self.steer_func(node.state, self.node_list[snn].state)
            if new_node_state != self.node_list[snn].state:  # rewiring can happen only if we steer to the exact state
                continue
            new_node_total_cost = new_node_path_cost + node.total_cost
            if new_node_total_cost < self.node_list[snn].total_cost:
                self.node_list[snn].total_cost = new_node_total_cost
                self.node_list[snn].parent = node.id
                self.node_list[snn].path_cost = new_node_path_cost
                self.node_list[snn].path = new_node_path
                self.propagate_cost_to_leaves(snn)
                "@Thought: When a node is rewired, its cost is decreased so it may in turn be considered as a better " \
                "parent for its neighbouring nodes. In the original algorithm however this is not done and may be the "\
                "reason is to avoid the increased computational complexity."
                # self.rewire(snn)

    def propagate_cost_to_leaves(self, parent_node_index: int):
        parent_node = self.node_list[parent_node_index]
        parent_cost = parent_node.total_cost
        for n_index in range(len(self.node_list)):
            node = self.node_list[n_index]
            if node.parent == parent_node_index:
                node.total_cost = parent_cost + node.path_cost
                self.propagate_cost_to_leaves(n_index)
                break
