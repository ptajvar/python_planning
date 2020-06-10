from rrt_star_template import RRTStar
import numpy as np
import matplotlib.pyplot as plt


class State:
    def __init__(self, x, y):
        self.x = x
        self.y = y


def euclidean_distance(state1: State, state2: State) -> float:
    distance = np.sqrt((state1.x - state2.x) ** 2 + (state1.y - state2.y) ** 2)
    return distance


def steer_func(state1: State, state2: State):
    if euclidean_distance(state1, state2) > 5:
        return None, None, None
    new_state = state2
    new_path = list([])
    new_path.append(state1)
    new_path.append(state2)
    new_path_cost = euclidean_distance(state1, state2)
    return new_state, new_path, new_path_cost


def sample_biased(x_bound: list, y_bound: list, target: State, bias_p=0.1):
    def sampler():
        if np.random.rand() < bias_p:
            return target
        sample_state = State(0, 0)
        rand_x = np.random.rand()
        rand_y = np.random.rand()
        sample_state.x = x_bound[0] * (1-rand_x) + x_bound[1] * rand_x
        sample_state.y = y_bound[0] * (1-rand_y) + y_bound[1] * rand_y
        return sample_state
    return sampler


def drawfunc(trajectory_list: list):
    plt.clf()
    for trajectory in trajectory_list:
        if trajectory is None:
            continue
        plt.plot([point.x for point in trajectory], [point.y for point in trajectory])
        plt.plot(trajectory[-1].x, trajectory[-1].y, "ok")
    plt.ylim(top=20, bottom=0)
    plt.xlim(right=20, left=0)
    plt.pause(0.1)


s0 = State(0, 0)
goalx = 10
goaly = 12
s_target = State(goalx, goaly)
x_bound = [0, 20]
y_bound = [0, 20]
problem = RRTStar(steer_func=steer_func, sample_func=sample_biased(x_bound=x_bound, y_bound=y_bound, target=s_target),
                  draw_func=drawfunc, dist_func=euclidean_distance, root_state=s0)
problem.run(max_iter=200)
