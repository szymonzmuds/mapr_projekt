import numpy as np
from sparse_rrt.systems.system_interface import BaseSystem
from sparse_rrt.visualization import svg_rectangle


class FreePoint(BaseSystem):
    '''
    A simple system implementing a 2d point. It's controls include velocity and direction.
    '''
    MIN_X, MAX_X = -10, 10  # min and max X coordinate of a point
    MIN_Y, MAX_Y = -10, 10  # min and max Y coordinate of a point
    MIN_V, MAX_V = 0., 10.  # min and max control velocity
    MIN_THETA, MAX_THETA = -3.14, 3.14  # min and max control direction

    def __init__(self, available_obstacles, number_of_obstacles, map_data=None):
        BaseSystem.__init__(self)
        # define available obstacle rectangles
        assert number_of_obstacles <= len(available_obstacles)
        # use only limited number of obstacles
        if map_data is not None:
            self.MIN_X = map_data["min_x"]
            self.MIN_Y = map_data["min_y"]
            self.MAX_X = map_data["max_x"]
            self.MAX_Y = map_data["max_y"]
        self._obstacles = available_obstacles[:number_of_obstacles]


    def propagate(self, start_state, control, num_steps, integration_step):
        '''
        Integrate system dynamics
        :param start_state: numpy array with the start state for the integration
        :param control: numpy array with constant controls to be applied during integration
        :param num_steps: number of steps to integrate
        :param integration_step: dt of integration
        :return: new state of the system
        '''
        control_v = np.array([control[0] * np.cos(control[1]), control[0] * np.sin(control[1])])
        trajectory = start_state + np.arange(num_steps)[:, None]*integration_step*control_v
        for (low_x, low_y, high_x, high_y) in self._obstacles:
            low_x_bound = trajectory[:, 0] >= low_x
            high_x_bound = trajectory[:, 0] <= high_x
            low_y_bound = trajectory[:, 1] >= low_y
            high_y_bound = trajectory[:, 1] <= high_y

            collisions = low_x_bound & high_x_bound & low_y_bound & high_y_bound
            if np.any(collisions):
                return None
        state = np.clip(trajectory[-1], [self.MIN_X, self.MIN_Y], [self.MAX_X, self.MAX_Y])
        return state


    def visualize_point(self, state):
        '''
        Project state space point to 2d visualization plane
        :param state: numpy array of the state point
        :return: x, y of visualization coordinates for this state point
        '''
        x = (state[0] - self.MIN_X) / (self.MAX_X - self.MIN_X)
        y = (state[1] - self.MIN_Y) / (self.MAX_Y - self.MIN_Y)
        return x, y

    def visualize_obstacles(self, image_width, image_height):
        '''
        Draw rectangles of obstacles
        '''
        output = ''
        for (low_x, low_y, high_x, high_y) in self._obstacles:
            x, y = self.visualize_point((low_x, low_y))
            output += svg_rectangle(
                (x * image_width, y * image_height),
                (image_width * (high_x - low_x) / float(self.MAX_X - self.MIN_X),
                 image_height * (high_y - low_y) / float(self.MAX_Y - self.MIN_Y)),
                (image_width, image_height),
                fill='red'
            )
        return output

    def get_state_bounds(self):
        '''
        Return bounds for the state space
        :return: list of (min, max) bounds for each coordinate in the state space
        '''
        return [(self.MIN_X, self.MAX_X),
                (self.MIN_Y, self.MAX_Y)]

    def get_control_bounds(self):
        '''
        Return bounds for the control space
        :return: list of (min, max) bounds for each coordinate in the control space
        '''
        return [(self.MIN_V, self.MAX_V),
                (self.MIN_THETA, self.MAX_THETA)]

    def is_circular_topology(self):
        '''
        Indicate whether state system has planar or circular topology
        :return: boolean flag for each coordinate (False for planar topology)
        '''
        return [False, False]