import numpy as np
import scipy.interpolate
from canonical_system import *  # CanonicalSystem
from IPython import embed

class DMP(object):
    """
    Implementation of Dynamic Motor Primitives,
    as described in Dr. Stefan Schaal's (2002) paper.
    """

    def __init__(self, n_bfs, dt=0.001,
                 y_init=0, y_goal=1, weights=None,
                 alpha_z=None, beta_z=None, **kwargs):
        """
        n_bfs int: number of basis functions per DMP
        dt float: time step for simulation
        y_init list: initial state of DMPs
        y_goal list: goal state of DMPs
        w list: tunable parameters, control amplitude of basis functions
        alpha_z int: gain on attractor term y dynamics
        beta_z int: gain on attractor term y dynamics
        """

        self.n_bfs = n_bfs
        self.dt = dt

        self.y_des = None
        self.y = None
        self.dy = None
        self.ddy = None

        if isinstance(y_init, (int, float)):
            y_init = np.ones(1)*y_init
        self.y_init = y_init
        if isinstance(y_goal, (int, float)):
            y_goal = np.ones(1)*y_goal
        self.y_goal = y_goal

        # initialize an array to store weights
        if weights is None:
            weights = np.zeros((1, self.n_bfs))
        self.weights = weights

        self.alpha_z = np.ones(1) * 25.0 if alpha_z is None else alpha_z
        self.beta_z = self.alpha_z / 4.0 if beta_z is None else beta_z

        # set up scale
        self.scale = 1.0

        # set up Canonical System
        self.cs = CanonicalSystem(dt=self.dt, **kwargs)

        # set up time steps
        self.time_steps = int(self.cs.run_time / self.dt)

        # set up DMP system
        self.reset_state()

    def reset_state(self):
        """
        Reset the DMP system state
        """
        self.y = self.y_init.copy()
        self.dy = np.zeros(1)
        self.ddy = np.zeros(1)
        self.cs.reset_state()

    def gen_psi(self):
        raise NotImplementedError()

    def step(self, error=0.0):
        """
        Run DMP system for a single time step
        """
        new_tau = 0.5 / self.cs.tau

        error_coupling_value = 1.0 / (1.0 + error)

        # run canonical system
        x = self.cs.step_discrete(error_coupling=error_coupling_value)
        # print(x)

        # generate basis function activation
        # print("In DMP.step")
        psi = self.gen_psi(x)
        # print(psi)

        # generate the forcing term
        f = (self.gen_front_term(x) *
             (np.dot(psi, self.weights[0])) / np.sum(psi))

        # update DMP output trajectory state
        # update acceleration
        self.ddy[0] = (self.alpha_z[0] *
                       (self.beta_z[0] * (self.y_goal[0] - self.y[0]) -
                       self.dy[0]/new_tau) + f) * new_tau
        # print('f = ', f, ' ddy = ', self.ddy[0])
        # update velocity
        self.dy[0] += self.ddy[0] * new_tau * self.dt * error_coupling_value
        # update position
        self.y[0] += self.dy[0] * self.dt * error_coupling_value
        # print('f, y, dy, ddy = ', f, self.y, self.dy, self.ddy)
        # print('x, f = %7f, %7f' % (x, f))
        return self.y, self.dy, self.ddy, x

    def step_with_feedback(self, y_feedback, dy_feedback, simulated, error=0.0):
        """
        Run DMP system for a single time step with sensor feedback info
        """
        new_tau = 0.5 / self.cs.tau

        # if error >= 0:
        #     error_coupling_value = 1.0 / (1.0 + error)
        # else:
        #     error_coupling_value = 1.0 / (1.0 + abs(error))

        error_coupling_value = 1.0 / (1.0 + abs(error))
        adjust_k = 1.0 / (1.0 + error)

        # print('error_coupling = ', error_coupling_value)
        # error_coupling_value = 1.0
        # run canonical system
        x = self.cs.step_discrete(error_coupling=error_coupling_value)
        # print(x)

        # generate basis function activation
        psi = self.gen_psi(x)

        # generate the forcing term
        f = (self.gen_front_term(x) *
             (np.dot(psi, self.weights[0])) / np.sum(psi))

        # update DMP output trajectory state
        # update acceleration

        if simulated:
            self.ddy[0] = (self.alpha_z[0] *
                           (self.beta_z[0] * (self.y_goal[0] - y_feedback) -
                            dy_feedback / new_tau) + f) * new_tau * adjust_k
        else:
        # Only y with feedback
            self.ddy[0] = (self.alpha_z[0] *
                           (self.beta_z[0] * (self.y_goal[0] - y_feedback) -
                            self.dy[0] / new_tau) + f) * new_tau  * adjust_k

        # update velocity
        self.dy[0] += self.ddy[0] * new_tau * self.dt * (adjust_k**2)
        # update position
        self.y[0] += self.dy[0] * self.dt * adjust_k

        return self.y, self.dy, self.ddy, x

    def gen_goal(self, y_des):
        raise NotImplementedError()

    def check_offset(self):
        """
        Check to see if initial position and goal are the same
        if they are, offset slightly so that the forcing term is not 0
        """
        if self.y_init[0] == self.y_goal[0]:
            self.y_goal[0] += 1e-10   # 1e-10 previously seems to be much faster in learning

    def gen_weights(self, f_target):
        raise NotImplementedError()

    def gen_front_term(self, x, dmp_num):
        raise NotImplementedError()

    def learn_path(self, y_des):
        """
        Take in a desired trajectory and generate the set of
        system parameters that best realize this path.
        y_des list/array: the dimension of desired trajectory of DMP
                          should be shaped [1, run_time]
        """

        # set initial state and goal
        if y_des.ndim == 1:
            y_des = y_des.reshape(1, len(y_des))
        self.y_init = y_des[:, 0].copy()
        self.y_des = y_des.copy()
        self.y_goal = self.gen_goal(y_des)

        self.check_offset()

        # generate function to interpolate the desired trajectory
        path = np.zeros((1, self.time_steps))
        x = np.linspace(0, self.cs.run_time, y_des.shape[1])
        path_gen = scipy.interpolate.interp1d(x, y_des[0])
        for t in range(self.time_steps):
            path[0, t] = path_gen(t * self.dt)
        y_des = path

        # compute velocity of y_des
        dy_des = np.diff(y_des) / self.dt
        # append zero to the beginning of dy_des
        dy_des = np.hstack((np.zeros((1, 1)), dy_des))

        # compute acceleration of y_des
        ddy_des = np.diff(dy_des) / self.dt
        # append zero to the beginning of ddy_des
        ddy_des = np.hstack((np.zeros((1, 1)), ddy_des))

        new_tau = 0.5 / self.cs.tau
        f_target = np.zeros((y_des.shape[1], 1))
        # compute the force term required to move along this trajectory
        f_target[:, 0] = (ddy_des[0]/(new_tau**2) - self.alpha_z[0] *
                          (self.beta_z[0] * (self.y_goal[0] - y_des[0]) -
                          dy_des[0]/new_tau))

        # generate weights to realize f_target
        self.gen_weights(f_target)

        self.reset_state()

    def construct_trajectory(self, time_steps=None, **kwargs):
        """
        Generate trajectory
        """
        self.reset_state()

        if time_steps is None:
            time_steps = self.time_steps

        # set up tracking vectors for y, dy and ddy
        y_track = np.zeros((time_steps, 1))
        dy_track = np.zeros((time_steps, 1))
        ddy_track = np.zeros((time_steps, 1))
        cs_x_track = np.zeros((time_steps, 1))
        # print('time steps = ', time_steps)
        for t in range(time_steps):
            # run and record time step
            y_track[t], dy_track[t], ddy_track[t], cs_x_track[t] = self.step(**kwargs)
            # print(dy_track[t])
        return y_track, dy_track, ddy_track, cs_x_track

    def construct_trajectory_one_step_with_feedback(self, y_feedback, dy_feedback, planned_y,
                                                    simulated, time_steps=None):
        """
        Generate trajectory with sensor feedback info
        """
        # run a time step
        alpha_err = 7.5
        error = alpha_err * (y_feedback-planned_y)
        # error = 0.0
        return self.step_with_feedback(y_feedback, dy_feedback, planned_y, simulated, error=error)

    def construct_trajectory_one_step(self, time_steps=None, **kwargs):
        y, dy, ddy, cs_x = self.step(**kwargs)
        print(dy)
        return y, dy, ddy, cs_x
