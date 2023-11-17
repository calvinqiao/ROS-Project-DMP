import numpy as np
import matplotlib.pyplot as plt
from IPython import embed

class CanonicalSystem(object):
    def __init__(self, dt, alpha_x=1.0, pattern='discrete'):
            '''

            :param dt: float, time step
            :param alpha_x: float, a gain term on the dynamical system
            :param pattern: 'discrete' or 'rhythmic'
            '''
            self.k_tau = 0.7

            self.alpha_x = alpha_x
            self.pattern = pattern

            if self.pattern == 'discrete':
                self.step = self.step_discrete
                # set run time of canonical system
                self.run_time = 10.0  ## change back to 40 later
            elif pattern == 'rhythmic':
                pass
            else:
                raise Exception("Invalid pattern type")

            self.tau = self.run_time * self.k_tau
            self.dt = dt
            self.time_steps = int(self.run_time / self.dt)

            self.x = None
            self.x_track = None

            self.reset_state()

    def compute_x_track(self, **kwargs):
        time_steps = self.time_steps

        if self.x_track == None:
            self.x_track = np.zeros(time_steps)

        # new_tau = 0.5 / self.tau
        self.reset_state()
        for t in range(time_steps):
            self.x_track[t] = self.x
            self.step(**kwargs)
            #self.x += self.alpha_x * (0 - self.x) * new_tau * self.dt

        return self.x_track

    def reset_state(self):
        '''
        Reset x to 1.0
        '''
        self.x = 1.0

    def step_discrete(self, error_coupling=1.0):
        '''
        Generate a single step of x for discrete movements
        Decay from 1.0 to 0 according to dx = -alpha_x*x
        :param tau: float gain on execution time
                    increase tau to make the system execute faster
        :return: self.x
        '''
        # embed()
        new_tau = 0.5 / self.tau
        self.x += self.alpha_x * (0-self.x) * error_coupling * new_tau * self.dt
        return self.x


if __name__ == "__main__":
    cs = CanonicalSystem(dt=0.001, pattern="discrete")

    x_track1 = cs.compute_x_track()

    cs.reset_state()

    time_steps = int(cs.run_time/.001)

    x_track2 = np.zeros(time_steps)
    err = np.zeros(time_steps)
    err[200:400] = 2
    err_coup = 1.0 / (1 + err)

    for i in range(time_steps):
        x_track2[i] = cs.step(error_coupling=err_coup[i])


    fig, ax = plt.subplots(figsize=(6, 3))

    ax.plot(x_track1, lw=2)
    ax.plot(x_track2, lw=2)

    plt.grid()

    plt.legend(['normal x_track', 'error_coupling'])

    plt.ylim(0, 1)

    plt.xlabel('time (s)')
    plt.ylabel('x')

    plt.title('Canonical system - discrete')

    plt.tight_layout()

    plt.show()