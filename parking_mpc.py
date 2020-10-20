import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 10
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 10, 3.14/2]
        self.reference2 = None #[10, 2, 3.14/2]

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        x_t += v_t * dt * np.cos(psi_t) 
        y_t += v_t * dt * np.sin(psi_t)
        v_t += (pedal * dt - v_t/25.0)
        psi_t += v_t * 1/2.5 * np.tan(steering) * dt 

        return [x_t, y_t, psi_t, v_t]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0
        ref_np = np.array(ref)

        for k in range(self.horizon):
            state = self.plant_model(state, self.dt, u[2*k], u[2*k+1])
            state_np = np.array(state)
            pos = state_np[0:2]
            goal = ref[0:2]
            cost += (np.linalg.norm(goal-pos))**2

            heading = state_np[2]
            he_ref = ref_np[2]

            if (np.linalg.norm(goal-pos) < 7):
                cost += (np.linalg.norm(he_ref - heading))





            



            




        return cost

sim_run(options, ModelPredictiveControl)
