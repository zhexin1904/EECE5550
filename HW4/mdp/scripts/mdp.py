import numpy as np
import matplotlib.pyplot as plt
import pdb



class MDP:
    def __init__(self, map, test, factor, actionType, plans):
        self.map = map
        self.actionType = actionType
        self.plans = plans
        self.test = test
        self.factor = factor


    def initialize_value_function(self):
        return np.zeros_like(self.map)

    def initialize_map(self):
        return np.array([[-0.01, -0.01, -1, 2],
                         [-0.01, -0.01, -0.01, -0.01],
                         [1, -0.01, -0.01, -0.01],
                         [-0.01, -0.01, -0.01, -0.01]])

    def initialize_parameters(self):
        return {'test_1': 0.1, 'test_2': 0.4, 'factor': 0.9, 'actionType': ['up', 'down', 'right', 'left'],
                'plans': {'up': (-1, 0), 'down': (1, 0), 'right': (0, 1), 'left': (0, -1)}}

    def update(self):
        value = self.initialize_value_function()
        delta_threshold = self.test * (1 - self.factor) / self.factor
        # pdb.set_trace()
        while True:
            delta = 0
            updated_value = np.copy(value)
            for i in range(self.map.shape[0]):
                for j in range(self.map.shape[1]):
                    if self.map[i, j] != -0.01:
                        continue
                    # pdb.set_trace()
                    updated_value[i, j] = max(self.q_value((i, j), a, value) for a in self.actionType)
                    delta = max(delta, abs(value[i, j] - updated_value[i, j]))
            value = np.copy(updated_value)
            if delta < delta_threshold:
                break

        return value

    def futurestates(self, state, action):
        i, j = state
        if action in self.plans:
            i_spot, j_spot = i + self.plans[action][0], j + self.plans[action][1]
            return [(i_spot, j_spot)] if 0 <= i_spot < self.map.shape[0] and 0 <= j_spot < self.map.shape[1] else [(i, j)]
        else:
            return [(i, j)]

    def value_(self, state, action, V):
        future_states = self.futurestates(state, action)
        value = sum(self.decision(action, spot, state) * V[spot[0], spot[1]] for spot in future_states)
        return value

    def decision(self, action, state_prime, state):
        i, j = state
        i_spot, j_spot = state_spot
        if action in ['up', 'down', 'right', 'left']:
            return 1 - self.test if (i_spot - i, j_spot - j) == self.plans[action] else self.test / 2
        else:
            return 0 
        
    def best_decision(self, V):
        policy = np.empty_like(self.map, dtype='<U5')
        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                if not np.isnan(V[i, j]):
                #  pdb.set_trace()
                    best_action = max(self.actionType, key=lambda a: self.q_value((i, j), a, V))
                    policy[i, j] = best_action
        return policy

    def visualize(self, policy, title):
        plt.imshow(self.map, cmap='viridis', origin='upper', extent=[0, self.map.shape[1], 0, self.map.shape[0]])
        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                self.visualize_direction(policy[i, j], j + 0.5, self.map.shape[0] - (i + 0.5))
        plt.title(title)
        plt.show()
        plt.savefig('1.png', dpi=300)

    def visualize_direction(self, action, x, y):
        if action == 'up':
            plt.arrow(x, y, 0, 0.3, head_width=0.1, head_length=0.1, fc='red', ec='red')
        elif action == 'down':
            plt.arrow(x, y, 0, -0.3, head_width=0.1, head_length=0.1, fc='red', ec='red')
        elif action == 'right':
            plt.arrow(x, y, 0.3, 0, head_width=0.1, head_length=0.1, fc='red', ec='red')
        elif action == 'left':
            plt.arrow(x, y, -0.3, 0, head_width=0.1, head_length=0.1, fc='red', ec='red')

def main():
    mdp_vi = MDP(None, None, None, None, None)
    map = mdp_vi.initialize_map()
    parameters = mdp_vi.initialize_parameters()

    mdp_vi = MDP(map, parameters['test_2'], parameters['factor'], parameters['actionType'], parameters['plans'])
    value = mdp_vi.update()
    test_1 = mdp_vi.best_decision(value)

    print("Final value:\n", value)
    print("Best :\n", test_1)

    mdp_vi.visualize(test_1, 'ϵ = 0.4')


if __name__ == "__main__":
    main()
