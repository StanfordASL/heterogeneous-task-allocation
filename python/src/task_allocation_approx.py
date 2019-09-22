from task_allocation_homogeneous import cplex_lp_homogeneous_centralized

class approx_centralized(object):
    '''
    Implementation of Centralized PFSF (Algorithm 1)
    '''

    def __init__(self, network, rewards, agents, common_task_key=0, verbose=False):
        '''
        Initialize the optimization problem.
        @ param network A networkx network containing the locations the agents can visit and their connections.
        @ param rewards A nested dictionary containing the task rewards. The dictionary has three keys.
                        The first key denotes the type of task considered.
                          Task type 0 is the "common" task type and can be performed by any agent.
                          Task types 1 through T can only be performed by agents of the corresponding type.
                        The second key denotes the location of the task.
                        The third key denotes the time at which the task is performed.
                        rewards[task_type][location][time] is the reward for performing task(s) of type
                          task_type at location location at time time.
        @ param agents A nested dictionary containing the description of the agents.
                       The dictionary has a key denoting the type of agent.
                       Each entry is a tuple of agent descriptions.
                       Each agent's description is a tuple with two entries, the agent's name and the initial location of the agent.
        @ param common_task_key=0 the key identifying the "common" tasks
        @ param verbose (Boolean) sets the verbosity of the problem
       
        The function asserts that the "location" and "task_type" keys in rewards and initial_locations be the same.
        '''

        self.network = network
        self.rewards = rewards
        self.agents = agents
        self.common_task_key = common_task_key
        self.verbose = verbose

        self._verbprint("  Initializing")
        #self.problem = self.initialize_problem()

        # Check that rewards has at least one task type
        assert len(rewards.keys())

        # Infer task types and check for consistency
        task_types = list(rewards.keys())
        agent_types = list(task_types)
        agent_types.remove(common_task_key)

        assert list(agents.keys()
                    ) == agent_types, f"ERROR: agent types do not match ({list(agents.keys())} vs {agent_types})"

        # Infer locations and check for consistency
        locations = list(network.nodes)
        assert locations == list(rewards[common_task_key].keys(
        )), "ERROR: locations for agent {} do not match (rewards)".format(agent_types)
        for agent_type in agent_types:
            for agent in agents[agent_type]:
                assert agent[1] in locations,  "ERROR: locations for agent {} do not match (initial_locations)".format(
                    agent_type)
        for task_type in task_types:
            assert list(rewards[task_type].keys(
            )) == locations, "ERROR: locations for agent {} do not match (task_types)".format(agent_type)

        # Infer time horizon and check for consistency
        assert len(locations)
        time_horizon = list(rewards[task_types[0]][locations[0]].keys())
        self.time_horizon = time_horizon
        for task_type in task_types:
            for location in locations:
                assert time_horizon == list(
                    rewards[task_type][location].keys()), "ERROR: inconsistent time horizon"

        self.locations = locations
        self.task_types = task_types
        self.agent_types = agent_types
        self.agent_num = len(agent_types)

        
    def _verbprint(self, msg):
        if self.verbose:
            print(msg)

    def no_repetition_Y(self, Y, Z):
        for time in self.time_horizon:
            for location in self.locations:
                breaker = False
                for agent_type in self.agents.keys():
                    for agent in self.agents[agent_type]:
                        if Z[agent_type][agent][location][time] == 1:
                            Y[agent_type][agent][location][time] = 1
                            breaker = True
                            break
                    if breaker:
                        break

        return Y


    # compute gained reward for the given assignment
    def get_val(self, Y,Z):
        val = 0
        for agent_type in self.agents.keys():
            for agent in self.agents[agent_type]:
                for location in self.locations:
                    for time in self.time_horizon:
                        val += Z[agent_type][agent][location][time] * self.rewards[agent_type][location][time]
                        val += Y[agent_type][agent][location][time] * self.rewards[self.common_task_key][location][time]

        return val

    def private_first(self):
        X, Y, Z = self.get_xyz()

        # prepare rewards with T^f = T^f + T^0 \cdot F^-1
        rewards_hat = self.rewards
        for task in self.task_types:
            for location in self.network.nodes():
                for time in self.time_horizon:
                    if task != self.common_task_key:
                        rewards_hat[task][location][time] += self.rewards[self.common_task_key][location][time] / self.agent_num

        # solve homogenous problems
        for f in self.agent_types:
            rewards_f = rewards_hat[f]
            agents_f = self.agents[f]
            problem_f = cplex_lp_homogeneous_centralized(
                network = self.network,
                rewards = rewards_f,
                agents = agents_f,
                verbose = False
            )
            opt_f, X_f, Z_f, duals = problem_f.solve()

            # update global solution of X,Z
            X[f] = X_f
            Z[f] = Z_f[self.common_task_key]

        Y = self.no_repetition_Y(Y,Z)
            
        # compute gained reward
        val = self.get_val(Y,Z)
        return val, X, Y, Z

    def shared_first(self):
        X, Y, Z = self.get_xyz()

        # assign shared task
        rewards_shared = self.rewards[self.common_task_key]
        agents_shared = []
        for agent_type in self.agents.keys():
            for agent in self.agents[agent_type]:
                agents_shared.append(agent)

        problem_shared = cplex_lp_homogeneous_centralized(
            network=self.network,
            rewards=rewards_shared,
            agents=agents_shared,
            verbose=False
        )

        v_s, X_s, Y_s_temp, duals = problem_shared.solve()

        # transform Y_s_temp to a partition according to agent types
        Y_s = Y

        for agent in agents_shared:
            for location in self.locations:
                for time in self.time_horizon:
                    agent_type = int(str.split(agent[0],':')[0])
                    Y_s[agent_type][agent][location][time] =                   Y_s_temp[list(Y_s_temp.keys())[0]][agent][location][time]

        # prepare rewards with T^f = T^f + T^0 \sum y^q
        rewards_bar = self.rewards
        for agent_type in self.agents.keys():
            for location in self.network.nodes():
                for time in self.time_horizon:
                    for agent in self.agents[agent_type]: 
                        rewards_bar[agent_type][location][time] +=  self.rewards[self.common_task_key][location][time] * Y_s[agent_type][agent][location][time]

        # solve for updated private tasks
        for f in self.agent_types:
            rewards_f = rewards_bar[f]
            agents_f = self.agents[f]
            problem_f = cplex_lp_homogeneous_centralized(
                network= self.network,
                rewards= rewards_f,
                agents= agents_f,
                verbose= False
            )
            opt_f, X_f, Z_f, duals = problem_f.solve()

            # update global solution of X,Z
            X[f] = X_f
            Z[f] = Z_f[self.common_task_key]

        Y = self.no_repetition_Y(Y,Z)

        # compute gained reward
        val = self.get_val(Y,Z)
        return val, X, Y, Z

    def solve(self):
        v_p, X_p, Y_p, Z_p = self.private_first()
        v_s, X_s, Y_s, Z_s = self.shared_first()

        if v_p > v_s:
            return v_p, X_p, Y_p, Z_p
        else:
            return v_s, X_s, Y_s, Z_s

    def get_xyz(self):
        # Create variables
        # X: transition variables
        X = {}
        for agent_type in self.agents.keys():
            X[agent_type] = {}
            for agent in self.agents[agent_type]:
                X[agent_type][agent] = {}
                for start_location in self.locations:
                    X[agent_type][agent][start_location] = {}
                    for end_location in self.network.successors(start_location):
                        X[agent_type][agent][start_location][end_location] = {}
                        for time in self.time_horizon:
                            X[agent_type][agent][start_location][end_location][time] = 0

        # Y: do I perform a shared task?
        Y = {}
        for agent_type in self.agents.keys():
            Y[agent_type] = {}
            for agent in self.agents[agent_type]:
                Y[agent_type][agent] = {}
                for location in self.locations:
                    Y[agent_type][agent][location] = {}
                    for time in self.time_horizon:
                        Y[agent_type][agent][location][time] = 0

        # Z: do I perform a private task?
        Z = {}
        for agent_type in self.agents.keys():
            Z[agent_type] = {}
            for agent in self.agents[agent_type]:
                Z[agent_type][agent] = {}
                for location in self.locations:
                    Z[agent_type][agent][location] = {}
                    for time in self.time_horizon:
                        Z[agent_type][agent][location][time] = 0

        return X, Y, Z
