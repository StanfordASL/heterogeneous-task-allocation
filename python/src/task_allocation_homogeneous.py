"""
 Copyright 2019 by California Institute of Technology.  ALL RIGHTS RESERVED.
 United  States  Government  sponsorship  acknowledged.   Any commercial use
 must   be  negotiated  with  the  Office  of  Technology  Transfer  at  the
 California Institute of Technology.

 This software may be subject to  U.S. export control laws  and regulations.
 By accepting this document,  the user agrees to comply  with all applicable
 U.S. export laws and regulations.  User  has the responsibility  to  obtain
 export  licenses,  or  other  export  authority  as may be required  before
 exporting  such  information  to  foreign  countries or providing access to
 foreign persons.

 This  software  is a copy  and  may not be current.  The latest  version is
 maintained by and may be obtained from the Mobility  and  Robotics  Sytstem
 Section (347) at the Jet  Propulsion  Laboratory.   Suggestions and patches
 are welcome and should be sent to the software's maintainer.

"""
try:
    import cplex
except:
    raise ImportWarning(
        "WARNING: CPLEX not available. You will probably want to use another module")


class abstract_lp_homogeneous_centralized(object):
    '''
    Implementation of Problem 2: homogeneous (single-fleet) task allocation
    '''

    #def __init__(self, network, rewards, agents, verbose=False, linear_program=True):
    def __init__(self, network, rewards, agents, verbose=False, linear_program=True):
        '''
        Initialize the optimization problem.
        @ param network A networkx network containing the locations the agents can visit and their connections.
        @ param rewards A nested dictionary containing the task rewards. The dictionary has two keys.
                        The first key denotes the location of the task.
                        The second key denotes the time at which the task is performed.
                        rewards[location][time] is the reward for performing task(s)
                        at location location at time time.
        @ param agents A list containing the description of the agents.
                       Each entry is a tuple of agent descriptions.
                       Each agent's description is a tuple with two entries, the agent's name and the initial location of the agent.
        @ param verbose (Boolean) sets the verbosity of the problem
        @ param linear_program (Boolean) sets whether to solve the problem as a MILP or a LP
        The function asserts that the "location" and "task_type" keys in rewards and initial_locations be the same.
        '''
        self.network = network
        self.rewards = rewards
        self.agents = agents
        self.verbose = verbose
        self.linear_program = linear_program
        self.solved = False
        
        self._verbprint("  Initializing")
        self.problem = self.initialize_problem()

        # Infer locations and check for consistency
        locations = list(network.nodes)
        assert locations == list(rewards.keys(
        )), "ERROR: locations do not match (rewards)"
        for agent in agents:
            assert agent[1] in locations,  "ERROR: locations for agent {} do not match (initial_locations)".format(
                agent)
        assert list(rewards.keys(
        )) == locations, "ERROR: locations do not match (task_types)"

        # Infer time horizon and check for consistency
        assert len(locations)
        time_horizon = list(rewards[locations[0]].keys())
        self.time_horizon = time_horizon
        for location in locations:
            assert time_horizon == list(
                rewards[location].keys()), "ERROR: inconsistent time horizon"

        # Create variables
        # X: transition variables
        self._verbprint("  Creating variables")
        X = {}
        agent_type = 'C'
        self.agent_type = agent_type

        X[agent_type] = {}
        for agent in agents:
            X[agent_type][agent] = {}
            for start_location in locations:
                X[agent_type][agent][start_location] = {}
                for end_location in network.successors(start_location):
                    X[agent_type][agent][start_location][end_location] = {}
                    for time in time_horizon:
                        X[agent_type][agent][start_location][end_location][time] = self.create_variables(
                            names="X[{}][{}][{}][{}][{}]".format(
                                agent_type, agent, start_location, end_location, time),
                            rewards=0,
                            types='C',
                            lbs=0,
                            ubs=1,
                        )[0]

        # Z: do I perform a private task?
        Z = {}
        Z[agent_type] = {}
        for agent in agents:
            Z[agent_type][agent] = {}
            for location in locations:
                Z[agent_type][agent][location] = {}
                for time in time_horizon:
                    Z[agent_type][agent][location][time] = self.create_variables(
                        names="Z[{}][{}][{}][{}]".format(
                            agent_type, agent, location, time),
                        rewards=rewards[location][time],
                        types='C',
                        lbs=0,
                        ubs=1,
                    )[0]

        # Constraints
        self._verbprint("  Creating constraints")
        # 1d: initial location
        self._verbprint("    Initial Location")
        initial_location_constraints = {}
        for start_location in locations:
            initial_location_constraints[start_location] = {}
            for agent in agents:
                cvars = [X[agent_type][agent][start_location][end_location][time_horizon[0]]
                         for end_location in network.successors(start_location)]
                ccoeffs = [
                    1 for end_location in network.successors(start_location)]
                if start_location == agent[1]:
                    rhs = 1.
                else:
                    rhs = 0.
                initial_location_constraints[start_location][agent] = self.add_constraint(
                    coefficients=ccoeffs,
                    variables=cvars,
                    rhs=rhs,
                    senses='E',
                )[0]

        # 1e: flow continuity
        self._verbprint("    Flow Continuity")
        flow_continuity_constraints = {}
        for location in locations:
            flow_continuity_constraints[location] = {}
            for agent in agents:
                flow_continuity_constraints[location][agent] = {}
                for time_index, time in enumerate(time_horizon[:-1]):
                    cvars = [X[agent_type][agent][start_location][location][time_horizon[time_index]]
                             for start_location in network.predecessors(location)]
                    ccoeffs = [1
                               for start_location in network.predecessors(location)]
                    cvars += [X[agent_type][agent][location][end_location][time_horizon[time_index+1]]
                              for end_location in network.successors(location)]
                    ccoeffs += [-1
                                for end_location in network.successors(location)]
                    flow_continuity_constraints[location][agent][time] = self.add_constraint(
                        coefficients=ccoeffs,
                        variables=cvars,
                        rhs=0,
                        senses='E',
                    )

        # 1k: can only do a task if I am present (1/2, private)
        self._verbprint("    Private Tasks: presence 1/2")
        task_presence_1_private_constraints = {}
        for location in locations:
            task_presence_1_private_constraints[location] = {}
            task_presence_1_private_constraints[location] = {}
            for agent in agents:
                task_presence_1_private_constraints[location][agent] = {
                }
                for time in time_horizon:
                    cvars = [Z[agent_type][agent][location][time]]
                    ccoeffs = [1]
                    cvars += [X[agent_type][agent][location][end_location][time]
                              for end_location in network.successors(location)]
                    ccoeffs += [-1
                                for end_location in network.successors(location)]
                    task_presence_1_private_constraints[location][agent][time] = self.add_constraint(
                        coefficients=ccoeffs,
                        variables=cvars,
                        rhs=0,
                        senses='L',
                    )

        # 1l: can only do a task if I am present (2/2, redundant, private)
        self._verbprint("    Private Tasks: presence 2/2")
        task_presence_2_private_constraints = {}
        for location in locations:
            task_presence_2_private_constraints[location] = {}
            for agent in agents:
                cvars = [Z[agent_type][agent][location][time_horizon[-1]]]
                ccoeffs = [1]
                cvars += [X[agent_type][agent][start_location][location][time_horizon[-2]]
                          for start_location in network.predecessors(location)]
                ccoeffs += [-1
                            for start_location in network.predecessors(location)]
                task_presence_2_private_constraints[location][agent] = self.add_constraint(
                    coefficients=ccoeffs,
                    variables=cvars,
                    rhs=0,
                    senses='L',
                )

        # 1m: private tasks are only done once
        self._verbprint("    Private Tasks: completion")
        task_completion_private_constraints = {}
        for location in locations:
            task_completion_private_constraints[location] = {}
            for time in time_horizon:
                task_completion_private_constraints[location][time] = {}
                cvars = [Z[agent_type][agent][location][time]
                         for agent in agents]
                ccoeffs = [1
                           for agent in agents]

                task_completion_private_constraints[location][time][agent_type] = self.add_constraint(
                    coefficients=ccoeffs,
                    variables=cvars,
                    rhs=1,
                    senses='L',
                )

    def compute_trajectories(self):
        assert self.solved, "ERROR: cannot compute trajectories before solving problem. Call self.solve()"
        agent_trajectories = {}
        agent_type = self.agent_type
        for agent in self.agents:
            trajectory = []
            locs = []
            for time in self.time_horizon:
                for start_location in self.network.nodes():
                    for end_location in self.network.successors(start_location):
                        if self.Xval[agent_type][agent][start_location][end_location][time] != 0:
                            if abs(self.Xval[agent_type][agent][start_location][end_location][time] - 1) > 1e-3:
                                print("WARNING: non-integer solution")
                                print(self.Xval[agent_type][agent][start_location][end_location][time])
                                import pdb
                                pdb.set_trace()
                            trajectory.append(
                                (time, (start_location, end_location)))
                            if len(locs) == 0:
                                locs.append(start_location)
                            else:
                                assert locs[-1] == start_location, "ERROR: discontinuous trajectory"

                            locs.append(end_location)

            agent_trajectories[agent] = {
                'trajectory': trajectory, 'locations': locs}
        return agent_trajectories

    def solve(self):
        raise NotImplementedError

    def initialize_problem(self):
        raise NotImplementedError

    def create_variables(self, names, rewards, types, lbs, ubs, quadratic_costs=None):
        raise NotImplementedError

    def add_constraint(self, coefficients, variables, rhs, senses):
        raise NotImplementedError

    def _verbprint(self, msg):
        if self.verbose:
            print(msg)


class cplex_lp_homogeneous_centralized(abstract_lp_homogeneous_centralized):
    def initialize_problem(self):
        problem = cplex.Cplex()
        problem.objective.set_sense(problem.objective.sense.maximize)
        if self.linear_program is True:
            problem.set_problem_type(problem.problem_type.LP)
            problem.parameters.barrier.crossover = 1
        else:
            problem.set_problem_type(problem.problem_type.MILP)
        return problem

    def create_variables(self, names, rewards, types, lbs, ubs, quadratic_costs=None):
        start_num = self.problem.variables.get_num()

        def _listify(entry):
            if type(entry) is not list:
                return [entry]
            else:
                return entry

        rewards = _listify(rewards)
        names = _listify(names)
        types = _listify(types)
        lbs = _listify(lbs)
        ubs = _listify(ubs)

        self.problem.variables.add(
            obj=rewards,
            lb=lbs,
            ub=ubs,
            types=types,
            # ['X[{},{},{}]'.format(i, m, t) if t >= 0 else None for i in AgentsList for m in TasksList for t in range(Thor - AgentCapabilities.ComputationTime[m][i])])
            names=names,
        )
        end_num = self.problem.variables.get_num()

        if quadratic_costs is not None:
            for ix, var in enumerate(range(start_num, end_num)):
                self.problem.objective.set_quadratic_coefficients(
                    var, var, quadratic_costs[ix])
        return [el for el in range(start_num, end_num)]

    def add_constraint(self, coefficients, variables, rhs, senses):
        def _listify(entry):
            if type(entry) is not list:
                return [entry]
            else:
                return entry
        start_num = self.problem.linear_constraints.get_num()
        rhs = _listify(rhs)
        senses = _listify(senses)
        self.problem.linear_constraints.add(
            lin_expr=[cplex.SparsePair(variables, coefficients)],
            senses=senses,
            rhs=rhs
        )
        end_num = self.problem.linear_constraints.get_num()
        return [el for el in range(start_num, end_num)]

    def solve(self):
        self._verbprint("  Solving")
        if self.linear_program:
            self.problem.set_problem_type(self.problem.problem_type.LP)
            assert self.problem.get_problem_type(
            ) == self.problem.problem_type.LP, "ERROR: could not convert problem to QP"
        self.problem.parameters.mip.tolerances.mipgap.set(0.05)

        if self.verbose is False:
            self.problem.set_log_stream(None)
            self.problem.set_warning_stream(None)
            self.problem.set_results_stream(None)
        self.problem.solve()
        self._verbprint('Solution status:                   %d' %
                        self.problem.solution.get_status())
        if self.problem.solution.is_primal_feasible():
            opt_val = self.problem.solution.get_objective_value()
            self._verbprint(
                'Optimal value:                     {}'.format(opt_val))
            # values = self.problem.solution.get_values()
            # Solution in MIP Start format
            varNames = self.problem.variables.get_names()
            varValues = self.problem.solution.get_values(varNames)
            self.CPLEXSol = [varNames, varValues]
        else:
            opt_val = None
            varNames = self.problem.variables.get_names()
            varValues = [0. for var in varNames]

        self._verbprint("  Rearranging variables")
        agents = self.agents
        network = self.network
        locations = network.nodes()
        time_horizon = self.time_horizon

        # Process variables for human consumption
        Xval = {}
        agent_type = self.agent_type
        Xval[agent_type] = {}
        for agent in agents:
            Xval[agent_type][agent] = {}
            for start_location in locations:
                Xval[agent_type][agent][start_location] = {}
                for end_location in network.successors(start_location):
                    Xval[agent_type][agent][start_location][end_location] = {}
                    for time in time_horizon:
                        X_name = "X[{}][{}][{}][{}][{}]".format(
                            agent_type, agent, start_location, end_location, time)
                        # WIll raise an error if name is not present
                        Xval[agent_type][agent][start_location][end_location][time] = self.problem.solution.get_values(
                            X_name)

        Zval = {}
        Zval[agent_type] = {}
        for agent in agents:
            Zval[agent_type][agent] = {}
            for location in locations:
                Zval[agent_type][agent][location] = {}
                for time in time_horizon:
                    Z_name = "Z[{}][{}][{}][{}]".format(
                        agent_type, agent, location, time)
                    Zval[agent_type][agent][location][time] = self.problem.solution.get_values(
                        Z_name)

        initial_location_duals = None
        flow_continuity_duals = None
        task_presence_1_public_duals = None
        task_presence_2_public_duals = None
        task_completion_public_duals = None
        task_presence_1_private_duals = None
        task_presence_2_private_duals = None
        task_completion_private_duals = None
        duals = {
            'initial_location_duals': initial_location_duals,
            'flow_continuity_duals': flow_continuity_duals,
            'task_presence_1_public_duals': task_presence_1_public_duals,
            'task_presence_2_public_duals': task_presence_2_public_duals,
            'task_completion_public_duals': task_completion_public_duals,
            'task_presence_1_private_duals': task_presence_1_private_duals,
            'task_presence_2_private_duals': task_presence_2_private_duals,
            'task_completion_private_duals': task_completion_private_duals,
        }

        self.opt_val = opt_val
        self.Xval = Xval
        self.Zval = Zval
        self.solved = True
        return opt_val, Xval, Zval, duals
