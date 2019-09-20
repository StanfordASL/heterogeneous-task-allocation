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
import networkx as nx
import random
from task_allocation_milp import cplex_milp_centralized
from task_allocation_homogeneous import cplex_lp_homogeneous_centralized
from task_allocation_distributed import cplex_distributed_task_allocation_subroutine, distributed_task_allocation_sim
from task_allocation_utilities import plot_trajectories, video_trajectories
from datetime import datetime


def solve_intruders_problem(rows_num, cols_num, num_agent_types, agents_per_type, max_intruders_per_type, Thor, common_task_label, solver="MILP", _plot=True):

    times = range(Thor)

    # Create a NetworkX network
    print("Preparing the problem")
    print("  Graph...")
    G = create_lattice_graph(rows_num, cols_num)

    # Add rewards to it
    print("  Rewards...")
    agent_types = list(range(num_agent_types))
    task_types = list(common_task_label)+agent_types

    rewards = create_intruder_reward(
        G, task_types, times, max_intruders_per_type)

    # Create agents
    print("  Agents.")
    def r(): return random.randint(0, 255)
    agent_colors = {}

    agents = {}
    for agent_type in agent_types:
        agent_color = '#%02X%02X%02X' % (r(), r(), r())
        agent_colors[agent_type] = agent_color
        agents[agent_type] = []
        for agent_id in range(agents_per_type):
            agent_name = f'{agent_type}:{agent_id}'
            agent_initial_location = (
                random.randint(0, rows_num-1),
                random.randint(0, cols_num-1)
            )
            agents[agent_type].append(
                (agent_name, agent_initial_location, agent_color)
            )

    # Solve!
    print(f"Solve the problem with solver {solver}")
    if solver == "MILP":
        problem = cplex_milp_centralized(
            network=G,
            rewards=rewards,
            agents=agents,
            common_task_key=common_task_label,
            verbose=False,
        )

        opt_val, Xval, Yval, Zval, duals = problem.solve()
        agent_trajectories = problem.compute_trajectories()
    elif solver == "Homogeneous":
        # Only solve for the common tasks
        rewards_homogeneous = rewards[common_task_label]
        agents_homogeneous = agents[list(agents.keys())[0]]
        problem = cplex_lp_homogeneous_centralized(
            network=G,
            rewards=rewards_homogeneous,
            agents=agents_homogeneous,
            verbose=False,
        )
        opt_val, Xval, Zval, duals = problem.solve()
        agent_trajectories = problem.compute_trajectories()
        # Add back type for plotting
        agent_trajectories = {common_task_label: agent_trajectories}
        agents = {common_task_label: agents_homogeneous}
        rewards = {common_task_label: rewards_homogeneous}
        agent_colors = {common_task_label: '#123456'}

    elif solver == "Homogeneous_distributed":
        print("We start by test-calling the single-agent subroutine...")
        # Only solve for the common tasks with a single agent and a lagrangian
        rewards_homogeneous = rewards[common_task_label]
        agents_homogeneous = agents[list(agents.keys())[0]]
        agents_distributed = [agents[list(agents.keys())[0]][0]]
        lagrangian = {}
        for location in G.nodes():
            lagrangian[location] = {}
            for time in times:
                lagrangian[location][time] = random.random()
        problem = cplex_distributed_task_allocation_subroutine(
            network=G,
            rewards=rewards_homogeneous,
            agents=agents_distributed,
            lagrangian=lagrangian,
            verbose=False,
        )
        opt_val, Xval, Zval, duals = problem.solve()
        agent_trajectories = problem.compute_trajectories()
        # Add back type for plotting
        agent_trajectories = {common_task_label: agent_trajectories}
        agents = {common_task_label: agents_distributed}
        rewards = {common_task_label: rewards_homogeneous}
        agent_colors = {common_task_label: '#123456'}

        # Actually, we do try and call the full iterative scheme
        print("We now try the full iterative scheme:")
        agent_trajectories = distributed_task_allocation_sim(
            network=G,
            rewards=rewards_homogeneous,
            agents=agents_homogeneous,
            alpha=0.05,
            alpha_decay=1.,
            error_tolerance=0.01,
            reward_tolerance=0.01,
            verbose=True
        )
        print(agent_trajectories)
        agent_trajectories = {common_task_label: agent_trajectories}

    elif solver == "PTAS":
        print("Good luck, Kiril!")
        return
    elif solver == "PTAS_distributed":
        print("Good luck, Kiril!")
        return
    else:
        print(f"Solver {solver} not recognized. Supported solver types are MILP, Homogeneous, Homogeneous_distributed, PTAS, and PTAS_distributed")
        return

    # Plot
    if _plot:
        print("Plot the problem")

        plot_trajectories(G, agents, rewards, agent_trajectories)
        print("Make a movie")
        reward_colors = {}
        reward_colors[common_task_label] = '#010101'
        for agent_type, agent_color in agent_colors.items():
            reward_colors[agent_type] = agent_color

        video_time = datetime.now().strftime("%m_%d_%Y_%H-%M-%S")

        video_trajectories(
            G,
            agents,
            rewards,
            agent_trajectories,
            reward_colors,
            video_name=f"video_trajectories_{solver}_{video_time}.mp4"
        )

    return


def create_lattice_graph(rows_num, cols_num):
    G = nx.DiGraph()

    for row in range(rows_num):
        for col in range(cols_num):
            G.add_node((row, col), location=[row, col], x=col, y=row)
            # Add link down
            if row > 0:
                G.add_edge((row, col), (row-1, col))
            # Add link up
            if row < rows_num-1:
                G.add_edge((row, col), (row+1, col))
            # Add link left
            if col > 0:
                G.add_edge((row, col), (row, col-1))
            # Add link right
            if col < cols_num-1:
                G.add_edge((row, col), (row, col+1))

    # Convert to directed graph
    G = G.to_directed()
    return G


def create_random_reward(G, task_types, times):
    rewards = {}
    for task in task_types:
        rewards[task] = {}
        for location in G.nodes():
            rewards[task][location] = {}
            for time in times:
                rewards[task][location][time] = random.random()
    return rewards


def create_intruder_reward(G, task_types, times, max_intruders_per_type):
    rewards = {}
    # Initialize
    for task in task_types:
        rewards[task] = {}
        for location in G.nodes():
            rewards[task][location] = {}
            for time in times:
                rewards[task][location][time] = 0.
    # Intrude
    for task in task_types:
        intruders_number = random.randint(0, max_intruders_per_type)
        for intruder in range(intruders_number):
            starting_position = random.choice(list(G.nodes()))
            rewards[task][starting_position][times[0]] += 1.
        for time in times[1:]:
            for location in G.nodes():
                for neighbor in G.predecessors(location):
                    num_successors = float(len(list(G.successors(neighbor))))
                    rewards[task][location][time] += rewards[task][neighbor][time-1]/num_successors
    return rewards


if __name__ == "__main__":
    # Problem parameters

    # We live in a lattice with this many rows and columns
    rows_num = 10
    cols_num = 8

    # These are the agent types
    common_task_label = 'C'
    num_agent_types = 4
    # And there are this many agents per type
    agents_per_type = 4
    max_intruders_per_type = 3

    # And this is the time horizon
    Thor = 15

    # Random seed for problem generation
    seed = 1

    # Solve the problem with a MILP
    random.seed(seed)
    solve_intruders_problem(rows_num, cols_num, num_agent_types,
                            agents_per_type, max_intruders_per_type, Thor, common_task_label, solver="MILP")

    # # Solve a piece of the problem with the homogeneous solver
    random.seed(seed)
    solve_intruders_problem(rows_num, cols_num, num_agent_types,
                            agents_per_type, max_intruders_per_type, Thor, common_task_label, solver="Homogeneous")

    # Solve a piece of the problem with the _distributed_ homogeneous solver
    random.seed(seed)
    solve_intruders_problem(rows_num, cols_num, num_agent_types,
                            agents_per_type, max_intruders_per_type, Thor, common_task_label, solver="Homogeneous_distributed")

    # Solve the full problem with the polynomial-time approximation scheme
    random.seed(seed)
    solve_intruders_problem(rows_num, cols_num, num_agent_types,
                            agents_per_type, max_intruders_per_type, Thor, common_task_label, solver="PTAS")

    # Solve the full problem with the _distributed_ polynomial-time approximation scheme
    random.seed(seed)
    solve_intruders_problem(rows_num, cols_num, num_agent_types,
                            agents_per_type, max_intruders_per_type, Thor, common_task_label, solver="PTAS_distributed")
