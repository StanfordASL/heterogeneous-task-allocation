import networkx as nx
import random
import time
import argparse
import sys
import csv

# Rewards structure:
# [task_type] [graph_vertex] [time]
def create_intruder_reward(G, fleets_num, times, max_intruders_per_type):
    rewards = {}
    # Initialize
    for task in range(fleets_num+1):
        rewards[task] = {}
        for time in times:
            rewards[task][time] = {}
            for location in G.nodes():
                rewards[task][time][location] = 0.
    # Intruders
    for task in range(fleets_num+1):
        #intruders_number = random.randint(0, max_intruders_per_type)
        intruders_number = max_intruders_per_type
        for intruder in range(intruders_number):
            starting_position = random.choice(list(G.nodes()))
            rewards[task][0][starting_position] -= 1.
        for time in times[1:]:
            for location in G.nodes():
                for neighbor in G.predecessors(location):
                    num_successors = float(len(list(G.successors(neighbor))))
                    rewards[task][time][location] += rewards[task][time-1][neighbor]/num_successors
    return rewards


def generate_problem(dimension, fleets_num, agents_per_fleet, max_intruders_per_type, Thor):
    times = range(Thor)

    # graph
    G = nx.grid_2d_graph(dimension, dimension).to_directed()
    rewards = create_intruder_reward(G, fleets_num, times, max_intruders_per_type)

    initial_locations = {}
    for f in range(fleets_num):
        initial_locations[f] = []
        for a in range(agents_per_fleet):
            start = random.choice(range(dimension**2))
            initial_locations[f].append(start)

    return G, rewards, initial_locations;

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('-d', action='store', dest='dimension', help='Dimension of grid graph', default=3)
    parser.add_argument('-f', action='store', dest='fleets', help='Number of fleets', default=2)
    parser.add_argument('-t', action='store', dest='time', help='Time horizon', default=3)
    parser.add_argument('-a', action='store', dest='agents', help='Number of agents per fleet', default=2)
    parser.add_argument('-i', action='store', dest='intruders', help='Number of intruders per task', default=3)
    parser.add_argument('-s', action='store', dest='seed', help='Seed for random generator', default=3)
    parser.add_argument('-path', action='store',dest='path', help='Directory for output files', default='.')

    results = parser.parse_args()

    # generate information
    ######################
    dimension = int(results.dimension)
    
    # agents and fleets
    fleets_num = int(results.fleets)
    agents_per_fleet = int(results.agents)
    max_intruders_per_fleet = int(results.intruders)

    # time horizon 
    Thor = int(results.time)

    # Random seed for problem generation
    seed = int(results.seed)

    path = results.path

    G, rewards, init = generate_problem(dimension, fleets_num, agents_per_fleet, max_intruders_per_fleet, Thor)

    # write graph
    with open('edges.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',',
                                quotechar='|', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(['origin','destination'])
        writer.writerow([0,dimension**2])

        for e in G.edges():
            o = dimension * e[0][0] + e[0][1]
            d = dimension * e[1][0] + e[1][1]
            writer.writerow([o,d])

    # write rewards
    with open('rewards.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(['task_type','time','vertex','value'])
        writer.writerow([fleets_num+1,Thor,dimension**2,0])
        for task in range(fleets_num+1):
            for time in range(Thor):
                for location in G.nodes():
                    v = dimension*location[0]+location[1]
                    value = rewards[task][time][location]
                    writer.writerow([task,time,v,value])
    
    # write start poisitions
    with open('init.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(['fleet','vertex'])
        for fleet in range(fleets_num):
            for val in init[fleet]:
                writer.writerow([fleet,val])       

   
