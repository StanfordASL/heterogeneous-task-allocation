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
import time
import random
import csv
import argparse
import networkx as nx
import numpy as np
from task_allocation_milp import cplex_milp_centralized
from datetime import datetime


def importer(path):
    init_file = path + "init.csv"
    edge_file = path + "edges.csv"
    reward_file = path + "rewards.csv"

    graph = nx.DiGraph()
    
    with open(edge_file, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='|')

        for row in list(reader)[2:]:
            o = int(row[0])
            d = int(row[1])
            graph.add_edge(o,d)
    
    with open(reward_file, newline='') as csvfile:
        reader_rewards = list(csv.reader(csvfile, delimiter=',', quotechar='|'))
        types = int(reader_rewards[1][0])
        fleets = types - 1
        times = range(int(reader_rewards[1][1]))
        vertices = int(reader_rewards[1][2])

        rewards = {}
        for task in range(types):
            rewards[task] = {}
            for location in graph.nodes():
                rewards[task][int(location)] = {}
                for time in times:
                    rewards[task][int(location)][time] = 0.
        for row in reader_rewards[2:]:
            ty = int(row[0])
            ti = int(row[1])
            ve = int(row[2])

            rewards[ty][ve][ti] = float(row[3])

    init = [ [ ] for i in range(fleets)]
    with open(init_file, newline='') as csvfile:
        reader_init = list(csv.reader(csvfile, delimiter=',', quotechar='|'))
        for row in reader_init[1:]:
            f = int(row[0])
            v = int(row[1])
            init[f].append(v)

    agents = {}
    for agent_type in range(fleets):
        agents[agent_type] = []

        agent_id = 0
        for v in init[agent_type]:
            agent_name = f'{agent_type}:{agent_id}'
            agent_initial_location = v
            agents[agent_type].append(
                (agent_name, agent_initial_location)
            )

            agent_id += 1

    return graph, rewards, agents

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', action='store', dest='path', help='path to read csv files from', default=".")

    path = parser.parse_args().path
    graph, rewards, agents = importer(path)
    
    problem = cplex_milp_centralized(
            network=graph,
            rewards=rewards,
            agents=agents,
            common_task_key=len(rewards)-1,
            verbose=False,
            linear_program=False
        )

    #start = time.time()
        
    opt_val, Xval, Yval, Zval, duals = problem.solve()
    print(opt_val)
    #end = time.time()

    #elaspsed = end - start
    #print(elaspsed)
        
