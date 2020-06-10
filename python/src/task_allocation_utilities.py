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
import matplotlib.pyplot as plt
import numpy as np
import random
import os
import subprocess
from datetime import datetime


def plot_trajectories(network, agents, rewards, agent_trajectories, plot_name=None):

    if plot_name is None:
        plot_name = "plot.png"

    task_types = list(rewards.keys())
    locations = list(network.nodes)
    time_horizon = list(rewards[task_types[0]][locations[0]].keys())

    plt.clf()
    nx.draw_networkx(
        network,
        pos=nx.get_node_attributes(network, 'location'),
        with_labels=False,
        node_color='k',
        alpha=.2,
        node_size=100,
    )
    for agent_type in agent_trajectories.keys():
        for agent in agent_trajectories[agent_type]:
            if len(agent) > 2:
                agent_color = agent[2]
            else:
                def r(): return random.randint(0, 255)
                agent_color = '#%02X%02X%02X' % (r(), r(), r())
            times, edges = zip(
                *agent_trajectories[agent_type][agent]['trajectory'])
            nodes_traj = agent_trajectories[agent_type][agent]['locations']
            nx.draw_networkx_edges(
                network,
                pos=nx.get_node_attributes(network, 'location'),
                edgelist=edges,
                edge_color=agent_color,
                width=3.,
                # edge_cmap=plt.cm.Blues
            )
            nx.draw_networkx_nodes(
                network,
                pos=nx.get_node_attributes(network, 'location'),
                nodelist=nodes_traj,
                node_color=agent_color,
                node_size=300,
                alpha=0.2+0.8*np.array(
                    range(len(nodes_traj)),
                    dtype=float,
                )/len(nodes_traj)
                # cmap=plt.cm.Blues
            )
    plt.savefig(plot_name)


def video_trajectories(network, agents, rewards, agent_trajectories, reward_colors=None, video_name=None):
    task_types = list(rewards.keys())
    locations = list(network.nodes)
    time_horizon = list(rewards[task_types[0]][locations[0]].keys())

    if reward_colors is None:
        reward_colors = {}
        for reward_type in rewards.keys():
            def r(): return random.randint(0, 255)
            reward_colors[reward_type] = '#%02X%02X%02X' % (r(), r(), r())
    plt.clf()
    agent_colors = {}

    file_names = []

    for agent_type in agent_trajectories.keys():
        agent_colors[agent_type] = {}
        for agent in agent_trajectories[agent_type]:
            if len(agent) > 2:
                agent_color = agent[2]
            else:
                def r(): return random.randint(0, 255)
                agent_color = '#%02X%02X%02X' % (r(), r(), r())
            agent_colors[agent_type][agent] = agent_color

    # Reward statistics
    min_reward = np.inf
    max_reward = -np.inf
    for reward_type, rewards_by_type in rewards.items():
        for _loc, rewards_by_location in rewards_by_type.items():
            for _time, reward in rewards_by_location.items():
                min_reward = np.minimum(min_reward, reward)
                max_reward = np.maximum(max_reward, reward)

    # For every time step
    for time_index, time in enumerate(time_horizon):
        file_name = '_tmp%04d.png' % time_index
        file_names.append(file_name)
        plt.clf()
        # Plot background network
        nx.draw_networkx(
            network,
            pos=nx.get_node_attributes(network, 'location'),
            with_labels=False,
            node_color='k',
            alpha=.1,
            node_size=1,
        )
        # Plot reward
        for reward_type in rewards.keys():
            # if reward_type != list(rewards.keys())[0] and reward_type != list(rewards.keys())[1]:
            #     continue
            #location_rewards = {}
            nodelist = []
            rewardlist = []
            for loc in rewards[reward_type].keys():
                rew = rewards[reward_type][loc][time]
                normlized_reward = float(
                    rew-min_reward)/float(max_reward-min_reward)
                #location_rewards[loc] = rew
                nodelist.append(loc)
                rewardlist.append(normlized_reward)
            nx.draw_networkx_nodes(
                network,
                pos=nx.get_node_attributes(network, 'location'),
                nodelist=nodelist,
                node_color=reward_colors[reward_type],
                node_size=300,
                alpha=rewardlist,
                # alpha=0.2+0.8*np.array(
                #     range(len(nodes_traj)),
                #     dtype=float,
                # )/len(nodes_traj)
                # cmap=plt.cm.Blues
            )

        # Plot every agent
        for agent_type in agent_trajectories.keys():
            for agent in agent_trajectories[agent_type]:
                agent_color = agent_colors[agent_type][agent]
                for ttime_and_tedge in agent_trajectories[agent_type][agent]['trajectory']:
                    ttime = ttime_and_tedge[0]
                    tedge = ttime_and_tedge[1]
                    if ttime != time:
                        continue

                    nx.draw_networkx_edges(
                        network,
                        pos=nx.get_node_attributes(network, 'location'),
                        edgelist=[tedge],
                        edge_color=agent_color,
                        width=3.,
                        # edge_cmap=plt.cm.Blues
                    )

                    nx.draw_networkx_nodes(
                        network,
                        pos=nx.get_node_attributes(network, 'location'),
                        nodelist=[tedge[0]],
                        node_color=agent_color,
                        node_size=150,
                        node_shape="D",
                        # alpha=0.2+0.8*np.array(
                        #     range(len(nodes_traj)),
                        #     dtype=float,
                        # )/len(nodes_traj)
                        # cmap=plt.cm.Blues
                    )

        plt.title(f"t={time}")
        plt.savefig(file_name, format="PNG")
    # Assemble in an animation
    video_time = datetime.now().strftime("%m_%d_%Y_%H-%M-%S")
    if video_name is None:
        video_name = f"video_trajectories_{video_time}.mp4"
    if len(video_name) < 3 or video_name[-4:] != '.mp4':
        video_name += '.mp4'
    subprocess.call([
        "ffmpeg",
        "-framerate", "1",
        "-i", "_tmp%04d.png",
        "-vcodec", "libx264",
        #"-vsync", "vfr",
        #"-crf", "25",
        "-crf", "10",
        #"-preset", "ultraslow",
        "-pix_fmt", "yuv420p",
        "-r", "24",
        video_name])

    # Cleanup
    for fname in file_names:
        os.remove(fname)
