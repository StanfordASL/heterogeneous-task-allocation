# Distributed Task Allocation - in Python

![Intruders chasing bad guys](img/python_intruders.gif)

This repository contains a set of Python functions implementing the distributed, heterogeneous, predictive task allocation algorithm by Solovey et al.
The repository contains four files:

## MILP implementation

The functions in `task_allocation_milp.py` implement a centralized MILP solution to the heterogeneous, predictive task allocation problem (Problem 1 in the paper).

## Homogeneous LP implementation

The functions in `task_allocation_homogeneous.py` implement a centralized solution to the (totally unimodular) problem of homogeneous task allocation (Problem 2 in the paper).

## Distributed homogeneous LP implementation

The functions in `task_allocation_distributed.py` implement the dual decomposition distributed solution to the (totally unimodular) problem of homogeneous task allocation (Problem 4 in the paper). At the time of writing, only the optimization subroutine is implemented.

## Utilities

The file `task_allocation_utilities.py` contains functions for plotting agents' trajectories.

## Examples

The file `intruders_chasing.py` creates an example problem with agents chasing intruders on a lattice and calls each of the implemented solvers above.
