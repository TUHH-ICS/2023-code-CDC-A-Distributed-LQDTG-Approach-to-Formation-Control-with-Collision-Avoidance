[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.7789655.svg)](https://doi.org/10.5281/zenodo.7789655)
# A Distributed Linear Quadratic Discrete-Time Game Approach to Formation Control with Collision Avoidance

## General
This repository contains an implementation of the algorithms and simulations described in 
> Prima Aditya and Herbert Werner, "A Distributed Linear Quadratic Discrete-Time Game Approach to Multi-Agent Formation with Collision Avoidance" submitted for CDC 2023

## Simulation
The main code `main.m` contains everything in one file with additional functions:
- `sig.m` to compute the sigma-norm 
- `zoom_plot.m` for zooming window 
- To plot the formation without collision avoidance, go to
`unconstrained.m`

This distributed framework based on iterative-scheme of steepest descent method. In this case, we took four vertices and five edges as an example. 

## Prerequisites
The simulation code in this repository was tested in the following environment:
- *Windows 11* Pro Version 21H2
- *Matlab* 2022b
