# TaskGraphs

<!-- [![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://kylejbrown17.github.io/TaskGraphs.jl/stable) -->
[![Dev](https://img.shields.io/badge/docs-dev-blue.svg)](https://kylejbrown17.github.io/TaskGraphs.jl/dev)
[![Build Status](https://github.com/kylejbrown17/TaskGraphs.jl/workflows/CI/badge.svg)](https://github.com/kylejbrown17/TaskGraphs.jl/actions)

A Julia package for modeling and solving precedence constrained multi-agent task assignment and path finding (PC-TAPF) problems.

## Installation

Enter the Julia REPL
```Bash
$ julia
```
Type "]" to enter Pkg mode
```Julia
julia> ]
```
Add the dependent repository GraphUtil with the command `add https://github.com/sisl/GraphUtils.jl.git`
```Julia
pkg> add https://github.com/kylejbrown17/GraphUtils.jl.git
```
Add the dependent repository CRCBS with the command `add add https://github.com/sisl/CRCBS.jl.git`
```Julia
pkg> add add https://github.com/kylejbrown17/CRCBS.jl.git
```
Add the repository with the command `add https://github.com/kylejbrown17/TaskGraphs.jl.git`
```Julia
pkg> add https://github.com/kylejbrown17/TaskGraphs.jl.git
```
If you wish to contribute to the package, use `dev`:
```Julia
pkg> dev https://github.com/kylejbrown17/TaskGraphs.jl.git
```

## Quick Start

```julia
julia> using TaskGraphs

julia> solver = NBSSolver(); # initialize a solver

julia> prob = pctapf_problem_1(solver); # initialize the problem

julia> solution, cost = solve!(solver,prob); # solve it

julia> optimal_status(solver) # check the solution status
```

## This package contains the source code for the following papers

Optimal Sequential Task Assignment and Path Finding for Multi Agent Robotic Assembly Planning, _Kyle Brown, Oriana Peltzer, Martin Sehr, Mac Schwager, Mykel Kochenderfer_, in International Conference on Robotics and Automation (ICRA), 2020 [arXiv](https://arxiv.org/abs/2006.08845)