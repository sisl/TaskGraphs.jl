# Getting Started

## Installation

To install
[TaskGraphs.jl](https://github.com/kylejbrown17/TaskGraphs.jl), start up
Julia and type the following code-snipped into the REPL.

```julia
julia>
] # enter package mode by typing "]"
(@v1.4) pkg> add https://github.com/kylejbrown17/TaskGraphs.jl.git
```
To take full advantage of the package, you will also need to install 
[Gurobi](https://www.gurobi.com/downloads/).

## Example

To solve a **precedence-constrained multi agent task assignment and pathfinding** 
(PC-TAPF) problem.

```julia
julia> using TaskGraphs

julia> solver = NBSSolver() # initialize a solver

julia> prob = pctapf_problem_1(solver) # initialize the problem

julia> solution, cost = solve!(solver,prob)
```