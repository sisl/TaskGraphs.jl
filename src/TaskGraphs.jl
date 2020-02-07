module TaskGraphs

using Reexport
using Parameters
using LightGraphs, MetaGraphs
using GraphUtils
using LinearAlgebra
using DataStructures
using JuMP
using TOML

export
    EXPERIMENT_DIR,
    ENVIRONMENT_DIR,
    PROBLEM_DIR,
    RESULTS_DIR,
    VIDEO_DIR

global EXPERIMENT_DIR   = joinpath(dirname(pathof(TaskGraphs)),"..","experiments")
global ENVIRONMENT_DIR  = joinpath(EXPERIMENT_DIR,"environments")
global PROBLEM_DIR      = joinpath(EXPERIMENT_DIR,"problem_instances")
global RESULTS_DIR      = joinpath(EXPERIMENT_DIR,"results")
global VIDEO_DIR        = joinpath(EXPERIMENT_DIR,"videos")

include("factory_worlds.jl")
@reexport using TaskGraphs.FactoryWorlds
include("pccbs.jl")
include("planning_predicates.jl")
# include("task_graphs_core.jl")
@reexport using TaskGraphs.PlanningPredicates
include("task_graphs_core.jl")
@reexport using TaskGraphs.TaskGraphsCore
include("utils.jl")
@reexport using TaskGraphs.TaskGraphsUtils
include("path_planning.jl")
@reexport using TaskGraphs.PathPlanning
include("helpers.jl")
@reexport using TaskGraphs.Helpers
include("profiling.jl")
@reexport using TaskGraphs.SolverProfiling

end
