module TaskGraphs

using Reexport
using Parameters
using LightGraphs, MetaGraphs
using GraphUtils
using LinearAlgebra
using DataStructures
using JuMP, MathOptInterface
using Gurobi
using TOML
using CRCBS
using SparseArrays
using JLD2, FileIO

export
    DEBUG_PATH,
    EXPERIMENT_DIR,
    ENVIRONMENT_DIR,
    PROBLEM_DIR,
    RESULTS_DIR,
    VIDEO_DIR

global DEBUG_PATH       = joinpath(dirname(pathof(TaskGraphs)),"..","debug")
global EXPERIMENT_DIR   = joinpath(dirname(pathof(TaskGraphs)),"..","experiments")
global ENVIRONMENT_DIR  = joinpath(EXPERIMENT_DIR,"environments")
global PROBLEM_DIR      = joinpath(EXPERIMENT_DIR,"problem_instances")
global RESULTS_DIR      = joinpath(EXPERIMENT_DIR,"results")
global VIDEO_DIR        = joinpath(EXPERIMENT_DIR,"videos")

include("planning_predicates.jl")
@reexport using TaskGraphs.PlanningPredicates
include("pccbs.jl")
include("task_graphs_core.jl")
include("utils.jl")
include("task_assignment_solvers.jl")
include("path_planning.jl")
include("pc_tapf_solvers.jl")
include("path_planners/dfs_planner.jl")
include("path_planners/pibt_planner.jl")
include("replanning.jl")
include("helpers.jl")
include("profiling.jl")
@reexport using TaskGraphs.SolverProfiling

end
