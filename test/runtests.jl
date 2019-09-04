using LightGraphs, MetaGraphs, GraphUtils
using TaskGraphs
using Gurobi
using JuMP, MathOptInterface
using TOML

using Test
using Logging

# Set logging level
global_logger(SimpleLogger(stderr, Logging.Debug))
# Define package tests
@time @testset "TaskGraphs Package Tests" begin
    testdir = joinpath(dirname(@__DIR__), "test")
    @time @testset "TaskGraphs.CoreTests" begin
        include(joinpath(testdir, "test_core.jl"))
    end
end
