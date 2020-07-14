using TaskGraphs
using CRCBS
using LightGraphs, MetaGraphs, GraphUtils
using JuMP, MathOptInterface
using Gurobi
using Random
using TOML

using Test
using Logging

# Check equality of two arrays
@inline function array_isapprox(x::AbstractArray{F},
                  y::AbstractArray{F};
                  rtol::F=sqrt(eps(F)),
                  atol::F=zero(F)) where {F<:AbstractFloat}

    # Easy check on matching size
    if length(x) != length(y)
        return false
    end

    for (a,b) in zip(x,y)
        if !isapprox(a,b, rtol=rtol, atol=atol)
            return false
        end
    end
    return true
end

# Check if array equals a single value
@inline function array_isapprox(x::AbstractArray{F},
                  y::F;
                  rtol::F=sqrt(eps(F)),
                  atol::F=zero(F)) where {F<:AbstractFloat}

    for a in x
        if !isapprox(a,y, rtol=rtol, atol=atol)
            return false
        end
    end
    return true
end

# Set logging level
global_logger(SimpleLogger(stderr, Logging.Debug))
# Define package tests
@time @testset "TaskGraphs Package Tests" begin
    testdir = joinpath(dirname(@__DIR__), "test")
    @time @testset "TaskGraphs.FactoryWorlds" begin
        include(joinpath(testdir, "test_factory_worlds.jl"))
    end
    @time @testset "TaskGraphs.PlanningPredicates" begin
        include(joinpath(testdir, "test_predicates.jl"))
    end
    @time @testset "TaskGraphs.PCCBS" begin
        include(joinpath(testdir, "test_pccbs.jl"))
    end
    @time @testset "TaskGraphs.CoreTests" begin
        include(joinpath(testdir, "test_core.jl"))
    end
    @time @testset "TaskGraphs.ExampleTests" begin
        include(joinpath(testdir, "test_milp_solvers.jl"))
    end
    @time @testset "TaskGraphs.PathPlanning" begin
        include(joinpath(testdir, "test_path_planning.jl"))
    end
    @time @testset "TaskGraphs.Solvers" begin
        include(joinpath(testdir, "test_solver.jl"))
    end
    @time @testset "TaskGraphs.Replanning" begin
        include(joinpath(testdir, "test_replanning.jl"))
    end
    # @time @testset "TaskGraphs.Profiling" begin
    #     include(joinpath(testdir, "test_profiling.jl"))
    # end
    # @time @testset "TaskGraphs.Overhaul" begin
    #     include(joinpath(testdir, "test_overhaul.jl"))
    # end
end
