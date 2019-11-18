using LightGraphs, MetaGraphs, GraphUtils
using TaskGraphs
using Gurobi
using JuMP, MathOptInterface
using TOML
using Random

using Compose
using GraphPlottingBFS

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
    # @time @testset "TaskGraphs.Overhaul" begin
    #     include(joinpath(testdir, "test_overhaul.jl"))
    # end
    @time @testset "TaskGraphs.CoreTests" begin
        include(joinpath(testdir, "test_core.jl"))
    end
#     @time @testset "TaskGraphs.ExampleTests" begin
#         include(joinpath(testdir, "test_examples.jl"))
#     end
    @show get_unique_operation_id()
end
