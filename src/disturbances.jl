export
    AbstractDisturbance

abstract type AbstractDisturbance end

export OilSpill
"""
    OilSpill

An obstruction that affects vertices `vtxs` and edges `edges`.
"""
struct OilSpill <: AbstractDisturbance
    vtxs::Set{Int}
    edges::Set{Edge}
end
function OilSpill(G::AbstractGraph,vtxs::Vector{Int})
    OilSpill(Set(vtxs),edge_cover(G,vtxs))
end

export Intruder
"""
    Intruder

An intruder that begins at location `start_vtx` and follows policy `policy`
"""
struct Intruder{P} <: AbstractDisturbance
    policy::P
    start_vtx::Int # start vtx
end

export DeadRobot
"""
    DeadRobot

Robot `id` is frozen.
"""
struct DeadRobot <: AbstractDisturbance
    id::Int
end

export DelayedRobot
"""
    DelayedRobot

Robot `id` is delayed by `dt` timesteps.
"""
struct DelayedRobot <: AbstractDisturbance
    id::Int
    dt::Int
end

export DroppedObject
"""
    DroppedObject

Object `id` dropped.
"""
struct DroppedObject <: AbstractDisturbance
    id::Int
end


function account_for_disturbance!(env_graph::GridFactoryEnvironment,d::OilSpill)
    for e in d.edges
        rem_edge!(env_graph.graph,e)
    end
    return env_graph
end
