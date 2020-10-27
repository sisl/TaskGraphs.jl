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


function apply_disturbance!(env_graph::GridFactoryEnvironment,d::OilSpill)
    remove_edges!(env_graph,d.edges)
    return env_graph
end
function remove_disturbance!(env_graph::GridFactoryEnvironment,d::OilSpill)
    add_edges!(env_graph,d.edges)
    return env_graph
end

function add_headless_cleanup_task!(
        sched::OperatingSchedule,
        spec::ProblemSpec,
        d::OilSpill,
        return_vtx::LocationID=LocationID(-1)
        )

    robot_id = CleanUpBotID(-1)
    action_id = get_unique_action_id()
    add_to_sched!(sched, spec, CLEAN_UP(robot_id, d.vtxs), action_id)

    prev_action_id = action_id
    action_id = get_unique_action_id()
    add_to_sched!(sched, spec, CUB_GO(robot_id, d.vtxs[1], return_vtx), action_id)
    add_edge!(sched, prev_action_id, action_id)

    return
end

function remove_robot!(env::SearchEnv,id::BotID,t::Int)
    G = get_graph(env.schedule)
    # schedule
    # - remove BOT_AT node
    # set assignment ids to -1 with replace_robot_id()
    # if in the middle of CARRY, another robot needs to get the object
end
