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

Effect:
- Freeze robot
- Add "no-go" constraint to CBS/PIBT (how to do consistently? Perhaps place in
    SearchEnv and add directly to PCCBSEnv) OR temporarily remove vertex from
    graph
- Set robot state to NULL state? How to avoid having CBS complain about
    conflicts? Maybe set State to NULL State and place DeadRobotObject at the
    collection site?
- Dispatch CleanUpBot to collect frozen robot
- When CleanUpBot returns to "garage", regenerate frozen Robot's ROBOT_AT node
    and valid state.
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
    sched = env.schedule
    G = get_graph(sched)
    # schedule
    to_remove = Set{AbstractID}(id)
    v = get_vtx(sched,id)
    for vp in edges(bfs_tree(G,v))
        node_id = get_vtx_id(sched,vp)
        if isa(node_id,ActionID)
            node = get_node_from_id(sched,node_id)
            if isa(node,BOT_GO)
                push!(to_remove,node_id)
            else
                new_node = replace_robot_id(node,-1)
                replace_in_schedule!(sched,env.problem_spec,new_node,node_id)
            end
        end
    end
    for node_id in to_remove
        delete_node!(sched,node_id)
    end
    # Verify that the robot is no longer in schedule
    for v in vertices(G)
        node_id = get_vtx_id(sched,v)
        if isa(node_id,ActionID)
            node = get_node_from_id(sched,node_id)
            @assert !(get_id(id) in get_robot_ids(node)) "Robot id $(string(id)) should be wiped from schedule, but is present in $(string(node))"
        end
    end
    # Remap other robots' IDs? TODO refactor OperatinSchedule and RoutePlan so that I don't have to remap ids? i.e., have both uids and idxs?
    # - remove BOT_AT node
    # set assignment ids to -1 with replace_robot_id()
    # if in the middle of CARRY, another robot needs to get the object
end
