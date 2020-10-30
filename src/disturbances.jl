export
    AbstractDisturbance,
    DisturbanceSequence

abstract type AbstractDisturbance end
const DisturbanceSequence = Vector{Pair{Int,AbstractDisturbance}}


export
    StochasticProblem,
    stochastic_problem

"""
    StochasticProblem{P<:AbstractPC_TAPF}

Defines a stochastic version of PC_TAPF, wherein different disturbances can
cause unexpected problems in the factory.
"""
struct StochasticProblem{P<:AbstractPC_TAPF}
    prob::P
    disturbances::DisturbanceSequence
end
CRCBS.get_env(spc_tapf::StochasticProblem) = get_env(spc_tapf.prob)

function stochastic_problem(ptype::Type{P},solver,prob,clean_up_bot_ICS,disturbances) where {P<:AbstractPC_MAPF}
    env = get_env(deepcopy(prob))
    env_graph = get_graph(env)
    sched = get_schedule(env)
    cache = get_cache(env)
    for pred in clean_up_bot_ICS
        add_new_robot_to_schedule!(sched,pred,get_problem_spec(env))
    end
    reinitialize_planning_cache!(sched,cache,
        vcat(cache.t0,zeros(nv(sched)-length(cache.t0))),
        vcat(cache.tF,zeros(nv(sched)-length(cache.tF)))
    )
    robot_ICs = get_robot_ICs(sched)
    prob_spec = ProblemSpec(
        get_problem_spec(env),
        r0=[get_id(get_location_id(robot_ICs[k])) for k in sort(collect(keys(robot_ICs)))],
        )
    # populate_environment_dict_layers!(env.env_layers,sched,env_graph,prob_spec)
    new_prob = P(prob,construct_search_env(solver,sched,prob_spec,env_graph,cache))
    StochasticProblem(new_prob,disturbances)
end

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


export handle_disturbance!

function remove_vtxs(sched,cache,remove_set::Set{A}) where {A<:AbstractID}
    remove_vtxs(sched,cache,Set{Int}(get_vtx(sched,id) for id in remove_set))
end
function remove_vtxs(sched,cache,remove_set)
    # Construct new graph
    new_sched = OperatingSchedule()
    keep_vtxs = setdiff(Set{Int}(collect(vertices(sched))), remove_set)
    # add all non-deleted nodes to new project schedule
    for v in keep_vtxs
        node_id = get_vtx_id(sched,v)
        node = get_node_from_id(sched, node_id)
        path_spec = get_path_spec(sched,v)
        add_to_schedule!(new_sched,path_spec,node,node_id)
    end
    # add all edges between nodes that still exist
    for e in edges(get_graph(sched))
        add_edge!(new_sched, get_vtx_id(sched, e.src), get_vtx_id(sched, e.dst))
    end
    set_leaf_operation_vtxs!(new_sched)
    G = get_graph(new_sched)
    # init planning cache with the existing solution
    t0 = map(v->get(cache.t0, get_vtx(sched, get_vtx_id(new_sched, v)), 0.0), vertices(G))
    tF = map(v->get(cache.tF, get_vtx(sched, get_vtx_id(new_sched, v)), 0.0), vertices(G))
    new_cache = initialize_planning_cache(new_sched,t0,tF)
    # @assert sanity_check(new_sched)
    new_sched, new_cache
end
function handle_disturbance!(solver,prob,env::SearchEnv,d::DroppedObject,t,
        env_state = get_env_state(env,t),
        )
    # Schedule surgery
    # - remove Delivery Task: OBJECT_AT(o,old_x) -> COLLECT -> CARRY -> DEPOSIT
    # - replace with equivalent CleanUpBot Delivery Task: OBJECT_AT(o,new_x) -> BOT_COLLECT{CleanUpBot} -> CARRY -> DEPOSIT
    sched = get_schedule(env)
    o = ObjectID(d.id)
    v = get_vtx(sched,o)
    vtxs = collect(capture_connected_nodes(get_graph(sched),v,
        v->check_object_id(get_node_from_vtx(sched,v),o)))
    setdiff!(vtxs,v)
    node_ids = Set{AbstractID}(get_vtx_id(sched,v) for v in vtxs)
    # identify deposit and operation nodes (will need new edges)
    # v_deposit = filter(v->isa(get_node_from_vtx(sched,v),BOT_DEPOSIT),vtxs)[1]
    # deposit_node = get_node_from_vtx(sched,v_deposit)
    # v_op = filter(v->isa(get_node_from_vtx(sched,v),Operation),
    #     outneighbors(sched,v_deposit))[1]
    # op_id = get_vtx_id(sched,v_op)
    # incoming and outgoing robot nodes -- for single and team robot tasks
    in_vtxs = setdiff(map(e->e.dst,collect(edge_cover(sched,vtxs,:in))),vtxs)
    in_ids = filter(id->isa(id,ActionID),
        map(v->get_vtx_id(sched,v),collect(in_vtxs)))
    out_vtxs = setdiff(map(e->e.src,collect(edge_cover(sched,vtxs,:out))),vtxs)
    out_ids = filter(id->isa(id,ActionID),
        map(v->get_vtx_id(sched,v),collect(out_vtxs)))
    op_id = filter(v->isa(get_node_from_vtx(sched,v),Operation),
        map(v->get_vtx_id(sched,v),collect(out_vtxs)))[1]
    # op_id = get_vtx_id(sched,v_op)
    # Add GO->GO edges for affected robot(s), as if they were never assigned
    for id1 in in_ids
        node1 = get_node_from_id(sched,id1)
        r = get_robot_id(node1)
        for i in 1:length(out_ids)
            id2 = out_ids[i]
            node2 = get_node_from_id(sched,id2)
            if get_robot_id(node2) == r
                x = get_location_id(env_state.robot_positions[r])
                add_edge!(sched,id1,id2)
                n1 = BOT_GO(r,get_initial_location_id(node1),x)
                n2 = BOT_GO(r,x,get_destination_location_id(node2))
                @log_info(-1,0,"connecting ",string(n1)," → ",string(n2))
                replace_in_schedule!(sched,n1,id1)
                replace_in_schedule!(sched,n2,id2)
                deleteat!(out_ids,i)
                break
            end
        end
    end
    @assert length(out_ids) == 0
    # remove old nodes
    new_sched, new_cache = remove_vtxs(sched,get_cache(env),vtxs)
    # add new CleanUpBot task nodes
    # r = get_robot_id(deposit_node)
    # x1 = get_location_id(robot_position(env_state,r))
    # @assert x1 == get_location_id(object_position(env_state,o))
    x = get_location_id(object_position(env_state,o))
    # x2 = get_destination_location_id(deposit_node)
    replace_in_schedule!(new_sched,OBJECT_AT(o,x1),x)
    add_headless_delivery_task!(
        new_sched,get_problem_spec(env,:CleanUpBot),o,op_id,CleanUpBotID#,x1,x2
    )
    t0 = map(v->get(new_cache.t0, v, t), vertices(get_graph(new_sched)))
    tF = map(v->get(new_cache.tF, v, t), vertices(get_graph(new_sched)))
    construct_search_env(
        solver,
        new_sched,
        env,
        initialize_planning_cache(new_sched,t0,tF)
    )
end


function apply_disturbance!(env_graph::GridFactoryEnvironment,d::OilSpill)
    remove_edges!(env_graph,d.edges)
    return env_graph
end
function remove_disturbance!(env_graph::GridFactoryEnvironment,d::OilSpill)
    add_edges!(env_graph,d.edges)
    return env_graph
end

"""
    regenerate_path_specs!(solver,env)

Recompute all paths specs (to account for changes in the env_graphs that will
be propagated to the ProblemSpec's distance function as well.)
"""
function regenerate_path_specs!(solver,env)
    sched = get_schedule(env)
    for v in vertices(sched)
        node = get_node_from_vtx(sched,v)
        prob_spec = get_problem_spec(env,graph_key(node))
        set_path_spec!(sched,v,generate_path_spec(sched,prob_spec,node))
    end
    update_planning_cache!(solver,env)
    return env
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
    sched = get_schedule(env)
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
                replace_in_schedule!(sched,get_problem_spec(env),new_node,node_id)
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
