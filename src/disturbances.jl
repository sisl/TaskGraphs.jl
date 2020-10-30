export
    AbstractDisturbance

abstract type AbstractDisturbance end

"""
    StochasticProblem{P<:AbstractPC_TAPF}

Defines a stochastic version of PC_TAPF, wherein different disturbances can
cause unexpected problems in the factory.
"""
struct StochasticProblem{P<:AbstractPC_TAPF}
    prob::P
    disturbances::Vector{AbstractDisturbance}
end
CRCBS.get_env(spc_tapf::StochasticProblem) = get_env(spc_tapf.prob)

# function spctapf_problem(env_graph,r0,cr0,s0,sF,ops)
#     tasks = config.tasks
#     ops = config.ops
#     s0 = map(t->t.first,tasks)
#     sF = map(t->t.second,tasks)
#     project_spec, _ = pctapf_problem(r0,s0,sF)
#     for op in ops
#         add_operation!(project_spec,construct_operation(project_spec,-1,
#             op.inputs,op.outputs,Δt_op))
#     end
#     def = SimpleProblemDef(project_spec,r0,s0,sF,config.shapes)
# end
#
function stochastic_problem(prob::P,args...) where {P<:AbstractPC_MAPF}
    stochastic_problem!(deepcopy(prob),args...)
end
function stochastic_problem!(prob::P,clean_up_bot_ICS,disturbances) where {P<:AbstractPC_MAPF}
    env = get_env(prob)
    sched = get_schedule(env)
    for pred in robot_ICs
        add_new_robot_to_schedule!(sched,pred,get_problem_spec(get_env(prob)))
    end
    # cache = initialize_planning
    sched = splice_schedules!()
end
function stochastic_problem(solver,env_graph,r0,cr0,s0,sF,ops,disturbances)
    object_ICs = [OBJECT_AT(o,x) for (o,x) in enumerate(s0)] # initial_conditions
    object_FCs = [OBJECT_AT(o,x) for (o,x) in enumerate(sF)] # final conditions
    robot_ICs = vcat(
        [ROBOT_AT(r,x) for (r,x) in enumerate(r0)],
        [CUB_AT(r,x) for (r,x) in enumerate(cr0)]
        )
    proj_spec = ProjectSpec(object_ICs,object_FCs)
    for op in ops
        add_operation!(proj_spec,construct_operation(proj_spec,-1,
            op.inputs,op.outputs,op.dt))
    end
    prob_spec = ProblemSpec(
        graph=construct_delivery_graph(proj_spec,length(s0)),
        D=get_dist_matrix(env_graph),
        Δt=map(op->op.dt,ops),
        r0=map(node->get_id(get_initial_location_id(node)),robot_ICs),
        s0=s0,
        sF=sF,
        )
    sched = construct_partial_project_schedule(proj_spec,prob_spec,robot_ICs)
    env = construct_search_env(solver,sched,prob_spec,env_graph)
    StochasticProblem{PC_TAPF}(env)
end
#
# function spctapf_problem(
#         solver,
#         project_spec::ProjectSpec,
#         problem_spec::ProblemSpec,
#         robot_ICs,
#         env_graph,
#         args...
#     )
#     project_schedule = construct_partial_project_schedule(
#         project_spec,
#         problem_spec,
#         robot_ICs,
#         )
#     env = construct_search_env(
#         solver,
#         project_schedule,
#         problem_spec,
#         env_graph
#         )
#     PC_TAPF(env)
# end

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
    node_ids = map(v->get_vtx_id(sched,v),vtxs)
    # nodes = map(id->get_node_from_id(sched,id),node_ids)
    # v_collect = filter(v->isa(get_node_from_vtx(sched,v),BOT_COLLECT),vtxs)[1]
    # collect_node = get_node_from_vtx(sched,v_deposit)
    v_deposit = filter(v->isa(get_node_from_vtx(sched,v),BOT_DEPOSIT),vtxs)[1]
    deposit_node = get_node_from_vtx(sched,v_deposit)
    v_op = filter(v->isa(get_node_from_vtx(sched,v),Operation),
        outneighbors(sched,v_deposit))[1]
    op_id = get_vtx_id(sched,v_op)
    # incoming and outgoing robot nodes -- for single and team robot tasks
    in_vtxs = setdiff(map(e->e.dst,collect(edge_cover(sched,vtxs,:in))),vtxs)
    in_ids = filter(id->isa(id,ActionID),
        map(v->get_vtx_id(sched,v),collect(in_vtxs)))
    out_vtxs = setdiff(map(e->e.dst,collect(edge_cover(sched,vtxs,:out))),vtxs)
    out_ids = filter(id->isa(id,ActionID),
        map(v->get_vtx_id(sched,v),collect(out_vtxs)))
    @show in_ids, out_ids
    # remove old nodes
    for id in node_ids
        delete_node!(sched,id)
    end
    # Add GO->GO edges for affected robot(s), as if they were never assigned
    for id1 in in_ids
        @show node1 = get_node_from_id(sched,id1)
        r = get_robot_id(node1)
        for i in 1:length(out_ids)
            id2 = out_ids[i]
            @show node2 = get_node_from_id(sched,id2)
            if get_robot_id(node2) == r
                add_edge!(sched,id1,id2)
                deleteat!(out_ids,i)
                break
            end
        end
    end
    @assert length(out_ids) == 0
    # add new CleanUpBot task nodes
    r = get_robot_id(deposit_node)
    x1 = get_location_id(robot_position(env_state,r))
    @assert x1 == get_location_id(object_position(env_state,o))
    x2 = get_destination_location_id(deposit_node)
    add_to_schedule!(sched,OBJECT_AT(o,x1),o)
    add_headless_delivery_task!(
        sched,get_problem_spec(env,:CleanUpBot),o,op_id,x1,x2,CleanUpBotID(-1)
    )
    env
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
