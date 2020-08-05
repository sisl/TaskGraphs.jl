# module PathPlanning
#
# using Parameters
# using LightGraphs
# using MetaGraphs
# using DataStructures
# using MathOptInterface, JuMP
# using TOML
# using JLD2, FileIO
#
# using GraphUtils
# using CRCBS
#
# using ..TaskGraphs


export
    PlanningCache,
    isps_queue_cost,
    initialize_planning_cache,
    reset_cache!


@with_kw struct PlanningCache
    closed_set::Set{Int}                    = Set{Int}()    # nodes that are completed
    active_set::Set{Int}                    = Set{Int}()    # active nodes
    node_queue::PriorityQueue{Int,Tuple{Int,Float64}} = PriorityQueue{Int,Tuple{Int,Float64}}() # active nodes prioritized by slack
    t0::Vector{Int}                         = Vector{Int}()
    tF::Vector{Int}                         = Vector{Int}()
    slack::Vector{Vector{Float64}}          = Vector{Vector{Float64}}()
    local_slack::Vector{Vector{Float64}}    = Vector{Vector{Float64}}()
    # max_deadline::Vector{Int}               = Vector{Int}() # Marks the time at which this node will begin accumulating a delay cost
end

function isps_queue_cost(schedule::OperatingSchedule,cache::PlanningCache,v::Int)
    path_spec = get_path_spec(schedule,v)
    return (Int(path_spec.plan_path), minimum(cache.slack[v]))
end

function initialize_planning_cache(schedule::OperatingSchedule,t0_=zeros(nv(schedule)),tF_=zeros(nv(schedule)))
    t0,tF,slack,local_slack = process_schedule(schedule,t0_,tF_)
    # allowable_slack = map(i->minimum(i),slack) # soft deadline (can be tightened as necessary)
    # allowable_slack = map(i->i==Inf ? typemax(Int) : Int(i), allowable_slack) # initialize deadline at infinity
    cache = PlanningCache(t0=t0,tF=tF,slack=slack,local_slack=local_slack) #,max_deadline=allowable_slack)
    for v in get_all_root_nodes(schedule)
        push!(cache.active_set,v)
        enqueue!(cache.node_queue,v=>isps_queue_cost(schedule,cache,v)) # need to store slack
    end
    cache
end

"""
    `reset_cache!(cache,schedule)`

    Resets the cache so that a solution can be repaired (otherwise calling
    low_level_search!() will return immediately because the cache says it's
    complete)
"""
function reset_cache!(cache::PlanningCache,schedule::OperatingSchedule,t0=cache.t0,tF=cache.tF)
    t0,tF,slack,local_slack = process_schedule(schedule,t0,tF)
    cache.t0            .= t0
    cache.tF            .= tF
    cache.slack         .= slack
    cache.local_slack   .= local_slack
    empty!(cache.closed_set)
    empty!(cache.active_set)
    empty!(cache.node_queue)
    # for v in get_all_terminal_nodes(schedule)
    for v in vertices(get_graph(schedule))
        if is_root_node(get_graph(schedule),v)
            push!(cache.active_set,v)
            enqueue!(cache.node_queue,v=>isps_queue_cost(schedule,cache,v)) # need to store slack
        end
    end
    cache
end

function initialize_route_plan end

const State = CRCBS.GraphState
const Action = CRCBS.GraphAction

export
    SearchEnv,
    construct_search_env,
    update_env!

@with_kw struct SearchEnv{G,C,H,S} <: GraphEnv{State,Action,C}
    schedule::OperatingSchedule     = OperatingSchedule()
    cache::PlanningCache            = PlanningCache()
    env_graph::G                    = GridFactoryEnvironment()
    problem_spec::ProblemSpec       = ProblemSpec()
    cost_model::C                   = C() #get_cost_model(env)
    heuristic_model::H              = H()
    num_agents::Int                 = length(get_robot_ICs(schedule))
    route_plan::S                   = initialize_route_plan(schedule,cost_model)
    # route_plan::S                   = LowLevelSolution(
    #     paths = map(i->Path{State,Action,cost_type(cost_model)}(),
    #         1:num_agents),
    #     cost_model = cost_model,
    #     costs = map(i->get_initial_cost(cost_model), 1:num_agents),
    #     cost = get_infeasible_cost(cost_model),
    #     )
end
CRCBS.get_graph(env::SearchEnv) = env.env_graph
function CRCBS.get_start(env::SearchEnv,v::Int)
    start_vtx   = get_path_spec(env.schedule,v).start_vtx
    start_time  = env.cache.t0[v]
    State(start_vtx,start_time)
end
CRCBS.get_cost_model(env::SearchEnv) = env.cost_model
CRCBS.get_heuristic_model(env::SearchEnv) = env.heuristic_model
CRCBS.num_agents(env::SearchEnv) = env.num_agents
for op in [
    :cost_type,:state_type,:action_type,:path_type,:get_cost,:get_paths,
    :get_path_costs,:set_cost!,:set_solution_path!,:set_path_cost!,
    :convert_to_vertex_lists,
    ]
    @eval CRCBS.$op(env::SearchEnv,args...) = $op(env.route_plan,args...)
end
GraphUtils.get_distance(env::SearchEnv,args...) = get_distance(env.problem_spec,args...)
GraphUtils.get_distance(env::SearchEnv,s::State,args...) = get_distance(env.problem_spec,get_vtx(s),args...)
GraphUtils.get_distance(env::SearchEnv,s::State,g::State) = get_distance(env.problem_spec,s,get_vtx(g))

initialize_planning_cache(env::SearchEnv) = initialize_planning_cache(env.schedule,deepcopy(env.cache.t0),deepcopy(env.cache.tF))

"""
    get_next_vtx_matching_agent_id(schedule,cache,agent_id)

Return the node_id of the active node assigned to an agent.
"""
function get_next_vtx_matching_agent_id(env::SearchEnv,agent_id)
    for v in env.cache.active_set
        if agent_id == get_path_spec(env.schedule, v).agent_id
            return v
        end
    end
    return -1
end

"""
    get_next_node_matching_agent_id(schedule,cache,agent_id)

Return the node_id of the active node assigned to an agent.
"""
function get_next_node_matching_agent_id(env::SearchEnv,agent_id)
    v = get_next_vtx_matching_agent_id(env::SearchEnv,agent_id)
    if has_vertex(env.schedule,v)
        return get_vtx_id(env.schedule,v)
    end
    return RobotID(agent_id)
end

export
    update_planning_cache!
# function reverse_propagate_delay!(solver,cache,schedule,delay_vec)
#     buffer = zeros(nv(schedule))
#     for v in reverse(topological_sort_by_dfs(get_graph(schedule)))
#         Δt_min = get_path_spec(schedule,v).min_path_duration
#         buffer[v] = (cache.tF[v] - (cache.t0[v] + Δt_min))
#         for v2 in outneighbors(schedule,v)
#             delay_vec[v] = max(delay_vec[v], delay_vec[v2] - buffer[v2])
#             delay_vec[v] = max(0, delay_vec[v] - (cache.t0[v2] - cache.tF[v]))
#         end
#     end
#     delay_vec
# end
# function get_delay_vec(solver,cache,schedule,v0)
#     delay = zeros(nv(schedule))
#     for v in reverse(topological_sort_by_dfs(get_graph(schedule)))
#         Δt_min = get_path_spec(schedule,v).min_path_duration
#         delay[v] = (cache.tF[v] - (cache.t0[v] + Δt_min + minimum(cache.local_slack[v])))
#     end
#     # zero out all vtxs that are not predecessors of v
#     delay_vec = zeros(Int,nv(schedule))
#     for v in map(e->e.dst,collect(edges(bfs_tree(get_graph(schedule),v0;dir=:in))))
#         if cache.max_deadline[v] > 0
#             delay_vec[v] = min(delay[v],typemax(Int))
#         end
#     end
#     delay_vec
# end
# function backtrack_deadlines(solver,cache,schedule,v)
#     frontier = Set{Int}([v])
#     delay_cut = Set{Int}()
#     while length(frontier) > 0
#         v = pop!(frontier)
#         Δt_min = get_path_spec(schedule,v).min_path_duration
#         buffer = (cache.tF[v] - (cache.t0[v] + Δt_min))
#         if get_path_spec(schedule,v).fixed
#             continue
#         elseif cache.max_deadline[v] > 0 # still has room for some delay
#             push!(delay_cut,v)
#         elseif indegree(schedule,v) == 0
#             # if v is a root_node, the deadlines cannot be tightened anymore
#             log_info(-1,solver.l3_verbosity,"ISPS: deadlines cannot be tightened any more.")
#             return Set{Int}()
#         else
#             for v2 in inneighbors(schedule,v)
#                 push!(frontier,v2)
#             end
#         end
#     end
#     return delay_cut
# end
# function tighten_deadline!(solver,env,route_plan,v,dt=1)
#     cache = env.cache
#     schedule = env.schedule
#     active_set = cache.active_set
#     closed_set = cache.closed_set
#     node_queue = cache.node_queue
#     graph = get_graph(schedule)
#     # adjust allowable_slack
#     cache.max_deadline[v] = max(0, cache.max_deadline[v]-1)
#     # remove from closed set
#     push!(active_set, v)
#     setdiff!(closed_set,v)
#     # Trim schedule at v.t0
#     agent_id = get_path_spec(schedule,v).agent_id
#     if agent_id != -1
#         cbs_env = typeof(env.env)(
#             graph = env.env.graph,
#             agent_idx = agent_id,
#             cost_model = get_cost_model(env.env),
#             heuristic = get_heuristic_model(env.env)
#         )
#         new_path = trim_path(cbs_env,get_paths(route_plan)[agent_id],cache.t0[v])
#         set_solution_path!(route_plan,new_path,agent_id)
#         set_path_cost!(route_plan,new_path.cost,agent_id)
#     end
#     # Update node queue
#     for v2 in map(e->e.dst,collect(edges(bfs_tree(graph,v;dir=:out))))
#         setdiff!(closed_set,v2)
#         setdiff!(active_set,v2)
#     end
#     empty!(node_queue)
#     for v2 in active_set
#         node_queue[v2] = isps_queue_cost(schedule,cache,v2)
#     end
#     env,route_plan
# end
"""
    update_planning_cache!(solver,env,v,path)
"""
function update_planning_cache!(solver,env::E,v::Int,path::P,t=get_t(get_final_state(path))) where {E<:SearchEnv,P<:Path}
    cache = env.cache
    schedule = env.schedule
    active_set = cache.active_set
    closed_set = cache.closed_set
    node_queue = cache.node_queue
    graph = get_graph(schedule)
    # update t0, tF, slack, local_slack
    Δt = t - cache.tF[v]
    if Δt > 0
        # delay = Δt - minimum(cache.slack[v])
        # if delay > 0
        #     log_info(-1,solver.l3_verbosity,"LOW LEVEL SEARCH: schedule delay of ",delay," time steps incurred by path for vertex v = ",v," - ",string(get_node_from_vtx(schedule,v)), " - tF = ",get_final_state(path).t)
        #     # Backtracking
        #     tightenable_set = backtrack_deadlines(solver,cache,schedule,v)
        #     delay_vec = get_delay_vec(solver,cache,schedule,v)
        #     # @show delay_vec
        #     if any(delay_vec .> 0)
        #         for v_ in topological_sort(graph)
        #             if delay_vec[v_] > 0
        #                 log_info(-1,solver,"ISPS: tightening at v = ",v_," - ",string(get_node_from_vtx(schedule,v_)))
        #                 tighten_deadline!(solver,env,solution,v_)
        #                 break
        #             end
        #         end
        #     # if length(tightenable_set) > 0
        #     #     for v_ in tightenable_set
        #     #         log_info(-1,solver,"ISPS: backtracking at v = ",v_," - ",string(get_node_from_vtx(schedule,v_)))
        #     #         # tighten_deadline!(solver,env,solution,v_)
        #     #     end
        #         t0,tF,slack,local_slack = process_schedule(schedule;
        #             t0=map(i->i in closed_set ? cache.t0[i] : 0, vertices(schedule)),
        #             tF=map(i->i in closed_set ? cache.tF[i] : 0, vertices(schedule)),
        #             # t0=map(i->is_root_node(schedule,v) ? cache.t0[i] : 0, vertices(schedule)),
        #             # tF=map(i->is_root_node(schedule,v) ? cache.tF[i] : 0, vertices(schedule)),
        #         )
        #         cache.t0            .= t0
        #         cache.tF            .= tF
        #         cache.slack         .= slack
        #         cache.local_slack   .= local_slack
        #         return cache
        #     end
        # end
        cache.tF[v] = t
        t0,tF,slack,local_slack = process_schedule(schedule,cache.t0,cache.tF)
        cache.t0            .= t0
        cache.tF            .= tF
        cache.slack         .= slack
        cache.local_slack   .= local_slack
    end
    # update closed_set
    activated_vtxs = Int[]
    push!(closed_set,v)
    # update active_set
    setdiff!(active_set, v)
    for v2 in outneighbors(graph,v)
        active = true
        for v1 in inneighbors(graph,v2)
            if !(v1 in closed_set)
                active = false
                break
            end
        end
        if active
            push!(activated_vtxs,v2)
            push!(active_set, v2)               # add to active set
        end
    end
    # update priority queue
    for v2 in active_set
        node_queue[v2] = isps_queue_cost(schedule,cache,v2)
    end
    log_info(2,solver,"moved ",v," to closed set, moved ",activated_vtxs," to active set")
    log_info(3,solver,string("cache.tF[v] = ",cache.tF))
    return cache
end
"""
    update_planning_cache!(solver,env)

Update cache continually. After a call to this function, the start and end times
of all schedule nodes will be updated to reflect the progress of active schedule
nodes (i.e., if a robot had not yet completed a GO task, the predicted final
time for that task will be updated based on the robot's current state and
distance to the goal).
All active nodes that don't require planning will be automatically marked as
complete.
"""
function update_planning_cache!(solver,env)
    cache = env.cache
    schedule = env.schedule
    dummy_path = path_type(env)()
    for v in collect(cache.active_set)
        path_spec = get_path_spec(schedule,v)
        agent_id = path_spec.agent_id
        if 1 <= agent_id <= num_agents(env)
            @show string(get_node_from_vtx(env.schedule,v))
            path = get_paths(env)[agent_id]
            s = get_final_state(path)
            t = get_t(s)
            d = get_distance(env,s,path_spec.final_vtx)
            cache.tF[v] = max(cache.tF[v],t+d)
        end
    end
    while true
        done = true
        for v in collect(cache.active_set)
            if get_path_spec(schedule,v).plan_path == false
                update_planning_cache!(solver,env,v,dummy_path) # NOTE I think this is all we need, since there is no actual path to update
                done = false
            end
        end
        if done
            break
        end
    end
    t0,tF,slack,local_slack = process_schedule(schedule,cache.t0,cache.tF)
    cache.t0 .= t0
    cache.tF .= tF
    cache.slack .= slack
    cache.local_slack .= local_slack

    cache
end

function CRCBS.SumOfMakeSpans(schedule::S,cache::C) where {S<:OperatingSchedule,C<:PlanningCache}
    SumOfMakeSpans(
        cache.tF,
        schedule.terminal_vtxs,
        map(k->schedule.weights[k], schedule.terminal_vtxs),
        cache.tF[schedule.terminal_vtxs])
end
function CRCBS.MakeSpan(schedule::S,cache::C) where {S<:OperatingSchedule,C<:PlanningCache}
    MakeSpan(
        cache.tF,
        schedule.terminal_vtxs,
        map(k->schedule.weights[k], schedule.terminal_vtxs),
        cache.tF[schedule.terminal_vtxs])
end

export
    construct_cost_model,
    construct_heuristic_model

function construct_cost_model end
function construct_heuristic_model end

function initialize_route_plan(schedule::OperatingSchedule,cost_model)
    starts = State[]
    robot_ics = get_robot_ICs(schedule)
    for k in sort(collect(keys(robot_ics)))
        push!(starts, State(vtx = get_id(get_location_id(robot_ics[k])), t = 0))
    end
    # cost_model = get_cost_model(env)
    paths = map(s->Path{State,Action,cost_type(cost_model)}(s0=s, cost=get_initial_cost(cost_model)), starts)
    costs = map(p->get_cost(p), paths)
    cost = aggregate_costs(cost_model, costs)
    LowLevelSolution(paths=paths, cost_model=cost_model,costs=costs, cost=cost)
end
function initialize_route_plan(env::SearchEnv)
    paths = map(p->path_type(env)(
        s0=get_initial_state(p),
        cost=get_initial_cost(env)
        ), get_paths(env.route_plan))
    costs = map(p->get_cost(p), paths)
    LowLevelSolution(
        paths=paths,
        cost_model=get_cost_model(env),
        costs=costs,
        cost=aggregate_costs(get_cost_model(env), costs)
        )
end
function construct_search_env(
        solver,
        schedule::OperatingSchedule,
        problem_spec::ProblemSpec,
        env_graph,
        cache::PlanningCache=initialize_planning_cache(schedule),
        primary_objective=problem_spec.cost_function,
        ;
        extra_T=400,
        kwargs...
    )
    N = problem_spec.N                                          # number of robots
    cost_model, heuristic_model = construct_cost_model(
        solver,
        schedule,
        cache,
        problem_spec,
        env_graph,
        primary_objective;
        extra_T=extra_T,
        )

    route_plan = initialize_route_plan(schedule,cost_model)
    @assert N == length(get_paths(route_plan))

    search_env = SearchEnv(
        schedule=schedule,
        cache=cache,
        env_graph=env_graph,
        problem_spec=problem_spec,
        cost_model=cost_model,
        heuristic_model=heuristic_model,
        num_agents=N,
        route_plan=route_plan)
    return search_env
end
function construct_search_env(
        solver,
        schedule::OperatingSchedule,
        env::SearchEnv,
        args...
        ;
        kwargs...)
    search_env = SearchEnv(
        construct_search_env(solver,schedule,env.problem_spec,env.env_graph,
        args...;kwargs...),
        route_plan = env.route_plan
        )
end

update_cost_model!(model::C,env::S) where {C,S<:SearchEnv} = nothing
function update_cost_model!(model::C,env::S) where {C<:MultiDeadlineCost,S<:SearchEnv}
    model.tF .= env.cache.tF
end
function update_cost_model!(model::C,env::S) where {C<:CompositeCostModel,S<:SearchEnv}
    for m in model.cost_models
        update_cost_model!(m,env)
    end
end
update_cost_model!(env::S) where {S<:SearchEnv} = update_cost_model!(get_cost_model(env),env)

"""
    `update_env!`

    `v` is the vertex id
"""
function update_env!(solver,env::SearchEnv,v::Int,path::P,
        agent_id::Int=get_path_spec(env.schedule,v).agent_id
        ) where {P<:Path}
    route_plan = env.route_plan
    cache = env.cache
    schedule = env.schedule
    # UPDATE CACHE
    update_planning_cache!(solver,env,v,path)
    update_cost_model!(env)
    # ADD UPDATED PATH TO HEURISTIC MODELS
    if agent_id != -1
        partially_set_path!(get_heuristic_model(env),agent_id,convert_to_vertex_lists(get_paths(route_plan)[agent_id]))
        partially_set_path!(get_cost_model(env),agent_id,convert_to_vertex_lists(get_paths(route_plan)[agent_id]))
    end

    env
end

################################################################################
############################### Low Level Search ###############################
################################################################################

include("pccbs.jl")

function get_base_path(solver,search_env::SearchEnv,env::PCCBSEnv)
    base_path = get_paths(search_env)[get_agent_id(env)]
    v = get_vtx(search_env.schedule, env.node_id)
    t0 = search_env.cache.t0[v]
    gap = t0 - get_end_index(base_path)
    if gap > 0
        log_info(1, solver, string("LOW LEVEL SEARCH: in node ",v," -- ",
            string(env.schedule_node),
            ": cache.t0[v] - get_end_index(base_path) = ", gap,
            ". Extending path to ",t0," ..."))
        extend_path!(env,base_path,t0)
    end
    return base_path
end
function CRCBS.build_env(
        solver,
        env::SearchEnv,
        node::ConstraintTreeNode,
        schedule_node::T,
        v::Int,
        path_spec=get_path_spec(env.schedule, v),
        ;
        heuristic = get_heuristic_model(env),
        cost_model = get_cost_model(env)
    ) where {T}
    agent_id = path_spec.agent_id
    goal_vtx = path_spec.final_vtx
    goal_time = env.cache.tF[v] # time after which goal can be satisfied
    # deadline = env.cache.tF[v] .+ min.(env.cache.max_deadline[v],env.cache.slack[v]) # NOTE iterative deadline tightening was causing problems with slack running out before the goal time, so this line has been replaced by the original
    # deadline = env.cache.tF[v] .+ min.(max.(env.cache.local_slack[v], env.cache.max_deadline[v]),env.cache.slack[v]) # This is a potential fix that would allow iterative tightening to keep working
    deadline = env.cache.tF[v] .+ env.cache.slack[v]         # deadline for DeadlineCost
    # Adjust deadlines if necessary:
    if path_spec.tight == true
        goal_time += minimum(env.cache.local_slack[v])
    end
    for v_next in outneighbors(get_graph(env.schedule),v)
        if get_path_spec(env.schedule, v_next).static == true
            duration_next = get_path_spec(env.schedule,v_next).min_path_duration
            for c in sorted_state_constraints(env,get_constraints(node, agent_id)) #.sorted_state_constraints
                if get_sp(get_path_node(c)).vtx == goal_vtx
                    if 0 < get_time_of(c) - goal_time < duration_next
                        log_info(1,solver,string("extending goal_time for node ",v," from ",goal_time," to ",get_time_of(c)," to avoid constraints"))
                        goal_time = max(goal_time, get_time_of(c))
                    end
                end
            end
        end
    end
    if (path_spec.free == true) && is_terminal_node(get_graph(env.schedule),v)
        goal_time = maximum(env.cache.tF)
        goal_vtx = -1
        # deadline = Inf # already taken care of, perhaps?
        log_info(3,solver,string("BUILD ENV: setting goal_vtx = ",goal_vtx,", t = maximum(cache.tF) = ",goal_time))
    end
    cbs_env = PCCBSEnv(
        # graph       = env.env_graph,
        search_env = env,
        schedule_node = schedule_node,
        node_id     = get_vtx_id(env.schedule,v),
        agent_idx   = agent_id, # this is only used for the HardConflictTable, which can be updated via the combined search node
        constraints = get_constraints(node, agent_id), # agent_id represents the whole path
        goal        = State(goal_vtx,goal_time),
        # heuristic   = heuristic,
        # cost_model  = cost_model
        )
    # update deadline in DeadlineCost
    set_deadline!(get_cost_model(cbs_env),deadline)
    return cbs_env
end
function CRCBS.build_env(
    solver,
    env::SearchEnv,
    node::ConstraintTreeNode,
    v::VtxID,
    schedule_node=get_node_from_vtx(env.schedule,get_id(v)),
    args...
    ;
    kwargs...
    )
    id = get_id(v)
    env = build_env(solver,env,node,schedule_node,id,args...;kwargs...)
end
function CRCBS.build_env(
        solver,
        env::SearchEnv,
        node::ConstraintTreeNode,
        agent_id::AgentID
        )
    node_id = get_next_node_matching_agent_id(env,get_id(agent_id))
    build_env(solver,env,node,VtxID(get_vtx(env.schedule,node_id)))
    # get_node_from_id(env.schedule,node_id),
end
function get_base_path(solver,search_env::SearchEnv,meta_env::MetaAgentCBS.AbstractMetaEnv)
    starts = Vector{state_type(search_env)}()
    meta_cost = MetaCost(Vector{cost_type(search_env)}(),get_initial_cost(search_env))
    for cbs_env in MetaAgentCBS.get_envs(meta_env)
        sub_node = cbs_env.schedule_node
        base_path = get_base_path(solver,search_env,cbs_env)
        push!(starts, get_final_state(base_path))
        push!(meta_cost.independent_costs, get_cost(base_path))
    end
    meta_path = Path{state_type(meta_env),action_type(meta_env),MetaCost{cost_type(search_env)}}(
        s0 = MetaAgentCBS.State(starts),
        cost = MetaCost(meta_cost.independent_costs, aggregate_costs(get_cost_model(meta_env), meta_cost.independent_costs))
    )
end

"""
For COLLABORATIVE transport problems
"""
function CRCBS.build_env(
    solver,
    env::E,
    node::N,
    schedule_node::TEAM_ACTION,
    v::Int,
    ;
    kwargs...) where {E<:SearchEnv,N<:ConstraintTreeNode}
    envs = []
    agent_idxs = Int[]
    for (i, sub_node) in enumerate(schedule_node.instructions)
        ph = PerfectHeuristic(env.env_graph.dist_function.dist_mtxs[schedule_node.shape][i])
        heuristic = construct_heuristic_model(solver,env.env_graph,ph)
        cbs_env = build_env(solver,env,node,VtxID(v),sub_node,generate_path_spec(env.schedule,env.problem_spec,sub_node);
            heuristic=heuristic,
            kwargs...
        )
        base_path = get_base_path(solver,env,cbs_env)
        push!(envs, cbs_env)
        push!(agent_idxs, get_id(get_robot_id(sub_node)))
    end
    meta_env = MetaAgentCBS.construct_team_env(
        [envs...],
        agent_idxs,
        get_cost_model(env)
        )
    return meta_env
end

export
    reset_route_plan!

"""
    Helper to reset the solution in a constraint node between re-runs of ISPS
"""
function reset_route_plan!(node::N,base_route_plan) where {N<:ConstraintTreeNode}
    for (agent_id,path) in enumerate(get_paths(base_route_plan))
        set_solution_path!(node.solution, deepcopy(path), agent_id)
        set_path_cost!(node.solution, get_cost(path), agent_id)
    end
    set_cost!(node, get_cost(base_route_plan))
    node
end

################################################################################
############################## CBS Wrapper Stuff ###############################
################################################################################
export
    PC_TAPF

"""
    PC_TAPF{L<:LowLevelSolution}

Defines an instance of a Precedence-Constrained Multi-Agent Task
    Assignment and Path-Finding problem.
"""
struct PC_TAPF{E<:SearchEnv} <: AbstractMAPF
    env::E
end

export
    PC_MAPF

"""
    `PC_MAPF`

    A precedence-constrained multi-agent path-finding problem. All agents have
    assigned tasks, but there are precedence constraints between tasks.
"""
struct PC_MAPF{E<:SearchEnv} <: AbstractMAPF
    env::E
end
CRCBS.build_env(solver, pc_mapf::PC_MAPF, args...) = build_env(solver,pc_mapf.env,args...)
CRCBS.get_initial_solution(pc_mapf::PC_MAPF) = pc_mapf.env
function CRCBS.initialize_root_node(env::SearchEnv)
    solution = SearchEnv(
        env,
        cache=initialize_planning_cache(env),
        route_plan=initialize_route_plan(env)
        )
    ConstraintTreeNode(
        solution    = solution,
        constraints = Dict(
            # i=>ConstraintTable{node_type(solution)}(agent_id=i) for i in 1:num_agents(env)
            i=>discrete_constraint_table(env,i,num_states(env)*8) for i in 1:num_agents(env)
            ),
        id = 1)
end
CRCBS.initialize_root_node(solver,pc_mapf::PC_MAPF) = initialize_root_node(pc_mapf.env)
function Base.copy(env::SearchEnv)
    SearchEnv(
        env,
        cache=deepcopy(env.cache),
        route_plan=deepcopy(env.route_plan)
        )
end
function CRCBS.default_solution(env::SearchEnv)
    solution = deepcopy(env)
    set_cost!(solution.route_plan,get_infeasible_cost(solution.route_plan))
    solution
end
CRCBS.default_solution(pc_mapf::M) where {M<:PC_MAPF} = default_solution(pc_mapf.env)
function CRCBS.cbs_update_conflict_table!(solver,mapf::PC_MAPF,node,constraint)
    search_env = node.solution
    idxs = collect(1:num_agents(search_env))
    t0 = max(minimum(search_env.cache.t0), 1) # This is particularly relevant for replanning, where we don't care to look for conflicts way back in the past.
    detect_conflicts!(node.conflict_table,search_env.route_plan,idxs,t0)
end
CRCBS.detect_conflicts!(table,env::SearchEnv,args...) = detect_conflicts!(table,env.route_plan,args...)
CRCBS.serialize(pc_mapf::PC_MAPF,args...) = serialize(pc_mapf.env,args...)
CRCBS.deserialize(pc_mapf::PC_MAPF,args...) = serialize(pc_mapf.env,args...)

CRCBS.get_env(pc_mapf::PC_MAPF)             = pc_mapf.env
CRCBS.action_type(pc_mapf::PC_MAPF)         = action_type(pc_mapf.env)
CRCBS.state_type(pc_mapf::PC_MAPF)          = state_type(pc_mapf.env)
CRCBS.cost_type(pc_mapf::PC_MAPF)           = cost_type(pc_mapf.env)
CRCBS.num_agents(pc_mapf::PC_MAPF)          = num_agents(pc_mapf.env)
for op in [
    :cost_type,:state_type,:action_type,:path_type,:num_states,:num_actions
    ]
    @eval CRCBS.$op(prob::PC_MAPF,args...) = $op(prob.env,args...)
    @eval CRCBS.$op(prob::PC_TAPF,args...) = $op(prob.env,args...)
end
# CRCBS.num_goals(pc_mapf::PC_MAPF)           = length(pc_mapf.goals)
# CRCBS.get_starts(pc_mapf::PC_MAPF)          = pc_mapf.starts
# CRCBS.get_goals(pc_mapf::PC_MAPF)           = pc_mapf.goals
# CRCBS.get_start(pc_mapf::PC_MAPF, i)        = get_starts(pc_mapf)[i]
# CRCBS.get_goal(pc_mapf::PC_MAPF, i)         = get_goals(pc_mapf)[i]
# CRCBS.get_start(pc_mapf::PC_MAPF, env, i)   = get_start(pc_mapf,i)
CRCBS.base_env_type(pc_mapf::PC_MAPF)       = PCCBSEnv

include("legacy/pc_tapf_solver.jl")

# end # PathPlanning module
