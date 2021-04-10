@with_kw struct MState <: CRCBS.AbstractGraphState
    vtx     ::Int           = -1
    t       ::Int           = -1
    stage   ::Int           = -1
    # node    ::ScheduleNode  = ScheduleNode(RobotID(-1),ROBOT_AT(-1,-1))
    node    ::Union{Nothing,ScheduleNode}  = nothing
    active  ::Bool          = true
end
MState(s::GraphState) = MState(s.vtx,s.t)
get_stage(s::MState) = s.stage
Base.convert(::Type{MState},s::GraphState) = MState(s)
MState(vtx,t) = MState(vtx=vtx,t=t)
_state_active(s::MState) = s.active
Base.string(s::MState) = "(v=$(get_vtx(s)),t=$(get_t(s)),stage=$(get_stage(s)),node=$(summary(s.node)))"
const MAction = CRCBS.GraphAction

# function CRCBS.detect_state_conflict(n1::N,n2::N) where {S<:MState,A<:AbstractGraphAction,N<:PathNode{S,A}}
#     if _state_active(n1.sp) && _state_active(n2.sp)
#         if get_vtx(n1.sp) == get_vtx(n2.sp) && get_t(n1.sp) == get_t(n2.sp)
#             return true
#         end
#     end
#     return false
# end
# function CRCBS.detect_action_conflict(n1::N,n2::N) where {S<:MState,A<:AbstractGraphAction,N<:PathNode{S,A}}
#     if _state_active(n1.sp) && _state_active(n2.sp)
#         if (get_e(n1.a).src == get_e(n2.a).dst) && (get_e(n1.a).dst == get_e(n2.a).src) && (get_t(n1.sp) == get_t(n2.sp))
#             return true
#         end
#     end
#     return false
# end

@with_kw struct MPCCBSEnv{E,ID<:BotID,T,C<:AbstractCostModel,H<:AbstractCostModel} <: GraphEnv{MState,MAction,C}
    search_env::E                   = nothing
    agent_id::ID                    = RobotID(-1)
    agent_idx::Int                  = get_id(agent_id)
    itineraries::Dict{BotID,Vector{ScheduleNode}} = Dict{BotID,Vector{ScheduleNode}}() # robot mission
    # itinerary::Vector{ScheduleNode} = Vector{ScheduleNode}() # robot mission
    # goal_sequence::Vector{GraphState} = map(construct_goal,itinerary)
    constraints::T                  = discrete_constraint_table(search_env,get_id(agent_id))
    cost_model::C                   = get_cost_model(search_env)
    heuristic::H                    = get_heuristic_model(search_env)
end
CRCBS.get_cost_model(env::MPCCBSEnv)       = env.cost_model
CRCBS.get_heuristic_model(env::MPCCBSEnv)  = env.heuristic
GraphUtils.get_graph(env::MPCCBSEnv)       = get_graph(env.search_env,graph_key(env.agent_id))
get_itinerary(env::MPCCBSEnv) = env.itineraries[env.agent_id]

function CRCBS.get_heuristic_cost(m::MultiStageEnvDistanceHeuristic,
    env::MPCCBSEnv,
    s::MState)
    goal_vtx = get_id(get_destination_location_id(get_itinerary(env)[get_stage(s)]))
    d = get_distance(get_graph(env),get_vtx(s),goal_vtx)
    d + CRCBS.cost_from_stage(m,get_id(env.agent_id),get_stage(s))
end

CRCBS.EnvDeadlineCost(sched::OperatingSchedule,args...) = EnvDeadlineCost()
CRCBS.EnvDeadlineCost{T}(sched::OperatingSchedule,args...) where {T} = EnvDeadlineCost{T}()
function CRCBS.compute_heuristic_cost(m::EnvDeadlineCost,h::MultiStageEnvDistanceHeuristic,env::MPCCBSEnv,cost,sp)
    # cost is travel time, h_cost is cost to go
    # need information about the current stage here...
    goal_vtx = get_id(get_destination_location_id(get_itinerary(env)[get_stage(sp)]))
    d = get_distance(get_graph(env),get_vtx(sp),goal_vtx)
    c = cost .+ d .- (get_tF(sp.node) .+ get_slack(sp.node))
    return m.f(max.(0.0, c))
end

construct_goal(::MPCCBSEnv,s::MState) = construct_goal(s.node)
function construct_goal(node::ScheduleNode)
    goal_idx = get_id(get_destination_location_id(node))
    goal_time = get_tF(node)
    GraphState(goal_idx,goal_time)
end
function check_stage_goal(::MPCCBSEnv,s,node=s.node)
    if get_t(s) >= get_tF(node)
        goal = construct_goal(node)
        if states_match(s, goal)
            return true
        elseif !CRCBS.is_valid(goal)
            return true
        end
    end
    return false
end
function CRCBS.get_next_state(env::MPCCBSEnv,s::MState,a::MAction)
    stage = get_stage(s)
    node = s.node
    sp = GraphState(get_e(a).dst,get_t(s)+get_dt(a))
    # update multiple nodes at once, if possible
    while (check_stage_goal(env,s,node) || check_stage_goal(env,sp,node)) && stage < length(get_itinerary(env))
        stage = stage + 1
        node = get_itinerary(env)[stage]
    end
    MState(
        get_vtx(sp),
        get_t(sp),
        stage,
        node,
        _state_active(s))
end
CRCBS.wait(::MPCCBSEnv,s)              = MAction(e=Edge(get_vtx(s),get_vtx(s)))

CRCBS.get_possible_actions(env::MPCCBSEnv,s::MState) = get_possible_actions(s.node,env,s)
function CRCBS.get_possible_actions(node,env::MPCCBSEnv,s::MState)
    if CRCBS.is_valid(env,s)
        return map(v2->MAction(e=Edge(s.vtx,v2)), outneighbors(get_graph(env),s.vtx))
    end
    return MAction[]
end
function CRCBS.get_possible_actions(node::Union{COLLECT,DEPOSIT},env::MPCCBSEnv,s::MState)
    # @assert get_vtx(s) == get_id(get_initial_location_id(node)) "Robot $(get_agent_id(env)) should not be at s=$(string(s)) when task=$(string(node))"
    if !(get_vtx(s) == get_id(get_initial_location_id(node)))
        @log_info(-1,0,"Robot $(get_agent_id(env)) should not be at s=$(string(s)) when task=$(string(node))")
    end
    if CRCBS.is_valid(env,s)
        return [CRCBS.wait(env,s)]
    end
    return MAction[]
end

function CRCBS.is_goal(env::MPCCBSEnv,s)
    if get_stage(s) >= length(get_itinerary(env))
        return check_stage_goal(env,s)
    end
    return false
end



### 
###
###
# """
#     MultiGoalPCMAPFSolver
# """
# struct MultiGoalPCMAPFSolver{C}
#     logger::SolverException{C}

# end
function solve_with_multi_goal_solver!(solver,pc_mapf,
        base_env = build_base_multi_goal_env(solver,pc_mapf),
        )
    priority_queue = PriorityQueue{BotID,Float64}()
    for (id,node) in base_env.itinerary
    end
    # formulate
    env = build_base_multi_goal_env(solver,pc_mapf)
    # 
end
function build_base_multi_goal_env(solver,pc_mapf;
    kwargs...
    )
    # unpack
    search_env = get_env(pc_mapf)
    sched = get_schedule(search_env)
    env_graph = get_graph(search_env)
    # itineraries 
    itineraries = Dict(
        id=>TaskGraphs.extract_robot_itinerary(sched,id) for id in keys(get_robot_ICs(sched)))
    vtx_sequences = Dict(get_id(k)=>map(n->get_vtx(TaskGraphs.construct_goal(n)),v) for (k,v) in itineraries)
    cost_to_go = CRCBS.construct_multi_stage_env_distance_heuristic(env_graph,vtx_sequences)
    # cost and heuristic models
    cost_model, _ = construct_cost_model(solver,search_env,EnvDeadlineCost();kwargs...)
    heuristic = construct_composite_heuristic(
        cost_to_go,
        NullHeuristic(),
        cost_to_go,
        cost_to_go,
        NullHeuristic(),
    )
    # Change route plan type in SearchEnv (Need MState instead of PCCBS.State)
    search_env = SearchEnv(
        schedule = get_schedule(search_env),
        env_layers = search_env.env_layers,
        cost_model = cost_model,
        heuristic_model = heuristic,
        num_agents = search_env.num_agents,
        route_plan = initialize_multi_goal_route_plan(sched,cost_model)
    )
    # base env
    env = TaskGraphs.MPCCBSEnv(
        search_env = search_env,
        itineraries = itineraries,
        cost_model = cost_model,
        heuristic = heuristic 
        )
    return env
end
function initialize_multi_goal_route_plan(sched::OperatingSchedule,cost_model)
    starts = MState[]
    robot_ics = get_robot_ICs(sched)
    for k in sort(collect(keys(robot_ics)))
        node = get_node(sched,k)
        push!(starts, MState(
            vtx = get_id(get_initial_location_id(node)),
            t = 0, 
            stage = 1,
            node = node,
            ))
    end
    paths = map(s->Path{MState,Action,cost_type(cost_model)}(s0=s, cost=get_initial_cost(cost_model)), starts)
    costs = map(p->get_cost(p), paths)
    cost = aggregate_costs(cost_model, costs)
    LowLevelSolution(paths=paths, cost_model=cost_model,costs=costs, cost=cost)
end