module PathPlanning

using Parameters
using LightGraphs
using MetaGraphs
using DataStructures
using MathOptInterface, JuMP
using TOML

using GraphUtils
using CRCBS

# using ..PCCBS
# using ..TaskGraphsCore
using ..TaskGraphs

export
    PC_TAPF_Solver,
    # reset_solver!,
    read_solver

@with_kw mutable struct PC_TAPF_Solver{T} <: AbstractMAPFSolver
    LIMIT_assignment_iterations   ::Int = 50
    LIMIT_CBS_iterations          ::Int = 100
    LIMIT_A_star_iterations       ::Int = 5000

    num_assignment_iterations   ::Int = 0
    num_CBS_iterations          ::Int = 0
    num_A_star_iterations       ::Int = 0
    best_cost                   ::T   = (Inf,Inf,Inf,Inf)

    total_assignment_iterations ::Int = 0
    total_CBS_iterations        ::Int = 0
    total_A_star_iterations     ::Int = 0

    max_CBS_iterations          ::Int = 0
    max_A_star_iterations       ::Int = 0

    verbosity                   ::Int = 0
    l1_verbosity                ::Int = 0
    l2_verbosity                ::Int = 0
    l3_verbosity                ::Int = 0
    l4_verbosity                ::Int = 0
    DEBUG                       ::Bool = false
end
function reset_solver!(solver::S) where {S<:PC_TAPF_Solver}
    solver.num_assignment_iterations = 0
    solver.num_CBS_iterations = 0
    solver.num_A_star_iterations = 0
    solver.best_cost = (Inf,Inf,Inf,Inf)

    solver.total_assignment_iterations = 0
    solver.total_CBS_iterations = 0
    solver.total_A_star_iterations = 0

    solver.max_CBS_iterations = 0
    solver.max_A_star_iterations = 0

    solver
end
# update functions
function enter_assignment!(solver::S) where {S<:PC_TAPF_Solver}
    reset_solver!(solver)
end
function exit_assignment!(solver::S) where {S<:PC_TAPF_Solver}
end
function enter_cbs!(solver::S) where {S<:PC_TAPF_Solver}
    solver.num_assignment_iterations += 1
end
function exit_cbs!(solver::S) where {S<:PC_TAPF_Solver}
    solver.max_CBS_iterations = max(solver.max_CBS_iterations,solver.num_CBS_iterations)
    solver.total_CBS_iterations += solver.num_CBS_iterations
    solver.num_CBS_iterations = 0
end
function enter_low_level!(solver::S) where {S<:PC_TAPF_Solver}
    solver.num_CBS_iterations += 1
end
function exit_low_level!(solver::S) where {S<:PC_TAPF_Solver}
end
function enter_a_star!(solver::S) where {S<:PC_TAPF_Solver}
end
function exit_a_star!(solver::S) where {S<:PC_TAPF_Solver}
    solver.max_A_star_iterations = max(solver.max_A_star_iterations,solver.num_A_star_iterations)
    solver.total_A_star_iterations += solver.num_A_star_iterations
    solver.num_A_star_iterations = 0
end
function TOML.parse(solver::S) where {S<:PC_TAPF_Solver}
    toml_dict = Dict()

    toml_dict["LIMIT_assignment_iterations"] = solver.LIMIT_assignment_iterations
    toml_dict["LIMIT_CBS_iterations"] = solver.LIMIT_CBS_iterations
    toml_dict["LIMIT_A_star_iterations"] = solver.LIMIT_A_star_iterations
    toml_dict["num_assignment_iterations"] = solver.num_assignment_iterations
    toml_dict["num_CBS_iterations"] = solver.num_CBS_iterations
    toml_dict["num_A_star_iterations"] = solver.num_A_star_iterations
    if any(i->i==Inf, solver.best_cost)
        toml_dict["best_cost"] = map(i->-1,1:length(solver.best_cost))
    else
        toml_dict["best_cost"] = collect(solver.best_cost)
    end
    toml_dict["total_assignment_iterations"] = solver.total_assignment_iterations
    toml_dict["total_CBS_iterations"] = solver.total_CBS_iterations
    toml_dict["total_A_star_iterations"] = solver.total_A_star_iterations
    toml_dict["max_CBS_iterations"] = solver.max_CBS_iterations
    toml_dict["max_A_star_iterations"] = solver.max_A_star_iterations
    toml_dict["verbosity"] = solver.verbosity
    toml_dict["l1_verbosity"] = solver.l1_verbosity
    toml_dict["l2_verbosity"] = solver.l2_verbosity
    toml_dict["l3_verbosity"] = solver.l3_verbosity
    toml_dict["l4_verbosity"] = solver.l4_verbosity
    toml_dict["DEBUG"]        = solver.DEBUG

    toml_dict
end
function read_solver(toml_dict::Dict)
    solver = PC_TAPF_Solver(
        LIMIT_assignment_iterations = toml_dict["LIMIT_assignment_iterations"],
        LIMIT_CBS_iterations = toml_dict["LIMIT_CBS_iterations"],
        LIMIT_A_star_iterations = toml_dict["LIMIT_A_star_iterations"],

        num_assignment_iterations = toml_dict["num_assignment_iterations"],
        num_CBS_iterations = toml_dict["num_CBS_iterations"],
        num_A_star_iterations = toml_dict["num_A_star_iterations"],
        best_cost = Tuple(toml_dict["best_cost"]),

        total_assignment_iterations = toml_dict["total_assignment_iterations"],
        total_CBS_iterations = toml_dict["total_CBS_iterations"],
        total_A_star_iterations = toml_dict["total_A_star_iterations"],

        max_CBS_iterations = toml_dict["max_CBS_iterations"],
        max_A_star_iterations = toml_dict["max_A_star_iterations"],

        verbosity = toml_dict["verbosity"],
        l1_verbosity = toml_dict["l1_verbosity"],
        l2_verbosity = toml_dict["l2_verbosity"],
        l3_verbosity = toml_dict["l3_verbosity"],
        l4_verbosity = toml_dict["l4_verbosity"],
        DEBUG = toml_dict["DEBUG"]
    )
end
function read_solver(io)
    read_solver(TOML.parsefile(io))
end

# Helpers for printing
function log_info(limit::Int,verbosity::Int,msg...)
    if verbosity > limit
        println(msg...)
    end
end
function log_info(limit::Int,solver::S,msg...) where {S<:PC_TAPF_Solver}
    log_info(limit,solver.verbosity,msg...)
end

export
    PlanningCache,
    initialize_planning_cache,
    reset_cache!,
    update_planning_cache!,
    repair_solution!,
    default_pc_tapf_solution,
    plan_next_path!

@with_kw struct PlanningCache
    closed_set::Set{Int}    = Set{Int}()    # nodes that are completed
    active_set::Set{Int}    = Set{Int}()    # active nodes
    node_queue::PriorityQueue{Int,Float64} = PriorityQueue{Int,Float64}() # active nodes prioritized by slack
    t0::Vector{Int}         = Vector{Int}()
    tF::Vector{Int}         = Vector{Int}()
    slack::Vector{Vector{Float64}}       = Vector{Vector{Float64}}()
    local_slack::Vector{Vector{Float64}} = Vector{Vector{Float64}}()
end

function initialize_planning_cache(schedule::ProjectSchedule)
    t0,tF,slack,local_slack = process_schedule(schedule);
    cache = PlanningCache(t0=t0,tF=tF,slack=slack,local_slack=local_slack)
    for v in vertices(get_graph(schedule))
        if is_root_node(get_graph(schedule),v)
            push!(cache.active_set,v)
            enqueue!(cache.node_queue,v=>minimum(cache.slack[v])) # need to store slack
        end
    end
    cache
end

"""
    `reset_cache!(cache,schedule)`

    Resets the cache so that a solution can be repaired (otherwise calling
    low_level_search!() will return immediately because the cache says it's
    complete)
"""
function reset_cache!(cache::PlanningCache,schedule::ProjectSchedule)
    t0,tF,slack,local_slack = process_schedule(schedule;t0=cache.t0,tF=cache.tF)
    cache.t0            .= t0
    cache.tF            .= tF
    cache.slack         .= slack
    cache.local_slack   .= local_slack
    while length(cache.closed_set) > 0
        pop!(cache.closed_set)
    end
    while length(cache.active_set) > 0
        pop!(cache.active_set)
    end
    while length(cache.node_queue) > 0
        dequeue!(cache.node_queue)
    end
    for v in vertices(get_graph(schedule))
        if is_root_node(get_graph(schedule),v)
            push!(cache.active_set,v)
            enqueue!(cache.node_queue,v=>minimum(cache.slack[v])) # need to store slack
        end
    end
    cache
end
function update_planning_cache!(solver::M,cache::C,schedule::S,v::Int,path::P) where {M<:PC_TAPF_Solver,C<:PlanningCache,S<:ProjectSchedule,P<:Path}
    active_set = cache.active_set
    closed_set = cache.closed_set
    node_queue = cache.node_queue
    graph = get_graph(schedule)

    # update t0, tF, slack, local_slack
    if get_final_state(path).t > cache.tF[v]
        log_info(2,solver,string("# Updating cache.tF[v]"))
        cache.tF[v] = get_final_state(path).t
        t0,tF,slack,local_slack = process_schedule(schedule;t0=cache.t0,tF=cache.tF)
        cache.t0            .= t0
        cache.tF            .= tF
        cache.slack         .= slack
        cache.local_slack   .= local_slack
    end
    # update closed_set
    log_msg = string("# moving ", v," from active_set to closed_set ... moving (")
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
            log_msg = log_msg * string(" ",v2,",")
            push!(active_set, v2)               # add to active set
        end
    end
    log_info(2,solver,log_msg * string(") to active set"))
    # update priority queue
    for v2 in active_set
        node_queue[v2] = minimum(cache.slack[v2])
    end
    return cache
end

export
    SearchEnv,
    construct_search_env,
    update_env!

const State = PCCBS.State
const Action = PCCBS.Action
@with_kw struct SearchEnv{C,E<:AbstractLowLevelEnv{PCCBS.State,PCCBS.Action,C}} <: AbstractLowLevelEnv{PCCBS.State,PCCBS.Action,C}
    schedule::ProjectSchedule   = ProjectSchedule()
    cache::PlanningCache        = PlanningCache()
    env::E                      = PCCBS.LowLevelEnv()
    cost_model::C               = get_cost_model(env)
    num_agents::Int             = -1
end
function CRCBS.get_start(env::SearchEnv,v::Int)
    start_vtx   = get_path_spec(env.schedule,v).start_vtx
    start_time  = env.cache.t0[v]
    PCCBS.State(start_vtx,start_time)
end

function CRCBS.SumOfMakeSpans(schedule::S,cache::C) where {S<:ProjectSchedule,C<:PlanningCache}
    SumOfMakeSpans(
        cache.tF,
        schedule.root_nodes,
        map(k->schedule.weights[k], schedule.root_nodes),
        cache.tF[schedule.root_nodes])
end
function CRCBS.MakeSpan(schedule::S,cache::C) where {S<:ProjectSchedule,C<:PlanningCache}
    MakeSpan(
        cache.tF,
        schedule.root_nodes,
        map(k->schedule.weights[k], schedule.root_nodes),
        cache.tF[schedule.root_nodes])
end

function construct_search_env(schedule, problem_spec, env_graph;
    primary_objective=SumOfMakeSpans,
    extra_T=100)
    # schedule = construct_project_schedule(project_spec, problem_spec, robot_ICs, assignments)
    cache = initialize_planning_cache(schedule)
    # Paths stored as seperate chunks (will need to be concatenated before conflict checking)
    N = problem_spec.N                                          # number of robots
    starts = Vector{PCCBS.State}()
    goals = Vector{PCCBS.State}()
    for path_id in sort(collect(keys(schedule.path_id_to_vtx_map)))
        v = schedule.path_id_to_vtx_map[path_id]
        start_vtx = get_path_spec(schedule,v).start_vtx
        final_vtx = get_path_spec(schedule,v).final_vtx
        @assert (start_vtx != -1) string("v = ",v,", start_vtx = ",start_vtx,", path_id = ",path_id)
        push!(starts, PCCBS.State(vtx = start_vtx, t = cache.t0[v]))
        push!(goals, PCCBS.State(vtx = final_vtx, t = cache.tF[v]))
    end
    cost_model = construct_composite_cost_model(
        primary_objective(schedule,cache),
        # FullDeadlineCost(DeadlineCost(0.0)),
        HardConflictCost(env_graph,maximum(cache.tF)+extra_T, N),
        SumOfTravelDistance(),
        FullCostModel(sum,NullCost()) # SumOfTravelTime(),
    )
    heuristic_model = construct_composite_heuristic(
        DefaultPerfectHeuristic(PerfectHeuristic(env_graph,map(s->s.vtx,starts),map(s->s.vtx,goals))),
        NullHeuristic(),
        DefaultPerfectHeuristic(PerfectHeuristic(env_graph,map(s->s.vtx,starts),map(s->s.vtx,goals))),
        DefaultPerfectHeuristic(PerfectHeuristic(env_graph,map(s->s.vtx,starts),map(s->s.vtx,goals)))
    )
    # This mapf stores the starts and goals for the actual stages (indexed by path_id)
    mapf = MAPF(PCCBS.LowLevelEnv(
        graph=env_graph,
        cost_model=cost_model,
        heuristic=heuristic_model
        ),starts,goals)
    # This search node maintains the full path for each agent (indexed by agent_id)
    # node = initialize_root_node(MAPF(mapf.env,map(i->PCCBS.State(),1:N),map(i->PCCBS.State(),1:N)))

    env = SearchEnv(schedule=schedule, cache=cache, env=mapf.env, num_agents=N)
    return env, mapf #, node
end
function construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph;kwargs...)
    schedule = construct_project_schedule(project_spec, problem_spec, robot_ICs, assignments)
    construct_search_env(schedule, problem_spec, env_graph;kwargs...)
end

function default_pc_tapf_solution(N::Int;extra_T=100)
    c0 = (0.0, 0.0, 0.0, 0.0)
    LowLevelSolution(
        paths=map(i->Path{State,Action,typeof(c0)}(s0=State(),cost=c0),1:N),
        costs=map(i->c0,1:N),
        cost=c0,
        cost_model = construct_composite_cost_model(
                SumOfMakeSpans(Float64[0],Int64[1],Float64[1],Float64[0]),
                # FullDeadlineCost(DeadlineCost(0.0)),
                HardConflictCost(DiGraph(10), 10+extra_T, N),
                SumOfTravelDistance(),
                FullCostModel(sum,NullCost())
            )
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
update_cost_model!(env::S) where {S<:SearchEnv} = update_cost_model!(env.cost_model,env)
# function finalize_cost!(env::S,cost_model::M,cost::T) where {S<:SearchEnv,M<:AbstractCostModel,T}
#     cost
# end
# function finalize_cost!(env::S,cost_model::M,cost::T) where {S<:SearchEnv,M<:CompositeCostModel,T}
#     T(map(i->finalize_cost!(env,cost_model.models[i],cost[i]), 1:length(cost_model.models)))
# end
# function finalize_cost!(env::S,cost_model::M,cost::T) where {S<:SearchEnv,M<:SumOfMakeSpans,T}
#     t0,tF,slack,local_slack = process_schedule(env.schedule;t0=env.cache.t0,tF=env.cache.tF)
# end

"""
    `update_env!`

    `v` is the vertex id
"""
function update_env!(solver::S,env::E,v::Int,path::P) where {S<:PC_TAPF_Solver,E<:SearchEnv,P<:Path}
    cache = env.cache
    schedule = env.schedule

    # UPDATE CACHE
    update_planning_cache!(solver,cache,schedule,v,path)
    update_cost_model!(env)
    # ADD UPDATED PATH TO HEURISTIC MODELS
    agent_id    = get_path_spec(schedule, v).agent_id
    if agent_id != -1
        partially_set_path!(get_heuristic_model(env.env),agent_id,convert_to_vertex_lists(path))
        partially_set_path!(get_cost_model(env.env),agent_id,convert_to_vertex_lists(path))
    end

    env
end

function CRCBS.build_env(solver, env::E, mapf::M, node::N, agent_id::Int, path_id::Int,dt=0) where {E<:SearchEnv,M<:AbstractMAPF,N<:ConstraintTreeNode}
    v = env.schedule.path_id_to_vtx_map[path_id]
    node_id = get_vtx_id(env.schedule, v)
    schedule_node = get_node_from_id(env.schedule, node_id)
    # log_info(-1,solver,string("v= ",v,", path_id= ", path_id, ", agent_id= ",agent_id,", nodetype=",typeof(schedule_node)))
    # @show v,node_id,schedule_node,agent_id
    goal_vtx = mapf.goals[path_id].vtx
    goal_time = env.cache.tF[v]                             # time after which goal can be satisfied
    deadline = env.cache.tF[v] .+ env.cache.slack[v]         # deadline for DeadlineCost
    # Adjust deadlines if necessary:
    if get_path_spec(env.schedule, v).tight == true
        goal_time += minimum(env.cache.local_slack[v])
    end
    for v_next in outneighbors(get_graph(env.schedule),v)
        if get_path_spec(env.schedule, v_next).static == true
            duration_next = get_path_spec(env.schedule,v_next).min_path_duration
            for c in get_constraints(node, agent_id).sorted_state_constraints
                if get_sp(get_path_node(c)).vtx == goal_vtx
                    if 0 < get_time_of(c) - goal_time < duration_next
                        log_info(1,solver,string("extending goal_time for node ",v," from ",goal_time," to ",get_time_of(c)," to avoid constraints"))
                        goal_time = max(goal_time, get_time_of(c))
                    end
                end
            end
        end
    end
    if (get_path_spec(env.schedule, v).free == true) && is_terminal_node(get_graph(env.schedule),v)
        goal_time = maximum(env.cache.tF)
        goal_vtx = -1
        # deadline = Inf # already taken care of, perhaps?
        log_info(3,solver,string("BUILD ENV: setting goal_vtx = ",goal_vtx,", t = maximum(cache.tF) = ",goal_time))
    end
    cbs_env = typeof(mapf.env)(
        graph       = mapf.env.graph,
        constraints = get_constraints(node, agent_id), # agent_id represents the whole path
        goal        = PCCBS.State(goal_vtx,goal_time),
        agent_idx   = agent_id, # this is only used for the HardConflictTable, which can be updated via the combined search node
        heuristic   = get_heuristic_model(mapf.env),
        cost_model  = get_cost_model(mapf.env)       # TODO update the cost model
        )
    # update deadline in DeadlineCost
    set_deadline!(get_cost_model(cbs_env),deadline)
    # retrieve base_path
    if !is_root_node(get_graph(env.schedule),v) # && agent_id != -1
        # log_info(0,solver,string("retrieving base path for node ",v," of type ",typeof(schedule_node)))
        base_path = get_paths(node.solution)[agent_id]
    else
        # log_info(0,solver,string("initializing base path for node ",v," of type ",typeof(schedule_node)))
        base_path = Path{PCCBS.State,PCCBS.Action,get_cost_type(cbs_env)}(
            s0=get_start(env,v), cost=get_initial_cost(cbs_env))
    end
    # base_path = get(get_paths(node.solution), agent_id, Path{PCCBS.State,PCCBS.Action,get_cost_type(cbs_env)}(
    #     s0=get_start(env,v), cost=get_initial_cost(cbs_env)))
    log_info(2,solver,
    """
    agent_id = $(agent_id)
    path_id = $(path_id)
    node_type = $(typeof(schedule_node))
    v = $(v)
    deadline = $(deadline)
    slack = $(env.cache.slack[v])
    goal = ($(cbs_env.goal.vtx),$(cbs_env.goal.t))
    """
    )
    return cbs_env, base_path
end

"""
    `plan_next_path`

    Computes nxxt path specified by the project schedule and updates the
    solution in the ConstraintTreeNode::node accordingly.
"""
function plan_next_path!(solver::S, env::E, mapf::M, node::N;
        heuristic=get_heuristic_cost,
        path_finder=A_star
        ) where {S<:PC_TAPF_Solver,E<:SearchEnv,M<:AbstractMAPF,N<:ConstraintTreeNode}

    enter_a_star!(solver)
    cache = env.cache
    schedule = env.schedule

    if length(cache.node_queue) > 0
        v = dequeue!(cache.node_queue)
        path_id = get_path_spec(schedule, v).path_id
        if path_id != -1
            agent_id = get_path_spec(schedule, v).agent_id
            cbs_env, base_path = build_env(solver, env, mapf, node, agent_id, path_id)
            # make sure base_path hits t0 constraint
            if get_end_index(base_path) < cache.t0[v]
                node_id = get_vtx_id(schedule,v)
                schedule_node = get_node_from_id(schedule,node_id)
                log_info(-1, solver, string("# LOW LEVEL SEARCH: in schedule node ",v," of type ",
                    typeof(schedule_node),", cache.t0[v] - get_end_index(base_path) = ",cache.t0[v] - get_end_index(base_path),". Extending path to ",cache.t0[v]," ..."))
                # log_info(-1, solver, string("# base path = ",convert_to_vertex_lists(base_path), ",  vache.t0[v] = ",cache.t0[v]))
                # log_info(-1, solver, base_path.s0)
                # log_info(-1, solver, string(convert_to_vertex_lists(base_path)))
                # log_info(-1, solver, get_final_state(base_path).vtx )
                # log_info(-1, solver, cbs_env.cost_model.cost_models[2].model.table.CAT)
                extend_path!(cbs_env,base_path,cache.t0[v])
            end
            # Solve!
            node_id = get_vtx_id(schedule,v)
            schedule_node = get_node_from_id(schedule,node_id)
            if typeof(schedule_node) <: Union{COLLECT,DEPOSIT} # Must sit and wait the whole time
                path = base_path
                extend_path!(cbs_env,path,cache.tF[v])
                cost = get_cost(path)
                solver.DEBUG ? validate(path,v) : nothing
            else
                solver.DEBUG ? validate(base_path,v) : nothing
                path, cost = path_finder(solver, cbs_env, base_path, heuristic;verbose=(solver.verbosity > 3))
                # if !CRCBS.is_valid(path, get_final_state(base_path), cbs_env.goal)
                if cost == get_infeasible_cost(cbs_env)
                    log_info(-1,solver,"# A*: returned infeasible path ... Exiting early")
                    return false
                end
                solver.DEBUG ? validate(path,v,cbs_env) : nothing
            end
            log_info(2,solver,string("LOW LEVEL SEARCH: solver.num_A_star_iterations = ",solver.num_A_star_iterations))
            # Make sure every robot sticks around for the entire time horizon
            if is_terminal_node(get_graph(schedule),v)
                log_info(2,solver,string("LOW LEVEL SEARCH: Extending terminal node",
                " (SHOULD NEVER HAPPEN IF THINGS ARE WORKING CORRECTLY)"))
                extend_path!(cbs_env,path,maximum(cache.tF))
                solver.DEBUG ? validate(path,v) : nothing
            end
            # add to solution
            set_solution_path!(node.solution, path, agent_id)
            set_path_cost!(node.solution, cost, agent_id)
            # update
            update_env!(solver,env,v,path)
            # TODO incorporate multi-headed cost
            node.solution.cost = aggregate_costs(get_cost_model(env.env),get_path_costs(node.solution))
            node.cost = get_cost(node.solution)
            # Print for DEBUGging
            log_info(3,solver,string("agent_path = ", convert_to_vertex_lists(path)))
            log_info(3,solver,string("cost = ", get_cost(path)))
            if node.cost >= solver.best_cost
                log_info(0,solver,"# LOW LEVEL SEARCH: node.cost >= solver.best_cost ... Exiting early")
                return false
            end
            # Debugging
            # vtx_lists = convert_to_vertex_lists(node.solution)
            # for (i,p) in enumerate(vtx_lists)
            #     log_info(-1,solver,string("path_",i," = ",p))
            # end
            # log_info(-1,solver,"\n\n")
        else
            # TODO parameterize env so that I don't have to hard-code the types here
            path = Path{PCCBS.State,PCCBS.Action,get_cost_type(mapf.env)}(
                s0=get_start(env,v),
                cost=get_initial_cost(mapf.env)
                )
            # update
            update_env!(solver,env,v,path)
        end
    end
    exit_a_star!(solver)
    return true
end

"""
    Computes all paths specified by the project schedule and updates the
    solution in the ConstraintTreeNode::node accordingly.
    `invalidated_id` is the path_id corresponding to a path that has been
    invalidated by a constraint.
"""
function CRCBS.low_level_search!(
        solver::S, env::E, mapf::M, node::N;
        heuristic=get_heuristic_cost,
        path_finder=A_star
        ) where {S<:PC_TAPF_Solver,E<:SearchEnv,M<:AbstractMAPF,N<:ConstraintTreeNode}

    # log_info(-1,solver,"low_level_search!")
    # vtx_lists = convert_to_vertex_lists(node.solution)
    # for (i,p) in enumerate(vtx_lists)
    #     log_info(-1,solver,string("path_",i," = ",p))
    # end
    # log_info(-1,solver,"\n\n")

    while length(env.cache.node_queue) > 0
        if !(plan_next_path!(solver,env,mapf,node;heuristic=heuristic,path_finder=path_finder))
            return false
        end
    end
    return true
end

"""
    `repair_solution!(solver, env, mapf, node, invalidated_id;
        heuristic=heuristic, path_finder=path_finder)`

    Resets env.cache and runs low_level_search!() again so that conflicts can be
    repaired before CBS has to branch.
"""
function repair_solution!(
        solver::S, env::E, mapf::M, node::N;
        heuristic=get_heuristic_cost,
        path_finder=A_star
        ) where {S<:PC_TAPF_Solver,E<:SearchEnv,M<:AbstractMAPF,N<:ConstraintTreeNode}

    reset_cache!(env.cache,env.schedule)
    low_level_search!(solver, env, mapf, node;
        heuristic=heuristic, path_finder=path_finder)
end


################################################################################
############################## CBS Wrapper Stuff ###############################
################################################################################

export
    PC_MAPF

"""
    `PC_MAPF`

    A precedence-constrained multi-agent path-finding problem. All agents have
    assigned tasks, but there are precedence constraints between tasks.
"""
struct PC_MAPF{E<:SearchEnv,M<:AbstractMAPF} <: AbstractMAPF
    env::E
    mapf::M
end
CRCBS.get_cost_model(env::E) where {E<:SearchEnv}   = get_cost_model(env.env)
CRCBS.get_cost_type(env::E) where {E<:SearchEnv}    = get_cost_type(env.env)
function CRCBS.initialize_root_node(pc_mapf::P) where {P<:PC_MAPF}
    N = pc_mapf.env.num_agents
    # It is important to only have N starts! Does not matter if they are invalid states.
    initialize_root_node(MAPF(pc_mapf.mapf.env,map(i->PCCBS.State(),1:N),map(i->PCCBS.State(),1:N)))
end
CRCBS.default_solution(pc_mapf::M) where {M<:PC_MAPF} = default_solution(pc_mapf.mapf)

function CRCBS.check_termination_criteria(solver::S,env::E,cost_so_far,path,s) where {S<:PC_TAPF_Solver,E<:AbstractLowLevelEnv}
    solver.num_A_star_iterations += 1
    if solver.num_A_star_iterations > solver.LIMIT_A_star_iterations
        log_info(0,solver,string("A_star: max iterations exceeded: iterations = ",solver.num_A_star_iterations))
        return true
    end
    return false
end

function CRCBS.low_level_search!(
        solver::S, pc_mapf::M, node::N, idxs::Vector{Int}=Vector{Int}();
        heuristic=get_heuristic_cost, path_finder=A_star
        ) where {S<:PC_TAPF_Solver,M<:PC_MAPF,N<:ConstraintTreeNode}

    enter_low_level!(solver)
    reset_cache!(pc_mapf.env.cache, pc_mapf.env.schedule)
    valid_flag = low_level_search!(solver, pc_mapf.env, pc_mapf.mapf, node)
    if !valid_flag # return early
        return valid_flag
    end
    valid_flag = repair_solution!(solver, pc_mapf.env, pc_mapf.mapf, node)
    exit_low_level!(solver)
    return valid_flag
end

function CRCBS.is_valid(solution::S,pc_mapf::P) where {S<:LowLevelSolution,P<:PC_MAPF}
    # TODO actually implement this
    return true
end

function CRCBS.solve!(
        solver::PC_TAPF_Solver,
        mapf::M where {M<:PC_MAPF},
        path_finder=A_star)

    enter_cbs!(solver)

    priority_queue = PriorityQueue{ConstraintTreeNode,get_cost_type(mapf.env)}()

    root_node = initialize_root_node(mapf)
    valid_flag = low_level_search!(solver,mapf,root_node;path_finder=path_finder)
    detect_conflicts!(root_node.conflict_table,root_node.solution)
    if valid_flag
        enqueue!(priority_queue, root_node => root_node.cost)
    end

    while length(priority_queue) > 0
        node = dequeue!(priority_queue)
        log_info(1,solver,string("CBS: node.cost = ",get_cost(node.solution)))
        # check for conflicts
        conflict = get_next_conflict(node.conflict_table)
        if !CRCBS.is_valid(conflict)
            log_info(-1,solver,string("CBS: Optimal Solution Found! Cost = ",node.cost))
            return node.solution, node.cost
        end
        log_info(1,solver,string("CBS: ", string(conflict)))
            # "CBS: ",conflict_type(conflict),
            # ": agent1=",agent1_id(conflict),
            # ", agent2=", agent2_id(conflict),
            # ", v1=(",get_s(node1(conflict)).vtx,",",get_sp(node1(conflict)).vtx,")",
            # ", v2=(",get_s(node2(conflict)).vtx,",",get_sp(node2(conflict)).vtx,")",
            # ", t=",get_s(node1(conflict)).t))
        # otherwise, create constraints and branch
        constraints = generate_constraints_from_conflict(conflict)
        for constraint in constraints
            new_node = initialize_child_search_node(node)
            if CRCBS.add_constraint!(new_node,constraint)
                log_info(1,solver,string("CBS: iteration ", solver.num_CBS_iterations))
                log_info(1,solver,string("CBS: constraint on agent id = ",get_agent_id(constraint),", time index = ",get_time_of(constraint)))
                valid_flag = low_level_search!(solver, mapf, new_node, [get_agent_id(constraint)]; path_finder=path_finder)
                detect_conflicts!(new_node.conflict_table,new_node.solution,[get_agent_id(constraint)]) # update conflicts related to this agent
                if valid_flag && CRCBS.is_valid(new_node.solution, mapf)
                    # TODO update env (i.e. update heuristic, etc.)
                    enqueue!(priority_queue, new_node => new_node.cost)
                end
            end
        end
        if solver.num_CBS_iterations > solver.LIMIT_CBS_iterations
            println("CBS: Maximum allowable CBS iterations reached. Exiting with infeasible solution ... ")
            break
        end
    end
    log_info(0,solver,"No Solution Found. Returning default solution")
    exit_cbs!(solver)
    return default_solution(mapf)
end

################################################################################
############################## Outer Loop Search ###############################
################################################################################
export
    high_level_search!,
    high_level_search_mod!

"""
    `high_level_search!`
"""
function high_level_search!(solver::P, env_graph, project_spec, problem_spec,
        robot_ICs, optimizer;
        primary_objective=SumOfMakeSpans,
        kwargs...
        ) where {P<:PC_TAPF_Solver}

    enter_assignment!(solver)
    log_info(0,solver,string("\nHIGH LEVEL SEARCH: beginning search ..."))

    # forbidden_solutions = Vector{Matrix{Int}}() # solutions to exclude from assignment module output
    lower_bound = 0.0
    best_solution = default_pc_tapf_solution(problem_spec.N)
    best_env    = SearchEnv()
    best_assignment = Vector{Int}()
    model = formulate_optimization_problem(problem_spec,optimizer;cost_model=primary_objective,kwargs...);

    while solver.best_cost[1] > lower_bound
        solver.num_assignment_iterations += 1
        log_info(0,solver,string("HIGH LEVEL SEARCH: iteration ",solver.num_assignment_iterations,"..."))
        ############## Task Assignment ###############
        # add constraints to get next best solution
        exclude_solutions!(model) #, problem_spec.M, forbidden_solutions)
        optimize!(model)
        optimal = (termination_status(model) == MathOptInterface.OPTIMAL);
        if !optimal
            log_info(0,solver,string(
                "HIGH LEVEL SEARCH: Task Assignment failed. Returning best solution so far.\n",
                " * optimality gap = ", solver.best_cost[1] - lower_bound))
            return best_solution, best_assignment, solver.best_cost, best_env
        end
        optimal_TA_cost = Int(round(value(objective_function(model)))); # lower bound on cost (from task assignment module)
        lower_bound = max(lower_bound, optimal_TA_cost)
        log_info(0,solver,string("HIGH LEVEL SEARCH: Current lower bound cost = ",lower_bound))
        assignment_matrix = get_assignment_matrix(model);
        assignments = get_assignment_vector(assignment_matrix,problem_spec.M)
        log_info(0,solver,string("HIGH LEVEL SEARCH: Current assignment vector = ",assignments))
        if lower_bound < solver.best_cost[1] # TODO make sure that the operation duration is accounted for here
            ############## Route Planning ###############
            env, mapf = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph;
                primary_objective=primary_objective);
            pc_mapf = PC_MAPF(env,mapf);
            ##### Call CBS Search Routine (LEVEL 2) #####
            solution, cost = solve!(solver,pc_mapf);
            if cost < solver.best_cost
                best_solution = solution
                best_assignment = assignments
                solver.best_cost = cost # TODO make sure that the operation duration is accounted for here
                best_env = env
            end
            log_info(0,solver,string("HIGH LEVEL SEARCH: Best cost so far = ", solver.best_cost[1]))
        end
    end
    exit_assignment!(solver)
    log_info(-1,solver.verbosity,string("HIGH LEVEL SEARCH: optimality gap = ",solver.best_cost[1] - lower_bound,". Returning best solution with cost ", solver.best_cost,"\n"))
    return best_solution, best_assignment, solver.best_cost, best_env
end
"""
    This is the modified version of high-level search that uses the adjacency
    matrix MILP formulation.
"""
function high_level_search_mod!(solver::P, env_graph, project_spec, problem_spec,
        robot_ICs, optimizer;
        milp_model=AdjacencyMILP(),
        primary_objective=SumOfMakeSpans,
        kwargs...) where {P<:PC_TAPF_Solver}

    enter_assignment!(solver)
    log_info(0,solver,string("\nHIGH LEVEL SEARCH: beginning search ..."))

    lower_bound = 0.0
    best_solution = default_pc_tapf_solution(problem_spec.N)
    best_env    = SearchEnv()
    best_assignment = Vector{Int}()
    project_schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
    model = formulate_milp(milp_model,project_schedule,problem_spec;
        cost_model=primary_objective,optimizer=optimizer,kwargs...)

    while solver.best_cost[1] > lower_bound
        solver.num_assignment_iterations += 1
        log_info(0,solver,string("HIGH LEVEL SEARCH: iteration ",solver.num_assignment_iterations,"..."))
        ############## Task Assignment ###############
        exclude_solutions!(model) # exclude most recent solution in order to get next best solution
        optimize!(model)
        optimal = (termination_status(model) == MathOptInterface.OPTIMAL);
        if !optimal
            log_info(0,solver,string("HIGH LEVEL SEARCH: Task assignment failed. Returning best solution so far.\n",
                " * optimality gap = ", solver.best_cost[1] - lower_bound))
            return best_solution, best_assignment, solver.best_cost, best_env
        end
        optimal_TA_cost = Int(round(value(objective_function(model)))); # lower bound on cost (from task assignment module)
        lower_bound = max(lower_bound, optimal_TA_cost)
        log_info(0,solver,string("HIGH LEVEL SEARCH: Current lower bound cost = ",lower_bound))
        assignment_matrix = get_assignment_matrix(model);
        assignments = get_assignment_vector(assignment_matrix,problem_spec.M)
        if lower_bound < solver.best_cost[1]
            ############## Route Planning ###############
            update_project_schedule!(project_schedule,problem_spec,assignment_matrix)
            env, mapf = construct_search_env(project_schedule, problem_spec, env_graph;
                primary_objective=primary_objective);
            pc_mapf = PC_MAPF(env,mapf);
            ##### Call CBS Search Routine (LEVEL 2) #####
            solution, cost = solve!(solver,pc_mapf);
            if cost < solver.best_cost
                best_solution = solution
                best_assignment = assignments
                solver.best_cost = cost # TODO make sure that the operation duration is accounted for here
                best_env = env
            end
            log_info(0,solver,string("HIGH LEVEL SEARCH: Best cost so far = ", solver.best_cost[1]))
        end
    end
    exit_assignment!(solver)
    log_info(-1,solver.verbosity,string("HIGH LEVEL SEARCH: optimality gap = ",solver.best_cost[1] - lower_bound,". Returning best solution with cost ", solver.best_cost,"\n"))
    return best_solution, best_assignment, solver.best_cost, best_env
end

high_level_search!(milp_model::AssignmentMILP, args...;kwargs...) = high_level_search!(args...;kwargs...)
high_level_search!(milp_model::AdjacencyMILP, args...;kwargs...) = high_level_search_mod!(args...;milp_model=milp_model,kwargs...)
high_level_search!(milp_model::SparseAdjacencyMILP, args...;kwargs...) = high_level_search_mod!(args...;milp_model=milp_model,kwargs...)

export
    freeze_assignments!,
    freeze!

"""
    freeze_assignments!
"""
function freeze_assignments!(schedule::P,cache::C,t) where {P<:ProjectSchedule,C<:PlanningCache}
    assignments = Set{Int}()
    for v in topological_sort_by_dfs(get_graph(schedule))
        node = get_node_from_id(schedule, get_vtx_id(schedule, v))
        if cache.t0[v] <= t
            if typeof(node) <: COLLECT
                # if !(typeof(node) <: GO)
                    # assignments[get_id(get_robot_id(node))] = get_id(get_object_id(node))
                push!(assignments, get_id(get_object_id(node)))
                # end
            end
        end
    end
    assignments
end

"""
    get_env_snapshot
"""
function get_env_snapshot(solution::S,t) where {S<:LowLevelSolution}
    r0 = map(path->get_vtx(get_s(get_path_node(path,t))), get_paths(solution))
end

"""
    `freeze!`

    Prepares a search environment for replanning.

    Args:
    - `solution` - must be a VALID solution
    - `schedule::ProjectSchedule`
    - `Ta::Int` - assignment freeze deadline. All delivery tasks that will have
        begun by time `Ta` (i.e., the COLLECT action begins before then) must be
        completed by the same robot.
    - `Tr::Int` - routing freeze deadline. All route plans must be frozen up to
        time `Tr`.
"""
function freeze!(solution::S,env::E,Ta::Int,Tr::Int) where {S<:LowLevelSolution,E<:SearchEnv}
    frozen_assignments = freeze_assignments() # set of object (delivery task) ids
    active_nodes = Set{Int}() #  set of project schedule node ids
    frozen_routes = default_solution(length(get_paths(solution)))

    schedule = env.schedule
    cache = env.cache

    for v in topological_sort_by_dfs(get_graph(schedule))
        node = get_node_from_id(schedule, get_vtx_id(schedule, v))
        if cache.t0[v] <= Ta
            if typeof(node) <: COLLECT
                push!(assignments, get_id(get_object_id(node)))
            end
            if Ta <= cache.tF[v]
                push!(active_nodes, v)
            end
        end
    end

    for v in active_nodes
        node = get_node_from_id(schedule, get_vtx_id(schedule, v))
        if typeof(node) <: AbstractRobotAction
            agent_id = get_path_spec(schedule, v).agent_id
            path = get_paths(solution)[v]
        end
    end
end

end # PathPlanning module
