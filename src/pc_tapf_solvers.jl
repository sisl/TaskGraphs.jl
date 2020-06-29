module Solvers

using Parameters
using MathOptInterface, JuMP
using GraphUtils
using DataStructures

using CRCBS
using ..TaskGraphs

export
    SolverException

"""
    SolverException

Custom exception type for tracking solver timeouts, etc.
"""
struct SolverException <: Exception
    msg::String
end

"""
    AbstractPCTAPFSolver

Abstract type of which all PC-TAPF solvers must be concrete subtypes. All
concrete solvers must implement the following interface for solving PC-TAPF
problems:
- `solution, cost = solve!(solver,problem_def)`
- `check_runtime(solver)` should trigger an interrupt + early return if the
    allowable runtime has been exceeded

Also, we need a good Logger type for keeping track of thing like runtime,
iterations, optimality gap (including upper and lower bound), etc.
"""
abstract type AbstractPCTAPFSolver end

export
    SolverLogger

"""
    SolverLogger

A logger type for keeping track of thing like runtime, iterations, optimality
gap (including upper and lower bound), etc.
"""
@with_kw mutable struct SolverLogger{C}
    iterations      ::Int           = 0
    iteration_limit ::Int           = 100
    max_iterations  ::Int           = 0
    start_time      ::Float64       = time()
    runtime_limit   ::Float64       = 100.0
    deadline        ::Float64       = Inf
    lower_bound     ::C             = typemin(C)
    best_cost       ::C             = typemax(C)
    verbosity       ::Int           = 0
    DEBUG           ::Bool          = false
end

get_logger(solver) = solver.logger
get_logger(logger::SolverLogger) = logger

export
    iterations,
    iteration_limit,
    max_iterations,
    start_time,
    runtime_limit,
    deadline,
    best_cost,
    verbosity

iterations(logger)        = get_logger(logger).iterations
iteration_limit(logger)   = get_logger(logger).iteration_limit
max_iterations(logger)    = get_logger(logger).max_iterations
start_time(logger)        = get_logger(logger).start_time
runtime_limit(logger)     = get_logger(logger).runtime_limit
deadline(logger)          = get_logger(logger).deadline
JuMP.lower_bound(logger)  = get_logger(logger).lower_bound
best_cost(logger)         = get_logger(logger).best_cost
verbosity(logger)         = get_logger(logger).verbosity
debug(logger)             = get_logger(logger).DEBUG

abstract type SearchTrait end
struct Prioritized <: SearchTrait end
struct NonPrioritized <: SearchTrait end
function search_trait end

get_primary_cost(::NonPrioritized,model,cost) = cost[1]
get_primary_cost(::Prioritized,model,cost) = cost[2]
get_primary_cost(solver,cost) = get_primary_cost(search_trait(solver),solver,cost)

export
    optimality_gap,
    set_lower_bound!,
    set_iteration_limit!,
    set_runtime_limit!,
    increment_iteration_count!,
    set_best_cost!,
    reset_solver!,
    hard_reset_solver!

optimality_gap(logger) = best_cost(logger) .- lower_bound(logger)
function check_time(logger)
    t = time()
    if t >= deadline(logger) || t - start_time(logger) >= runtime_limit(logger)
        return true
    end
    return false
end
function enforce_time_limit(logger)
    if check_time(logger)
        throw(SolverException("Solver time limit exceeded!"))
    end
end
function check_iterations(logger)
    iterations(logger) > iteration_limit(logger)
end
function enforce_iteration_limit(logger)
    if check_iterations(logger)
        throw(SolverException("Solver iterations exceeded!"))
    end
end

function set_max_iterations!(logger::SolverLogger,val::Int)
    logger.max_iterations = val
end
function increment_iteration_count!(logger::SolverLogger)
    logger.iterations += 1
    set_max_iterations!(logger,max(iterations(logger),max_iterations(logger)))
end
function set_lower_bound!(logger::SolverLogger,val)
    logger.lower_bound = val
end
# function set_runtime_time_limit!(logger::SolverLogger,val)
#     logger.runtime_limit = val
# end
function CRCBS.set_deadline!(solver,val)
    get_logger(solver).deadline = val
end
function set_runtime_limit!(solver,val)
    get_logger(solver).runtime_limit = val
end
function set_lower_bound!(logger::SolverLogger{NTuple{N,T}},val::R) where {N,T<:Real,R<:Real}
    logger.lower_bound = NTuple{N,R}((R(val),zeros(R,N-1)...))
end
function set_best_cost!(logger::SolverLogger{C},val::C) where {C}
    logger.best_cost = val
end
function set_best_cost!(logger::SolverLogger{NTuple{N,T}},val::R) where {N,T<:Real,R<:Real}
    logger.best_cost = NTuple{N,R}((R(val),zeros(R,N-1)...))
end
function set_iteration_limit!(logger,val)
    get_logger(logger).iteration_limit = val
end

"""
    reset_solver!(solver)

Resets iteration counts and start times.
"""

function reset_solver!(logger::SolverLogger)
    logger.iterations = 0
    logger.best_cost = typemax(logger.best_cost)
    logger.lower_bound = typemin(logger.lower_bound)
    logger.start_time = time()
    logger
end
"""
    hard_reset_solver!(solver)

To be called when no information (other than iteration and time limits) needs to
be stored.
"""
function hard_reset_solver!(logger::SolverLogger)
    reset_solver!(logger)
    set_max_iterations!(logger,0)
end
CRCBS.get_infeasible_cost(logger::SolverLogger{C}) where {C} = typemax(C)
CRCBS.get_infeasible_cost(solver) = get_infeasible_cost(get_logger(solver))
CRCBS.get_cost_type(logger::SolverLogger{C}) where {C} = C
CRCBS.get_cost_type(solver) = get_cost_type(get_logger(solver))
Base.typemin(c::Type{NTuple{N,R}}) where {N,R} = NTuple{N,R}(map(i->typemin(R),1:N))
Base.typemin(c::NTuple{N,R}) where {N,R} = typemin(typeof(c))
Base.typemax(c::Type{NTuple{N,R}}) where {N,R} = NTuple{N,R}(map(i->typemax(R),1:N))
Base.typemax(c::NTuple{N,R}) where {N,R} = typemax(typeof(c))
Base.NTuple{N,R}() where {N,R} = NTuple{N,R}(map(i->R(0), 1:N))

for op = (:(>), :(<), :(>=), :(<=))
    eval(quote
        Base.$op(a::NTuple{N,T},b::R) where {N,T<:Real,R<:Real} = $op(a[1],b)
        Base.$op(b::R,a::NTuple{N,T}) where {N,T<:Real,R<:Real} = $op(b,a[1])
    end)
end

increment_iteration_count!(solver)  = increment_iteration_count!(solver.logger)
set_lower_bound!(solver,val)        = set_lower_bound!(solver.logger,val)
set_best_cost!(solver,val)          = set_best_cost!(solver.logger,val)
reset_solver!(solver)               = reset_solver!(solver.logger)
hard_reset_solver!(solver)          = hard_reset_solver!(solver.logger)

TaskGraphs.log_info(limit::Int,solver,msg...) = log_info(limit,verbosity(solver),msg...)

# """
#     BiLevelPlanner
#
# Generic type of bi-level planner.
# """
# struct BiLevelPlanner{A,B,C}
#     high_level::A
#     low_level::B
#     logger::SolverLogger{C}
# end

low_level(solver) = solver.low_level_planner
primary_cost(solver,cost) = cost[1]
primary_cost_type(solver) = Float64

export
    AStarSC

"""
    AStarSC

Low-level path planner that employs Slack-and-Collision-Aware A*.
Fields:
- logger
- replan : if true, planner will replan with an empty conflict table following
    timeout.
"""
@with_kw struct AStarSC{C}
    logger::SolverLogger{C} = SolverLogger{C}()
    replan::Bool            = false
end
AStarSC() = AStarSC{NTuple{5,Float64}}()
search_trait(solver::AStarSC) = NonPrioritized()
function CRCBS.check_termination_criteria(solver,env::E,cost_so_far,s) where {E<:AbstractLowLevelEnv}
    if iterations(solver) > iteration_limit(solver)
        # throw(SolverAstarMaxOutException(string("# MAX OUT: A* limit of ",solver.LIMIT_A_star_iterations," exceeded.")))
        return true
    end
    return false
end
function CRCBS.logger_step_a_star!(solver::AStarSC, env::MetaAgentCBS.LowLevelEnv, base_path, s, q_cost)
    increment_iteration_count!(solver)
    check_iterations(solver)
    log_info(2,solver,"A* iter $(iterations(solver)): s = $(string(s)), q_cost = $q_cost")
end
function CRCBS.logger_step_a_star!(solver::AStarSC, env, base_path, s, q_cost)
    increment_iteration_count!(solver)
    # if solver.DEBUG
    #     push!(solver.search_history, s)
    # end
    if check_iterations(solver)
        # if solver.DEBUG
        #     # Dump env to JLD2 environment
        #     filename = joinpath(DEBUG_PATH,string("A_star_dump_",get_debug_file_id(),".jld2"))
        #     mkpath(DEBUG_PATH)
        #     log_info(-1,solver,"Dumping A* env to $filename")
        #     agent_id = env.agent_idx
        #     history = map(s->(s.vtx,s.t),solver.search_history)
        #     start   = (get_final_state(base_path).vtx,get_final_state(base_path).t)
        #     goal    = (env.goal.vtx,env.goal.t)
        #     idx = isa(solver.AStarSC) ? 2 : 1
        #     paths   = env.cost_model.cost_models[idx].model.table.paths
        #     state_constraints = map(c->(c.a,(c.v.s.vtx,c.v.sp.vtx),c.t),collect(env.constraints.state_constraints))
        #     action_constraints = map(c->(c.a,(c.v.s.vtx,c.v.sp.vtx),c.t),collect(env.constraints.action_constraints))
        #     @save filename agent_id history start goal paths state_constraints action_constraints
        #     # @assert false "making a bogus assertion to hack my way out of this block"
        # end
        # throw(SolverAstarMaxOutException(string("# MAX OUT: A* limit of ",solver.LIMIT_A_star_iterations," exceeded.")))
    end
    log_info(2,solver,"A* iter $(iterations(solver)): s = $(string(s)), q_cost = $q_cost")
end
function CRCBS.logger_enter_a_star!(solver::AStarSC)
    log_info(1,solver,"A*: entering...")
    @assert(iterations(solver) == 0, "A*: ERROR: iterations = $(iterations(solver)) at entry")
end
function CRCBS.logger_enqueue_a_star!(solver::AStarSC,env,s,a,sp,h_cost)
    log_info(2,solver,"A* exploring $(string(s)) -- $(string(sp)), h_cost = $h_cost")
end
function CRCBS.logger_exit_a_star!(solver::AStarSC, path, cost, status)
    # empty!(solver.search_history)
    if status == false
        log_info(-1,solver,"A*: failed to find feasible path. Returning path of cost $cost")
    else
        log_info(0,solver,"A*: returning optimal path with cost $cost")
    end
end

export
    PrioritizedAStarSC

"""
    PrioritizedAStarSC

Low-level proritized path planner that employs Slack-and-Collision-Aware A*.
"""
@with_kw struct PrioritizedAStarSC{C}
    logger::SolverLogger{C} = SolverLogger{C}()
    replan::Bool            = false
end
search_trait(solver::PrioritizedAStarSC) = Prioritized()
primary_cost(::PrioritizedAStarSC,cost::NTuple{5,Float64}) = cost[2]

# export
#     construct_heuristic_model

"""
    construct_heuristic_model(solver,env_graph;kwargs...)

Construct the heuristic model to be used by solver.
"""
function TaskGraphs.construct_heuristic_model(trait::NonPrioritized,solver,env_graph;
        ph = PerfectHeuristic(get_dist_matrix(env_graph)),
        kwargs...)
    construct_composite_heuristic(ph,NullHeuristic(),ph,ph,NullHeuristic())
end
function TaskGraphs.construct_heuristic_model(trait::Prioritized,args...;kwargs...)
    h = construct_heuristic_model(NonPrioritized(),args...;kwargs...)
    construct_composite_heuristic(
        h.cost_models[2],
        h.cost_models[1],
        h.cost_models[3:end]...
    )
end
function TaskGraphs.construct_heuristic_model(solver, args...;kwargs...)
    construct_heuristic_model(search_trait(solver),solver,args...;kwargs...)
end

# export
#     construct_cost_model

"""
    construct_cost_model(solver::AStarSC, args...;kwargs...)

Defines the cost model used by Slack- and Collision-aware A*.
This particular setting of cost model is crucial for good performance of A_star,
because it encourages depth first search. If we were to replace terms 3-5 with
SumOfTravelTime(), we would get worst-case exponentially slow breadth-first
search!
"""
function TaskGraphs.construct_cost_model(trait::NonPrioritized,
        solver, schedule, cache, problem_spec, env_graph;
        extra_T=400, primary_objective=SumOfMakeSpans())
    N = problem_spec.N
    cost_model = construct_composite_cost_model(
        typeof(primary_objective)(schedule,cache),
        HardConflictCost(env_graph,maximum(cache.tF)+extra_T, N),
        SumOfTravelDistance(),
        FullCostModel(sum,NullCost()), # SumOfTravelTime(),
        FullCostModel(sum,TransformCostModel(c->-1*c,TravelTime()))
    )
    heuristic_model = construct_heuristic_model(trait,solver,env_graph)
    # ph = PerfectHeuristic(get_dist_matrix(env_graph))
    # heuristic_model = construct_composite_heuristic(ph,NullHeuristic(),ph,ph,NullHeuristic())
    cost_model, heuristic_model
end
function TaskGraphs.construct_cost_model(trait::Prioritized,args...;kwargs...)
    c, h = construct_cost_model(NonPrioritized(),args...;kwargs...)
    # switch the first two elements of the cost and heuristic models
    cost_model = construct_composite_cost_model(
        c.cost_models[2],
        c.cost_models[1],
        c.cost_models[3:end]...
    )
    heuristic = construct_composite_heuristic(
        h.cost_models[2],
        h.cost_models[1],
        h.cost_models[3:end]...
    )
    cost_model, heuristic
end
function TaskGraphs.construct_cost_model(solver, args...;kwargs...)
    construct_cost_model(search_trait(solver),solver,args...;kwargs...)
end

export
    update_route_plan!

"""
    update_route_plan!()
"""
function update_route_plan!(solver,env,route_plan,schedule,v,path,cost,schedule_node,agent_id = get_path_spec(schedule, v).agent_id)
    # add to solution
    # log_info(3,solver,string("agent_path = ", convert_to_vertex_lists(path)))
    # log_info(3,solver,string("cost = ", get_cost(path)))
    set_solution_path!(route_plan, path, agent_id)
    set_path_cost!(route_plan, cost, agent_id)
    # update
    update_env!(solver,env,route_plan,v,path)
    route_plan.cost = aggregate_costs(get_cost_model(env.env),get_path_costs(route_plan))
    return route_plan
end
function update_route_plan!(solver,env,route_plan,schedule,v,meta_path,meta_cost,schedule_node::TEAM_ACTION)
    # add to solution
    paths = MetaAgentCBS.split_path(meta_path)
    # @show length(paths)
    # @show length(meta_env.envs)
    # @show length(meta_cost.independent_costs)
    # @show length(schedule_node.instructions)
    for (new_path, sub_node) in zip(paths, schedule_node.instructions)
        agent_id = get_id(get_robot_id(sub_node))
        path = get_paths(route_plan)[agent_id]
        for p in new_path.path_nodes
            push!(path, p)
            # path.cost = accumulate_cost(cbs_env, get_cost(path), get_transition_cost(cbs_env, p.s, p.a, p.sp))
        end
        path.cost = get_cost(new_path)
        update_route_plan!(solver,env,route_plan,schedule,v,path,path.cost,sub_node,agent_id)
        # update_env!(solver,env,route_plan,v,path,agent_id)
        # Print for debugging
        # @show agent_id
        log_info(3,solver,"agent_id = ", agent_id)
        log_info(3,solver,string("agent_path = ", convert_to_vertex_lists(path)))
        log_info(3,solver,string("cost = ", get_cost(new_path)))
    end
    return route_plan
end

export
    plan_path!

"""
    plan_path!

    Computes nxxt path specified by the project schedule and updates the
    solution in the ConstraintTreeNode::node accordingly.
"""
function plan_path!(solver::AStarSC, env::SearchEnv, node::N,
        schedule_node::T, v::Int;
        path_finder=CRCBS.A_star,
        kwargs...) where {N<:ConstraintTreeNode,T}

    cache = env.cache
    schedule = env.schedule
    node_id = get_vtx_id(schedule,v)

    reset_solver!(solver)
    cbs_env, base_path = build_env(solver, env, node, schedule_node, v)
    ### PATH PLANNING ###
    # solver.DEBUG ? validate(base_path,v) : nothing
    path, cost = path_finder(solver, cbs_env, base_path, get_heuristic_cost)
    @assert get_cost(path) == cost
    if cost == get_infeasible_cost(cbs_env)
        if solver.replan == true
            log_info(-1,solver,"A*: replanning without conflict cost", string(schedule_node))
            reset_solver!(solver)
            cost_model, _ = construct_cost_model(
                solver, schedule, cache, env.problem_spec, env.env.graph;
                primary_objective=env.problem_spec.cost_function)
            cbs_env, base_path = build_env(solver, env, node, schedule_node, v;cost_model=cost_model)
            path, cost = path_finder(solver, cbs_env, base_path, heuristic)
        end
        if cost == get_infeasible_cost(cbs_env)
            log_info(-1,solver,"A*: returned infeasible path for node ", string(schedule_node))
            return false
        end
    end
    # solver.DEBUG ? validate(path,v,cbs_env) : nothing
    #####################
    log_info(2,solver,string("A* iterations = ",iterations(solver)))
    # Make sure every robot sticks around for the entire time horizon
    if is_terminal_node(get_graph(schedule),v)
        log_info(2,solver,"ISPS: Extending terminal node")
        extend_path!(cbs_env,path,maximum(cache.tF))
        # solver.DEBUG ? validate(path,v) : nothing
    end
    # add to solution
    update_route_plan!(solver,env,node.solution,schedule,v,path,cost,schedule_node)
    # node.solution.cost = aggregate_costs(get_cost_model(env.env),get_path_costs(node.solution))
    node.cost = get_cost(node.solution)
    if node.cost >= best_cost(solver)
        log_info(0,solver,"ISPS: node.cost >= best_cost(solver) ... Exiting early")
        return false
    end

    return true
end

export
    ISPS

"""
    ISPS

Path planner that employs Incremental Slack-Prioritized Search.
"""
@with_kw struct ISPS{L,C}
    low_level_planner::L = AStarSC()
    logger::SolverLogger{C} = SolverLogger{get_cost_type(low_level_planner)}(
        iteration_limit = 2
    )
end
ISPS(planner) = ISPS(low_level_planner=planner)
TaskGraphs.construct_cost_model(solver::ISPS,args...;kwargs...) = construct_cost_model(low_level(solver),args...;kwargs...)
search_trait(solver::ISPS) = search_trait(low_level(solver))
primary_cost(solver::ISPS,cost) = primary_cost(low_level(solver),cost)
primary_cost_type(solver::ISPS) = primary_cost_type(low_level(solver))
function set_best_cost!(solver::ISPS,cost)
    set_best_cost!(get_logger(solver),cost)
    set_best_cost!(low_level(solver),cost)
end
function hard_reset_solver!(solver::ISPS)
    hard_reset_solver!(get_logger(solver))
    hard_reset_solver!(low_level(solver))
end

export
    plan_next_path!

"""
    `plan_next_path`
"""
function plan_next_path!(solver::ISPS, env::SearchEnv, node::N;
        heuristic=get_heuristic_cost,
        kwargs...
        ) where {N<:ConstraintTreeNode}

    valid_flag = true
    if length(env.cache.node_queue) > 0
        v,priority = dequeue_pair!(env.cache.node_queue)
        node_id = get_vtx_id(env.schedule,v)
        schedule_node = get_node_from_id(env.schedule,node_id)
        # log_info(1,solver,"ISPS: dequeuing v = ",v," - ",string(schedule_node)," with priority ",priority)
        # step_low_level!(solver,schedule_node,env.cache.t0[v],env.cache.tF[v],get_path_spec(env.schedule, v).plan_path)
        if get_path_spec(env.schedule, v).plan_path == true
            # enter_a_star!(solver)
            try
                valid_flag = plan_path!(low_level(solver),env,node,schedule_node,v;
                    heuristic=heuristic)
            catch e
                if isa(e, SolverException)
                    # log_info(-1, solver, e.msg)
                    showerror(stdout, e, catch_backtrace())
                    valid_flag = false
                    return valid_flag
                else
                    rethrow(e)
                end
            end
        else
            # enter_a_star!(solver)
            # dummy path
            path = Path{PCCBS.State,PCCBS.Action,get_cost_type(env.env)}(
                s0=PCCBS.State(-1, -1),
                cost=get_initial_cost(env.env)
                )
            # update planning cache only
            update_planning_cache!(solver,env,v,path) # NOTE I think this is all we need, since there is no actual path to update
        end
        # exit_a_star!(solver)
    end
    return valid_flag
end

"""
    Computes all paths specified by the project schedule and updates the
    solution in the ConstraintTreeNode::node accordingly.
"""
function CRCBS.low_level_search!(solver::ISPS, env::E, node::N;
        heuristic=get_heuristic_cost,
        kwargs...
        ) where {E<:SearchEnv,N<:ConstraintTreeNode}

    reset_solution!(node,env.route_plan)
    while length(env.cache.node_queue) > 0
        if !(plan_next_path!(solver,env,node;heuristic=heuristic))
            return false
        end
        enforce_time_limit(solver)
    end
    # log_info(0,solver,"LOW_LEVEL_SEARCH: Returning consistent route plan with cost ", get_cost(node.solution))
    # log_info(1,solver,"LOW_LEVEL_SEARCH: max path length = ", maximum(map(p->length(p), convert_to_vertex_lists(node.solution))))
    return true
end

"""
    low_level_search!
"""
function CRCBS.low_level_search!(solver::ISPS, pc_mapf::M, node::N,
        idxs::Vector{Int}=Vector{Int}();
        heuristic=get_heuristic_cost,
        kwargs...
        ) where {M<:PC_MAPF,N<:ConstraintTreeNode}

    # enter_low_level!(solver)
    reset_solver!(solver)
    valid_flag = true
    cache = initialize_planning_cache(pc_mapf.env.schedule;
        t0=deepcopy(pc_mapf.env.cache.t0),
        tF=deepcopy(pc_mapf.env.cache.tF)
    )
    search_env = SearchEnv(pc_mapf.env,cache=cache)
    for i in 1:iteration_limit(solver)
        increment_iteration_count!(solver)
        cache = initialize_planning_cache(pc_mapf.env.schedule;
            t0=deepcopy(pc_mapf.env.cache.t0),
            tF=deepcopy(pc_mapf.env.cache.tF)
        )
        search_env = SearchEnv(pc_mapf.env,cache=cache)
        valid_flag = low_level_search!(solver, search_env, node)
        if valid_flag == false
            log_info(0,solver,"ISPS: failed on ",i,"th repair iteration.")
            break
        end
    end
    # exit_low_level!(solver)
    return search_env, valid_flag
end

export
    CBSRoutePlanner

"""
    CBSRoutePlanner

Path planner that employs Conflict-Based Search
"""
@with_kw struct CBSRoutePlanner{L,C}
    low_level_planner::L    = ISPS()
    logger::SolverLogger{C} = SolverLogger{get_cost_type(low_level_planner)}()
end
CBSRoutePlanner(planner) = CBSRoutePlanner(low_level_planner=planner)
TaskGraphs.construct_cost_model(solver::CBSRoutePlanner,args...;kwargs...) = construct_cost_model(low_level(solver),args...;kwargs...)
search_trait(solver::CBSRoutePlanner) = search_trait(low_level(solver))
primary_cost(solver::CBSRoutePlanner,cost) = primary_cost(low_level(solver),cost)
primary_cost_type(solver::CBSRoutePlanner) = primary_cost_type(low_level(solver))
function set_best_cost!(solver::CBSRoutePlanner,cost)
    set_best_cost!(get_logger(solver),cost)
    set_best_cost!(low_level(solver),cost)
end
function hard_reset_solver!(solver::CBSRoutePlanner)
    hard_reset_solver!(get_logger(solver))
    hard_reset_solver!(low_level(solver))
end

function CRCBS.solve!(
        solver::CBSRoutePlanner,
        mapf::M where {M<:PC_MAPF},
        )

    # enter_cbs!(solver)
    reset_solver!(solver)
    priority_queue = PriorityQueue{Tuple{ConstraintTreeNode,PlanningCache},get_cost_type(mapf.env)}()
    root_node = initialize_root_node(mapf) #TODO initialize with a partial solution when replanning
    search_env, valid_flag = low_level_search!(
        low_level(solver),mapf,root_node)
    detect_conflicts!(root_node.conflict_table,root_node.solution;t0=max(minimum(mapf.env.cache.t0), 1))
    if valid_flag
        enqueue!(priority_queue, (root_node,search_env.cache) => root_node.cost)
    else
        # log_info(-1,solver,"CBS: first call to low_level_search returned infeasible.")
    end

    while length(priority_queue) > 0
        try
            node,cache = dequeue!(priority_queue)
            # log_info(1,solver,string("CBS: node.cost = ",get_cost(node.solution)))
            # check for conflicts
            conflict = get_next_conflict(node.conflict_table)
            if !CRCBS.is_valid(conflict)
                # log_info(-1,solver,string("CBS: valid route plan found after ",solver.num_CBS_iterations," CBS iterations! Cost = ",node.cost))
                return SearchEnv(search_env,cache=cache,route_plan=node.solution)
                # return node.solution, cache, node.cost
            end
            # log_info(1,solver,string("CBS: ", string(conflict)))
            # otherwise, create constraints and branch
            constraints = generate_constraints_from_conflict(conflict)
            for constraint in constraints
                new_node = initialize_child_search_node(node,mapf)
                if CRCBS.add_constraint!(new_node,constraint)
                    increment_iteration_count!(solver)
                    # log_info(1,solver,string("CBS: adding constraint", string(constraint)))
                    # step_cbs!(solver,constraint)
                    search_env, valid_flag = low_level_search!(
                        low_level(solver),
                        mapf, new_node,
                        [get_agent_id(constraint)])
                    if valid_flag
                        detect_conflicts!(new_node.conflict_table,new_node.solution,[get_agent_id(constraint)];t0=max(minimum(search_env.cache.t0), 1)) # update conflicts related to this agent
                        enqueue!(priority_queue, (new_node,search_env.cache) => new_node.cost)
                    end
                end
            end
            enforce_time_limit(solver)
            enforce_iteration_limit(solver)
        catch e
            if isa(e, SolverException)
                showerror(stdout, e, catch_backtrace())
                break
            else
                rethrow(e)
            end
        end
    end
    # log_info(-1,solver,"CBS: no solution found. Returning default solution")
    # exit_cbs!(solver)
    solution, cost = default_solution(mapf)
    return SearchEnv(search_env,cache=mapf.env.cache,route_plan=solution)
    # return solution, mapf.env.cache, cost
end

export
    TaskGraphsMILPSolver

"""
    TaskGraphsMILPSolver

Wrapper for MILP solver for assignment problem.
"""
@with_kw struct TaskGraphsMILPSolver{M,C}
    milp::M = SparseAdjacencyMILP()
    logger::SolverLogger{C} = SolverLogger{Int}()
end
TaskGraphsMILPSolver(milp) = TaskGraphsMILPSolver(milp,SolverLogger{Int}())
function TaskGraphs.formulate_milp(solver::TaskGraphsMILPSolver,args...;kwargs...)
    formulate_milp(solver.milp,args...;kwargs...)
end

export
    NBSSolver,
    assignment_solver,
    route_planner

"""
    NBSSolver{A,P}

A hierarchical PC-TAPF solver with an assignment level and a path-planning
level.
The solver works by alternating between assignment and path-planning until the
optimality gap between the lower bound (from task assignment) and the lower
bound (from path planning) disappears.
The input to the assignment problem is the full PC-TAPF problem specification.
The output of the assignment problem is a valid `OperatingSchedule`--that is,
an operating schedule wherein all assignments have been made in a legal way.
The input to the route planner is the PC-TAPF problem spec along with the
`OperatingSchedule` that comes from the assignment solution.
"""
@with_kw struct NBSSolver{A,P,C} <: AbstractPCTAPFSolver
    assignment_model::A     = TaskGraphsMILPSolver()
    path_planner    ::P     = CBSRoutePlanner()
    logger          ::SolverLogger{C} = SolverLogger{primary_cost_type(path_planner)}()
end
NBSSolver(a,b) = NBSSolver(assignment_model=a,path_planner=b)
assignment_solver(solver::NBSSolver) = solver.assignment_model
route_planner(solver::NBSSolver) = solver.path_planner
TaskGraphs.construct_cost_model(solver::NBSSolver,args...;kwargs...) = construct_cost_model(route_planner(solver),args...;kwargs...)
search_trait(solver::NBSSolver) = search_trait(route_planner(solver))
primary_cost(solver::NBSSolver,cost) = primary_cost(route_planner(solver),cost)
primary_cost_type(solver::NBSSolver) = primary_cost_type(route_planner(solver))
function set_best_cost!(solver::NBSSolver,cost)
    set_best_cost!(get_logger(solver),cost)
    set_best_cost!(route_planner(solver),cost)
end
function hard_reset_solver!(solver::NBSSolver)
    hard_reset_solver!(get_logger(solver.logger))
    hard_reset_solver!(route_planner(solver))
end

"""
    solve!(solver, base_search_env::SearchEnv;kwargs...) where {A,P}

Use the planner defined by `solver` to solve the PC-TAPF problem encoded by
`base_search_env`. For solvers of type `NBSSolver`, the algorithm involves
repeatedly solving an assignment problem followed by a route-planning problem.
Within the generic `solve!` method it is possible to initialize an assignment
problem (the type is not constrained) and then modify it via
`update_assignment_problem!` prior to each new call to
`solve_assignment_problem!`. This is the approach taken for various MILP-based
assignment solvers. It is also possible to reconstruct the assignment problem
from scratch within each call to `solve_assignment_problem!`.

Arguments:
- solver <: AbstractPCTAPFSolver
- base_search_env::SearchEnv : a PC-TAPF problem

Outputs:
- best_env : a `SearchEnv` data structure that encodes a solution to the problem
- cost : the cost of the solution encoded by `best_env`
"""
function CRCBS.solve!(solver::NBSSolver{A,P,C}, base_search_env::SearchEnv;kwargs...) where {A,P,C}
    best_env = SearchEnv() # TODO match type of base_search_env for type stability
    set_best_cost!(solver, primary_cost(solver, get_cost(best_env)))
    assignment_problem = formulate_assignment_problem(assignment_solver(solver), base_search_env;
        kwargs...)
    while optimality_gap(solver) > 0
        try
            update_assignment_problem!(assignment_solver(solver), assignment_problem, base_search_env)
            schedule, l_bound = solve_assignment_problem!(
                assignment_solver(solver),
                assignment_problem,
                base_search_env;kwargs...)
            set_lower_bound!(solver,l_bound)
            if optimality_gap(solver) > 0
                env, cost = plan_route!(
                    route_planner(solver),
                    schedule,
                    base_search_env;kwargs...)
                if cost < best_cost(solver)
                    best_env = env
                    set_best_cost!(solver,cost)
                end
            end
            increment_iteration_count!(solver)
            enforce_time_limit(solver)
            if check_iterations(solver)
                log_info(1,solver,
                    "NBS: Reached $(iteration_limit(solver))-iteration limit.")
                break
            end
        catch e
            if isa(e, SolverException)
                bt = catch_backtrace()
                showerror(stdout, e, bt)
                # log_info(-1, solver, sprint(showerror, e, bt))
                break
            else
                rethrow(e)
            end
        end
    end
    return best_env, best_cost(solver)
end
function CRCBS.solve!(solver, env_graph, schedule::OperatingSchedule, problem_spec::ProblemSpec;
    kwargs...)
    base_search_env = construct_search_env(solver,schedule,problem_spec,env_graph;kwargs...)
    # @show base_search_env.cache.t0
    solve!(solver, base_search_env;kwargs...)
end
function CRCBS.solve!(solver, env_graph, project_spec::ProjectSpec, problem_spec::ProblemSpec,
        robot_ICs;kwargs...)
    schedule = construct_partial_project_schedule(
        project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
    solve!(solver, env_graph, schedule, problem_spec;kwargs...)
end

export
    formulate_assignment_problem

"""
    formulate_assignment_problem(solver,base_search_env::SearchEnv;

Returns an assignment problem instance that can be updated (as opposed to being
reconstructed from scratch) on each call to `update_assignment_problem!` prior
to being resolved.
"""
function formulate_assignment_problem(solver,base_search_env::SearchEnv;
        # cost_model=base_search_env.problem_spec.cost_function,
        # optimizer=get_optimizer(solver), #TODO where to pass in the optimizer?
        kwargs...)

    project_schedule    = base_search_env.schedule
    problem_spec        = base_search_env.problem_spec
    formulate_milp(solver,project_schedule,problem_spec;
        cost_model=base_search_env.problem_spec.cost_function,
        # optimizer=optimizer,
        kwargs...)
end

export
    update_assignment_problem!

"""
    update_assignment_problem!(solver, assignment_problem)

A helper method for updating an instance of an assignment problem. In the case
    of MILP-based models, this method simply excludes all previous solutions by
    adding new constraints on the assignment/adjacency matrix.
"""
function update_assignment_problem!(solver, model::T, base_search_env) where {T<:TaskGraphsMILP}
    exclude_solutions!(model) # exclude most recent solution in order to get next best solution
end

export
    solve_assignment_problem!

"""
    solve_assignment_problem!(solver,base_search_env;kwargs...)

Solve the "assignment problem"--i.e., the relaxation of the full PC-TAPF problem
wherein we ignore collisions--using the algorithm encoded by solver.
"""
function solve_assignment_problem!(solver::S, model, base_search_env;
        t0_ = Dict{AbstractID,Int}(get_vtx_id(base_search_env.schedule, v)=>t0 for (v,t0) in enumerate(base_search_env.cache.t0)),
        tF_ = Dict{AbstractID,Int}(get_vtx_id(base_search_env.schedule, v)=>tF for (v,tF) in enumerate(base_search_env.cache.tF)),
        TimeLimit=min(deadline(solver)-time(),runtime_limit(solver)),
        buffer=5.0, # to give some extra time to the path planner if the milp terminates late.
        kwargs...) where {S<:TaskGraphsMILPSolver}

    # enter_assignment!(solver)
    l_bound = lower_bound(solver)
    optimize!(model)
    if primal_status(model) != MOI.FEASIBLE_POINT
        throw(SolverException("Assignment problem is infeasible -- in `solve_assignment_problem!()`"))
    end
    if termination_status(model) == MOI.OPTIMAL
        @assert Int(round(value(objective_function(model)))) == Int(round(value(objective_bound(model))))
    #     l_bound = max(l_bound, Int(round(value(objective_function(model)))) )
    # else
    #     l_bound = max(l_bound, Int(round(value(objective_bound(model)))) )
    end
    set_lower_bound!(solver, Int(round(value(objective_bound(model)))) )
    set_best_cost!(solver, Int(round(value(objective_function(model)))) )
    schedule = deepcopy(base_search_env.schedule)
    update_project_schedule!(model, schedule, base_search_env.problem_spec)
    schedule, lower_bound(solver) 
end


export
    plan_route!

"""
    plan_route!

Compute a route plan that corresponds to the OperatingSchedule.
Arguments:
- solver
- schedule
- search_env

Outputs:
- A `SearchEnv` the contains a valid solution
"""
function plan_route!(
        solver,
        schedule,
        base_search_env;
        kwargs...)

    env = construct_search_env(solver, schedule, base_search_env;kwargs...)
    solution = solve!(solver, PC_MAPF(env))
    solution, primary_cost(solver,get_cost(solution))
end

end
