module PathPlanning

using Parameters
using LightGraphs
using MetaGraphs
using DataStructures
using MathOptInterface, JuMP
using TOML
using JLD2, FileIO

using GraphUtils
using CRCBS

# using ..PCCBS
# using ..TaskGraphsCore
using ..TaskGraphs


export
    PC_TAPF
    # default_pc_tapf_solution

const State = PCCBS.State
const Action = PCCBS.Action
Base.string(s::State) = "($(s.vtx),$(s.t))"
Base.string(a::Action) = "($(a.e.src)->$(a.e.dst))"
Base.string(s::MetaAgentCBS.State) = string("(",prod(map(s->"$(string(s)),",s.states)),")")
Base.string(a::MetaAgentCBS.Action) = string("(",prod(map(a->"$(string(a)),",s.actions)),")")

"""
    `PC_TAPF{L<:LowLevelSolution}`

    The struct defining an instance of a Precedence-Constrained Multi-Agent Task
        Assignment and Path-Finding problem.
"""
struct PC_TAPF{L<:LowLevelSolution}
    env::GridFactoryEnvironment
    schedule::ProjectSchedule       # partial project schedule
    initial_solution::L             # initial condition
end

function CRCBS.get_initial_solution(schedule::ProjectSchedule,env::E) where {E<:PCCBS.LowLevelEnv}
    starts = State[]
    robot_ics = get_robot_ICs(schedule)
    for k in sort(collect(keys(robot_ics)))
        push!(starts, State(vtx = get_id(get_location_id(robot_ics[k])), t = 0))
    end
    cost_model = get_cost_model(env)
    paths = map(s->Path{State,Action,get_cost_type(cost_model)}(s0=s, cost=get_initial_cost(cost_model)), starts)
    costs = map(p->get_cost(p), paths)
    cost = aggregate_costs(cost_model, costs)
    LowLevelSolution(paths=paths, cost_model=cost_model,costs=costs, cost=cost)
end

export
    AbstractCBSModel,
    DefaultCBSModel,
    PrioritizedDFSPlanner,
    AbstractISPSModel,
    DefaultISPSModel,
    PrioritizedPlannerISPSModel,
    AbstractPathFinderModel,
    AStarPathFinderModel,
    PrioritizedAStarModel,
    DFS_PathFinder,
    PC_TAPF_Solver

abstract type AbstractCBSModel end
struct DefaultCBSModel <: AbstractCBSModel end
@with_kw struct PrioritizedDFSPlanner <: AbstractCBSModel
    max_iters::Int = 2000
end
abstract type AbstractISPSModel end
@with_kw struct DefaultISPSModel <: AbstractISPSModel
    n_repair_iters::Int = 2
end
abstract type AbstractPathFinderModel end
@with_kw struct AStarPathFinderModel <: AbstractPathFinderModel
    search_history::Vector{State}   = State[]
    replan::Bool                    = false # Flag for replanning with empty conflict table after timeout
end
@with_kw struct PrioritizedAStarModel <: AbstractPathFinderModel
    search_history::Vector{State}   = State[]
    replan::Bool                    = false # Flag for replanning with empty conflict table after timeout
end
struct DFS_PathFinder <: AbstractPathFinderModel end

@with_kw mutable struct PC_TAPF_Solver{M,C,I,A,T} <: AbstractMAPFSolver
    # TODO parameterize by MILP Solver, CBS solver, ISPS solver, A_star solver
    nbs_model                   ::M = SparseAdjacencyMILP()
    cbs_model                   ::C = DefaultCBSModel()
    isps_model                  ::I = DefaultISPSModel()
    astar_model                 ::A = AStarPathFinderModel()
    LIMIT_assignment_iterations ::Int = 50
    LIMIT_CBS_iterations        ::Int = 100
    LIMIT_A_star_iterations     ::Int = 5000

    num_assignment_iterations   ::Int = 0
    num_CBS_iterations          ::Int = 0
    num_A_star_iterations       ::Int = 0
    best_cost                   ::T   = get_infeasible_cost(astar_model)

    total_assignment_iterations ::Int = 0
    total_CBS_iterations        ::Int = 0
    total_A_star_iterations     ::Int = 0

    max_CBS_iterations          ::Int = 0
    max_A_star_iterations       ::Int = 0

    start_time                  ::Float64 = time()
    time_limit                  ::Float64 = 200.0
    nbs_time_limit              ::Float64 = 190.0

    verbosity                   ::Int = 0
    l1_verbosity                ::Int = 0
    l2_verbosity                ::Int = 0
    l3_verbosity                ::Int = 0
    l4_verbosity                ::Int = 0
    DEBUG                       ::Bool = false
end


export
    SolverException,
    SolverTimeOutException,
    SolverMilpMaxOutException,
    SolverAstarMaxOutException,
    read_solver

abstract type SolverException <: Exception end

@with_kw struct SolverTimeOutException <: SolverException
    msg::String = ""
end
@with_kw struct SolverMilpMaxOutException <: SolverException
    msg::String = ""
end
@with_kw struct SolverCBSMaxOutException <: SolverException
    msg::String = ""
end
@with_kw struct SolverAstarMaxOutException <: SolverException
    msg::String = ""
end
# Helpers for printing
export
    log_info

function log_info(limit::Int,verbosity::Int,msg...)
    if verbosity > limit
        println("[ logger ]: ",msg...)
    end
end
function log_info(limit::Int,solver::S,msg...) where {S<:PC_TAPF_Solver}
    log_info(limit,solver.verbosity,msg...)
end

export
    reset_solver!

function reset_solver!(solver::S) where {S<:PC_TAPF_Solver}
    solver.num_assignment_iterations = 0
    solver.num_CBS_iterations = 0
    solver.num_A_star_iterations = 0
    solver.best_cost = map(c->Inf,solver.best_cost)

    solver.total_assignment_iterations = 0
    solver.total_CBS_iterations = 0
    solver.total_A_star_iterations = 0

    solver.max_CBS_iterations = 0
    solver.max_A_star_iterations = 0

    # solver.start_time = time() # Set solver start time to current time

    solver
end
function check_time(solver::PC_TAPF_Solver)
    if time() - solver.start_time >= solver.time_limit
        throw(SolverTimeOutException(string("# TIME OUT: Overall time limit of ",solver.time_limit," seconds exceeded.")))
    elseif solver.num_assignment_iterations > solver.LIMIT_assignment_iterations
        throw(SolverMilpMaxOutException(string("# MAX OUT: milp solver iteration limit of ",solver.LIMIT_assignment_iterations," exceeded.")))
    elseif solver.num_CBS_iterations > solver.LIMIT_CBS_iterations
        throw(SolverCBSMaxOutException(string("# MAX OUT: cbs iteration limit of ",solver.LIMIT_CBS_iterations," exceeded.")))
    # elseif solver.num_A_star_iterations > solver.LIMIT_A_star_iterations
    #     throw(SolverAstarMaxOutException(string("# MAX OUT: A* iteration limit of ",solver.LIMIT_A_star_iterations," exceeded.")))
    end
end
# update functions
function enter_assignment!(solver::S) where {S<:PC_TAPF_Solver}
    reset_solver!(solver)
end
function exit_assignment!(solver::S) where {S<:PC_TAPF_Solver}
end
function enter_cbs!(solver::S) where {S<:PC_TAPF_Solver}
    # solver.num_assignment_iterations += 1
    log_info(0,solver.l2_verbosity,"CBS: solver.num_CBS_iterations = ",solver.num_CBS_iterations)
    check_time(solver)
end
function step_cbs!(solver::S,constraint) where {S<:PC_TAPF_Solver}
    solver.num_CBS_iterations += 1
    log_info(1,solver.l2_verbosity,"CBS: iteration ", solver.num_CBS_iterations)
    log_info(1,solver.l2_verbosity,string("CBS: constraint on agent id = ",get_agent_id(constraint),", time index = ",get_time_of(constraint)))
    # check_time(solver)
end
function exit_cbs!(solver::S) where {S<:PC_TAPF_Solver}
    solver.max_CBS_iterations = max(solver.max_CBS_iterations,solver.num_CBS_iterations)
    solver.total_CBS_iterations += solver.num_CBS_iterations
    solver.num_CBS_iterations = 0
    check_time(solver)
end
function enter_low_level!(solver::S) where {S<:PC_TAPF_Solver}
    # solver.num_CBS_iterations += 1
    check_time(solver)
end
function step_low_level!(solver::S) where {S<:PC_TAPF_Solver}
    # check_time(solver)
end
function step_low_level!(solver::S,schedule_node::N,t0,tF,plan_path,args...) where {S<:PC_TAPF_Solver,N<:AbstractPlanningPredicate}
    step_low_level!(solver)
    log_info(1,solver.l3_verbosity,"LOW_LEVEL_SEARCH: plan path = ",plan_path," -- node ", string(schedule_node), " t0 = ", t0, ", tF = ",tF)
end
function exit_low_level!(solver::S) where {S<:PC_TAPF_Solver}
    check_time(solver)
end
function enter_a_star!(solver::S,args...) where {S<:PC_TAPF_Solver}
    check_time(solver)
end
function enter_a_star!(solver::S,schedule_node::N,t0,tF,args...) where {S<:PC_TAPF_Solver,N<:AbstractPlanningPredicate}
    enter_a_star!(solver)
    log_info(0,solver.l4_verbosity,"A*: planning for node ",string(schedule_node), " t0 = ", t0, ", tF = ",tF)
end
function exit_a_star!(solver::S,args...) where {S<:PC_TAPF_Solver}
    solver.max_A_star_iterations = max(solver.max_A_star_iterations,solver.num_A_star_iterations)
    solver.total_A_star_iterations += solver.num_A_star_iterations
    solver.num_A_star_iterations = 0
    check_time(solver)
end
function CRCBS.logger_step_a_star!(solver::PC_TAPF_Solver, env::MetaAgentCBS.LowLevelEnv, base_path, s, q_cost)
    solver.num_A_star_iterations += 1
    if solver.num_A_star_iterations > solver.LIMIT_A_star_iterations
        # throw(SolverAstarMaxOutException(string("# MAX OUT: A* limit of ",solver.LIMIT_A_star_iterations," exceeded.")))
    end
    log_info(2,solver.l4_verbosity,"A* iter $(solver.num_A_star_iterations): s = $(string(s)), q_cost = $q_cost")
end
function CRCBS.logger_step_a_star!(solver::PC_TAPF_Solver, env, base_path, s, q_cost)
    solver.num_A_star_iterations += 1
    if solver.DEBUG
        push!(solver.astar_model.search_history, s)
    end
    if solver.num_A_star_iterations > solver.LIMIT_A_star_iterations
        if solver.DEBUG
            # Dump env to JLD2 environment
            filename = joinpath(DEBUG_PATH,string("A_star_dump_",get_debug_file_id(),".jld2"))
            mkpath(DEBUG_PATH)
            log_info(-1,solver.l4_verbosity,"Dumping A* env to $filename")
            agent_id = env.agent_idx
            history = map(s->(s.vtx,s.t),solver.astar_model.search_history)
            start   = (get_final_state(base_path).vtx,get_final_state(base_path).t)
            goal    = (env.goal.vtx,env.goal.t)
            idx = isa(solver.astar_model,AStarPathFinderModel) ? 2 : 1
            paths   = env.cost_model.cost_models[idx].model.table.paths
            state_constraints = map(c->(c.a,(c.v.s.vtx,c.v.sp.vtx),c.t),collect(env.constraints.state_constraints))
            action_constraints = map(c->(c.a,(c.v.s.vtx,c.v.sp.vtx),c.t),collect(env.constraints.action_constraints))
            @save filename agent_id history start goal paths state_constraints action_constraints
            # @assert false "making a bogus assertion to hack my way out of this block"
        end
        # throw(SolverAstarMaxOutException(string("# MAX OUT: A* limit of ",solver.LIMIT_A_star_iterations," exceeded.")))
    end
    log_info(2,solver.l4_verbosity,"A* iter $(solver.num_A_star_iterations): s = $(string(s)), q_cost = $q_cost")
end
function CRCBS.logger_enter_a_star!(solver::PC_TAPF_Solver)
    log_info(1,solver.l4_verbosity,"A*: entering...")
    @assert(solver.num_A_star_iterations == 0, string("A*: ERROR: iterations = ", solver.num_A_star_iterations, " at entry"))
    # if solver.num_A_star_iterations > 0
    #     log_info(-1,solver.l4_verbosity,"A*: ERROR: iterations = ", solver.num_A_star_iterations, " at entry")
    # end
end
function CRCBS.logger_enqueue_a_star!(solver::PC_TAPF_Solver,env,s,a,sp,h_cost)
    log_info(2,solver.l4_verbosity,"A* exploring $(string(s)) -- $(string(sp)), h_cost = $h_cost")
end
function CRCBS.logger_exit_a_star!(solver::PC_TAPF_Solver, path, cost, status)
    empty!(solver.astar_model.search_history)
    if status == false
        log_info(-1,solver.l4_verbosity,"A*: failed to find feasible path. Returning path of cost ",cost)
    else
        log_info(0,solver.l4_verbosity,"A*: returning optimal path with cost ",cost)
    end
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
    toml_dict["time_limit"] = solver.time_limit
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

        time_limit = toml_dict["time_limit"],

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


export
    PlanningCache,
    isps_queue_cost,
    initialize_planning_cache,
    reset_cache!,
    update_planning_cache!,
    repair_solution!,
    plan_path!,
    plan_next_path!

@with_kw struct PlanningCache
    closed_set::Set{Int}                    = Set{Int}()    # nodes that are completed
    active_set::Set{Int}                    = Set{Int}()    # active nodes
    node_queue::PriorityQueue{Int,Tuple{Int,Float64}} = PriorityQueue{Int,Tuple{Int,Float64}}() # active nodes prioritized by slack
    t0::Vector{Int}                         = Vector{Int}()
    tF::Vector{Int}                         = Vector{Int}()
    slack::Vector{Vector{Float64}}          = Vector{Vector{Float64}}()
    local_slack::Vector{Vector{Float64}}    = Vector{Vector{Float64}}()
    max_deadline::Vector{Int}               = Vector{Int}() # Marks the time at whidh this node will begin accumulating a delay cost
end

function isps_queue_cost(schedule::ProjectSchedule,cache::PlanningCache,v::Int)
    path_spec = get_path_spec(schedule,v)
    return (Int(path_spec.plan_path), minimum(cache.slack[v]))
end

function initialize_planning_cache(schedule::ProjectSchedule;kwargs...)
    t0,tF,slack,local_slack = process_schedule(schedule;kwargs...);
    allowable_slack = map(i->minimum(i),slack) # soft deadline (can be tightened as necessary)
    allowable_slack = map(i->i==Inf ? typemax(Int) : Int(i), allowable_slack) # initialize deadline at infinity
    cache = PlanningCache(t0=t0,tF=tF,slack=slack,local_slack=local_slack,max_deadline=allowable_slack)
    for v in vertices(get_graph(schedule))
        if is_root_node(get_graph(schedule),v)
            push!(cache.active_set,v)
            enqueue!(cache.node_queue,v=>isps_queue_cost(schedule,cache,v)) # need to store slack
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
    empty!(cache.closed_set)
    empty!(cache.active_set)
    empty!(cache.node_queue)
    for v in vertices(get_graph(schedule))
        if is_root_node(get_graph(schedule),v)
            push!(cache.active_set,v)
            enqueue!(cache.node_queue,v=>isps_queue_cost(schedule,cache,v)) # need to store slack
        end
    end
    cache
end

export
    SearchEnv,
    construct_search_env,
    update_env!

# @with_kw struct SearchEnv{C,E<:AbstractLowLevelEnv{State,Action,C},S<:LowLevelSolution} <: AbstractLowLevelEnv{State,Action,C}
@with_kw struct SearchEnv{C,E<:AbstractLowLevelEnv{State,Action,C},S} <: AbstractLowLevelEnv{State,Action,C}
    schedule::ProjectSchedule       = ProjectSchedule()
    cache::PlanningCache            = PlanningCache()
    env::E                          = PCCBS.LowLevelEnv()
    problem_spec::ProblemSpec       = ProblemSpec()
    dist_function::DistMatrixMap    = env.graph.dist_function # DistMatrixMap(env.graph.vtx_map, env.graph.vtxs)
    cost_model::C                   = get_cost_model(env)
    num_agents::Int                 = length(get_robot_ICs(schedule))
    base_solution::S                = LowLevelSolution(
        paths = map(i->Path{State,Action,get_cost_type(cost_model)}(), 1:num_agents),
        cost_model = cost_model,
        costs = map(i->get_initial_cost(cost_model), 1:num_agents),
        cost  = aggregate_costs(cost_model, map(i->get_initial_cost(cost_model), 1:num_agents)),
        )
end
function CRCBS.get_start(env::SearchEnv,v::Int)
    start_vtx   = get_path_spec(env.schedule,v).start_vtx
    start_time  = env.cache.t0[v]
    PCCBS.State(start_vtx,start_time)
end

"""
    Trim path for replanning
"""
function trim_path(cbs_env, old_path, T)
    new_path = typeof(old_path)(
        s0=get_initial_state(old_path),
        cost=get_initial_cost(cbs_env)
        )
    for t in 1:max(1, min(T, length(old_path)))
        p = get_path_node(old_path,t)
        push!(new_path, p)
        new_path.cost = accumulate_cost(cbs_env, get_cost(new_path), get_transition_cost(cbs_env, p.s, p.a, p.sp))
        # set_path_cost!(trimmed_solution, new_path.cost, agent_id)
    end
    if T > length(new_path)
        println("Extending path in trim_path.")
        extend_path!(cbs_env,new_path,T)
    end
    new_path
end
function reverse_propagate_delay!(solver,cache,schedule,delay_vec)
    buffer = zeros(nv(schedule))
    for v in reverse(topological_sort_by_dfs(get_graph(schedule)))
        Δt_min = get_path_spec(schedule,v).min_path_duration
        buffer[v] = (cache.tF[v] - (cache.t0[v] + Δt_min))
        for v2 in outneighbors(schedule,v)
            delay_vec[v] = max(delay_vec[v], delay_vec[v2] - buffer[v2])
            delay_vec[v] = max(0, delay_vec[v] - (cache.t0[v2] - cache.tF[v]))
        end
    end
    delay_vec
end
function get_delay_vec(solver,cache,schedule,v0)
    delay = zeros(nv(schedule))
    for v in reverse(topological_sort_by_dfs(get_graph(schedule)))
        Δt_min = get_path_spec(schedule,v).min_path_duration
        delay[v] = (cache.tF[v] - (cache.t0[v] + Δt_min + minimum(cache.local_slack[v])))
    end
    # zero out all vtxs that are not predecessors of v
    delay_vec = zeros(Int,nv(schedule))
    for v in map(e->e.dst,collect(edges(bfs_tree(get_graph(schedule),v0;dir=:in))))
        if cache.max_deadline[v] > 0
            delay_vec[v] = min(delay[v],typemax(Int))
        end
    end
    delay_vec
end
function backtrack_deadlines(solver,cache,schedule,v)
    frontier = Set{Int}([v])
    delay_cut = Set{Int}()
    while length(frontier) > 0
        v = pop!(frontier)
        Δt_min = get_path_spec(schedule,v).min_path_duration
        buffer = (cache.tF[v] - (cache.t0[v] + Δt_min))
        if get_path_spec(schedule,v).fixed
            continue
        elseif cache.max_deadline[v] > 0 # still has room for some delay
            push!(delay_cut,v)
        elseif indegree(schedule,v) == 0
            # if v is a root_node, the deadlines cannot be tightened anymore
            log_info(-1,solver.l3_verbosity,"ISPS: deadlines cannot be tightened any more.")
            return Set{Int}()
        else
            for v2 in inneighbors(schedule,v)
                push!(frontier,v2)
            end
        end
    end
    return delay_cut
end
function tighten_deadline!(solver,env,solution,v,dt=1)
    cache = env.cache
    schedule = env.schedule
    active_set = cache.active_set
    closed_set = cache.closed_set
    node_queue = cache.node_queue
    graph = get_graph(schedule)
    # adjust allowable_slack
    cache.max_deadline[v] = max(0, cache.max_deadline[v]-1)
    # remove from closed set
    push!(active_set, v)
    setdiff!(closed_set,v)
    # Trim schedule at v.t0
    agent_id = get_path_spec(schedule,v).agent_id
    if agent_id != -1
        cbs_env = typeof(env.env)(
            graph = env.env.graph,
            agent_idx = agent_id,
            cost_model = get_cost_model(env.env),
            heuristic = get_heuristic_model(env.env)
        )
        new_path = trim_path(cbs_env,get_paths(solution)[agent_id],cache.t0[v])
        set_solution_path!(solution,new_path,agent_id)
        set_path_cost!(solution,new_path.cost,agent_id)
    end
    # Update node queue
    for v2 in map(e->e.dst,collect(edges(bfs_tree(graph,v;dir=:out))))
        setdiff!(closed_set,v2)
        setdiff!(active_set,v2)
    end
    empty!(node_queue)
    for v2 in active_set
        node_queue[v2] = isps_queue_cost(schedule,cache,v2)
    end
    env,solution
end
function update_planning_cache!(solver::M,env::E,v::Int,path::P) where {M<:PC_TAPF_Solver,E<:SearchEnv,P<:Path}
    cache = env.cache
    schedule = env.schedule
    active_set = cache.active_set
    closed_set = cache.closed_set
    node_queue = cache.node_queue
    graph = get_graph(schedule)

    # update t0, tF, slack, local_slack
    Δt = get_final_state(path).t - cache.tF[v]
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
        cache.tF[v] = get_final_state(path).t
        t0,tF,slack,local_slack = process_schedule(schedule;t0=cache.t0,tF=cache.tF)
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

export
    get_primary_cost

get_primary_cost(model,cost)                        = cost[1]
get_primary_cost(solver::PC_TAPF_Solver,args...)    = get_primary_cost(solver.astar_model,args...)

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
CRCBS.get_infeasible_cost(astar_model,args...) = (Inf,Inf,Inf,Inf,Inf)
function construct_a_star_cost_model(astar_model, schedule, cache, problem_spec, env_graph; extra_T=400, primary_objective=SumOfMakeSpans)
    N = problem_spec.N
    # NOTE: This particular setting of cost model is crucial for good performance of A_star, because it encourages depth first search. If we were to replace them with SumOfTravelTime(), we would get worst-case exponentially slow breadth-first search!
    cost_model = construct_composite_cost_model(
        primary_objective(schedule,cache),
        HardConflictCost(env_graph,maximum(cache.tF)+extra_T, N),
        SumOfTravelDistance(),
        FullCostModel(sum,NullCost()), # SumOfTravelTime(),
        FullCostModel(sum,TransformCostModel(c->-1*c,TravelTime()))
    )
    ph = PerfectHeuristic(get_dist_matrix(env_graph))
    heuristic_model = construct_composite_heuristic(ph,NullHeuristic(),ph,ph,NullHeuristic())
    cost_model, heuristic_model
end
get_primary_cost(model::PrioritizedAStarModel,cost) = cost[2]
function construct_a_star_cost_model(astar_model::PrioritizedAStarModel, schedule, cache, problem_spec, env_graph; extra_T=400, primary_objective=SumOfMakeSpans)
    N = problem_spec.N
    cost_model = construct_composite_cost_model(
        HardConflictCost(env_graph,maximum(cache.tF)+extra_T, N),
        primary_objective(schedule,cache),
        SumOfTravelDistance(),
        FullCostModel(sum,NullCost()), # SumOfTravelTime(),
        FullCostModel(sum,TransformCostModel(c->-1*c,TravelTime())),
    )
    ph = PerfectHeuristic(get_dist_matrix(env_graph))
    heuristic_model = construct_composite_heuristic(
        NullHeuristic(),
        ph,
        ph,
        ph,
        NullHeuristic(),
        )
    cost_model, heuristic_model
end
CRCBS.get_infeasible_cost(astar_model::DFS_PathFinder,args...) = (Inf,Inf,Inf)
function construct_a_star_cost_model(astar_model::DFS_PathFinder, schedule, cache, problem_spec, env_graph; extra_T=400, primary_objective=SumOfMakeSpans)
    cost_model = construct_composite_cost_model(
        primary_objective(schedule,cache),
        FullCostModel(maximum,TravelTime()),
        FullCostModel(maximum,TravelDistance())
        )
    ph = PerfectHeuristic(get_dist_matrix(env_graph))
    heuristic_model = construct_composite_heuristic(ph,ph,NullHeuristic())
    cost_model, heuristic_model
end
construct_a_star_cost_model(solver::PC_TAPF_Solver, args...; kwargs...) = construct_a_star_cost_model(solver.astar_model, args...; kwargs...)

function construct_search_env(solver,schedule, problem_spec, env_graph;
        primary_objective=SumOfMakeSpans,
        extra_T=400,
        kwargs...
    )
    cache = initialize_planning_cache(schedule;kwargs...)
    N = problem_spec.N                                          # number of robots
    cost_model, heuristic_model = construct_a_star_cost_model(solver, schedule, cache, problem_spec, env_graph; primary_objective=primary_objective, extra_T=extra_T)
    low_level_env = PCCBS.LowLevelEnv(
        graph=env_graph, cost_model=cost_model, heuristic=heuristic_model)
    base_solution = get_initial_solution(schedule,low_level_env)

    search_env = SearchEnv(schedule=schedule, cache=cache, env=low_level_env,
        problem_spec=problem_spec, num_agents=N, base_solution=base_solution)
    return search_env
end
function construct_search_env(solver,project_spec, problem_spec, robot_ICs, assignments, env_graph;kwargs...)
    schedule = construct_project_schedule(project_spec, problem_spec, robot_ICs, assignments)
    construct_search_env(solver,schedule, problem_spec, env_graph;kwargs...)
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

"""
    `update_env!`

    `v` is the vertex id
"""
function update_env!(solver::S,env::E,solution::L,v::Int,path::P,agent_id::Int=get_path_spec(env.schedule,v).agent_id) where {S<:PC_TAPF_Solver,E<:SearchEnv,P<:Path,L<:LowLevelSolution}
    cache = env.cache
    schedule = env.schedule

    # UPDATE CACHE
    update_planning_cache!(solver,env,v,path)
    update_cost_model!(env)
    # ADD UPDATED PATH TO HEURISTIC MODELS
    if agent_id != -1
        partially_set_path!(get_heuristic_model(env.env),agent_id,convert_to_vertex_lists(get_paths(solution)[agent_id]))
        partially_set_path!(get_cost_model(env.env),agent_id,convert_to_vertex_lists(get_paths(solution)[agent_id]))
    end

    env
end

function CRCBS.build_env(solver, env::E, node::N, schedule_node::T, v::Int, path_spec=get_path_spec(env.schedule, v);
        heuristic = get_heuristic_model(env.env),
        cost_model = get_cost_model(env.env)
    ) where {E<:SearchEnv,N<:ConstraintTreeNode,T}
    agent_id = path_spec.agent_id
    goal_vtx = path_spec.final_vtx
    goal_time = env.cache.tF[v]                             # time after which goal can be satisfied

    # deadline = env.cache.tF[v] .+ min.(env.cache.max_deadline[v],env.cache.slack[v]) # NOTE iterative deadline tightening was causing problems with slack running out before the goal time, so this line has been replaced by the original
    # deadline = env.cache.tF[v] .+ min.(max.(env.cache.local_slack[v], env.cache.max_deadline[v]),env.cache.slack[v]) # This is a potential fix that would allow iterative tightening to keep working
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
    if (path_spec.free == true) && is_terminal_node(get_graph(env.schedule),v)
        goal_time = maximum(env.cache.tF)
        goal_vtx = -1
        # deadline = Inf # already taken care of, perhaps?
        log_info(3,solver,string("BUILD ENV: setting goal_vtx = ",goal_vtx,", t = maximum(cache.tF) = ",goal_time))
    end
    cbs_env = PCCBS.LowLevelEnv(
        graph       = env.env.graph,
        schedule_node = schedule_node,
        node_id     = get_vtx_id(env.schedule,v),
        constraints = get_constraints(node, agent_id), # agent_id represents the whole path
        goal        = PCCBS.State(goal_vtx,goal_time),
        agent_idx   = agent_id, # this is only used for the HardConflictTable, which can be updated via the combined search node
        heuristic   = heuristic,
        cost_model  = cost_model
        )
    # update deadline in DeadlineCost
    set_deadline!(get_cost_model(cbs_env),deadline)
    # retrieve base_path
    if !is_root_node(get_graph(env.schedule),v) # && agent_id != -1
        # log_info(0,solver,string("retrieving base path for node ",v," of type ",typeof(schedule_node)))
        base_path = get_paths(node.solution)[agent_id]
    else
        base_path = get_paths(node.solution)[agent_id]
        # log_info(0,solver,string("initializing base path for node ",v," of type ",typeof(schedule_node)))
        # start_vtx = path_spec.start_vtx
        # base_path = Path{PCCBS.State,PCCBS.Action,get_cost_type(cbs_env)}(
        #     s0=PCCBS.State(start_vtx, 0), # TODO fix start time
        #     cost=get_initial_cost(cbs_env)
        #     )
        @assert PCCBS.State(path_spec.start_vtx, 0) == get_final_state(base_path)
    end
    # make sure base_path hits t0 constraint
    if get_end_index(base_path) < env.cache.t0[v]
        log_info(1, solver, string("LOW LEVEL SEARCH: in schedule node ",v," -- ",
            string(schedule_node),": cache.t0[v] - get_end_index(base_path) = ",env.cache.t0[v] - get_end_index(base_path),". Extending path to ",env.cache.t0[v]," ..."))
        # base_path = extend_path(cbs_env,base_path,env.cache.t0[v])
        extend_path!(cbs_env,base_path,env.cache.t0[v])
    end
    log_info(2,solver.l3_verbosity,
    "LOW LEVEL SEARCH: v=$(v), node=$(string(schedule_node)), deadline=$(deadline), ",
    "local_slack=$(env.cache.slack[v]), slack=$(env.cache.slack[v]), goal=$(string(cbs_env.goal)),
    ")
    return cbs_env, base_path
end
CRCBS.build_env(solver, env::E, node::N, v::Int) where {E<:SearchEnv,N<:ConstraintTreeNode} = build_env(solver, env, node, get_node_from_vtx(env.schedule,v), v)
function CRCBS.build_env(solver, env::E, node::N, schedule_node::TEAM_ACTION, v::Int) where {E<:SearchEnv,N<:ConstraintTreeNode}
    envs = []
    starts = Vector{PCCBS.State}()
    meta_cost = MetaCost(Vector{get_cost_type(env)}(),get_initial_cost(env.env))
    # path_specs = Vector{PathSpec}()
    for (i, sub_node) in enumerate(schedule_node.instructions)
        # if i == 1 # leader
            ph = PerfectHeuristic(env.dist_function.dist_mtxs[schedule_node.shape][i])
            heuristic = construct_composite_heuristic(ph,NullHeuristic(),ph,ph,NullHeuristic())
        # else
        #     heuristic = get_heuristic_model(env.env)
        # end
        cbs_env, base_path = build_env(solver,env,node,sub_node,v,generate_path_spec(env.schedule,env.problem_spec,sub_node);
            heuristic=heuristic,
        ) # TODO need problem_spec here
        push!(envs, cbs_env)
        push!(starts, get_final_state(base_path))
        push!(meta_cost.independent_costs, get_cost(base_path))
    end
    meta_env = MetaAgentCBS.construct_meta_env([envs...], get_cost_model(env))
    meta_path = Path{MetaAgentCBS.State{PCCBS.State},MetaAgentCBS.Action{PCCBS.Action},MetaCost{get_cost_type(env)}}(
        s0 = MetaAgentCBS.State(starts),
        cost = MetaCost(meta_cost.independent_costs, aggregate_costs(get_cost_model(meta_env), meta_cost.independent_costs))
    )
    return meta_env, meta_path
end

"""
    plan_path!

    Computes nxxt path specified by the project schedule and updates the
    solution in the ConstraintTreeNode::node accordingly.
"""
function plan_path!(solver::PC_TAPF_Solver, env::SearchEnv, node::N, schedule_node::T, v::Int;
        heuristic=get_heuristic_cost,
        path_finder=A_star
        ) where {N<:ConstraintTreeNode,T}

    cache = env.cache
    schedule = env.schedule
    node_id = get_vtx_id(schedule,v)

    # if get_path_spec(schedule, v).plan_path == true
    cbs_env, base_path = build_env(solver, env, node, schedule_node, v)
    ### PATH PLANNING ###
    # if typeof(schedule_node) <: Union{COLLECT,DEPOSIT} # Must sit and wait the whole time
    #     path = base_path
    #     extend_path!(cbs_env,path,cache.tF[v]) # NOTE looks like this might work out of the box for replanning. No need for node surgery
    #     cost = get_cost(path)
    #     CRCBS.logger_exit_a_star!(solver, path, cost, true)
    #     solver.DEBUG ? validate(path,v) : nothing
    # else # if typeof(schedule_node) <: Union{GO,CARRY}
        solver.DEBUG ? validate(base_path,v) : nothing
        path, cost = path_finder(solver, cbs_env, base_path, heuristic)
        if cost == get_infeasible_cost(cbs_env)
            if solver.astar_model.replan == true
                # TODO replan with empty conflict table
                log_info(-1,solver.l4_verbosity,"A*: replanning without conflict cost", string(schedule_node))
                solver.num_A_star_iterations = 0
                cost_model, heuristic_model = construct_a_star_cost_model(solver.astar_model, schedule, cache, env.problem_spec, env.env.graph;
                    primary_objective=env.problem_spec.cost_function)
                cbs_env, base_path = build_env(solver, env, node, schedule_node, v;cost_model=cost_model,heuristic=heuristic_model)
                path, cost = path_finder(solver, cbs_env, base_path, heuristic)
            end
            if cost == get_infeasible_cost(cbs_env)
                log_info(-1,solver.l4_verbosity,"A*: returned infeasible path for node ", string(schedule_node))
                return false
            end
        end
        solver.DEBUG ? validate(path,v,cbs_env) : nothing
    # end
    #####################
    log_info(2,solver.l3_verbosity,string("LOW LEVEL SEARCH: solver.num_A_star_iterations = ",solver.num_A_star_iterations))
    # Make sure every robot sticks around for the entire time horizon
    if is_terminal_node(get_graph(schedule),v)
        log_info(2,solver.l3_verbosity,string("LOW LEVEL SEARCH: Extending terminal node",
        " (SHOULD NEVER HAPPEN IF THINGS ARE WORKING CORRECTLY)"))
        extend_path!(cbs_env,path,maximum(cache.tF))
        solver.DEBUG ? validate(path,v) : nothing
    end
    # add to solution
    agent_id = get_path_spec(schedule, v).agent_id
    set_solution_path!(node.solution, path, agent_id)
    set_path_cost!(node.solution, cost, agent_id)
    # update
    update_env!(solver,env,node.solution,v,path)
    node.solution.cost = aggregate_costs(get_cost_model(env.env),get_path_costs(node.solution))
    node.cost = get_cost(node.solution)
    # Print for debugging
    log_info(3,solver.l3_verbosity,string("agent_path = ", convert_to_vertex_lists(path)))
    log_info(3,solver.l3_verbosity,string("cost = ", get_cost(path)))
    if node.cost >= solver.best_cost
        log_info(0,solver.l3_verbosity,"LOW LEVEL SEARCH: node.cost >= solver.best_cost ... Exiting early")
        return false
    end

    return true
end
function plan_path!(solver::PC_TAPF_Solver, env::SearchEnv, node::N, schedule_node::TEAM_ACTION{A}, v::Int;
        heuristic=get_heuristic_cost,
        path_finder=A_star
        ) where {N<:ConstraintTreeNode,A}

    cache = env.cache
    schedule = env.schedule
    node_id = get_vtx_id(schedule,v)

    meta_env, base_path = build_env(solver, env, node, schedule_node, v)
    ### PATH PLANNING ###
    # if A <: Union{COLLECT,DEPOSIT} # Must sit and wait the whole time
    #     meta_path = base_path
    #     extend_path!(meta_env,meta_path,cache.tF[v]-(cache.t0[v]-length(base_path)))
    #     meta_cost = get_cost(meta_path)
    # else
        meta_path, meta_cost = path_finder(solver, meta_env, base_path, heuristic;verbose=(solver.verbosity > 3))
        if meta_cost == get_infeasible_cost(meta_env)
            log_info(-1,solver.l4_verbosity,"A*: returned infeasible path for node ", string(schedule_node))
            return false
        end
    # end
    paths = MetaAgentCBS.split_path(meta_path)
    # @show length(paths)
    # @show length(meta_env.envs)
    # @show length(meta_cost.independent_costs)
    # @show length(schedule_node.instructions)
    for (cbs_env, new_path, cost, sub_node) in zip(meta_env.envs, paths, meta_cost.independent_costs, schedule_node.instructions)
        agent_id = get_id(get_robot_id(sub_node))
        path = get_paths(node.solution)[agent_id]
        for p in new_path.path_nodes
            push!(path, p)
            path.cost = accumulate_cost(cbs_env, get_cost(path), get_transition_cost(cbs_env, p.s, p.a, p.sp))
        end
        set_solution_path!(node.solution, path, agent_id)
        set_path_cost!(node.solution, get_cost(path), agent_id)
        update_env!(solver,env,node.solution,v,path,agent_id)
        # Print for debugging
        # @show agent_id
        log_info(3,solver.l3_verbosity,"agent_id = ", agent_id)
        log_info(3,solver.l3_verbosity,string("agent_path = ", convert_to_vertex_lists(path)))
        log_info(3,solver.l3_verbosity,string("cost = ", get_cost(path)))
    end
    # solver.DEBUG ? validate(path,v,meta_env) : nothing
    #####################
    log_info(2,solver.l3_verbosity,string("LOW LEVEL SEARCH: solver.num_A_star_iterations = ",solver.num_A_star_iterations))
    # update
    node.solution.cost = aggregate_costs(get_cost_model(env.env),get_path_costs(node.solution))
    node.cost = get_cost(node.solution)
    if node.cost >= solver.best_cost
        log_info(0,solver,"LOW LEVEL SEARCH: node.cost >= solver.best_cost ... Exiting early")
        return false
    end

    return true
end

"""
    `plan_next_path`
"""
function plan_next_path!(solver::S, env::E, node::N;
        heuristic=get_heuristic_cost,
        path_finder=A_star
        ) where {S<:PC_TAPF_Solver,E<:SearchEnv,N<:ConstraintTreeNode}

    valid_flag = true
    if length(env.cache.node_queue) > 0
        v,priority = dequeue_pair!(env.cache.node_queue)
        node_id = get_vtx_id(env.schedule,v)
        schedule_node = get_node_from_id(env.schedule,node_id)
        # log_info(1,solver.l3_verbosity,"ISPS: dequeuing v = ",v," - ",string(schedule_node)," with priority ",priority)
        step_low_level!(solver,schedule_node,env.cache.t0[v],env.cache.tF[v],get_path_spec(env.schedule, v).plan_path)
        if get_path_spec(env.schedule, v).plan_path == true
            enter_a_star!(solver)
            # enter_a_star!(solver,schedule_node,env.cache.t0[v],env.cache.tF[v])
            try
                valid_flag = plan_path!(solver,env,node,schedule_node,v;
                    heuristic=heuristic,path_finder=path_finder)
            catch e
                if isa(e, SolverAstarMaxOutException)
                    log_info(-1,solver.l4_verbosity, e.msg)
                    log_info(-1,solver.l4_verbosity,"A*: planning timed out for node ",string(schedule_node))
                    valid_flag = false
                    exit_a_star!(solver)
                    return valid_flag
                else
                    throw(e)
                end
            end
        else
            enter_a_star!(solver)
            # dummy path
            path = Path{PCCBS.State,PCCBS.Action,get_cost_type(env.env)}(
                s0=PCCBS.State(-1, -1),
                cost=get_initial_cost(env.env)
                )
            # update planning cache only
            update_planning_cache!(solver,env,v,path) # NOTE I think this is all we need, since there is no actual path to update
        end
        exit_a_star!(solver)
    end
    return valid_flag
end

export
    reset_solution!

"""
    Helper to reset the solution in a constraint node between re-runs of ISPS
"""
function reset_solution!(node::N,base_solution) where {N<:ConstraintTreeNode}
    for (agent_id,path) in enumerate(get_paths(base_solution))
        set_solution_path!(node.solution, deepcopy(path), agent_id)
        set_path_cost!(node.solution, get_cost(path), agent_id)
        node.solution.cost = base_solution.cost
        node.cost = get_cost(node.solution)
    end
    node
end

"""
    Computes all paths specified by the project schedule and updates the
    solution in the ConstraintTreeNode::node accordingly.
"""
function CRCBS.low_level_search!(
        solver::S, env::E, node::N;
        heuristic=get_heuristic_cost,
        path_finder=A_star
        ) where {S<:PC_TAPF_Solver,E<:SearchEnv,N<:ConstraintTreeNode}

    reset_solution!(node,env.base_solution)
    while length(env.cache.node_queue) > 0
        if !(plan_next_path!(solver,env,node;heuristic=heuristic,path_finder=path_finder))
            return false
        end
    end
    log_info(0,solver.l3_verbosity,"LOW_LEVEL_SEARCH: Returning consistent route plan with cost ", get_cost(node.solution))
    log_info(1,solver.l3_verbosity,"LOW_LEVEL_SEARCH: max path length = ", maximum(map(p->length(p), convert_to_vertex_lists(node.solution))))
    return true
end


################################################################################
############################## CBS Wrapper Stuff ###############################
################################################################################

export
    PC_MAPF


CRCBS.get_cost_model(env::E) where {E<:SearchEnv}   = get_cost_model(env.env)
CRCBS.get_cost_type(env::E) where {E<:SearchEnv}    = get_cost_type(env.env)
function CRCBS.initialize_root_node(env::SearchEnv)
    N = env.num_agents
    # It is important to only have N starts! Does not matter if they are invalid states.
    dummy_mapf = MAPF(env.env,map(i->PCCBS.State(),1:N),map(i->PCCBS.State(),1:N))
    initialize_root_node(dummy_mapf,deepcopy(env.base_solution))
end
function CRCBS.initialize_child_search_node(node::N,env::SearchEnv) where {N<:ConstraintTreeNode}
    initialize_child_search_node(node,deepcopy(env.base_solution))
end
function CRCBS.default_solution(env::SearchEnv)
    N = env.num_agents
    dummy_mapf = MAPF(env.env,map(i->PCCBS.State(),1:N),map(i->PCCBS.State(),1:N))
    default_solution(dummy_mapf)
end

"""
    `PC_MAPF`

    A precedence-constrained multi-agent path-finding problem. All agents have
    assigned tasks, but there are precedence constraints between tasks.
"""
struct PC_MAPF{E<:SearchEnv} <: AbstractMAPF
    env::E
end
CRCBS.initialize_root_node(pc_mapf::P) where {P<:PC_MAPF} = initialize_root_node(pc_mapf.env)
CRCBS.default_solution(pc_mapf::M) where {M<:PC_MAPF} = default_solution(pc_mapf.env)
CRCBS.initialize_child_search_node(node::N,pc_mapf::PC_MAPF) where {N<:ConstraintTreeNode} = initialize_child_search_node(node,pc_mapf.env)

function CRCBS.check_termination_criteria(solver::S,env::E,cost_so_far,s) where {S<:PC_TAPF_Solver,E<:AbstractLowLevelEnv}
    # solver.num_A_star_iterations += 1
    if solver.num_A_star_iterations > solver.LIMIT_A_star_iterations
        # throw(SolverAstarMaxOutException(string("# MAX OUT: A* limit of ",solver.LIMIT_A_star_iterations," exceeded.")))
        return true
    end
    return false
end

"""
    low_level_search_with_repair
"""
function CRCBS.low_level_search!(
        solver::S, pc_mapf::M, node::N, idxs::Vector{Int}=Vector{Int}();
        heuristic=get_heuristic_cost, path_finder=A_star
        ) where {S<:PC_TAPF_Solver,M<:PC_MAPF,N<:ConstraintTreeNode}

    # TODO make the number of "repair" iterations a parameter of PC_TAPF_Solver
    enter_low_level!(solver)
    valid_flag = true
    cache = initialize_planning_cache(pc_mapf.env.schedule;
        t0=deepcopy(pc_mapf.env.cache.t0),
        tF=deepcopy(pc_mapf.env.cache.tF)
    )
    search_env = SearchEnv(pc_mapf.env,cache=cache)
    for i in 1:solver.isps_model.n_repair_iters
        # reset_cache!(pc_mapf.env.cache, pc_mapf.env.schedule) # DEBUG This line appears to be carrying over all delays introduced in the cache...
        # valid_flag = low_level_search!(solver, pc_mapf.env, node)
        cache = initialize_planning_cache(pc_mapf.env.schedule;
            t0=deepcopy(pc_mapf.env.cache.t0),
            tF=deepcopy(pc_mapf.env.cache.tF)
        )
        search_env = SearchEnv(pc_mapf.env,cache=cache)
        valid_flag = low_level_search!(solver, search_env, node)
        if valid_flag == false
            log_info(0,solver.l3_verbosity,"LOW LEVEL SEARCH: failed on ",i,"th repair iteration.")
            break
        end
    end
    exit_low_level!(solver)
    return search_env, valid_flag
end

function CRCBS.solve!(
        solver::PC_TAPF_Solver,
        mapf::M where {M<:PC_MAPF},
        path_finder=A_star)

    enter_cbs!(solver)

    priority_queue = PriorityQueue{Tuple{ConstraintTreeNode,PlanningCache},get_cost_type(mapf.env)}()

    root_node = initialize_root_node(mapf) #TODO initialize with a partial solution when replanning
    search_env, valid_flag = low_level_search!(solver,mapf,root_node;path_finder=path_finder)
    detect_conflicts!(root_node.conflict_table,root_node.solution;t0=max(minimum(mapf.env.cache.t0), 1))
    if valid_flag
        enqueue!(priority_queue, (root_node,search_env.cache) => root_node.cost)
    else
        log_info(-1,solver.l2_verbosity,"CBS: first call to low_level_search returned infeasible.")
    end

    while length(priority_queue) > 0
        try
            node,cache = dequeue!(priority_queue)
            log_info(1,solver.l2_verbosity,string("CBS: node.cost = ",get_cost(node.solution)))
            # check for conflicts
            conflict = get_next_conflict(node.conflict_table)
            if !CRCBS.is_valid(conflict)
                log_info(-1,solver.l2_verbosity,string("CBS: valid route plan found after ",solver.num_CBS_iterations," CBS iterations! Cost = ",node.cost))
                return node.solution, cache, node.cost
            end
            log_info(1,solver.l2_verbosity,string("CBS: ", string(conflict)))
            # otherwise, create constraints and branch
            constraints = generate_constraints_from_conflict(conflict)
            for constraint in constraints
                # new_node = initialize_child_search_node(node)
                new_node = initialize_child_search_node(node,mapf)
                if CRCBS.add_constraint!(new_node,constraint)
                    step_cbs!(solver,constraint)
                    search_env, valid_flag = low_level_search!(solver, mapf, new_node, [get_agent_id(constraint)]; path_finder=path_finder)
                    if valid_flag
                        detect_conflicts!(new_node.conflict_table,new_node.solution,[get_agent_id(constraint)];t0=max(minimum(search_env.cache.t0), 1)) # update conflicts related to this agent
                        enqueue!(priority_queue, (new_node,search_env.cache) => new_node.cost)
                    end
                end
            end
        catch e
            if isa(e, SolverCBSMaxOutException)
                log_info(-1,solver.l2_verbosity,e.msg)
                break
            else
                throw(e)
            end
        end
    end
    log_info(-1,solver.l2_verbosity,"CBS: no solution found. Returning default solution")
    exit_cbs!(solver)
    solution, cost = default_solution(mapf)
    return solution, mapf.env.cache, cost
end
# CRCBS.solve!(solver,mapf::PC_MAPF,args...) = CRCBS.solve!(solver,mapf.env,args...)

################################################################################
######################### prioritized joint dfs planner ########################
################################################################################
export
    sorted_actions,
    get_conflict_idx,
    select_action_dfs!,
    get_next_node_matching_agent_id,
    update_envs!,
    prioritized_dfs_search

@with_kw struct DFS_SearchState
    pickup_i::Int   = 0 # if i < start, iterate from current action (inclusive). Defines where to "pick back up"
    reset_i::Int    = 0 # if i > reset_i, iterate through all possible actions. Defines the point below which to "reset" the action vector
end
function update_search_state(s,i)
    s = DFS_SearchState(s,reset_i = max(s.reset_i,s.pickup_i+1))
    if s.pickup_i > i # reset pickup_i once it has been exceeded
        s = DFS_SearchState(s, pickup_i = 0)
    end
    if s.reset_i < i
       s = DFS_SearchState(s,reset_i = i)
    end
    s
end
function sorted_actions(env,s)
    f = (s,a,sp)->add_heuristic_cost(env,get_transition_cost(env,s,a,sp),get_heuristic_cost(env,sp))
    sort(
        collect(get_possible_actions(env,s)),
        by=a->f(s,a,get_next_state(env,s,a))
    )
end
function get_conflict_idx(envs,states,actions,i,ordering,idxs)
    idx = ordering[i]
    env = envs[idx]
    s = states[idx]
    a = actions[idx]
    sp = get_next_state(env,s,a)
    pi = PathNode(s,a,sp)
    for (j,idx) in enumerate(ordering[1:max(1,i-1)])
        if j == i
            continue
        end
        env = envs[idx]
        s = states[idx]
        a = actions[idx]
        sp = get_next_state(env,s,a)
        pj = PathNode(s,a,sp)
        if detect_state_conflict(pi,pj) || detect_action_conflict(pi,pj)
            return j
        end
    end
    return -1
end
function get_next_node_matching_agent_id(schedule,cache,agent_id)
    node_id = RobotID(agent_id)
    for v in cache.active_set
        if agent_id == get_path_spec(schedule, v).agent_id
            return get_vtx_id(schedule,v)
        end
    end
    return node_id
end
function update_planning_cache!(solver,env)
    cache = env.cache
    schedule = env.schedule
    while true
        done = true
        for v in collect(cache.active_set)
            if get_path_spec(schedule,v).plan_path==false
                path = Path{PCCBS.State,PCCBS.Action,get_cost_type(env.env)}(
                    s0=PCCBS.State(-1, -1),
                    cost=get_initial_cost(env.env)
                    )
                # update planning cache only
                update_planning_cache!(solver,env,v,path) # NOTE I think this is all we need, since there is no actual path to update
                done = false
            end
        end
        if done
            break
        end
    end
end
function update_envs!(solver,search_env,envs,paths)
    cache = search_env.cache
    schedule = search_env.schedule
    cbs_node = initialize_root_node(search_env)
    update_planning_cache!(solver,search_env)
    i = 0
    while i < length(envs)
        i += 1
        env = envs[i]
        path = paths[i]
        if is_goal(envs[i],get_final_state(path))
            # update schedule and cache
            v = get_vtx(schedule,env.node_id)
            update_planning_cache!(solver,search_env,v,path)
            update_planning_cache!(solver,search_env)
            # TODO update all vertices that don't require a robot
            @assert v in cache.closed_set
            # swap out env for new env
            @assert i == env.agent_idx
            node_id = get_next_node_matching_agent_id(schedule,cache,env.agent_idx)
            @assert node_id != env.node_id
            if get_vtx(schedule,node_id) in cache.active_set
                envs[i],_ = build_env(solver,search_env,cbs_node,get_vtx(schedule,node_id))
                i = 0
            end
        end
    end
    update_cost_model!(search_env)
    envs,paths
end
function select_ordering(solver,search_env,envs)
    schedule = search_env.schedule
    cache = search_env.cache
    ordering = sort(
        collect(1:search_env.num_agents),
        by = i->(
            ~isa(envs[i].schedule_node,Union{COLLECT,DEPOSIT}),
            minimum(cache.slack[get_vtx(schedule,envs[i].node_id)])
            )
        )
    log_info(4,solver.l2_verbosity,"ordering = $ordering")
    ordering
end
function select_action_dfs!(solver,envs,states,actions,i,ordering,idxs,search_state=SearchState())
    # search_states = map(e->search_state,1:length(envs)+1)
    # while true
    search_state = update_search_state(search_state,i)
    # search_states[i] = update_search_state(search_states[i],i)
    if i <= 0
        return false
    elseif i > length(states)
        for (env,s,a) in zip(envs,states,actions)
            if a != CRCBS.wait(env,s) || length(get_possible_actions(env,s)) == 1
                return true
            end
            # log_info(5,solver.l2_verbosity,"action ",string(a)," ineligible with env ",string(env.schedule_node)," |A| = $(length(get_possible_actions(env,s)))")
        end
        log_info(5,solver.l2_verbosity,"action vector $(map(a->string(a),actions)) not eligible")
        return false
    elseif !(ordering[i] in idxs)
        # search_states[i+1] = search_states[i]
        # i = i + 1
        return select_action_dfs!(solver,envs,states,actions,i+1,ordering,idxs,search_state)
    else
        # idx = env.ordering_map[i]zs
        j = 0
        idx = ordering[i]
        env = envs[idx]
        s = states[idx]
        for ai in sorted_actions(env,s)
            a = actions[idx]
            c = get_transition_cost(env,s,ai)
            c0 = get_transition_cost(env,s,a)
            if (i >= search_state.reset_i) || (i < search_state.pickup_i && a == ai) || ((c >= c0 || is_valid(env,a)) && a != ai)
                actions[idx] = ai
                log_info(5,solver.l2_verbosity,"$(repeat(" ",i))i = $i, trying a=",string(ai)," for env ",string(env.schedule_node))
                k = get_conflict_idx(envs,states,actions,i,ordering,idxs)
                # @assert k < i "should only check for conflicts with 1:$i, but found conflict with $k"
                if k <= 0
                    # search_states[i+1] = search_states[i]
                    # i = i+1
                    # break
                    if select_action_dfs!(solver,envs,states,actions,i+1,ordering,idxs,search_state)
                        return true
                    end
                elseif !(ordering[k] in idxs)
                    # i = i - 1
                    # break
                    # return false
                else
                    log_info(5,solver.l2_verbosity,"--- conflict betweeh $i and $k")
                    j = max(k,j)
                end
            end
        end
        if j <= 0
            return false
        end
        # if j > 0
        search_state = DFS_SearchState(pickup_i=j,reset_i=0)
        # search_states[j] = search_states[i]
        # i = j
        # break
        # end
        return select_action_dfs!(solver,envs,states,actions,j,ordering,idxs,search_state) #
    end
    # end
end
function prioritized_dfs_search(solver,search_env,envs,paths;
        t0 = max(0, minimum(map(path->length(path),paths))),
        max_iters = 4*(maximum(search_env.cache.tF)-minimum(map(p->length(p),paths))),
        search_state = DFS_SearchState()
        )
     tip_times = map(path->length(path),paths)
     t = t0
     log_info(3,solver.l2_verbosity,"start time t0=$t0")
     states     = map(path->get_s(get_path_node(path,t+1)), paths)
     actions    = map(path->get_a(get_path_node(path,t+1)), paths)
     iter = 0
     while true && iter < max_iters
         iter += 1
         update_envs!(solver,search_env,envs,paths)
         # if all(map(i->is_goal(envs[i],states[i]),1:length(paths)))
         if length(search_env.cache.active_set) == 0
             return envs, paths, true
         end
         log_info(4,solver.l2_verbosity,"envs: $(string(map(e->string(e.schedule_node),envs))...)")
         log_info(4,solver.l2_verbosity,"path_lengths = $(map(path->length(path),paths))")
         log_info(4,solver.l2_verbosity,"states: $(string(map(s->string(s),states))...)")
         @assert all(map(path->length(path)>=t,paths))
         ordering   = select_ordering(solver,search_env,envs)
         idxs       = Set(findall(tip_times .<= t))
         log_info(4,solver.l2_verbosity,"idxs: $idxs")
         if select_action_dfs!(solver,envs,states,actions,1,ordering,idxs,search_state)
             log_info(4,solver.l2_verbosity,"actions: $(string(map(a->string(a),actions))...)")
             # step forward in search
             for idx in idxs
                 env = envs[idx]
                 path = paths[idx]
                 s = states[idx]
                 a = actions[idx]
                 sp = get_next_state(envs[idx],s,a)
                 push!(path.path_nodes,PathNode(s,a,sp))
                 path.cost = accumulate_cost(env,path.cost,get_transition_cost(env,s,a,sp))
             end
             t += 1
             search_state = DFS_SearchState()
             log_info(4,solver.l2_verbosity,"stepping forward, t = $t")
         else
             # step backward in search
             all(tip_times .> t) == 0 ? break : nothing
             for (idx,path) in enumerate(paths)
                 if tip_times[idx] < t
                     pop!(path.path_nodes)
                     @assert length(path) == t-1
                 end
             end
             t -= 1
             idxs    = Set(findall(tip_times .<= t))
              # start new search where previous search left off
             search_state = DFS_SearchState(pickup_i = maximum(idxs))
             log_info(3,solver.l2_verbosity,"stepping backward, t = $t")
         end
         states     = map(path->get_s(get_path_node(path,t+1)), paths)
         actions    = map(path->get_a(get_path_node(path,t+1)), paths)
     end
     return envs, paths, false
end
"""
    Iterate over agents in order of priority, allowing them to fill in a
    reservation table for the vtxs they would like to occupy at the next time
    step(s).
"""
function CRCBS.solve!(
    solver::PC_TAPF_Solver{M,C,I,A,T},
    mapf::P;kwargs...) where {M,C<:PrioritizedDFSPlanner,I,A,T,P<:PC_MAPF}

    search_env = mapf.env

    route_plan = deepcopy(search_env.base_solution)
    paths = get_paths(route_plan)
    envs = Vector{PCCBS.LowLevelEnv}([PCCBS.LowLevelEnv() for p in paths])
    cbs_node = initialize_root_node(search_env)
    for i in 1:search_env.num_agents
        node_id = get_next_node_matching_agent_id(search_env.schedule,search_env.cache,i)
        envs[i], _ = build_env(solver,search_env,cbs_node,get_vtx(search_env.schedule,node_id))
    end

    envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;
        max_iters = solver.cbs_model.max_iters
    )
    @show validate(search_env.schedule,convert_to_vertex_lists(route_plan),search_env.cache.t0,search_env.cache.tF)
    # for (path,base_path,env) in zip(paths,search_env.base_solution.paths,envs)
    #     c = base_path.cost
    #     for p in path.path_nodes
    #         c = accumulate_cost(env,c,get_transition_cost(env,p.s,p.a,p.sp))
    #     end
    #     path.cost = c
    # end
    cost = aggregate_costs(get_cost_model(search_env),map(p->get_cost(p),paths))

    return route_plan, search_env.cache, cost
end



################################################################################
############################## Outer Loop Search ###############################
################################################################################
export
    high_level_search!

"""
    This is the modified version of high-level search that uses the adjacency
    matrix MILP formulation.
"""
function high_level_search!(solver::P, base_search_env::SearchEnv,  optimizer;
        t0_ = Dict{AbstractID,Int}(get_vtx_id(base_search_env.schedule, v)=>t0 for (v,t0) in enumerate(base_search_env.cache.t0)),
        tF_ = Dict{AbstractID,Int}(get_vtx_id(base_search_env.schedule, v)=>tF for (v,tF) in enumerate(base_search_env.cache.tF)),
        primary_objective=SumOfMakeSpans,
        TimeLimit=solver.nbs_time_limit,
        buffer=5.0, # to give some extra time to the path planner if the milp terminates late.
        kwargs...) where {P<:PC_TAPF_Solver}

    enter_assignment!(solver)
    log_info(0,solver.l1_verbosity,string("HIGH LEVEL SEARCH: beginning search with ",typeof(solver.nbs_model)," ..."))

    project_schedule    = base_search_env.schedule
    env_graph           = base_search_env.env.graph
    problem_spec        = base_search_env.problem_spec

    lower_bound = 0.0
    # best_solution = default_pc_tapf_solution(problem_spec.N)
    best_solution   = default_solution(base_search_env)
    best_env        = SearchEnv()
    best_assignment = adjacency_matrix(get_graph(project_schedule))

    TimeLimit       = min(TimeLimit,solver.time_limit-(buffer+time()-solver.start_time))
    @assert(TimeLimit >= 0.0, "TimeLimit=$TimeLimit, solver.time_limit=$(solver.time_limit), time()=$(time()), solver.start_time=$(solver.start_time), buffer=$buffer")
    model = formulate_milp(solver.nbs_model,project_schedule,problem_spec;
        cost_model=primary_objective,optimizer=optimizer,t0_=t0_,tF_=tF_,
        TimeLimit=TimeLimit,
        kwargs...) #TODO pass t0_ in replanning mode

    base_schedule = deepcopy(project_schedule)

    solver.start_time = time()
    while get_primary_cost(solver,solver.best_cost) > lower_bound
        try
            solver.num_assignment_iterations += 1
            check_time(solver)
            log_info(0,solver.l1_verbosity,string("HIGH LEVEL SEARCH: iteration ",solver.num_assignment_iterations,"..."))
            ############## Task Assignment ###############
            exclude_solutions!(model) # exclude most recent solution in order to get next best solution
            optimize!(model)
            optimal = (termination_status(model) == MathOptInterface.OPTIMAL);
            feasible = (primal_status(model) == MOI.FEASIBLE_POINT) # TODO use this!
            if !feasible
                log_info(0,solver.l1_verbosity,string("HIGH LEVEL SEARCH: Task assignment infeasible. Returning best solution so far.\n",
                    " * optimality gap = ", get_primary_cost(solver,solver.best_cost) - lower_bound))
                break
            end
            if optimal
                lower_bound = max(lower_bound, Int(round(value(objective_function(model)))) )
            else
                log_info(0,solver.l1_verbosity,string("HIGH LEVEL SEARCH: MILP not optimally solved. Current lower bound cost = ",lower_bound))
                lower_bound = max(lower_bound, Int(round(value(objective_bound(model)))) )
                solver.LIMIT_assignment_iterations = solver.num_assignment_iterations
            end
            log_info(0,solver.l1_verbosity,string("HIGH LEVEL SEARCH: Current lower bound cost = ",lower_bound))
            # if lower_bound < get_primary_cost(solver,solver.best_cost)
            if lower_bound < get_primary_cost(solver,solver.best_cost)
                ############## Route Planning ###############
                adj_matrix = get_assignment_matrix(model);
                project_schedule = deepcopy(base_schedule)
                if update_project_schedule!(model,project_schedule,problem_spec,adj_matrix)
                    env = construct_search_env(solver,project_schedule, problem_spec, env_graph;
                        primary_objective=primary_objective,
                        t0 = base_search_env.cache.t0,
                        tF = base_search_env.cache.tF
                        ) # TODO pass in previous solution
                    env = SearchEnv(env,base_solution=base_search_env.base_solution)
                    pc_mapf = PC_MAPF(env);
                    ##### Call CBS Search Routine (LEVEL 2) #####
                    solution, cache, cost = solve!(solver,pc_mapf);
                    if cost < solver.best_cost
                        best_solution = solution
                        best_assignment = adj_matrix # this represents an assignment matrix in AssignmentMILP
                        solver.best_cost = cost # TODO make sure that the operation duration is accounted for here
                        best_env = SearchEnv(env,cache=cache)
                    end
                    optimality_gap = get_primary_cost(solver,solver.best_cost) - lower_bound
                    log_info(0,solver,string("HIGH LEVEL SEARCH: Best cost so far = ", get_primary_cost(solver,solver.best_cost), ". Optimality gap = ",optimality_gap))
                end
            end
        catch e
            if isa(e, SolverException)
                log_info(-1,solver.l1_verbosity,e.msg)
                # if isa(e, SolverAstarMaxOutException)
                #     throw(e)
                # end
                break
            else
                throw(e)
            end
        end
    end
    exit_assignment!(solver)
    optimality_gap = get_primary_cost(solver,solver.best_cost) - lower_bound
    log_info(-1,solver.l1_verbosity,string("HIGH LEVEL SEARCH: optimality gap = ",optimality_gap,". Returning best solution with cost ", solver.best_cost,"\n"))
    return best_solution, best_assignment, solver.best_cost, best_env, optimality_gap
end

function high_level_search!(solver::PC_TAPF_Solver, env_graph, project_schedule::ProjectSchedule, problem_spec,  optimizer;
    primary_objective=SumOfMakeSpans,
    kwargs...)
    base_search_env = construct_search_env(solver,project_schedule,problem_spec,env_graph;primary_objective=primary_objective)
    # @show base_search_env.cache.t0
    high_level_search!(solver, base_search_env, optimizer;
        primary_objective=primary_objective,
        kwargs...
    )
end
function high_level_search!(solver::P, env_graph, project_spec::ProjectSpec, problem_spec,
        robot_ICs, optimizer;kwargs...) where {P<:PC_TAPF_Solver}

    project_schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
    high_level_search!(solver, env_graph, project_schedule, problem_spec, optimizer;
        kwargs...
    )
end

function high_level_search!(milp_model::M, args...;kwargs...) where {M<:TaskGraphsMILP}
    high_level_search!(args...;milp_model=milp_model,kwargs...)
end

export
    get_env_snapshot,
    trim_solution


"""
    get_env_snapshot
"""
function get_env_snapshot(solution::S,t) where {S<:LowLevelSolution}
    Dict(RobotID(i)=>ROBOT_AT(i, get_sp(get_path_node(path,t)).vtx) for (i,path) in enumerate(get_paths(solution)))
end

"""
    `trim_solution`

    construct a trimmed solution that stops at a certain time step
"""
function trim_solution(search_env, solution, T)
    N = length(get_paths(solution))
    env = search_env.env
    trimmed_solution = get_initial_solution(MAPF(env, map(i->PCCBS.State(),1:N),map(i->PCCBS.State(),1:N)))
    for agent_id in 1:N
        cbs_env = typeof(env)(
            graph = env.graph,
            agent_idx = agent_id,
            cost_model = get_cost_model(env),
            heuristic = get_heuristic_model(env)
        )
        old_path = get_paths(solution)[agent_id]
        new_path = get_paths(trimmed_solution)[agent_id]
        for t in 1:max(1, min(T, length(old_path)))
            p = get_path_node(old_path,t)
            push!(new_path, p)
            new_path.cost = accumulate_cost(cbs_env, get_cost(new_path), get_transition_cost(cbs_env, p.s, p.a, p.sp))
            set_path_cost!(trimmed_solution, new_path.cost, agent_id)
        end
        if T > length(new_path)
            println("Extending path in trim_solution. Agent id = ",agent_id)
            extend_path!(cbs_env,new_path,T)
        end
    end
    # trimmed_solution.cost = solution.cost
    trimmed_solution
end


export
    get_active_and_fixed_vtxs,
    split_active_vtxs!,
    fix_precutoff_nodes!,
    break_assignments!,
    prune_schedule,
    prune_project_schedule,
    splice_schedules!

"""
    get all vertices that "straddle" the query time t
"""
function get_active_and_fixed_vtxs(project_schedule::ProjectSchedule,cache::PlanningCache,t)
    active_vtxs = Set{Int}()
    fixed_vtxs = Set{Int}()
    for v in vertices(get_graph(project_schedule))
        if cache.tF[v] + minimum(cache.local_slack[v]) <= t
            push!(fixed_vtxs, v)
        elseif cache.t0[v] <= t < cache.tF[v] + minimum(cache.local_slack[v])
            push!(active_vtxs, v)
        end
    end
    @assert all(map(v->cache.tF[v] + minimum(cache.local_slack[v]), collect(active_vtxs)) .>= t)
    active_vtxs, fixed_vtxs
end

"""
    Split all GO nodes that "straddle" the cutoff time.
"""
function split_active_vtxs!(project_schedule::ProjectSchedule,problem_spec::ProblemSpec,cache::PlanningCache,t;
    robot_positions::Dict{RobotID,ROBOT_AT}=Dict{RobotID,ROBOT_AT}()
    )
    G = get_graph(project_schedule)
    # identify nodes "cut" by timestep t
    active_vtxs, fixed_vtxs = get_active_and_fixed_vtxs(project_schedule,cache,t)
    # split active nodes
    t0 = deepcopy(cache.t0)
    tF = deepcopy(cache.tF)
    for v in active_vtxs
        node_id = get_vtx_id(project_schedule,v)
        node = get_node_from_id(project_schedule, node_id)
        # if isa(node,Union{GO,CARRY,COLLECT,DEPOSIT}) # split
        if isa(node,GO) # split
            x = robot_positions[node.r].x
            node1,node2 = split_node(node,x)
            replace_in_schedule!(project_schedule,problem_spec,node1,node_id)
            node_id2 = ActionID(get_unique_action_id())
            add_to_schedule!(project_schedule,problem_spec,node2,node_id2)
            # remap edges
            v2 = get_vtx(project_schedule, node_id2)
            for vp in outneighbors(G,v)
                rem_edge!(G,v,vp)
                add_edge!(G,v2,vp)
            end
            add_edge!(G,v,v2)
            # fix start and end times
            push!(t0,t)
            push!(tF,tF[v])
            tF[v] = t
            # reset path specs
            set_path_spec!(project_schedule,v,PathSpec(get_path_spec(project_schedule,v),fixed=true,plan_path=false,min_path_duration=tF[v]-t0[v]))
            set_path_spec!(project_schedule,v2,PathSpec(get_path_spec(project_schedule,v2),tight=true,min_path_duration=tF[v2]-t0[v2]))
        end
    end
    set_leaf_operation_nodes!(project_schedule)
    new_cache = initialize_planning_cache(project_schedule;t0=t0,tF=tF)
    project_schedule, new_cache
end

"""
    identify all nodes that end before the cutoff time, and change their path
    spec so that the route planner will not actually plan a path for them.
"""
function fix_precutoff_nodes!(project_schedule::ProjectSchedule,problem_spec::ProblemSpec,cache::PlanningCache,t)
    # active_vtxs = Set{Int}()
    active_vtxs, fixed_vtxs = get_active_and_fixed_vtxs(project_schedule,cache,t)
    # set all fixed_vtxs to plan_path=false
    for v in fixed_vtxs
        set_path_spec!(project_schedule,v,PathSpec(get_path_spec(project_schedule,v), plan_path=false, fixed=true))
    end
    # verify that all vertices following active_vtxs have a start time > 0
    @assert all(map(v->cache.t0[v], collect(fixed_vtxs)) .<= t)
    @assert all(map(v->cache.tF[v] + minimum(cache.local_slack[v]), collect(active_vtxs)) .>= t)
    project_schedule, cache
end

"""
    Break all assignments that are eligible for replanning
"""
function break_assignments!(project_schedule::ProjectSchedule,problem_spec::ProblemSpec)
    G = get_graph(project_schedule)
    for v in vertices(G)
        path_spec = get_path_spec(project_schedule,v)
        if path_spec.fixed == true
            continue
        end
        node_id = get_vtx_id(project_schedule,v)
        node = get_node_from_id(project_schedule,node_id)
        if isa(node, AbstractRobotAction)
            new_node = typeof(node)(node,r=RobotID(-1))
            if isa(node,GO)
                new_node = GO(new_node,x2=StationID(-1))
                for v2 in outneighbors(G,v)
                    rem_edge!(G,v,v2)
                end
            end
            replace_in_schedule!(project_schedule,problem_spec,new_node,node_id)
        elseif isa(node,TEAM_ACTION)
            for i in 1:length(node.instructions)
                n = node.instructions[i]
                node.instructions[i] = typeof(n)(n,r=RobotID(-1))
            end
        end
    end
    propagate_valid_ids!(project_schedule,problem_spec)

    project_schedule
end

"""
    Prune schedule

    remove nodes that don't need to be kept around any longer
"""
function prune_schedule(project_schedule::ProjectSchedule,problem_spec::ProblemSpec,cache::PlanningCache,t)
    G = get_graph(project_schedule)

    # identify nodes "cut" by timestep
    active_vtxs, fixed_vtxs = get_active_and_fixed_vtxs(project_schedule,cache,t)
    # construct set of all nodes to prune out.
    remove_set = Set{Int}()
    for v in active_vtxs
    # for v in fixed_vtxs
        if isa(get_node_from_vtx(project_schedule,v),Operation)
            push!(remove_set, v)
        end
        for e in edges(bfs_tree(G,v;dir=:in))
            if isa(get_node_from_vtx(project_schedule, e.dst),Operation)
                push!(remove_set, e.dst)
            end
        end
    end
    for v in collect(remove_set)
        for e in edges(bfs_tree(G,v;dir=:in))
            if !(isa(get_node_from_vtx(project_schedule, e.dst),ROBOT_AT))
                push!(remove_set, e.dst)
            end
        end
    end
    # Construct new graph
    new_schedule = ProjectSchedule()
    keep_vtxs = setdiff(Set{Int}(collect(vertices(G))), remove_set)
    # add all non-deleted nodes to new project schedule
    for v in keep_vtxs
        node_id = get_vtx_id(project_schedule,v)
        node = get_node_from_id(project_schedule, node_id)
        path_spec = get_path_spec(project_schedule,v)
        add_to_schedule!(new_schedule,path_spec,node,node_id)
    end
    # add all edges between nodes that still exist
    for e in edges(get_graph(project_schedule))
        add_edge!(new_schedule, get_vtx_id(project_schedule, e.src), get_vtx_id(project_schedule, e.dst))
    end
    # Initialize new cache
    G = get_graph(new_schedule)
    t0 = map(v->get(cache.t0, get_vtx(project_schedule, get_vtx_id(new_schedule, v)), 0.0), vertices(G))
    tF = map(v->get(cache.tF, get_vtx(project_schedule, get_vtx_id(new_schedule, v)), 0.0), vertices(G))
    # draw new ROBOT_AT -> GO edges where necessary
    for v in vertices(get_graph(new_schedule))
        node_id = get_vtx_id(new_schedule,v)
        node = get_node_from_id(new_schedule, node_id)
        if isa(node,GO) && indegree(get_graph(new_schedule),v) == 0
            robot_id = get_robot_id(node)
            replace_in_schedule!(new_schedule, ROBOT_AT(robot_id, node.x1), robot_id)
            t0[get_vtx(new_schedule,robot_id)] = t0[v]
            add_edge!(new_schedule, robot_id, node_id)
        end
        if isa(node,Operation) && indegree(get_graph(new_schedule),v) < num_required_predecessors(node)
            input_ids = Set(map(v2->get_object_id(get_node_from_vtx(new_schedule,v2)), inneighbors(G,v)))
            for o in node.pre
                if !(get_object_id(o) in input_ids)
                    add_to_schedule!(new_schedule,o,get_object_id(o))
                    push!(t0,t0[v])
                    push!(tF,t0[v])
                    add_edge!(new_schedule, get_object_id(o), node_id)
                end
            end
        end
    end
    set_leaf_operation_nodes!(new_schedule)
    # init planning cache with the existing solution
    new_cache = initialize_planning_cache(new_schedule;t0=t0,tF=tF)

    @assert sanity_check(new_schedule,string(" in prune_schedule() at t = ",t,":\n",[string(string(get_node_from_vtx(project_schedule,v))," - t0 = ",cache.t0[v]," - tF = ",cache.tF[v]," - local_slack = ",cache.local_slack[v],"\n") for v in active_vtxs]...))

    new_schedule, new_cache
end

"""
    `prune_project_schedule`

    Remove all vertices that have already been completed. The idea is to identify
    all `Operation`s that are completed before `t`, remove all nodes upstream of
    them (except for ROBOT_AT nodes), and create new edges between the ROBOT_AT
    nodes and their first GO assignments.
"""
function prune_project_schedule(project_schedule::ProjectSchedule,problem_spec::ProblemSpec,cache::PlanningCache,t;
        robot_positions::Dict{RobotID,ROBOT_AT}=Dict{RobotID,ROBOT_AT}()
    )
    new_schedule, new_cache = prune_schedule(project_schedule,problem_spec,cache,t)
    # split active nodes
    new_schedule, new_cache = split_active_vtxs!(new_schedule,problem_spec,new_cache,t;robot_positions=robot_positions)
    # freeze nodes that terminate before cutoff time
    fix_precutoff_nodes!(new_schedule,problem_spec,new_cache,t)
    # Remove all "assignments" from schedule
    break_assignments!(new_schedule,problem_spec)

    new_cache = initialize_planning_cache(new_schedule;t0=new_cache.t0,tF=min.(new_cache.tF,t))
    new_schedule, new_cache
end

"""
    Merge next_schedule into project_schedule
"""
function splice_schedules!(project_schedule::P,next_schedule::P) where {P<:ProjectSchedule}
    for v in vertices(get_graph(next_schedule))
        node_id = get_vtx_id(next_schedule, v)
        add_to_schedule!(project_schedule, get_path_spec(next_schedule,v), get_node_from_id(next_schedule, node_id), node_id)
    end
    for e in edges(get_graph(next_schedule))
        node_id1 = get_vtx_id(next_schedule, e.src)
        node_id2 = get_vtx_id(next_schedule, e.dst)
        add_edge!(project_schedule, node_id1, node_id2)
    end
    set_leaf_operation_nodes!(project_schedule)
    project_schedule
end
splice_schedules!(project_schedule,next_schedule) = project_schedule

export
    ReplannerModel,
    DeferUntilCompletion,
    ReassignFreeRobots,
    MergeAndBalance,
    Oracle,
    FallBackPlanner,
    get_commit_time,
    replan!

abstract type ReplannerModel end
@with_kw struct DeferUntilCompletion <: ReplannerModel
    max_time_limit::Float64 = 100
    time_out_buffer::Float64 = 1
    route_planning_buffer::Float64 = 2
end
@with_kw struct ReassignFreeRobots   <: ReplannerModel
    time_out_buffer::Float64 = 1
    route_planning_buffer::Float64 = 2
end
@with_kw struct MergeAndBalance      <: ReplannerModel
    time_out_buffer::Float64 = 1
    route_planning_buffer::Float64 = 2
end
@with_kw struct Oracle               <: ReplannerModel
    time_out_buffer::Float64 = -110
    route_planning_buffer::Float64 = 10
end
@with_kw struct FallBackPlanner      <: ReplannerModel
    time_out_buffer::Float64 = 1
    route_planning_buffer::Float64 = 2
end
@with_kw struct NullReplanner        <: ReplannerModel
    time_out_buffer::Float64 = 1
    route_planning_buffer::Float64 = 2
end

get_timeout_buffer(replan_model) = replan_model.time_out_buffer
get_route_planning_buffer(replan_model) = replan_model.route_planning_buffer

get_commit_time(replan_model, search_env, t_request, commit_threshold) = t_request + commit_threshold
get_commit_time(replan_model::Oracle, search_env, t_request, args...) = t_request
get_commit_time(replan_model::DeferUntilCompletion, search_env, t_request, commit_threshold) = max(t_request + commit_threshold,maximum(search_env.cache.tF))

break_assignments!(replan_model::ReplannerModel,args...) = break_assignments!(args...)
break_assignments!(replan_model::ReassignFreeRobots,args...) = nothing
break_assignments!(replan_model::DeferUntilCompletion,args...) = nothing

function set_time_limits!(solver,replan_model,t_request,t_commit)
    solver.time_limit = (t_commit - t_request) - get_timeout_buffer(replan_model)
    solver.nbs_time_limit = solver.time_limit - get_route_planning_buffer(replan_model)
    solver
end
function set_time_limits!(solver,replan_model::DeferUntilCompletion,t_request,t_commit)
    solver.time_limit = (t_commit - t_request) - get_timeout_buffer(replan_model)
    solver.time_limit = min(solver.time_limit,replan_model.max_time_limit)
    solver.nbs_time_limit = solver.time_limit - get_route_planning_buffer(replan_model)
    solver
end

split_active_vtxs!(replan_model::ReplannerModel,new_schedule,problem_spec,new_cache,t_commit;kwargs...) = split_active_vtxs!(new_schedule,problem_spec,new_cache,t_commit;kwargs...)

fix_precutoff_nodes!(replan_model,new_schedule,problem_spec,new_cache,t_commit) = fix_precutoff_nodes!(new_schedule,problem_spec,new_cache,t_commit)
# function fix_precutoff_nodes!(replan_model::DeferUntilCompletion,project_schedule,problem_spec,new_cache,t_commit)
#     fix_precutoff_nodes!(project_schedule,problem_spec,new_cache,t_commit)
#     # prune all nodes but the tip GO nodes
#     new_schedule = ProjectSchedule()
#     for v in vertices(project_schedule)
#         if ~get_path_spec(project_schedule,v).fixed
#             node_id = get_vtx_id(project_schedule,v)
#             node = get_node_from_id(project_schedule,node_id)
#             @assert isa(node,GO)
#             add_to_schedule!(new_schedule,problem_spec,node,node_id)
#             robot_id = get_robot_id(node)
#             robot_node = ROBOT_AT(get_robot_id(node),get_initial_location_id(node))
#             robot_path_spec = PathSpec(generate_path_spec(new_schedule,problem_spec,robot_node),fixed=true,plan_path=false)
#             add_to_schedule!(new_schedule,robot_path_spec,robot_node,robot_id)
#             add_edge!(new_schedule,robot_id,node_id)
#         end
#     end
#     new_cache = initialize_planning_cache(new_schedule;t0=map(v->t_commit,vertices(new_schedule)))
#     new_schedule, new_cache
#     # try
#     #     @assert(all(map(v->get_path_spec(new_schedule,v).fixed,vertices(new_schedule))))
#     # catch e
#     #     for v in vertices(new_schedule)
#     #         if ~get_path_spec(new_schedule,v).fixed
#     #             @show string(get_node_from_vtx(new_schedule,v)), get_path_spec(new_schedule,v).fixed, new_cache.t0[v],new_cache.tF[v]
#     #             for v2 in inneighbors(new_schedule,v)
#     #                @show string(get_node_from_vtx(new_schedule,v2)), new_cache.t0[v2],new_cache.tF[v2]
#     #             end
#     #         end
#     #     end
#     #     throw(e)
#     # end
# end

function replan!(solver, replan_model, search_env, env_graph, problem_spec, solution, next_schedule, t_request, t_arrival; commit_threshold=5,kwargs...)
    project_schedule = search_env.schedule
    cache = search_env.cache
    @assert sanity_check(project_schedule," in replan!()")
    # Freeze solution and schedule at t_commit
    t_commit = get_commit_time(replan_model, search_env, t_request, commit_threshold)
    reset_solver!(solver)
    set_time_limits!(solver,replan_model,t_request,t_commit)
    # Update operating schedule
    new_schedule, new_cache = prune_schedule(project_schedule,problem_spec,cache,t_commit)
    @assert sanity_check(new_schedule," after prune_schedule()")
    # split active nodes
    robot_positions=get_env_snapshot(solution,t_commit)
    new_schedule, new_cache = split_active_vtxs!(replan_model,new_schedule,problem_spec,new_cache,t_commit;robot_positions=robot_positions)
    @assert sanity_check(new_schedule," after split_active_vtxs!()")
    # freeze nodes that terminate before cutoff time
    new_schedule, new_cache = fix_precutoff_nodes!(replan_model,new_schedule,problem_spec,new_cache,t_commit)
    # Remove all "assignments" from schedule
    break_assignments!(replan_model,new_schedule,problem_spec)
    @assert sanity_check(new_schedule," after break_assignments!()")
    new_cache = initialize_planning_cache(new_schedule;t0=new_cache.t0,tF=min.(new_cache.tF,t_commit))
    # splice projects together!
    splice_schedules!(new_schedule,next_schedule)
    @assert sanity_check(new_schedule," after splice_schedules!()")
    # t0 = map(v->get(new_cache.t0, v, t_arrival), vertices(get_graph(new_schedule)))
    # tF = map(v->get(new_cache.tF, v, t_arrival), vertices(get_graph(new_schedule)))
    # NOTE: better performance is obtained when t_commit is the default t0 (tighter constraint on milp)
    t0 = map(v->get(new_cache.t0, v, t_commit), vertices(get_graph(new_schedule)))
    tF = map(v->get(new_cache.tF, v, t_commit), vertices(get_graph(new_schedule)))
    base_search_env = construct_search_env(solver, new_schedule, search_env.problem_spec, env_graph;t0=t0,tF=tF)
    trimmed_solution = trim_solution(base_search_env, solution, t_commit)
    base_search_env = SearchEnv(base_search_env, base_solution=trimmed_solution)
    base_search_env, solver
end
replan!(solver, replan_model::NullReplanner, search_env, args...;kwargs...) = search_env

end # PathPlanning module
