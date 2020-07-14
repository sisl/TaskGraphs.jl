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
    max_iters::Int = 3000
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

@with_kw mutable struct PC_TAPF_Solver{M,C,I,A,T} 
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
    reset_solver!

# function reset_solver!(solver::S) where {S<:PC_TAPF_Solver}
#     solver.num_assignment_iterations = 0
#     solver.num_CBS_iterations = 0
#     solver.num_A_star_iterations = 0
#     solver.best_cost = map(c->Inf,solver.best_cost)
#
#     solver.total_assignment_iterations = 0
#     solver.total_CBS_iterations = 0
#     solver.total_A_star_iterations = 0
#
#     solver.max_CBS_iterations = 0
#     solver.max_A_star_iterations = 0
#
#     # solver.start_time = time() # Set solver start time to current time
#
#     solver
# end
# function check_time(solver::PC_TAPF_Solver)
#     if time() - solver.start_time >= solver.time_limit
#         throw(SolverTimeOutException(string("# TIME OUT: Overall time limit of ",solver.time_limit," seconds exceeded.")))
#     elseif solver.num_assignment_iterations > solver.LIMIT_assignment_iterations
#         throw(SolverMilpMaxOutException(string("# MAX OUT: milp solver iteration limit of ",solver.LIMIT_assignment_iterations," exceeded.")))
#     elseif solver.num_CBS_iterations > solver.LIMIT_CBS_iterations
#         throw(SolverCBSMaxOutException(string("# MAX OUT: cbs iteration limit of ",solver.LIMIT_CBS_iterations," exceeded.")))
#     # elseif solver.num_A_star_iterations > solver.LIMIT_A_star_iterations
#     #     throw(SolverAstarMaxOutException(string("# MAX OUT: A* iteration limit of ",solver.LIMIT_A_star_iterations," exceeded.")))
#     end
# end
# # update functions
# function enter_assignment!(solver::S) where {S<:PC_TAPF_Solver}
#     reset_solver!(solver)
# end
# function exit_assignment!(solver::S) where {S<:PC_TAPF_Solver}
# end
# function enter_cbs!(solver::S) where {S<:PC_TAPF_Solver}
#     # solver.num_assignment_iterations += 1
#     log_info(0,solver.l2_verbosity,"CBS: solver.num_CBS_iterations = ",solver.num_CBS_iterations)
#     check_time(solver)
# end
# function step_cbs!(solver::S,constraint) where {S<:PC_TAPF_Solver}
#     solver.num_CBS_iterations += 1
#     log_info(1,solver.l2_verbosity,"CBS: iteration ", solver.num_CBS_iterations)
#     log_info(1,solver.l2_verbosity,string("CBS: constraint on agent id = ",get_agent_id(constraint),", time index = ",get_time_of(constraint)))
#     # check_time(solver)
# end
# function exit_cbs!(solver::S) where {S<:PC_TAPF_Solver}
#     solver.max_CBS_iterations = max(solver.max_CBS_iterations,solver.num_CBS_iterations)
#     solver.total_CBS_iterations += solver.num_CBS_iterations
#     solver.num_CBS_iterations = 0
#     check_time(solver)
# end
# function enter_low_level!(solver::S) where {S<:PC_TAPF_Solver}
#     # solver.num_CBS_iterations += 1
#     check_time(solver)
# end
# function step_low_level!(solver::S) where {S<:PC_TAPF_Solver}
#     # check_time(solver)
# end
# function step_low_level!(solver::S,schedule_node::N,t0,tF,plan_path,args...) where {S<:PC_TAPF_Solver,N<:AbstractPlanningPredicate}
#     step_low_level!(solver)
#     log_info(1,solver.l3_verbosity,"LOW_LEVEL_SEARCH: plan path = ",plan_path," -- node ", string(schedule_node), " t0 = ", t0, ", tF = ",tF)
# end
# function exit_low_level!(solver::S) where {S<:PC_TAPF_Solver}
#     check_time(solver)
# end
# function enter_a_star!(solver::S,args...) where {S<:PC_TAPF_Solver}
#     check_time(solver)
# end
# function enter_a_star!(solver::S,schedule_node::N,t0,tF,args...) where {S<:PC_TAPF_Solver,N<:AbstractPlanningPredicate}
#     enter_a_star!(solver)
#     log_info(0,solver.l4_verbosity,"A*: planning for node ",string(schedule_node), " t0 = ", t0, ", tF = ",tF)
# end
# function exit_a_star!(solver::S,args...) where {S<:PC_TAPF_Solver}
#     solver.max_A_star_iterations = max(solver.max_A_star_iterations,solver.num_A_star_iterations)
#     solver.total_A_star_iterations += solver.num_A_star_iterations
#     solver.num_A_star_iterations = 0
#     check_time(solver)
# end
# function CRCBS.logger_step_a_star!(solver::PC_TAPF_Solver, env::MetaAgentCBS.LowLevelEnv, base_path, s, q_cost)
#     solver.num_A_star_iterations += 1
#     if solver.num_A_star_iterations > solver.LIMIT_A_star_iterations
#         # throw(SolverAstarMaxOutException(string("# MAX OUT: A* limit of ",solver.LIMIT_A_star_iterations," exceeded.")))
#     end
#     log_info(2,solver.l4_verbosity,"A* iter $(solver.num_A_star_iterations): s = $(string(s)), q_cost = $q_cost")
# end
# function CRCBS.logger_step_a_star!(solver::PC_TAPF_Solver, env, base_path, s, q_cost)
#     solver.num_A_star_iterations += 1
#     if solver.DEBUG
#         push!(solver.astar_model.search_history, s)
#     end
#     if solver.num_A_star_iterations > solver.LIMIT_A_star_iterations
#         if solver.DEBUG
#             # Dump env to JLD2 environment
#             filename = joinpath(DEBUG_PATH,string("A_star_dump_",get_debug_file_id(),".jld2"))
#             mkpath(DEBUG_PATH)
#             log_info(-1,solver.l4_verbosity,"Dumping A* env to $filename")
#             agent_id = env.agent_idx
#             history = map(s->(s.vtx,s.t),solver.astar_model.search_history)
#             start   = (get_final_state(base_path).vtx,get_final_state(base_path).t)
#             goal    = (env.goal.vtx,env.goal.t)
#             idx = isa(solver.astar_model,AStarPathFinderModel) ? 2 : 1
#             paths   = env.cost_model.cost_models[idx].model.table.paths
#             state_constraints = map(c->(c.a,(c.v.s.vtx,c.v.sp.vtx),c.t),collect(env.constraints.state_constraints))
#             action_constraints = map(c->(c.a,(c.v.s.vtx,c.v.sp.vtx),c.t),collect(env.constraints.action_constraints))
#             @save filename agent_id history start goal paths state_constraints action_constraints
#             # @assert false "making a bogus assertion to hack my way out of this block"
#         end
#         # throw(SolverAstarMaxOutException(string("# MAX OUT: A* limit of ",solver.LIMIT_A_star_iterations," exceeded.")))
#     end
#     log_info(2,solver.l4_verbosity,"A* iter $(solver.num_A_star_iterations): s = $(string(s)), q_cost = $q_cost")
# end
# function CRCBS.logger_enter_a_star!(solver::PC_TAPF_Solver)
#     log_info(1,solver.l4_verbosity,"A*: entering...")
#     @assert(solver.num_A_star_iterations == 0, string("A*: ERROR: iterations = ", solver.num_A_star_iterations, " at entry"))
#     # if solver.num_A_star_iterations > 0
#     #     log_info(-1,solver.l4_verbosity,"A*: ERROR: iterations = ", solver.num_A_star_iterations, " at entry")
#     # end
# end
# function CRCBS.logger_enqueue_a_star!(solver::PC_TAPF_Solver,env,s,a,sp,h_cost)
#     log_info(2,solver.l4_verbosity,"A* exploring $(string(s)) -- $(string(sp)), h_cost = $h_cost")
# end
# function CRCBS.logger_exit_a_star!(solver::PC_TAPF_Solver, path, cost, status)
#     empty!(solver.astar_model.search_history)
#     if status == false
#         log_info(-1,solver.l4_verbosity,"A*: failed to find feasible path. Returning path of cost ",cost)
#     else
#         log_info(0,solver.l4_verbosity,"A*: returning optimal path with cost ",cost)
#     end
# end
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

function CRCBS.check_termination_criteria(solver::S,env::E,cost_so_far,s) where {S<:PC_TAPF_Solver,E<:AbstractLowLevelEnv}
    # solver.num_A_star_iterations += 1
    if solver.num_A_star_iterations > solver.LIMIT_A_star_iterations
        # throw(SolverAstarMaxOutException(string("# MAX OUT: A* limit of ",solver.LIMIT_A_star_iterations," exceeded.")))
        return true
    end
    return false
end
