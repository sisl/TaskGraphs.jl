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
