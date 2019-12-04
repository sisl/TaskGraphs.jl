module TaskGraphsSolvers

using CRCBS

export
    PC_TAPF_Solver,
    reset_solver!,
    enter_assignment!,
    exit_assignment!,
    enter_cbs!,
    exit_cbs!,
    enter_low_level!,
    exit_low_level!,
    enter_a_star!,
    exit_a_star!,
    read_solver,
    read_solver,
    log_info

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
        l4_verbosity = toml_dict["l4_verbosity"]
    )
end
function read_solver(io)
    read_solver(TOML.parsefile(io))
end

# Helpers for printing
function log_info(limit::Int,verbosity::Int,msg::String)
    if verbosity > limit
        println(msg)
    end
end
function log_info(limit::Int,solver::S,msg::String) where {S<:PC_TAPF_Solver}
    log_info(limit,solver.verbosity,msg)
end

end
