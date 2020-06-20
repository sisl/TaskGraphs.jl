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

"""
    SolverLogger

A logger type for keeping track of thing like runtime, iterations, optimality
gap (including upper and lower bound), etc.
"""
mutable struct SolverLogger{S,C}
    iterations      ::Int
    iteration_limit ::Int
    start_time      ::Float64
    runtime_limit   ::Float64
    lower_bound     ::C
    best_cost       ::C
end

"""
    CBSPlanner

A path planner that employs Conflict-Based Search
"""
struct CBSPlanner{L}
    low_level_planner::L
end

"""
    ISPSPlanner

A path planner that employs Incremental Slack-Prioritized Search.
"""
struct ISPSPlanner{L}
    low_level_planner::L

end

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
@with_kw struct NBSSolver{A,P} <: AbstractPCTAPFSolver
    assigment_model ::A     = SparseAdjacencyMILP()
    path_planner    ::P     = CBSPlanner{ISPS{AStarSC}}()

    # These fields should be reusable across solver types, including at each
    # level of a nested/hierarchical solver
    iteration_limit ::Int = 50
    verbosity       ::Int = 0
    DEBUG           ::Bool = false
    # LOGGER:
    # num_iterations   ::Int = 0
    # best_cost         ::CostTracker{C}
    # total_iterations  ::Int = 0
    # start_time        ::Float64 = time()
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

"""
    solve!(solver, base_search_env::SearchEnv;kwargs...) where {A,P}

Use the planner defined by `solver` to solve the PC-TAPF problem encoded by
`base_search_env`.

Arguments:
- solver <: AbstractPCTAPFSolver
- base_search_env::SearchEnv : a PC-TAPF problem

Outputs:
- best_env : a `SearchEnv` data structure that encodes a solution to the problem
- cost : the cost of the solution encoded by `best_env`
"""
function solve!(solver::NBSSolver{A,P}, base_search_env::SearchEnv;kwargs...) where {A,P}
    best_env = SearchEnv()
    assignment_problem = formulate_assignment_problem(solver, base_search_env;
        kwargs...)
    while optimality_gap(solver) > 0
        try
            update_assignment_problem!(solver, assignment_problem, base_search_env)
            schedule = solve_assignment_problem!(solver, assignment_problem, base_search_env;kwargs...)
            if optimality_gap(solver) > 0
                env = plan_route!(solver, schedule, base_search_env;kwargs...);
                if get_cost(env) < get_cost(best_env)
                    best_env = env
                end
            end
        catch e
            if isa(e, SolverException)
                log_info(-1, solver.verbosity, e.msg)
                break
            else
                throw(e)
            end
        end
    end
    cost = get_cost(best_env)
    return best_env, cost
end

"""
    solve_assignment_problem!(solver,base_search_env;kwargs...)

Solve the "assignment problem"--i.e., the relaxation of the full PC-TAPF problem
wherein we ignore collisions--using the algorithm encoded by solver.
"""
function solve_assignment_problem!(solver::S, model, base_search_env;
        t0_ = Dict{AbstractID,Int}(get_vtx_id(base_search_env.schedule, v)=>t0 for (v,t0) in enumerate(base_search_env.cache.t0)),
        tF_ = Dict{AbstractID,Int}(get_vtx_id(base_search_env.schedule, v)=>tF for (v,tF) in enumerate(base_search_env.cache.tF)),
        TimeLimit=solver.nbs_time_limit,
        buffer=5.0, # to give some extra time to the path planner if the milp terminates late.
        kwargs...) where {S<:TaskGraphsMILP}

    enter_assignment!(solver)
    optimize!(model)
    optimal = (termination_status(model) == MathOptInterface.OPTIMAL);
    feasible = (primal_status(model) == MOI.FEASIBLE_POINT) # TODO use this!
    if !feasible
        throw(SolverException("Assignment problem is infeasible")))
    end
    if optimal
        lower_bound = max(lower_bound, Int(round(value(objective_function(model)))) )
    else
        lower_bound = max(lower_bound, Int(round(value(objective_bound(model)))) )
    end
    schedule = deepcopy(base_search_env.schedule)
    update_project_schedule!(model, schedule, base_search_env.problem_spec)
end

"""
    formulate_assignment_problem(solver,base_search_env::SearchEnv;

Returns an assignment problem instance that can be updated (as opposed to being
reconstructed from scratch) on each call to `update_assignment_problem!` prior
to being resolved.
"""
function formulate_assignment_problem(solver,base_search_env::SearchEnv;
        cost_model=base_search_env.problem_spec.cost_function,
        optimizer=get_optimizer(solver),
        kwargs...)

    project_schedule    = base_search_env.schedule
    problem_spec        = base_search_env.problem_spec
    formulate_milp(solver,project_schedule,problem_spec;
        cost_model=cost_model,
        optimizer=optimizer,
        kwargs...)
end

"""
    update_assignment_problem!(solver, assignment_problem)

A helper method for updating an instance of an assignment problem. In the case
    of MILP-based models, this method simply excludes all previous solutions by
    adding new constraints on the assignment/adjacency matrix.
"""
function update_assignment_problem!(solver, model::TaskGraphsMILP, base_search_env)
    exclude_solutions!(model) # exclude most recent solution in order to get next best solution
end


end
