let
    SolverLogger{Int}()
    SolverLogger{Float64}()
    logger = SolverLogger{NTuple{3,Float64}}()
    TaskGraphs.Solvers.iterations(logger)
    TaskGraphs.Solvers.iterations(logger)
    TaskGraphs.Solvers.iteration_limit(logger)
    TaskGraphs.Solvers.start_time(logger)
    TaskGraphs.Solvers.runtime_limit(logger)
    TaskGraphs.Solvers.deadline(logger)
    JuMP.lower_bound(logger)
    TaskGraphs.Solvers.best_cost(logger)
    TaskGraphs.Solvers.verbosity(logger)
    TaskGraphs.Solvers.debug(logger)
    TaskGraphs.Solvers.increment_iteration_count!(logger)
    TaskGraphs.Solvers.set_lower_bound!(logger,(0.0,0.0,0.0))
    TaskGraphs.Solvers.set_best_cost!(logger,(0.0,0.0,0.0))
    TaskGraphs.Solvers.reset_solver!(logger)
end
let
    # init search env
    for (i, f) in enumerate([
        initialize_toy_problem_1,
        initialize_toy_problem_2,
        initialize_toy_problem_3,
        initialize_toy_problem_4,
        initialize_toy_problem_5,
        initialize_toy_problem_6,
        initialize_toy_problem_7,
        initialize_toy_problem_8,
        ])
        for cost_model in [SumOfMakeSpans, MakeSpan]
            let
                costs = Float64[]
                project_spec, problem_spec, robot_ICs, assignments, env_graph = f(;verbose=false);
                for solver in [
                        TaskGraphsMILPSolver(AssignmentMILP()),
                        # TaskGraphsMILPSolver(AdjacencyMILP()),
                        TaskGraphsMILPSolver(SparseAdjacencyMILP()),
                        # GreedyAssignment(),
                        ]
                    @show i, f, solver
                    # construct search env
                    schedule = construct_partial_project_schedule(
                        project_spec,
                        problem_spec,
                        robot_ICs,
                        )
                    search_env = construct_search_env(
                        solver,
                        schedule,
                        problem_spec,
                        env_graph;
                        primary_objective=cost_model,
                        )
                    # solve task assignment problem
                    prob = formulate_assignment_problem(solver,search_env;
                        optimizer=Gurobi.Optimizer,
                    )
                    sched, cost = solve_assignment_problem!(solver,prob,search_env)
                    # solve
                    # solution, cost = solve!(solver,search_env;
                    #     optimizer=Gurobi.Optimizer,
                    #     )
                    if !isa(solver.milp,GreedyAssignment)
                        push!(costs, cost[1])
                    end
                    @test validate(sched)
                    @test cost[1] != Inf
                end
                @test all(costs .== costs[1])
            end
        end
    end
end
