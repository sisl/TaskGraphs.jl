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
    NBSSolver(
        assignment_model=TaskGraphsMILPSolver(AssignmentMILP()),
        route_planner=
    )
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
                        NBSSolver(assignment_model=AssignmentMILP()),
                        NBSSolver(assignment_model=AdjacencyMILP()),
                        NBSSolver(assignment_model=SparseAdjacencyMILP()),
                        NBSSolver(assignment_model=GreedyAssignment(),iteration_limit=2),
                        ]
                    # @show i, f, solver.nbs_model
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
                    # solve
                    solution, cost = solve!(solver,search_env;
                        optimizer=Gurobi.Optimizer,
                        )

                    if !isa(solver.assignment_model,GreedyAssignment)
                        push!(costs, cost[1])
                    end
                    @test validate(solution.schedule)
                    @test validate(solution.schedule, convert_to_vertex_lists(solution.route_plan), solution.cache.t0, solution.cache.tF)
                    @test cost[1] != Inf
                end
                @test all(costs .== costs[1])
            end
        end
    end
end
