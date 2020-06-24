
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
    @test (1,0,0,0,0) < 2
    @test (1,0,0,0,0) <= 1
    @test (1,0,0,0,0) >= 1
    @test (1,0,0,0,0) > 0

    @test 2 > (1,0,0,0,0)
    @test 1 >= (1,0,0,0,0)
    @test 1 <= (1,0,0,0,0)
    @test 0 < (1,0,0,0,0)
end
let
    AStarSC()
    AStarSC{Int}()
    ISPS()
    CBSRoutePlanner()
    TaskGraphsMILPSolver()
    NBSSolver()
    solver = NBSSolver()
    TaskGraphs.Solvers.best_cost(solver)

    NBSSolver(TaskGraphsMILPSolver(),CBSRoutePlanner())
    # NBSSolver(
    #     TaskGraphsMILPSolver(),
    #     CBSRoutePlanner(),
    #     iteration_limit=2
    #     )

end

let
    # init search env
    f = initialize_toy_problem_4
    cost_model = SumOfMakeSpans()
    project_spec, problem_spec, robot_ICs, assignments, env_graph = f(;verbose=false)
    solver = NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment()))
    project_schedule = construct_partial_project_schedule(
        project_spec,
        problem_spec,
        robot_ICs,
        )
    base_search_env = construct_search_env(
        solver,
        project_schedule,
        problem_spec,
        env_graph;
        primary_objective=cost_model,
        )
    prob = formulate_assignment_problem(solver.assignment_model,base_search_env;
        optimizer=Gurobi.Optimizer,
    )
    sched, cost = solve_assignment_problem!(solver.assignment_model,prob,base_search_env)
    @test validate(sched)
    # Test AStarSC
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_search_env)
        path_planner = AStarSC()
        node = initialize_root_node(search_env)
        # plan for Robot Start
        node_id = RobotID(1)
        schedule_node = get_node_from_id(search_env.schedule, node_id)
        println(string(schedule_node))
        v = get_vtx(search_env.schedule, node_id)
        @test plan_path!(path_planner, search_env, node, schedule_node, v)
        # plan for GO
        v2 = outneighbors(search_env.schedule,v)[1]
        schedule_node = get_node_from_vtx(search_env.schedule, v2)
        println(string(schedule_node))
        @test plan_path!(path_planner, search_env, node, schedule_node, v2)
        @show convert_to_vertex_lists(node.solution)
    end
    # Test ISPS
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_search_env)
        path_planner = ISPS()
        node = initialize_root_node(search_env)
        low_level_search!(path_planner,search_env,node)
        @show convert_to_vertex_lists(node.solution)
    end
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_search_env)
        path_planner = ISPS()
        node = initialize_root_node(search_env)
        low_level_search!(path_planner,PC_MAPF(search_env),node)
        @show convert_to_vertex_lists(node.solution)
    end
    # Test CBS
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_search_env)
        path_planner = CBSRoutePlanner()
        env = solve!(path_planner,PC_MAPF(search_env))
        @show convert_to_vertex_lists(env.route_plan)
    end
    let
        path_planner = CBSRoutePlanner()
        env = plan_route!(path_planner,deepcopy(sched),base_search_env)
        @show convert_to_vertex_lists(env.route_plan)
    end
    # test NBS
    let
        solver = NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment()))
        TaskGraphs.Solvers.set_iteration_limit!(solver,3)
        env, cost = solve!(solver,base_search_env;optimizer=Gurobi.Optimizer)
        @show convert_to_vertex_lists(env.route_plan)
        # @show TaskGraphs.Solvers.get_logger(solver)
        # @show TaskGraphs.Solvers.optimality_gap(solver) > 0
        @test validate(env.schedule)
        @test validate(env.schedule,convert_to_vertex_lists(env.route_plan), env.cache.t0, env.cache.tF)
    end

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
        for cost_model in [SumOfMakeSpans(), MakeSpan()]
            let
                costs = Float64[]
                project_spec, problem_spec, robot_ICs, assignments, env_graph = f(;verbose=false);
                for solver in [
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(AssignmentMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment())),
                        ]
                    # @show i, f, solver
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
                    prob = formulate_assignment_problem(solver.assignment_model,search_env;
                        optimizer=Gurobi.Optimizer,
                    )
                    sched, cost = solve_assignment_problem!(solver.assignment_model,prob,search_env)
                    # # break down the solution process here:
                    # solution, cost = solve!(solver,search_env;
                    #     optimizer=Gurobi.Optimizer,
                    #     )
                    if !isa(solver.assignment_model.milp,GreedyAssignment)
                        push!(costs, cost)
                    end
                    @test validate(sched)
                    @test cost != typemax(Int)
                    @test cost != Inf
                end
                @show costs
                @test all(costs .== costs[1])
            end
        end
    end
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
        for cost_model in [SumOfMakeSpans(), MakeSpan()]
            let
                costs = Float64[]
                project_spec, problem_spec, robot_ICs, assignments, env_graph = f(;verbose=false);
                for solver in [
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(AssignmentMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment())),
                        ]
                    @show i, f, solver
                    TaskGraphs.Solvers.set_iteration_limit!(solver,1)
                    schedule = construct_partial_project_schedule(
                        project_spec,
                        problem_spec,
                        robot_ICs,
                        )
                    base_search_env = construct_search_env(
                        solver,
                        schedule,
                        problem_spec,
                        env_graph;
                        primary_objective=cost_model,
                        )
                    env, cost = solve!(solver,base_search_env;optimizer=Gurobi.Optimizer)
                    @show convert_to_vertex_lists(env.route_plan)
                    # prob = formulate_assignment_problem(solver.assignment_model,search_env;
                    #     optimizer=Gurobi.Optimizer,
                    # )
                    # sched, cost = solve_assignment_problem!(solver.assignment_model,prob,search_env)
                    # # break down the solution process here:
                    # solution, cost = solve!(solver,search_env;
                    #     optimizer=Gurobi.Optimizer,
                    #     )
                    if !isa(solver.assignment_model.milp,GreedyAssignment)
                        push!(costs, cost[1])
                    end
                    @test validate(env.schedule)
                    @test validate(
                        env.schedule,
                        convert_to_vertex_lists(env.route_plan),
                        env.cache.t0,
                        env.cache.tF
                        )
                    @test cost != typemax(Int)
                    @test cost != Inf
                end
                @show costs
                @test all(costs .== costs[1])
            end
        end
    end
end
