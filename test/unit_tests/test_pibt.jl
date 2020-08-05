let
    cost_model = SumOfMakeSpans()
    for (i, f) in enumerate(pctapf_test_problems())
        solver = NBSSolver(
            assignment_model = TaskGraphsMILPSolver(GreedyAssignment()),
            path_planner = PIBTPlanner{NTuple{3,Float64}}()
            )
        let
            pc_tapf = f(solver;cost_function=cost_model,verbose=false)
            base_search_env = pc_tapf.env
            prob = formulate_assignment_problem(solver.assignment_model,base_search_env;
                optimizer=Gurobi.Optimizer,
            )
            sched, _ = solve_assignment_problem!(solver.assignment_model,prob,base_search_env)

            pibt_planner = route_planner(solver)
            search_env = construct_search_env(solver,deepcopy(sched),base_search_env)
            pc_mapf = PC_MAPF(search_env)
            set_iteration_limit!(pibt_planner,50)
            set_verbosity!(pibt_planner,0)
            solution, valid_flag = pibt!(pibt_planner,deepcopy(pc_mapf))
            # @show i, f, get_cost(solution)
            # @show convert_to_vertex_lists(solution.route_plan)
            @test valid_flag

            reset_solver!(solver)
            solution, cost = solve!(pibt_planner,pc_mapf)
            @test cost != typemax(cost_type(pc_mapf))
        end
    end
end
