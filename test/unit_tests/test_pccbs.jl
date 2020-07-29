let
    f = pctapf_problem_1
    cost_model = SumOfMakeSpans()
    project_spec, problem_spec, robot_ICs, _, env_graph = f(;cost_function=cost_model,verbose=false)
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
        env_graph
        )
    prob = formulate_assignment_problem(solver.assignment_model,base_search_env;
        optimizer=Gurobi.Optimizer,
    )
    sched, cost = solve_assignment_problem!(solver.assignment_model,prob,base_search_env)
    @test validate(sched)
    search_env = construct_search_env(solver,deepcopy(sched),base_search_env)
    node = initialize_root_node(search_env)
    let
        env = build_env(solver,search_env,node,AgentID(1))

        s = PCCBS.State()
        a = PCCBS.Action()
        get_next_state(s,a)
        get_next_state(env,s,a)

        @test PCCBS.State(1,2) == PCCBS.State(1,2)
        @test PCCBS.State() == PCCBS.State()
        @test states_match(PCCBS.State(),PCCBS.State())
        @test states_match(env,PCCBS.State(),PCCBS.State())
        CRCBS.wait(s)
        CRCBS.wait(env,s)

        get_cost_model(env)
        get_heuristic_model(env)
        @test CRCBS.is_valid(env,PCCBS.State()) == false
        @test CRCBS.is_valid(env,PCCBS.Action()) == false
        @test CRCBS.is_valid(env,PCCBS.State(1,2))
        @test CRCBS.is_valid(env,PCCBS.Action(Edge(1,2),1))

        get_heuristic_cost(env,PCCBS.State(1,2))
        # @test is_goal(env,PCCBS.State(1,0)) == false
        # @test is_goal(env,PCCBS.State(2,3)) == false
        # @test is_goal(env,PCCBS.State(1,3))
        @test length(get_possible_actions(env,PCCBS.State(-1,-1))) == 0
    end
    let
        for v in vertices(search_env.schedule)
            schedule_node = get_node_from_vtx(search_env.schedule,v)
            if isa(schedule_node,COLLECT)
                env = build_env(solver,search_env,node,VtxID(v))
                @test length(get_possible_actions(env,PCCBS.State(1,0))) == 1
                @test length(get_possible_actions(env,PCCBS.State(-1,-1))) == 0
                break
            end
        end
    end
end
