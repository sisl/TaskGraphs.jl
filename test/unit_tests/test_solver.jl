# test solver initialization
let
    DefaultAStarSC()
    AStarSC{Int}()
    ISPS()
    CBSSolver()
    TaskGraphsMILPSolver()
    NBSSolver()
    solver = NBSSolver()
    best_cost(solver)
    NBSSolver(TaskGraphsMILPSolver(),CBSSolver())
end
let
    solver = NBSSolver()
    increment_iteration_count!(solver)
    increment_iteration_count!(route_planner(solver))
    @test max_iterations(solver) == 1
    @test iterations(solver) == 1
    reset_solver!(solver)
    @test max_iterations(solver) == 1
    @test iterations(solver) == 0
    hard_reset_solver!(solver)
    @test max_iterations(solver) == 0
    @test max_iterations(route_planner(solver)) == 0
    @test iterations(solver) == 0
end
# in-depth testing of solver stages on a single problem
let
    # init search env
    f = pctapf_problem_4
    cost_model = SumOfMakeSpans()
    solver = NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment()))
    pc_tapf = f(solver;cost_function=cost_model,verbose=false)
    base_env = pc_tapf.env
    prob = formulate_assignment_problem(solver.assignment_model,
        pc_tapf;
        optimizer=Gurobi.Optimizer,
    )
    sched, cost = solve_assignment_problem!(solver.assignment_model,prob,pc_tapf)
    @test validate(sched)
    # Test AStarSC
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_env)
        pc_mapf = PC_MAPF(search_env)
        path_planner = DefaultAStarSC()
        node = initialize_root_node(search_env)
        # plan for Robot Start
        node_id = RobotID(1)
        schedule_node = get_node_from_id(search_env.schedule, node_id)
        v = get_vtx(search_env.schedule, node_id)
        @test plan_path!(path_planner, pc_mapf,search_env, node, schedule_node, v)
        # plan for GO
        v2 = outneighbors(search_env.schedule,v)[1]
        schedule_node = get_node_from_vtx(search_env.schedule, v2)
        @test plan_path!(path_planner, pc_mapf,search_env, node, schedule_node, v2)
        # @show convert_to_vertex_lists(node.solution)
    end
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_env)
        pc_mapf = PC_MAPF(search_env)
        node = initialize_root_node(search_env)
        path_planner = ISPS()
        for i in 1:nv(search_env.schedule)
            @test plan_next_path!(path_planner,pc_mapf,node.solution,node)
        end
    end
    # Test ISPS
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_env)
        path_planner = ISPS()
        node = initialize_root_node(search_env)
        @test compute_route_plan!(path_planner,PC_MAPF(search_env),node)#,node)
        # @show convert_to_vertex_lists(node.solution)
        # @show convert_to_vertex_lists(search_env.route_plan)
    end
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_env)
        path_planner = ISPS()
        solution, cost = solve!(path_planner,PC_MAPF(search_env))
        # @show convert_to_vertex_lists(solution.route_plan)
    end
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_env)
        path_planner = ISPS()
        node = initialize_root_node(search_env)
        @test low_level_search!(path_planner,PC_MAPF(search_env),node)
        # @show convert_to_vertex_lists(node.solution)
    end
    # Test CBS
    let
        search_env = construct_search_env(solver,deepcopy(sched),base_env)
        pc_mapf = PC_MAPF(search_env)
        path_planner = CBSSolver(ISPS())
        solution, cost = solve!(path_planner,pc_mapf)
        # solution, cost = CRCBS.cbs!(path_planner,pc_mapf)
        # @show convert_to_vertex_lists(solution.route_plan)
    end
    let
        # NOTE Freezing here
        path_planner = CBSSolver(ISPS())
        env, cost = plan_route!(path_planner,deepcopy(sched),pc_tapf)
        # @show convert_to_vertex_lists(env.route_plan)
    end
    # test NBS
    let
        solver = NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment()))
        # solver = NBSSolver(assignment_model = TaskGraphsMILPSolver())
        set_iteration_limit!(solver,3)
        env, cost = solve!(solver,base_env;optimizer=Gurobi.Optimizer)
        # @show convert_to_vertex_lists(env.route_plan)
        # @show get_logger(solver)
        # @show optimality_gap(solver) > 0
        @test validate(env.schedule)
        @test validate(env.schedule,convert_to_vertex_lists(env.route_plan), env.cache.t0, env.cache.tF)
    end
end
# test task assignment
let
    # init search env
    for (i, f) in enumerate(pctapf_test_problems())
        for cost_model in [SumOfMakeSpans(), MakeSpan()]
            let
                costs = Float64[]
                for solver in [
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(AssignmentMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment())),
                        ]
                    # @show i, f, solver
                    pc_tapf = f(solver;cost_function=cost_model,verbose=false);
                    search_env = pc_tapf.env
                    prob = formulate_assignment_problem(solver.assignment_model,
                        pc_tapf;
                        optimizer=Gurobi.Optimizer,
                    )
                    sched, cost = solve_assignment_problem!(solver.assignment_model,prob,pc_tapf)
                    if !isa(solver.assignment_model.milp,GreedyAssignment)
                        push!(costs, cost)
                    end
                    @test validate(sched)
                    @test cost != typemax(Int)
                    @test cost != Inf
                end
                # @show costs
                @test all(costs .== costs[1])
            end
        end
    end
end
# test full solver
let
    # init search env
    for (i, f) in enumerate([
        pctapf_problem_1,
        pctapf_problem_2,
        pctapf_problem_3,
        pctapf_problem_4,
        pctapf_problem_5,
        pctapf_problem_6,
        pctapf_problem_7,
        pctapf_problem_8,
        ])
        for cost_model in [SumOfMakeSpans(), MakeSpan()]
            let
                costs = Float64[]
                for solver in [
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(AssignmentMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(GreedyAssignment())),
                        NBSSolver(
                            assignment_model = TaskGraphsMILPSolver(GreedyAssignment()),
                            path_planner = PIBTPlanner{NTuple{3,Float64}}()
                            ),
                        ]
                    # @show i, f, solver
                    set_iteration_limit!(solver,1)
                    set_iteration_limit!(route_planner(solver),100)
                    pc_tapf = f(solver;cost_function=cost_model,verbose=false);
                    env, cost = solve!(solver,pc_tapf;optimizer=Gurobi.Optimizer)
                    # @show convert_to_vertex_lists(env.route_plan)
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
                # @show costs
                @test all(costs .== costs[1])
            end
        end
    end
end
# Test that optimal collision-free planners obtain the correct costs
let
    cost_model = MakeSpan()
    for (i, (f,expected_cost)) in enumerate([
        (pctapf_problem_1,5),
        (pctapf_problem_2,8),
        (pctapf_problem_3,10),
        (pctapf_problem_4,3),
        (pctapf_problem_5,4),
        ])
        for solver in [
                NBSSolver(assignment_model = TaskGraphsMILPSolver(AssignmentMILP())),
                NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                ]
            let
                pc_tapf = f(solver;cost_function=cost_model,verbose=false);
                env, cost = solve!(solver,pc_tapf;optimizer=Gurobi.Optimizer)
                @test expected_cost == cost
            end
        end
    end
    cost_model = SumOfMakeSpans()
    for (i, (f,expected_cost)) in enumerate([
        (pctapf_problem_1,5),
        (pctapf_problem_2,8),
        (pctapf_problem_3,10),
        (pctapf_problem_4,3),
        (pctapf_problem_5,4),
        (pctapf_problem_8,16),
        ])
        for solver in [
                NBSSolver(assignment_model = TaskGraphsMILPSolver(AssignmentMILP())),
                NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                ]
            let
                pc_tapf = f(solver;cost_function=cost_model,verbose=false);
                env, cost = solve!(solver,pc_tapf;optimizer=Gurobi.Optimizer)
                @test expected_cost == cost
            end
        end
    end
end
# verify that ISPS+A*sc avoids a collision by exploiting slack
let
    solver = NBSSolver()
    cost_model = MakeSpan()
    pc_tapf = pctapf_problem_3(solver;cost_function=cost_model,verbose=false);
    base_env = pc_tapf.env

    prob = formulate_assignment_problem(assignment_solver(solver),pc_tapf)
    project_schedule, cost = solve_assignment_problem!(assignment_solver(solver),prob,pc_tapf)
    @test termination_status(prob) == MOI.OPTIMAL

    env = construct_search_env(solver,project_schedule,base_env)
    pc_mapf = PC_MAPF(env)
    node = initialize_root_node(solver,pc_mapf)
    low_level_search!(route_planner(solver),pc_mapf,node)

    path1 = convert_to_vertex_lists(node.solution.route_plan.paths[1])
    path2 = convert_to_vertex_lists(node.solution.route_plan.paths[2])
    @test path2[2] == 5 # test that robot 2 will indeed wait for robot 1
    @test path1 == [2, 6, 10, 14, 18, 22, 26, 30, 31, 32, 32]
    @test path2 == [5, 5, 6,  7,  8,  12, 12, 12, 12, 12, 16]
    # @show path1
    # @show path2
end
# Non-zero process time
let
    """
    Verify that the algorithm correctly handles scenarios where process time
    is non-zero
    """
    println("Test non-zero wait time:")
    for Δt in [0,1,4]
        let
            solver = NBSSolver()
            cost_model=MakeSpan()
            pc_tapf = pctapf_problem_6(solver;
                cost_function=cost_model,
                verbose=false,
                Δt_op=Δt
                );
            env, cost = solve!(solver,pc_tapf;optimizer=Gurobi.Optimizer)
            paths = convert_to_vertex_lists(env.route_plan)
            # @show paths[1]
            # @show paths[2]
            @test paths[1][8+Δt] == 13
            @test paths[1][9+Δt] == 17
        end
    end
end
# Non-zero collection time
let
    """
    Verify that the algorithm correctly handles scenarios where collection time
    and deposit time are non-zero.
    """
    println("Test non-zero collection time:")
    Δt_deliver_1 = 1
    for (Δt_collect_2, true_cost) in zip([0,1,2,3,4],[6,7,8,8,8])
        solver = NBSSolver()
        cost_model = MakeSpan()
        pc_tapf = pctapf_problem_7(solver;
            cost_function=cost_model,
            verbose=false,
            Δt_op=0,
            Δt_collect=[0,Δt_collect_2,0],
            Δt_deliver=[Δt_deliver_1,0,0]
            );
        env, cost = solve!(solver,pc_tapf;optimizer=Gurobi.Optimizer)
        paths = convert_to_vertex_lists(env.route_plan)
        @test cost == true_cost
    end
end
# Collaborative transport problems
let
    cost_model = MakeSpan()
    solver = NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP()))
    set_runtime_limit!(solver,Inf)
    f = pctapf_problem_11
    pc_tapf = f(solver;cost_function=cost_model);
    c_pc_tapf = C_PC_TAPF(pc_tapf.env)
    for (i, (f,expected_cost,expected_paths)) in enumerate([
        (pctapf_problem_11, 6, [
            [1,2,6,10,14,13,9],
            [4,3,7,11,15],
            [15,11,10,9,5,1],
        ]),
        ])
        for solver in [
                NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP())),
                ]
            let
                pc_tapf = f(solver;cost_function=cost_model);
                c_pc_tapf = C_PC_TAPF(pc_tapf.env)
                # set_verbosity!(solver,4)
                env, cost = solve!(solver,c_pc_tapf;optimizer=Gurobi.Optimizer)
                paths = convert_to_vertex_lists(env.route_plan)
                @show paths
                @test cost == expected_cost
                for (expected_path, path) in zip(expected_paths,paths)
                    @test length(path) <= maximum(map(length,expected_paths))
                    @test path[1:length(expected_path)] == expected_path
                end
            end
        end
    end
end
# let
#     cost_model = MakeSpan()
#     solver = NBSSolver(assignment_model = TaskGraphsMILPSolver(SparseAdjacencyMILP()))
#     set_verbosity!(solver,4)
#     set_runtime_limit!(solver,Inf)
#     f = pctapf_problem_11
#     pc_tapf = f(solver;cost_function=cost_model);
#     c_pc_tapf = C_PC_TAPF(pc_tapf.env)
#     prob = formulate_assignment_problem(solver.assignment_model,
#         c_pc_tapf;
#         optimizer=Gurobi.Optimizer,
#     )
#     sched, cost = solve_assignment_problem!(solver.assignment_model,prob,pc_tapf)
#     @test validate(sched)
#     prob = c_pc_tapf
#     env = construct_search_env(solver, sched, prob.env)
#     pc_mapf = construct_routing_problem(prob,env)
#     node = initialize_root_node(solver,pc_mapf)
#     path_planner = low_level(route_planner(solver))
#
#     # valid_flag = compute_route_plan!(path_planner,pc_mapf,node)
#     while length(env.cache.node_queue) > 0
#         if !(plan_next_path!(path_planner,pc_mapf,env,node))
#             @show false
#         end
#         enforce_time_limit(path_planner)
#     end
#
#     plan_next_path!(path_planner,pc_mapf,env,node)
#
#     # solution, cost = solve!(solver, pc_mapf)
#     # env, cost = plan_route!(route_planner(solver), sched, c_pc_tapf)
# end
# let
#     mapf = init_mapf_1()
#     node = initialize_root_node(mapf)
#     i = 1
#     env = build_env(mapf,node,i)
#     s = CBSEnv.State(1,0)
#     a = CRCBS.wait(env,s)
#     sp = get_next_state(env,s,a)
#     n = PathNode(s,a,sp)
#
#     path = Path(
#         path_nodes = [n],
#         s0=s,
#         cost=1.0
#     )
#
#     meta_path = MetaAgentCBS.construct_meta_path([path,path],1.0)
#
#     meta_env = MetaAgentCBS.construct_meta_env([env],[1])
#     # s = get_start(mapf,meta_env,i)
#     s = MetaAgentCBS.State([CBSEnv.State(1,4)])
#     a = CRCBS.wait(meta_env,s)
#     sp = get_next_state(meta_env,s,a)
#     n = PathNode(s,a,sp)
#     path = Path(
#         path_nodes=[n],
#         s0=s,
#         cost=MetaCost{Float64}([0.0],0.0)
#         )
#
#     get_start_index(path), get_end_index(path), get_index_from_time(path,get_end_index(path))
#
#     length(path)
#     extend_path!(path,6)
#     length(path)
#
# end
