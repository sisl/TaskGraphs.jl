let

    println("WARMING UP")

    dummy_problem_dir = "dummy_problem_dir"
    dummy_results_dir = "dummy_results_dir"
    modes = [
        :write,
        :assignment_only,
        :low_level_search_without_repair,
        :low_level_search_with_repair,
        :full_solver
        ]
    for mode in modes
        run_profiling(mode;
            num_tasks=[10],
            num_robots=[10],
            depth_biases=[0.1],
            num_trials=1,
            problem_dir = dummy_problem_dir,
            results_dir = dummy_results_dir,
            solver_mode=:adjacency # :assignment
            )
    end
    run(pipeline(`rm -rf $dummy_problem_dir`, stdout=devnull, stderr=devnull))
    run(pipeline(`rm -rf $dummy_results_dir`, stdout=devnull, stderr=devnull))
end
let

    println("RUNNING PROFILING TESTS")

    modes = [
        # :write,
        # :assignment_only,
        # :low_level_search_without_repair,
        # :low_level_search_with_repair,
        :full_solver
        ]
    for mode in modes
        run_profiling(mode;
            num_tasks = [10,20,30,40,50,60],
            num_robots = [10,20,30,40],
            depth_biases = [0.1,0.4,0.7,1.0],
            max_parent_settings = [3],
            num_trials = 4,
            env_id = 2,
            initial_problem_id = 1,
            problem_dir = PROBLEM_DIR,
            results_dir = joinpath(EXPERIMENT_DIR,"adjacency_solver/results"),
            TimeLimit=100,
            OutputFlag=0,
            solver_mode=:adjacency
            )
    end
end
let

    env_id = 2
    env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
    factory_env = read_env(env_filename)
    env_graph = factory_env.graph
    dist_matrix = get_dist_matrix(env_graph)

    # problem_def = read_problem_def(joinpath(PROBLEM_DIR,"problem223.toml"))
    problem_def = read_problem_def(joinpath(PROBLEM_DIR,"problem254.toml"))
    project_spec, r0, s0, sF = problem_def.project_spec,problem_def.r0,problem_def.s0,problem_def.sF
    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(
            project_spec, r0, s0, sF, dist_matrix);
    # Solve the problem
    solver = PC_TAPF_Solver(DEBUG=true,verbosity=0,LIMIT_A_star_iterations=5*nv(env_graph));
    (solution, assignment, cost, search_env), elapsed_time, byte_ct, gc_time, mem_ct = @timed high_level_search_mod!(
        solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer);
    # TODO: the problem is that one of the COLLECT nodes has a -1 for the collection vertex
    # project_schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
    # @test validate(project_schedule)
    # model = formulate_schedule_milp(project_schedule,problem_spec;Presolve=1)
    # # optimize!(model)
    # retval, elapsed_time, byte_ct, gc_time, mem_ct = @timed optimize!(model)
    # @show elapsed_time
    # @test termination_status(model) == MathOptInterface.OPTIMAL
    # assignment_matrix = get_assignment_matrix(model);
    # ############## Route Planning ###############
    # update_project_schedule!(project_schedule,problem_spec,assignment_matrix)
    # env, mapf = construct_search_env(project_schedule, problem_spec, env_graph);
    # pc_mapf = PC_MAPF(env,mapf);
    ##### Call CBS Search Routine (LEVEL 2) #####
    # solution, cost = solve!(solver,pc_mapf);
    # root_node = initialize_root_node(pc_mapf)
    # vtx_lists = convert_to_vertex_lists(root_node.solution)
    # for (i,p) in enumerate(vtx_lists)
    #     @show p
    # end

    # valid_flag = low_level_search!(solver,pc_mapf,root_node)
    #
    # v = 130
    # path_id = get_path_spec(project_schedule, v).path_id
    # agent_id = get_path_spec(project_schedule, v).agent_id
    # cbs_env, base_path = build_env(solver, pc_mapf.env, pc_mapf.mapf, root_node, agent_id, path_id)
    # extend_path!(cbs_env, base_path, 216)
    # extend_path!(cbs_env, base_path, 217)
    # extend_path!(cbs_env, base_path, 218)
    # schedule_node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))


    # for i in 1:nv(project_schedule.graph)
    #     @show i
    #     plan_next_path!(solver,pc_mapf.env,pc_mapf.mapf,root_node;heuristic=get_heuristic_cost)
    # end
    # plan_next_path!(solver,pc_mapf.env,pc_mapf.mapf,root_node;heuristic=get_heuristic_cost)


    # solution, assignment, cost, search_env = high_level_search_mod!(
    #     solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer;);

end
