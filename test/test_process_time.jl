# Ensure that process_schedule works correctly on this matter
let

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
            project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_6(;
                verbose=false,
                Δt_op=Δt
                );

            env, mapf = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph)
            pc_mapf = PC_MAPF(env,mapf)
            node = initialize_root_node(pc_mapf)
            solver = PC_TAPF_Solver()

            low_level_search!(solver,env,mapf,node)
            path1 = convert_to_vertex_lists(node.solution.paths[1])
            path2 = convert_to_vertex_lists(node.solution.paths[2])
            @test path1[8+Δt] == 13
            @test path1[9+Δt] == 17
            @show path1
            @show path2
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
        project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_7(;
            verbose=false,
            Δt_op=0,
            Δt_collect=[0,Δt_collect_2,0],
            Δt_deliver=[Δt_deliver_1,0,0]
            );
        env, mapf = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph)
        pc_mapf = PC_MAPF(env,mapf);
        node = initialize_root_node(pc_mapf);
        solver = PC_TAPF_Solver(verbosity=2)

        solution, assignment, cost, search_env = high_level_search!(solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer);
        path1 = convert_to_vertex_lists(solution.paths[1])
        path2 = convert_to_vertex_lists(solution.paths[2])
        @show path1
        @show path2
        @show cost
        @test cost[1] == true_cost
    end
end
