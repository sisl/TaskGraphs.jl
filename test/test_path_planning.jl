# Verify that all solvers can solve all example problems
let

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
                for milp_model in [AssignmentMILP(),AdjacencyMILP(),SparseAdjacencyMILP(),GreedyAssignment()]
                    solver = PC_TAPF_Solver(nbs_model=milp_model,l3_verbosity=0)
                    solution, assignment, cost, env = high_level_search!(
                        solver,
                        env_graph,
                        project_spec,
                        problem_spec,
                        robot_ICs,
                        Gurobi.Optimizer;
                        primary_objective=cost_model,
                        )
                    if !isa(milp_model,GreedyAssignment)
                        push!(costs, cost[1])
                    end
                    @test validate(env.schedule)
                    @test validate(env.schedule, convert_to_vertex_lists(solution), env.cache.t0, env.cache.tF)
                    @test cost[1] != Inf
                end
                @test all(costs .== costs[1])
            end
        end
    end
end

# Test that optimal collision-free planners obtain the correct costs
let

    for (i, (f,cost)) in enumerate([
        (initialize_toy_problem_3,10),
        (initialize_toy_problem_4,3),
        (initialize_toy_problem_5,4),
        ])
        cost_model = MakeSpan
        milp_model = SparseAdjacencyMILP()
        let
            project_spec, problem_spec, robot_ICs, assignments, env_graph = f(;verbose=false);
            # get optimistic task assignment
            solver = PC_TAPF_Solver()
            schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
            model = formulate_milp(milp_model,schedule,problem_spec;cost_model=cost_model)
            optimize!(model)
            @test termination_status(model) == MOI.OPTIMAL
            update_project_schedule!(model,schedule,problem_spec,get_assignment_matrix(model))

            # test path planning given optimistic task assignment
            env = construct_search_env(solver,schedule,problem_spec,env_graph;primary_objective=cost_model);
            pc_mapf = PC_MAPF(env)
            solution, _, _ = solve!(solver,pc_mapf)
            # path1 = convert_to_vertex_lists(node.solution.paths[1])
            # path2 = convert_to_vertex_lists(node.solution.paths[2])
            # @show path1
            # @show path2

            @test get_primary_cost(solver,get_cost(solution)) == cost
        end
    end
end

# verify that ISPS+A*sc avoids a collision by exploiting slack
let
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_3(;verbose=false);

    solver = PC_TAPF_Solver()
    schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
    milp_model = SparseAdjacencyMILP()
    cost_model=MakeSpan
    model = formulate_milp(milp_model,schedule,problem_spec;cost_model=cost_model)
    optimize!(model)
    @test termination_status(model) == MOI.OPTIMAL
    cost = Int(round(value(objective_function(model))))
    update_project_schedule!(milp_model,schedule,problem_spec,get_assignment_matrix(model))
    env = construct_search_env(solver,schedule,problem_spec,env_graph;primary_objective=cost_model);
    pc_mapf = PC_MAPF(env)
    node = initialize_root_node(pc_mapf)
    low_level_search!(solver,env,node);

    path1 = convert_to_vertex_lists(node.solution.paths[1])
    path2 = convert_to_vertex_lists(node.solution.paths[2])
    @test path2[2] == 5 # test that robot 2 will indeed wait for robot 1
    @test path1 == [2, 6, 10, 14, 18, 22, 26, 30, 31, 32, 32]
    @test path2 == [5, 5, 6,  7,  8,  12, 12, 12, 12, 12, 16]
    # @show path1
    # @show path2
end

# # Test prioritized dfs planner
# let
#     cost_model = MakeSpan
#     project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_1(;verbose=false);
#
#     milp_model = SparseAdjacencyMILP()
#     # milp_model = GreedyAssignment()
#     project_schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
#     model = formulate_milp(milp_model,project_schedule,problem_spec;cost_model=cost_model)
#     optimize!(model)
#     adj_matrix = get_assignment_matrix(model)
#     update_project_schedule!(milp_model,project_schedule,problem_spec,adj_matrix)
#
#
#     for k in 1:2
#         solver = PC_TAPF_Solver(
#             cbs_model = PrioritizedDFSPlanner(),
#             astar_model = DFS_PathFinder(),
#             l2_verbosity=5
#         )
#         search_env = construct_search_env(solver, project_schedule, problem_spec, env_graph;primary_objective=cost_model)
#
#         env_graph = search_env.env.graph
#         project_schedule = search_env.schedule
#         N = search_env.num_agents
#
#         route_plan = deepcopy(search_env.base_solution)
#         paths = get_paths(route_plan)
#         envs = Vector{PCCBS.LowLevelEnv}([PCCBS.LowLevelEnv() for i in 1:N])
#         cbs_node = initialize_root_node(search_env)
#         for i in 1:N
#             node_id = get_next_node_matching_agent_id(project_schedule,search_env.cache,i)
#             envs[i], _ = build_env(solver,search_env,cbs_node,get_vtx(project_schedule,node_id))
#         end
#
#         if k == 1
#             # test with paths of equal length
#         elseif k == 2
#             # test with paths of different lengths
#             update_envs!(solver,search_env,envs,paths)
#             push!(paths[1].path_nodes,PathNode(
#                 PCCBS.State(1,0),
#                 PCCBS.Action(Edge(1,5),1),
#                 PCCBS.State(5,1),
#             ))
#         end
#         let
#             envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
#             @test !any(map(env->isa(env.schedule_node,ROBOT_AT),envs))
#             if k == 1
#                 @test all(map(env->isa(env.schedule_node,GO),envs))
#             elseif k == 2
#                 @test isa(envs[1].schedule_node,CARRY)
#                 @test isa(envs[2].schedule_node,GO)
#             end
#             @test get_sp(get_path_node(paths[1],1)) == PCCBS.State(vtx=5,t=1)
#             @test get_sp(get_path_node(paths[2],1)) == PCCBS.State(vtx=8,t=1)
#             @test get_vtx(search_env.schedule,ObjectID(1)) in search_env.cache.closed_set
#             @test get_vtx(search_env.schedule,ObjectID(2)) in search_env.cache.closed_set
#             @test get_vtx(search_env.schedule,RobotID(1)) in search_env.cache.closed_set
#             @test get_vtx(search_env.schedule,RobotID(2)) in search_env.cache.closed_set
#             @test status == false
#             map(v->get_node_from_vtx(search_env.schedule,v),collect(search_env.cache.active_set))
#
#             envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
#             @test !any(map(env->isa(env.schedule_node,ROBOT_AT),envs))
#             @test all(map(env->isa(env.schedule_node,CARRY),envs))
#             @test status == false
#
#             envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
#             @test !any(map(env->isa(env.schedule_node,ROBOT_AT),envs))
#             @test isa(envs[1].schedule_node,CARRY)
#             @test isa(envs[2].schedule_node,GO)
#             @test status == false
#
#             envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
#             @test !any(map(env->isa(env.schedule_node,ROBOT_AT),envs))
#             @test isa(envs[1].schedule_node,GO)
#             @test isa(envs[2].schedule_node,GO)
#             @test status == false
#
#             envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
#             @test !any(map(env->isa(env.schedule_node,ROBOT_AT),envs))
#             @test isa(envs[1].schedule_node,CARRY)
#             @test isa(envs[2].schedule_node,GO)
#             @test status == false
#
#             envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
#             @test !any(map(env->isa(env.schedule_node,ROBOT_AT),envs))
#             @test isa(envs[1].schedule_node,GO)
#             @test isa(envs[2].schedule_node,GO)
#             @test status == true
#         end
#     end
# end
# let
#     cost_model = MakeSpan
#     project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_4(;verbose=false);
#
#     milp_model = SparseAdjacencyMILP()
#     # milp_model = GreedyAssignment()
#     project_schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
#     model = formulate_milp(milp_model,project_schedule,problem_spec;cost_model=cost_model)
#     optimize!(model)
#     adj_matrix = get_assignment_matrix(model)
#     update_project_schedule!(milp_model,project_schedule,problem_spec,adj_matrix)
#
#     solver = PC_TAPF_Solver(
#         nbs_model = milp_model,
#         cbs_model = PrioritizedDFSPlanner(),
#         astar_model = DFS_PathFinder(),
#         l2_verbosity=6
#     )
#     search_env = construct_search_env(solver, project_schedule, problem_spec, env_graph;primary_objective=cost_model)
#
#     route_plan = deepcopy(search_env.base_solution)
#     paths = get_paths(route_plan)
#     envs = Vector{PCCBS.LowLevelEnv}([PCCBS.LowLevelEnv() for p in paths])
#     cbs_node = initialize_root_node(search_env)
#     for i in 1:search_env.num_agents
#         node_id = get_next_node_matching_agent_id(search_env.schedule,search_env.cache,i)
#         envs[i], _ = build_env(solver,search_env,cbs_node,get_vtx(search_env.schedule,node_id))
#     end
#
#     envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
#     vtxs = map(e->get_vtx(search_env.schedule,e.node_id),envs)
#     nodes = map(e->e.schedule_node,envs)
#     println(map(n->string(n),nodes)...)
#     println("t0=$(search_env.cache.t0[vtxs]), tF=$(search_env.cache.tF[vtxs])")
#     envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
#     envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
#     envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
#     @show string(get_node_from_vtx(search_env.schedule,v)), search_env.cache.tF[v]
#     cost = aggregate_costs(get_cost_model(search_env),map(p->get_cost(p),paths))
#
# end
# let
#     cost_model = MakeSpan
#     project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_3(;verbose=false);
#
#     milp_model = SparseAdjacencyMILP()
#     # milp_model = GreedyAssignment()
#     project_schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
#     model = formulate_milp(milp_model,project_schedule,problem_spec;cost_model=cost_model)
#     optimize!(model)
#     adj_matrix = get_assignment_matrix(model)
#     update_project_schedule!(milp_model,project_schedule,problem_spec,adj_matrix)
#
#
#     solver = PC_TAPF_Solver(
#         cbs_model = PrioritizedDFSPlanner(),
#         astar_model = DFS_PathFinder(),
#         l2_verbosity=6
#     )
#     search_env = construct_search_env(solver, project_schedule, problem_spec, env_graph;primary_objective=cost_model)
#
#     env_graph = search_env.env.graph
#     project_schedule = search_env.schedule
#     N = search_env.num_agents
#
#     route_plan = deepcopy(search_env.base_solution)
#     paths = get_paths(route_plan)
#     envs = Vector{PCCBS.LowLevelEnv}([PCCBS.LowLevelEnv() for i in 1:N])
#     cbs_node = initialize_root_node(search_env)
#     for i in 1:N
#         node_id = get_next_node_matching_agent_id(project_schedule,search_env.cache,i)
#         envs[i], _ = build_env(solver,search_env,cbs_node,get_vtx(project_schedule,node_id))
#     end
#
#     envs, paths, status = prioritized_dfs_search(solver,search_env,envs,paths;max_iters=1)
#     vtxs = map(e->get_vtx(search_env.schedule,e.node_id),envs)
#     t0 = map(v->search_env.cache.t0[v],vtxs)
#     tF = map(v->search_env.cache.tF[v],vtxs)
#     nodes = map(e->string(e.schedule_node),envs)
#     states = hcat(map(p->convert_to_vertex_lists(p),paths))
#     println("t0=$t0, tF=$tF, nodes=$nodes, states=$states")
#     # nodes, t0, tF, states
#
# end
# # Still testing DFS planner
# let
#
#     cost_model = MakeSpan
#     for (i, f) in enumerate([
#                 initialize_toy_problem_1,
#                 initialize_toy_problem_2,
#                 initialize_toy_problem_3,
#                 initialize_toy_problem_4,
#                 initialize_toy_problem_5,
#                 initialize_toy_problem_6,
#                 initialize_toy_problem_7,
#                 initialize_toy_problem_8,
#             ])
#         project_spec, problem_spec, robot_ICs, assignments, env_graph = f(;verbose=false);
#         milp_model = GreedyAssignment()
#         project_schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
#         model = formulate_milp(milp_model,project_schedule,problem_spec;cost_model=cost_model)
#         optimize!(model)
#         lower_bound = Int(round(objective_function(model)))
#         adj_matrix = get_assignment_matrix(model)
#         update_project_schedule!(milp_model,project_schedule,problem_spec,adj_matrix)
#
#         solver = PC_TAPF_Solver(
#             cbs_model = PrioritizedDFSPlanner(),
#             astar_model = DFS_PathFinder(),
#             l2_verbosity=4
#         )
#         env = construct_search_env(solver, project_schedule, problem_spec, env_graph;primary_objective=cost_model)
#         pc_mapf = PC_MAPF(env)
#
#         route_plan, cache, cost = CRCBS.solve!(solver,pc_mapf)
#         @show f, lower_bound, cost
#         @show convert_to_vertex_lists(route_plan)
#         @test length(cache.active_set) == 0
#         @test length(cache.closed_set) == nv(project_schedule)
#         @test maximum(cache.tF) == maximum(map(p->length(p),route_plan.paths))
#         @test validate(env.schedule, convert_to_vertex_lists(route_plan),env.cache.t0,env.cache.tF)
#     end
# end
