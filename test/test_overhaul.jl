# using Pkg
# Pkg.activate("/home/kylebrown/.julia/dev/TaskGraphs")
using TaskGraphs
using CRCBS
using LightGraphs, MetaGraphs, GraphUtils
using Gurobi
using JuMP, MathOptInterface
using TOML
using Random
using Test
using GraphPlottingBFS
using Compose

# load rendering tools
include(joinpath(pathof(TaskGraphs),"../..","test/notebooks/render_tools.jl"))

function show_times(sched,v)
    arr = process_schedule(sched)
    return string(map(a->string(a[v],","), arr[1:2])...)
end
function print_project_schedule(project_schedule,filename;mode=:root_aligned)
    rg = get_display_metagraph(project_schedule;
        f=(v,p)->string(v,",",get_path_spec(project_schedule,v).agent_id))
    plot_graph_bfs(rg;
        mode=mode,
        shape_function = (G,v,x,y,r)->Compose.circle(x,y,r),
        color_function = (G,v,x,y,r)->get_prop(G,v,:color),
        text_function = (G,v,x,y,r)->string(
            title_string(get_node_from_id(project_schedule, get_vtx_id(project_schedule, v)),true),
            "\n",show_times(project_schedule,v)
            )
    ) |> Compose.SVG(string(filename,".svg"))
    # `inkscape -z project_schedule1.svg -e project_schedule1.png`
    # OR: `for f in *.svg; do inkscape -z $f -e $f.png; done`
end
function print_project_schedule(project_schedule,model,filename;mode=:root_aligned)
    rg = get_display_metagraph(project_schedule;
        f=(v,p)->string(v,",",get_path_spec(project_schedule,v).agent_id))
    plot_graph_bfs(rg;
        mode=mode,
        shape_function = (G,v,x,y,r)->Compose.circle(x,y,r),
        color_function = (G,v,x,y,r)->get_prop(G,v,:color),
        text_function = (G,v,x,y,r)->string(
            title_string(get_node_from_id(project_schedule, get_vtx_id(project_schedule, v)),true),
            "\n",show_times(project_schedule,v),
            "-",Int(round(value(model[:t0][v]))),",",Int(round(value(model[:tF][v])))
            )
    ) |> Compose.SVG(string(filename,".svg"))
    # `inkscape -z project_schedule1.svg -e project_schedule1.png`
    # OR: `for f in *.svg; do inkscape -z $f -e $f.png; done`
end

let

    Random.seed!(0);
    # Define Environment
    vtx_grid = initialize_dense_vtx_grid(8,8);
    env_graph = initialize_grid_graph_from_vtx_grid(vtx_grid);
    dist_matrix = get_dist_matrix(env_graph);
    # Define project
    N = 1; M = 2;
    object_ICs = [OBJECT_AT(j,j) for j in 1:M];
    object_FCs = [OBJECT_AT(j,j+M) for j in 1:M];
    robot_ICs = [ROBOT_AT(i,i) for i in 1:N];
    spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3);
    operations = spec.operations;
    root_ops = map(op->op.id, spec.operations[collect(spec.root_nodes)])
    problem_spec = ProblemSpec(N=N,M=M,D=dist_matrix);

    # Construct Partial Project Schedule
    project_schedule = construct_partial_project_schedule(spec,problem_spec,robot_ICs)

    # edge_list = collect(edges(project_schedule.graph))
    # nodes = map(id->get_node_from_id(project_schedule, id), project_schedule.vtx_ids)
    # nodes, edge_list

    # Formulate MILP problem
    model = formulate_schedule_milp(project_schedule,problem_spec)

    # Optimize!
    optimize!(model)
    @show status = termination_status(model)
    obj_val = Int(round(value(objective_function(model))))
    @show adj_matrix = Int.(round.(value.(model[:X])))

    print_project_schedule(project_schedule,"project_schedule1";mode=:leaf_aligned)

    # Update Project Graph by adding all edges encoded by the optimized adjacency graph
    update_project_schedule!(project_schedule,problem_spec,adj_matrix)
    print_project_schedule(project_schedule,"project_schedule2")

    @test validate(project_schedule)
end
let

    env_id = 2
    env_file = joinpath(ENVIRONMENT_DIR,string("env_",env_id,".toml"))
    factory_env = read_env(env_file)
    env_graph = factory_env.graph
    dist_matrix = get_dist_matrix(env_graph)

    let
        filename = joinpath(PROBLEM_DIR,"problem2.toml")
        problem_def = read_problem_def(filename)

        project_spec, r0, s0, sF = problem_def.project_spec,problem_def.r0,problem_def.s0,problem_def.sF
        robot_ICs = [ROBOT_AT(i,x) for (i,x) in enumerate(r0)] # remove dummy robots
        project_spec, problem_spec, object_ICs, object_FCs, _ = construct_task_graphs_problem(project_spec, r0, s0, sF, dist_matrix)

        project_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
        model = formulate_milp(AssignmentMILP(),project_schedule,problem_spec)
        optimize!(model)
        @test termination_status(model) == MOI.OPTIMAL
        obj_val = Int(round(value(objective_function(model))))
        assignment_matrix = get_assignment_matrix(model);
        assignments = get_assignment_vector(assignment_matrix,problem_spec.M)

        env, mapf = construct_search_env(project_spec, problem_spec, robot_ICs, assignments, env_graph;
            primary_objective=primary_objective);
        pc_mapf = PC_MAPF(env,mapf);

        # adj_matrix = Int.(round.(value.(model[:X])))
        # # print_project_schedule(project_schedule,"project_schedule1")
        # update_project_schedule!(project_schedule,problem_spec,adj_matrix)
        # # print_project_schedule(project_schedule,"project_schedule2")
        # @test validate(project_schedule)
    end
    let
        project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_4(;
            verbose=false);
        robot_ICs = map(i->robot_ICs[i], 1:problem_spec.N)

        project_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
        model = formulate_schedule_milp(project_schedule,problem_spec)
        optimize!(model)
        @test termination_status(model) == MOI.OPTIMAL
        obj_val = Int(round(value(objective_function(model))))
        adj_matrix = Int.(round.(value.(model[:X])))

        # print_project_schedule(project_schedule,"project_schedule1")
        update_project_schedule!(project_schedule,problem_spec,adj_matrix)
        # print_project_schedule(project_schedule,"project_schedule2")
        @test validate(project_schedule)
    end
    # Sparse MILP formulation
    let
        project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_8(;
            verbose=false);
        # robot_ICs = map(i->robot_ICs[i], 1:problem_spec.N)

        # project_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
        cost_model = MakeSpan
        model = formulate_milp(AssignmentMILP(),project_schedule,problem_spec;cost_model=cost_model)
        optimize!(model)
        @test termination_status(model) == MOI.OPTIMAL
        obj_val = Int(round(value(objective_function(model))))
        adj_matrix = get_assignment_matrix(model)
        # assignments = get_assignment_vector(adj_matrix,problem_spec.M)
        assignment_dict, assignments = get_assignment_dict(adj_matrix,problem_spec.N,problem_spec.M)
        project_schedule = construct_project_schedule(project_spec,problem_spec,robot_ICs,assignments)

        solver = PC_TAPF_Solver(verbosity=0)
        env, mapf = construct_search_env(project_schedule, problem_spec, env_graph;primary_objective=cost_model)
        pc_mapf = PC_MAPF(env,mapf)
        constraint_node = initialize_root_node(pc_mapf)
        propagate_valid_ids!(project_schedule,problem_spec)
        print_project_schedule(project_schedule,"project_schedule1")
        low_level_search!(solver,pc_mapf,constraint_node)

        @show convert_to_vertex_lists(constraint_node.solution)

        # assignment_dict = get_assignment_dict(adj_matrix,problem_spec.N,problem_spec.M)
        #
        # G = get_graph(project_schedule)
        # leaf_type = GO
        # root_type = COLLECT
        # for (robot_idx,task_list) in assignment_dict
        #     robot_id = RobotID(robot_idx)
        #     v = get_vtx(project_schedule, robot_id)
        #     # find leaf node
        #     for task_idx in assignment_dict[get_id(robot_id)]
        #         for e in edges(bfs_tree(G, v))
        #             node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, e.dst))
        #             if typeof(node2) <: leaf_type && outdegree(G,e.dst) < sum([0, values(required_successors(node2))...])
        #                 v = e.dst
        #                 break
        #             end
        #         end
        #         object_id = get_object_id(project_spec.initial_conditions[task_idx]) # does not necessarily match task idx
        #         task_node = get_node_from_id(project_schedule, object_id)
        #         vo = get_vtx(project_schedule, object_id)
        #         for e in edges(bfs_tree(G, vo))
        #             node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, e.dst))
        #             if typeof(node2) <: root_type && indegree(G,e.dst) < sum([0, values(required_predecessors(node2))...])
        #                 vo = e.dst
        #                 break
        #             end
        #         end
        #         @show v,vo
        #         add_edge!(G,v,vo)
        #     end
        # end
        # propagate_valid_ids!(project_schedule,problem_spec)

        # update_project_schedule!(project_schedule,problem_spec,adj_matrix)
        # print_project_schedule(project_schedule,"project_schedule2")
        @test validate(project_schedule)
        @test obj_val == get_cost(constraint_node)[1]
    end

    # Verify that old method (assignment milp) and new method (adjacency matrix
    # milp) yield the same costs
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
                    # Compare against old method
                    # f = initialize_toy_problem_8
                    # i = 8
                    # cost_model = MakeSpan
                    project_spec, problem_spec, robot_ICs, assignments, env_graph = f(;verbose=false);
                    model1 = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;cost_model=cost_model);
                    optimize!(model1)
                    @test termination_status(model1) == MOI.OPTIMAL
                    assignment_matrix = Int.(round.(value.(model1[:X])))
                    obj_val1 = Int(round(value(objective_function(model1))))
                    # assignment_vector = map(j->findfirst(assignment_matrix[:,j] .== 1),1:problem_spec.M);
                    assignment_dict, assignments = get_assignment_dict(assignment_matrix,problem_spec.N,problem_spec.M)
                    schedule1 = construct_project_schedule(project_spec, problem_spec, robot_ICs, assignments)
                    # verify that the first cost element of low_level_search! matches the milp cost
                    solver = PC_TAPF_Solver(verbosity=0)
                    env, mapf = construct_search_env(schedule1, problem_spec, env_graph;primary_objective=cost_model)
                    pc_mapf = PC_MAPF(env,mapf)
                    constraint_node = initialize_root_node(pc_mapf)
                    low_level_search!(solver,pc_mapf,constraint_node)
                    # @show i, f, obj_val1, constraint_node.cost
                    @test constraint_node.cost[1] == obj_val1
                    @test validate(env.schedule)

                    # robot_ICs = map(i->robot_ICs[i], 1:problem_spec.N)
                    schedule2 = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
                    model2 = formulate_milp(AdjacencyMILP(),schedule2,problem_spec;cost_model=cost_model)
                    optimize!(model2)
                    @test termination_status(model2) == MOI.OPTIMAL
                    obj_val2 = Int(round(value(objective_function(model2))))
                    adj_matrix = Int.(round.(value.(model2[:X])))
                    update_project_schedule!(schedule2,problem_spec,adj_matrix)
                    @test validate(schedule2)

                    # test sparse adjacency matrix formulation
                    schedule3 = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
                    model3 = formulate_milp(SparseAdjacencyMILP(),schedule2,problem_spec;cost_model=cost_model)
                    optimize!(model3)
                    @test termination_status(model3) == MOI.OPTIMAL
                    obj_val3 = Int(round(value(objective_function(model2))))
                    adj_matrix = get_assignment_matrix(model3)
                    update_project_schedule!(schedule2,problem_spec,adj_matrix)
                    @test validate(schedule2)

                    @test obj_val1 == obj_val2
                    @test obj_val1 == obj_val3
                    # @show i, (obj_val1 == obj_val2), obj_val1, obj_val2
                    # @test schedule1 == schedule2
                    if !(obj_val1 == obj_val2)
                        print_project_schedule(schedule1,string("project_schedule1_",i))
                        print_project_schedule(schedule2,model2,string("project_schedule2_",i))
                    end
                    if !(obj_val1 == obj_val3)
                        print_project_schedule(schedule1,string("project_schedule1_",i))
                        print_project_schedule(schedule3,model3,string("project_schedule3_",i))
                    end
                end
            end
        end
    end

    # Test the adjacency matrix solver as a part of the full pipeline
    let
        project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_4(;verbose=false);
        solver = PC_TAPF_Solver(verbosity=1)
        solution, assignment, cost, env = high_level_search!(solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer)
        solution, assignment, cost, env = high_level_search!(solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer;primary_objective=MakeSpan)
    end
    let
        project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_4(;verbose=false);
        solver = PC_TAPF_Solver(verbosity=1)
        solution, assignment, cost, env = high_level_search_mod!(solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer)
        solution, assignment, cost, env = high_level_search_mod!(solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer;primary_objective=MakeSpan)
    end

    # Test that the full planning stack works with the new model and returns the same final cost
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
                    for milp_model in [AssignmentMILP(),AdjacencyMILP(),SparseAdjacencyMILP()]
                        solver = PC_TAPF_Solver(verbosity=0)
                        solution, assignment, cost, env = high_level_search!(
                            milp_model,
                            solver,
                            env_graph,
                            project_spec,
                            problem_spec,
                            robot_ICs,
                            Gurobi.Optimizer;
                            primary_objective=cost_model,
                            )
                        push!(costs, cost[1])
                        @test validate(env.schedule)
                        @test cost[1] != Inf
                    end
                    @test all(costs .== costs[1])
                end
            end
        end

    end

    # Verify deterministic behavior of AdjacencyMILP formulation
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
                    project_spec, problem_spec, robot_ICs, assignments, env_graph = f(;verbose=false);
                    solver = PC_TAPF_Solver(verbosity=0)
                    solution, assignment, cost, env = high_level_search_mod!(
                        solver,
                        env_graph,
                        project_spec,
                        problem_spec,
                        robot_ICs,
                        Gurobi.Optimizer;
                        primary_objective=cost_model
                        );

                    @test validate(env.schedule)

                    for i in 1:10
                        let
                            solver = PC_TAPF_Solver(verbosity=0)
                            solution2, assignment2, cost2, env2 = high_level_search_mod!(
                                solver,
                                env_graph,
                                project_spec,
                                problem_spec,
                                robot_ICs,
                                Gurobi.Optimizer;
                                primary_objective=cost_model
                                );
                            @test !any(adjacency_matrix(env.schedule.graph) .!= adjacency_matrix(env2.schedule.graph))
                            for v in vertices(env.schedule.graph)
                                spec1 = get_path_spec(env.schedule, v)
                                spec2 = get_path_spec(env2.schedule, v)
                                @test spec1 == spec2
                            end
                            @test cost[1] == cost2[1]
                            @test validate(env2.schedule)
                        end
                    end
                end
            end
        end
    end
end

# Debug: what is causing the cyclic graph?
let

    env_id = 2
    env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
    factory_env = read_env(env_filename)
    env_graph = factory_env.graph
    dist_matrix = get_dist_matrix(env_graph)

    # problem_def = read_problem_def(joinpath(PROBLEM_DIR,"problem223.toml"))
    problem_id = 74
    problem_def = read_problem_def(joinpath(PROBLEM_DIR,string("problem",problem_id,".toml")))
    project_spec, r0, s0, sF = problem_def.project_spec,problem_def.r0,problem_def.s0,problem_def.sF
    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(
            project_spec, r0, s0, sF, dist_matrix);
    # Solve the problem
    solver = PC_TAPF_Solver(DEBUG=true,verbosity=0,LIMIT_A_star_iterations=5*nv(env_graph));
    (solution, assignment, cost, search_env), elapsed_time, byte_ct, gc_time, mem_ct = @timed high_level_search!(
        AdjacencyMILP(), solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer;)

    # Note: the milp solver is called 15 times on problem 75. It must be
    # returning an invalid "optimal" solution on the final call. The behavior
    # should be modified to catch the error and return the next best solution.


    project_schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
    model = formulate_schedule_milp(project_schedule,problem_spec;Presolve=1)
    # optimize!(model)
    retval, elapsed_time, byte_ct, gc_time, mem_ct = @timed optimize!(model)
    @show elapsed_time
    @test termination_status(model) == MathOptInterface.OPTIMAL
    assignment_matrix = get_assignment_matrix(model);
    # ############## Route Planning ###############
    update_project_schedule!(project_schedule,problem_spec,assignment_matrix)
    @test validate(project_schedule)
    print_project_schedule(project_schedule,"project_schedule")
    env, mapf = construct_search_env(project_schedule, problem_spec, env_graph);
    pc_mapf = PC_MAPF(env,mapf);
    ##### Call CBS Search Routine (LEVEL 2) #####
    # solution, cost = solve!(solver,pc_mapf);
    ##### Call Level 3 ####
    root_node = initialize_root_node(pc_mapf)
    # vtx_lists = convert_to_vertex_lists(root_node.solution)
    # for (i,p) in enumerate(vtx_lists)
    #     @show p
    # end
    valid_flag = low_level_search!(solver,pc_mapf,root_node)

    v = 130
    path_id = get_path_spec(project_schedule, v).path_id
    agent_id = get_path_spec(project_schedule, v).agent_id
    cbs_env, base_path = build_env(solver, pc_mapf.env, pc_mapf.mapf, root_node, agent_id, path_id)
    extend_path!(cbs_env, base_path, 216)
    extend_path!(cbs_env, base_path, 217)
    extend_path!(cbs_env, base_path, 218)
    schedule_node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))


    for i in 1:nv(project_schedule.graph)
        @show i
        plan_next_path!(solver,pc_mapf.env,pc_mapf.mapf,root_node;heuristic=get_heuristic_cost)
    end
    plan_next_path!(solver,pc_mapf.env,pc_mapf.mapf,root_node;heuristic=get_heuristic_cost)

end
let
    env_id = 2
    env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
    factory_env = read_env(env_filename)
    env_graph = factory_env.graph
    dist_matrix = get_dist_matrix(env_graph)
    logfile = "log.txt"

    TimeLimit = 40
    OutputFlag = 0
    problem_dir = PROBLEM_DIR

    problematic_ids = [
        43, # one of the solvers gets stuck after A* returns an infeasible path (twice)
        146, # A* infeasible, again.
        197, # can't remember why I put this on here
        255, # more A* infeasible. These always seem to terminate with "bounds error"
        267, # just pausing here--nothing necessarily wrong.
        ]

    ##
    # for problem_id in problematic_ids[end]+1:384
    for problem_id in 1:384
        problem_filename = joinpath(problem_dir,string("problem",problem_id,".toml"))
        problem_def = read_problem_def(problem_filename)
        project_spec, r0, s0, sF = problem_def.project_spec,problem_def.r0,problem_def.s0,problem_def.sF
        project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec, r0, s0, sF, dist_matrix)
        println("PROBLEM ID: ", problem_id)
        for cost_model in [SumOfMakeSpans, MakeSpan]
            costs = Float64[]
            for milp_model in [AdjacencyMILP(),SparseAdjacencyMILP()]
                try
                    solver = PC_TAPF_Solver(verbosity=0)
                    solution, assignment, cost, env = high_level_search!(
                        milp_model,
                        solver,
                        env_graph,
                        project_spec,
                        problem_spec,
                        robot_ICs,
                        Gurobi.Optimizer;
                        primary_objective=cost_model,
                        TimeLimit=TimeLimit
                        )
                    push!(costs, cost[1])
                    @assert validate(env.schedule)
                    @assert cost[1] != Inf
                catch e
                    open(logfile, "a") do io
                        write(io, string("PROBLEM ", problem_id, " - ",
                            "cost model: ", cost_model, " - ",
                            typeof(milp_model), " - ", e.msg, "\n"))
                    end
                end
            end
            try
                @assert all(costs .== costs[1])
            catch e
                open(logfile, "a") do io
                    write(io, string("PROBLEM ", problem_id, " - ",
                        "cost model: ", cost_model, " - ",
                         e.msg, " costs: ", costs, "\n"))
                end
            end
        end
    end
    ##
end
