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
using Compose
using GraphPlottingBFS

function show_times(sched,v)
    arr = process_schedule(sched)
    return string(map(a->string(a[v],","), arr[1:2])...)
end
function print_project_schedule(project_schedule,filename;mode=:root_aligned)
    rg = get_display_metagraph(project_schedule;
        f=(v,p)->string(v,",",get_path_spec(project_schedule,v).path_id,",",get_path_spec(project_schedule,v).agent_id))
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
        f=(v,p)->string(v,",",get_path_spec(project_schedule,v).path_id,",",get_path_spec(project_schedule,v).agent_id))
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
        model = formulate_schedule_milp(project_schedule,problem_spec)
        optimize!(model)
        @show status = termination_status(model)
        @show obj_val = Int(round(value(objective_function(model))))
        adj_matrix = Int.(round.(value.(model[:X])))

        print_project_schedule(project_schedule,"project_schedule1")
        update_project_schedule!(project_schedule,problem_spec,adj_matrix)
        print_project_schedule(project_schedule,"project_schedule2")

    end
    let

        project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_4(;
            verbose=false);
        robot_ICs = map(i->robot_ICs[i], 1:problem_spec.N)

        project_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
        model = formulate_schedule_milp(project_schedule,problem_spec)
        optimize!(model)
        @show status = termination_status(model)
        @show obj_val = Int(round(value(objective_function(model))))
        adj_matrix = Int.(round.(value.(model[:X])))

        print_project_schedule(project_schedule,"project_schedule1")
        update_project_schedule!(project_schedule,problem_spec,adj_matrix)
        print_project_schedule(project_schedule,"project_schedule2")

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
                    @test termination_status(model1) == MathOptInterface.OPTIMAL
                    assignment_matrix = Int.(round.(value.(model1[:X])))
                    obj_val1 = Int(round(value(objective_function(model1))))
                    assignment_vector = map(j->findfirst(assignment_matrix[:,j] .== 1),1:problem_spec.M);
                    schedule1 = construct_project_schedule(project_spec, problem_spec, robot_ICs, assignment_vector)
                    # verify that the first cost element of low_level_search! matches the milp cost
                    solver = PC_TAPF_Solver(verbosity=0)
                    env, mapf = construct_search_env(schedule1, problem_spec, env_graph;primary_objective=cost_model)
                    pc_mapf = PC_MAPF(env,mapf)
                    constraint_node = initialize_root_node(pc_mapf)
                    low_level_search!(solver,pc_mapf,constraint_node)
                    # @show i, f, obj_val1, constraint_node.cost
                    @test constraint_node.cost[1] == obj_val1

                    # robot_ICs = map(i->robot_ICs[i], 1:problem_spec.N)
                    schedule2 = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
                    model2 = formulate_schedule_milp(schedule2,problem_spec;cost_model=cost_model)
                    optimize!(model2)
                    @test termination_status(model2) == MathOptInterface.OPTIMAL
                    obj_val2 = Int(round(value(objective_function(model2))))
                    adj_matrix = Int.(round.(value.(model2[:X])))
                    update_project_schedule!(schedule2,problem_spec,adj_matrix)

                    @test obj_val1 == obj_val2
                    # @show i, (obj_val1 == obj_val2), obj_val1, obj_val2
                    # @test schedule1 == schedule2
                    if !(obj_val1 == obj_val2)
                        print_project_schedule(schedule1,string("project_schedule1_",i))
                        print_project_schedule(schedule2,model2,string("project_schedule2_",i))
                    end
                end
            end
        end

    end

    # Test the adjacency matrix solver as a part of the full pipeline
    let
        project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_4(;verbose=false);
        solver = PC_TAPF_Solver(verbosity=1)
        high_level_search!(solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer);
        high_level_search!(solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer;primary_objective=MakeSpan);
    end
    let
        project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_4(;verbose=false);
        solver = PC_TAPF_Solver(verbosity=1)
        high_level_search_mod!(solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer);
        high_level_search_mod!(solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer;primary_objective=MakeSpan);
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
                    # Assignment method
                    project_spec, problem_spec, robot_ICs, assignments, env_graph = f(;verbose=false);
                    solver = PC_TAPF_Solver(verbosity=-1)
                    solution1, assignment1, cost1, env1 = high_level_search!(
                        solver,
                        env_graph,
                        project_spec,
                        problem_spec,
                        robot_ICs,
                        Gurobi.Optimizer;
                        primary_objective=cost_model
                        );

                    # Adjacency method
                    solver = PC_TAPF_Solver(verbosity=-1)
                    solution2, assignment2, cost2, env2 = high_level_search_mod!(
                        solver,
                        env_graph,
                        project_spec,
                        problem_spec,
                        robot_ICs,
                        Gurobi.Optimizer;
                        primary_objective=cost_model
                        );

                    # if cost2[1] != cost1[1]
                        @show f, cost_model, cost1, cost2
                    # end
                    @test cost2[1] == cost1[1]

                end
            end
        end

    end

end
