# using Pkg
# Pkg.activate("/home/kylebrown/.julia/dev/TaskGraphs")
using Test
using LightGraphs, MetaGraphs, GraphUtils
using TaskGraphs
using Gurobi
using JuMP, MathOptInterface
using TOML
using Random
using Compose
using GraphPlottingBFS


function print_project_schedule(project_schedule,filename="project_schedule1")
    rg = get_display_metagraph(project_schedule;
        f=(v,p)->string(v,",",get_path_spec(project_schedule,v).path_id,",",get_path_spec(project_schedule,v).agent_id))
    plot_graph_bfs(rg;
        mode=:root_aligned,
        shape_function = (G,v,x,y,r)->Compose.circle(x,y,r),
        color_function = (G,v,x,y,r)->get_prop(G,v,:color),
        text_function = (G,v,x,y,r)->title_string(get_node_from_id(project_schedule, get_vtx_id(project_schedule, v)),true)
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
    problem_spec = TaskGraphProblemSpec(N=N,M=M,D=dist_matrix);

    # Construct Partial Project Schedule
    project_schedule = construct_partial_project_schedule(spec,problem_spec,robot_ICs)

    # edge_list = collect(edges(project_schedule.graph))
    # nodes = map(id->get_node_from_id(project_schedule, id), project_schedule.vtx_ids)
    # nodes, edge_list

    # Formulate MILP problem
    model, job_shop_variables = formulate_schedule_milp(project_schedule,problem_spec)

    # Optimize!
    optimize!(model)
    @show status = termination_status(model)
    obj_val = Int(round(value(objective_function(model))))
    @show adj_matrix = Int.(round.(value.(model[:X])))

    print_project_schedule(project_schedule,"project_schedule1")

    # Update Project Graph by adding all edges encoded by the optimized adjacency graph
    update_project_schedule!(project_schedule,problem_spec,adj_matrix)

    print_project_schedule(project_schedule,"project_schedule2")

    job_shop_variables

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
        model, job_shop_variables = formulate_schedule_milp(project_schedule,problem_spec)
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
        model, job_shop_variables = formulate_schedule_milp(project_schedule,problem_spec)
        optimize!(model)
        @show status = termination_status(model)
        @show obj_val = Int(round(value(objective_function(model))))
        adj_matrix = Int.(round.(value.(model[:X])))

        print_project_schedule(project_schedule,"project_schedule1")
        update_project_schedule!(project_schedule,problem_spec,adj_matrix)
        print_project_schedule(project_schedule,"project_schedule2")

    end
end
