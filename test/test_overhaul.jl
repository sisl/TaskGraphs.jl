# using Pkg
# Pkg.activate("/home/kylebrown/.julia/dev/TaskGraphs")
using TaskGraphs
using CRCBS
using LightGraphs, MetaGraphs, GraphUtils
using ImageFiltering
using Gurobi
using JuMP, MathOptInterface
using TOML
using Random
using Test
using GraphPlottingBFS
using Compose

# load rendering tools
include(joinpath(pathof(TaskGraphs),"../..","test/notebooks/render_tools.jl"))
# for f in *.svg; do inkscape -z $f -e $f.png; done

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

    print_project_schedule(project_schedule,"project_schedule1";mode=:leaf_aligned)

    # Update Project Graph by adding all edges encoded by the optimized adjacency graph
    update_project_schedule!(project_schedule,problem_spec,adj_matrix)
    print_project_schedule(project_schedule,"project_schedule2")

    @test validate(project_schedule)
end
# let

# Verify that old method (assignment milp) and new method (adjacency matrix
    # milp) yield the same costs
let

    for (i, f) in enumerate([
        initialize_toy_problem_1,
        initialize_toy_problem_2,
        initialize_toy_problem_3,
        initialize_toy_problem_5,
        initialize_toy_problem_5,
        initialize_toy_problem_6,
        initialize_toy_problem_7,
        initialize_toy_problem_8,
        ])
        for cost_model in [MakeSpan, SumOfMakeSpans]
            costs = Float64[]
            schedules = ProjectSchedule[]
            project_spec, problem_spec, robot_ICs, assignments, env_graph = f(;verbose=false);
            for milp_model in [AssignmentMILP(),AdjacencyMILP(),SparseAdjacencyMILP()]
                # MILP formulations alone
                schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
                model = formulate_milp(milp_model,schedule,problem_spec;cost_model=cost_model)
                optimize!(model)
                @test termination_status(model) == MOI.OPTIMAL
                cost = Int(round(value(objective_function(model))))
                adj_matrix = get_assignment_matrix(model)
                update_project_schedule!(milp_model,schedule,problem_spec,adj_matrix)
                push!(costs, cost)
                push!(schedules, schedule)
                @test validate(schedule)
                @test cost != Inf

                # Check that it matches low_level_search
                solver = PC_TAPF_Solver(verbosity=0)
                env = construct_search_env(solver, schedule, problem_spec, env_graph;primary_objective=cost_model)
                pc_mapf = PC_MAPF(env)
                constraint_node = initialize_root_node(pc_mapf)
                low_level_search!(solver,pc_mapf,constraint_node)
                @show i, f, milp_model, cost_model, cost, constraint_node.cost
                @test constraint_node.cost[1] == cost
                @test validate(env.schedule)
            end
            if !(costs[1] == costs[2])
                print_project_schedule(schedules[1],string("project_schedule1_",i))
                print_project_schedule(schedules[2],string("project_schedule2_",i))
            end
            if !(costs[1] == costs[3])
                print_project_schedule(schedules[1],string("project_schedule1_",i))
                print_project_schedule(schedules[3],string("project_schedule3_",i))
            end
            @test all(costs .== costs[1])
        end
    end
end
# Test GreedyAssignment
let

    for (i, f) in enumerate([
        initialize_toy_problem_1,
        initialize_toy_problem_2,
        initialize_toy_problem_3,
        initialize_toy_problem_5,
        initialize_toy_problem_5,
        initialize_toy_problem_6,
        initialize_toy_problem_7,
        initialize_toy_problem_8,
        ])
        for cost_model in [MakeSpan, SumOfMakeSpans]
            costs = Float64[]
            project_spec, problem_spec, robot_ICs, assignments, env_graph = f(;verbose=false);
            for milp_model in [SparseAdjacencyMILP(),GreedyAssignment()]
                # MILP formulations alone
                schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
                model = formulate_milp(milp_model,schedule,problem_spec;cost_model=cost_model)
                optimize!(model)
                @test termination_status(model) == MOI.OPTIMAL
                cost = Int(round(value(objective_function(model))))
                adj_matrix = get_assignment_matrix(model)
                update_project_schedule!(milp_model,schedule,problem_spec,adj_matrix)
                @test validate(schedule)
                @test cost != Inf
                push!(costs, cost)

                # Check that it matches low_level_search
                solver = PC_TAPF_Solver(verbosity=0)
                env = construct_search_env(solver, schedule, problem_spec, env_graph;primary_objective=cost_model)
                pc_mapf = PC_MAPF(env)
                constraint_node = initialize_root_node(pc_mapf)
                low_level_search!(solver,pc_mapf,constraint_node)
                @show i, f, milp_model, cost_model, cost, constraint_node.cost
                @test get_primary_cost(solver,constraint_node.cost) == cost
                @test validate(env.schedule)
            end
            @show costs
        end
    end
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
                    push!(costs, cost[1])
                    @test validate(env.schedule)
                    @show convert_to_vertex_lists(solution)
                    @test validate(env.schedule, convert_to_vertex_lists(solution), env.cache.t0, env.cache.tF)
                    @test cost[1] != Inf
                end
                @test all(costs .== costs[1])
            end
        end
    end
end

let
    f = initialize_toy_problem_9
    cost_model = MakeSpan
    project_spec, problem_spec, robot_ICs, assignments, env_graph = f(;verbose=false)
    solver = PC_TAPF_Solver(nbs_model=AssignmentMILP(),l3_verbosity=0)
    solution, assignment, cost, env = high_level_search!(
        solver,
        env_graph,
        project_spec,
        problem_spec,
        robot_ICs,
        Gurobi.Optimizer;
        primary_objective=cost_model,
        )

    cache = env.cache
    tF = cache.tF
    slack = cache.slack
    deadline = tF .+ map(i->minimum(i),slack)
end

# end

# For catching troublesome problem instances
# let
#     env_id = 2
#     env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
#     factory_env = read_env(env_filename)
#     env_graph = factory_env.graph
#     dist_matrix = get_dist_matrix(env_graph)
#     logfile = "log.txt"
#
#     TimeLimit = 40
#     OutputFlag = 0
#     problem_dir = PROBLEM_DIR
#
#     problematic_ids = [
#         43, # one of the solvers gets stuck after A* returns an infeasible path (twice)
#         146, # A* infeasible, again.
#         197, # can't remember why I put this on here
#         255, # more A* infeasible. These always seem to terminate with "bounds error"
#         267, # just pausing here--nothing necessarily wrong.
#         146, # TODO why does this fail for SparseAdjacencyMILP?
#         ]
#
#     ##
#     # for problem_id in problematic_ids[end]+1:384
#     for problem_id in 1:10
#         problem_filename = joinpath(problem_dir,string("problem",problem_id,".toml"))
#         problem_def = read_problem_def(problem_filename)
#         project_spec, r0, s0, sF = problem_def.project_spec,problem_def.r0,problem_def.s0,problem_def.sF
#         project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec, r0, s0, sF, dist_matrix)
#         println("PROBLEM ID: ", problem_id)
#         for cost_model in [SumOfMakeSpans, MakeSpan]
#             costs = Float64[]
#             for milp_model in [AdjacencyMILP(),SparseAdjacencyMILP()]
#                 try
#                     solver = PC_TAPF_Solver(verbosity=0)
#                     solution, assignment, cost, env = high_level_search!(
#                         milp_model,
#                         solver,
#                         env_graph,
#                         project_spec,
#                         problem_spec,
#                         robot_ICs,
#                         Gurobi.Optimizer;
#                         primary_objective=cost_model,
#                         TimeLimit=TimeLimit
#                         )
#                     push!(costs, cost[1])
#                     @assert validate(env.schedule)
#                     @assert cost[1] != Inf
#                 catch e
#                     open(logfile, "a") do io
#                         write(io, string("PROBLEM ", problem_id, " - ",
#                             "cost model: ", cost_model, " - ",
#                             typeof(milp_model), " - ", e.msg, "\n"))
#                     end
#                 end
#             end
#             try
#                 @assert all(costs .== costs[1])
#             catch e
#                 open(logfile, "a") do io
#                     write(io, string("PROBLEM ", problem_id, " - ",
#                         "cost model: ", cost_model, " - ",
#                          e.msg, " costs: ", costs, "\n"))
#                 end
#             end
#         end
#     end
#     ##
# end

# for REPLANNING
let
    env_id = 2
    env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
    factory_env = read_env(env_filename)
    # factory_env = construct_regular_factory_world(;
    #     n_obstacles_x=2,
    #     n_obstacles_y=2,
    #     obs_width = [2;2],
    #     obs_offset = [2;2],
    #     env_pad = [1;1],
    #     env_offset = [1,1],
    #     env_scale = 1 # this is essentially the robot diameter
    # )
    env_graph = factory_env
    dist_matrix = get_dist_matrix(env_graph)

    # Experimental params

    # seed = 0
    solver_template = PC_TAPF_Solver(
        nbs_model=SparseAdjacencyMILP(),
        DEBUG=true,
        l1_verbosity=1,
        l2_verbosity=1,
        l3_verbosity=0,
        l4_verbosity=0,
        LIMIT_assignment_iterations=5,
        LIMIT_A_star_iterations=8000
        );
    fallback_solver_template = PC_TAPF_Solver(
        nbs_model=GreedyAssignment(),
        astar_model = PrioritizedAStarModel(),
        DEBUG=true,
        l1_verbosity=1,
        l2_verbosity=1,
        l3_verbosity=0,
        l4_verbosity=0,
        LIMIT_assignment_iterations=2,
        LIMIT_A_star_iterations=8000
        );

    # replan_model = MergeAndBalance()
    # replan_model = ReassignFreeRobots()
    replan_model = DeferUntilCompletion()
    fallback_model = ReassignFreeRobots()

    primary_objective = SumOfMakeSpans

    ################################################################################
    ############################## Define Project List #############################
    ################################################################################
    seed = 0
    Random.seed!(seed)
    # seed = seed + 1
    # N = 10
    # M = 10
    N = 4
    M = 6
    max_parents = 3
    depth_bias = 0.4
    Δt_min = 0
    Δt_max = 0
    task_sizes = (1=>1.0,2=>0.0,4=>0.0) # all single agent tasks for now
    stream_length = 5
    project_list = SimpleProblemDef[]
    for i in 1:stream_length
        r0,s0,sF = get_random_problem_instantiation(N,M,get_pickup_zones(factory_env),get_dropoff_zones(factory_env),
                get_free_zones(factory_env))
        project_spec = construct_random_project_spec(M,s0,sF;max_parents=max_parents,depth_bias=depth_bias,Δt_min=Δt_min,Δt_max=Δt_max)
        shapes = choose_random_object_sizes(M,Dict(task_sizes...))
        push!(project_list, SimpleProblemDef(project_spec,r0,s0,sF,shapes))
    end

    arrival_interval            = 30 # new project requests arrive every `arrival_interval` timesteps
    warning_time                = 20 # the project request arrives in the command center `warning_time` timesteps before the relevant objects become available
    commit_threshold            = 5 # freeze the current plan (route plan and schedule) at `t_arrival` + `commit_threshold`
    fallback_commit_threshold   = 2 # a tentative plan (computed with a fast heuristic) may take effect at t = t_arrival + fallback_commit_threshold--just in case the solver fails

    ################################################################################
    ############################## Simulate Replanning #############################
    ################################################################################

    solver = PC_TAPF_Solver(solver_template);
    fallback_solver = PC_TAPF_Solver(fallback_solver_template);

    idx = 1
    def = project_list[idx]
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(def, dist_matrix);
    partial_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
    base_search_env = construct_search_env(solver,partial_schedule,problem_spec,env_graph;primary_objective=primary_objective)

    # store solution for plotting
    project_ids = Dict{Int,Int}()
    for (object_id, pred) in get_object_ICs(partial_schedule)
        project_ids[get_id(object_id)] = idx
    end
    object_path_dict = Dict{Int,Vector{Vector{Int}}}()
    object_interval_dict = Dict{Int,Vector{Int}}()

    print_project_schedule(string("schedule",idx,"B"),base_search_env.schedule,base_search_env.cache;mode=:leaf_aligned)
    (solution, _, cost, search_env), elapsed_time, byte_ct, gc_time, mem_ct = @timed high_level_search!(
        solver, base_search_env, Gurobi.Optimizer;primary_objective=primary_objective)
    print_project_schedule(string("schedule",idx,"C"),search_env.schedule;mode=:leaf_aligned)

    while idx < length(project_list)
        project_schedule = search_env.schedule
        cache = search_env.cache
        idx += 1
        t_request = arrival_interval * (idx - 1) # time when request reaches command center
        t_arrival = t_request + warning_time # time when objects become available
        # load next project
        def = project_list[idx]
        project_spec, problem_spec, _, _, _ = construct_task_graphs_problem(def, dist_matrix);
        next_schedule = construct_partial_project_schedule(project_spec,problem_spec)
        remap_object_ids!(next_schedule,project_schedule)
        print_project_schedule(string("next_schedule",idx),next_schedule;mode=:leaf_aligned)
        # store solution for plotting
        object_path_dict, object_interval_dict = fill_object_path_dicts!(solution,project_schedule,cache,object_path_dict,object_interval_dict)
        for (object_id, pred) in get_object_ICs(next_schedule)
            project_ids[get_id(object_id)] = idx
        end
        # Fallback
        fallback_search_env = replan(fallback_solver, fallback_model, search_env, env_graph, problem_spec, solution, next_schedule, t_request, t_arrival;commit_threshold=fallback_commit_threshold)
        reset_solver!(fallback_solver)
        (fallback_solution, _, fallback_cost, fallback_search_env, _), fallback_elapsed_time, _, _, _ = @timed high_level_search!(
            fallback_solver, fallback_search_env, Gurobi.Optimizer;primary_objective=primary_objective)
        print_project_schedule(string("schedule",idx,"A"),fallback_search_env.schedule;mode=:leaf_aligned)
        # Replanning outer loop
        # TODO update fall_back plan
        # base_search_env = replan(solver, replan_model, search_env, env_graph, problem_spec, solution, next_schedule, t_request, t_arrival; commit_threshold=commit_threshold)
        base_search_env = replan(solver, replan_model, fallback_search_env, env_graph, problem_spec, fallback_solution, nothing, t_request, t_arrival;commit_threshold=commit_threshold)
        print_project_schedule(string("schedule",idx,"B"),base_search_env.schedule;mode=:leaf_aligned)
        # plan for current project
        reset_solver!(solver)
        (solution, _, cost, search_env, _), elapsed_time, _, _, _ = @timed high_level_search!(solver, base_search_env, Gurobi.Optimizer;primary_objective=primary_objective)
        print_project_schedule(string("schedule",idx,"C"),search_env.schedule;mode=:leaf_aligned)
        m = maximum(map(p->length(p), get_paths(solution)))
        @show m
    end

    robot_paths = convert_to_vertex_lists(solution)
    object_path_dict, object_interval_dict = fill_object_path_dicts!(solution,search_env.schedule,search_env.cache,object_path_dict,object_interval_dict)
    object_paths, object_intervals = convert_to_path_vectors(object_path_dict, object_interval_dict)
    project_idxs = map(k->project_ids[k], sort(collect(keys(project_ids))))

    # @show project_ids
    # @show project_idxs

    # # Render video clip
    # tf = maximum(map(p->length(p),robot_paths))
    # set_default_plot_size(24cm,24cm)
    # record_video(joinpath(VIDEO_DIR,string("replanning4.webm")),
    #     t->render_paths(t,factory_env,robot_paths,object_paths;
    #         object_intervals=object_intervals,
    #         colors_vec=map(i->LCHab(60,80,200),1:length(robot_paths)),
    #         project_idxs=project_idxs,
    #         show_paths=false);tf=tf)

end

# for REPLANNING
let

    env_id=2
    env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
    factory_env = read_env(env_filename)
    env_graph = factory_env
    dist_matrix = get_dist_matrix(env_graph)
    dist_mtx_map = DistMatrixMap(factory_env.vtx_map,factory_env.vtxs)

    base_solver_configs = [
        Dict(
        :nbs_time_limit=>8,
        :route_planning_buffer=>2,
        :env_id=>2,
        :OutputFlag => 0,
        :Presolve => -1,
        ),
    ]
    replan_configs = [
        Dict(:replan_model=>MergeAndBalance(),),
        Dict(:replan_model=>Oracle(),:time_out_buffer=>-105,:route_planning_buffer=>5),
        Dict(:replan_model=>ReassignFreeRobots(),),
        Dict(:replan_model=>DeferUntilCompletion(),),
    ]
    fallback_configs = [
        Dict(:fallback_model=>ReassignFreeRobots(),),
    ]
    solver_configs = Dict[]
    for dicts in Base.Iterators.product(base_solver_configs,replan_configs,fallback_configs)
        push!(solver_configs,merge(dicts...))
    end
    base_configs = [
        Dict(
            :warning_time=>20,
            :commit_threshold=>10,
            :fallback_commit_threshold=>10,
            :num_trials => 4,
            :max_parents => 3,
            :depth_bias => 0.4,
            :dt_min => 0,
            :dt_max => 0,
            :dt_collect => 0,
            :dt_deliver => 0,
            :task_sizes => (1=>1.0,2=>0.0,4=>0.0),
            )
    ]
    stream_configs = [
        Dict(:N=>30, :M=>10, :num_projects=>10, :arrival_interval=>40, ),
        Dict(:N=>30, :M=>15, :num_projects=>10, :arrival_interval=>50, ),
        Dict(:N=>30, :M=>20, :num_projects=>10, :arrival_interval=>60, ),
        Dict(:N=>30, :M=>25, :num_projects=>10, :arrival_interval=>70, ),
        Dict(:N=>30, :M=>30, :num_projects=>10, :arrival_interval=>80, ),
    ]
    problem_configs = Dict[]
    for dicts in Base.Iterators.product(base_configs,stream_configs)
        push!(problem_configs,merge(dicts...))
    end

    solver_template = PC_TAPF_Solver(
        nbs_model                   = SparseAdjacencyMILP(),
        DEBUG                       = true,
        l1_verbosity                = 1,
        l2_verbosity                = 1,
        l3_verbosity                = 0,
        l4_verbosity                = 0,
        LIMIT_assignment_iterations = 10,
        LIMIT_A_star_iterations     = 8000
        );
    fallback_solver_template = PC_TAPF_Solver(
        nbs_model                   = GreedyAssignment(),
        astar_model                 = PrioritizedAStarModel(),
        DEBUG                       = true,
        l1_verbosity                = 1,
        l2_verbosity                = 1,
        l3_verbosity                = 0,
        l4_verbosity                = 0,
        LIMIT_assignment_iterations = 2,
        LIMIT_A_star_iterations     = 8000
        );


    base_dir            = joinpath(EXPERIMENT_DIR,"replanning")
    base_problem_dir    = joinpath(base_dir,"problem_instances")
    base_results_dir    = joinpath(base_dir,"results")

    solver_config = solver_configs[4]
    problem_config = problem_configs[1]
    primary_objective = SumOfMakeSpans
    folder_id = 5
    # reset_operation_id_counter!()
    # reset_action_id_counter!()
    # run_replanner_profiling(:write;
    #     problem_configs=problem_configs,
    #     base_problem_dir=base_problem_dir,
    #     )
    modes = [:solve]
    # for solver_config in solver_configs
    replan_model = get(solver_config,:replan_model,  MergeAndBalance())
    fallback_model  = get(solver_config,:fallback_model,ReassignFreeRobots())
    results_dir     = get(solver_config,:results_dir,   joinpath(
        base_results_dir, string(typeof(replan_model),"-",typeof(fallback_model))))
        # for mode in modes
        #     reset_operation_id_counter!()
        #     reset_action_id_counter!()
        #     run_replanner_profiling(mode;
        #         solver_config=solver_config,
        #         problem_configs=problem_configs,
        #         base_problem_dir=base_problem_dir,
        #         base_results_dir=results_dir,
        #         solver_template=solver_template,
        #         fallback_solver_template=fallback_solver_template,
        #         primary_objective=SumOfMakeSpans,
        #         )
        # end
    # end

    Random.seed!(1)
    # folder_id = initial_problem_id-1;
    # run tests and push results into table
    # for problem_config in problem_configs

    num_trials          = get(problem_config,:num_trials,1)
    max_parents         = get(problem_config,:max_parents,3)
    depth_bias          = get(problem_config,:depth_bias,0.4)
    dt_min              = get(problem_config,:dt_min,0)
    dt_max              = get(problem_config,:dt_max,0)
    dt_collect          = get(problem_config,:dt_collect,0)
    dt_deliver          = get(problem_config,:dt_deliver,0)
    task_sizes          = get(problem_config,:task_sizes,(1=>1.0,2=>0.0,4=>0.0))
    N                   = get(problem_config,:N,30)
    M                   = get(problem_config,:M,10)
    num_projects        = get(problem_config,:num_projects,10)
    arrival_interval    = get(problem_config,:arrival_interval,40)
    # for trial in 1:num_trials
    # try
    # folder_id += 1 # moved this to the beginning so it doesn't get skipped
    problem_dir = joinpath(base_problem_dir,string("stream",folder_id))

    project_list = SimpleProblemDef[]
    for problem_id in 1:num_projects
        problem_filename = joinpath(problem_dir,string("problem",problem_id,".toml"))
        config_filename = joinpath(problem_dir,string("config",problem_id,".toml"))
        # config_filename = joinpath(problem_dir,string("config",problem_id,".toml"))
        problem_def = read_problem_def(problem_filename)
        push!(project_list, problem_def)
    end

    solver = PC_TAPF_Solver(solver_template,start_time=time());
    fallback_solver = PC_TAPF_Solver(fallback_solver_template,start_time=time());

    arrival_interval            = problem_config[:arrival_interval] # new project requests arrive every `arrival_interval` timesteps
    warning_time                = problem_config[:warning_time] # the project request arrives in the command center `warning_time` timesteps before the relevant objects become available
    commit_threshold            = problem_config[:commit_threshold] # freeze the current plan (route plan and schedule) at `t_arrival` + `commit_threshold`
    fallback_commit_threshold   = problem_config[:fallback_commit_threshold] # a tentative plan (computed with a fast heuristic) may take effect at t = t_arrival + fallback_commit_threshold--just in case the solver fails

    local_results = map(p->Dict{String,Any}(),project_list)
    final_times = map(p->Dict{AbstractID,Int}(),project_list)

    idx = 1
    t_arrival = 0
    def = project_list[idx]
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(def, dist_matrix;
        Δt_collect=map(i->get(problem_config,:dt_collect,0), def.s0),
        Δt_deliver=map(i->get(problem_config,:dt_deliver,0), def.s0),
        cost_function=primary_objective,
        task_shapes=def.shapes,
        shape_dict=env_graph.expanded_zones,
        );
    partial_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
    base_search_env = construct_search_env(solver,partial_schedule,problem_spec,env_graph;primary_objective=primary_objective)

    # store solution for plotting
    project_ids = Dict{Int,Int}()
    for (object_id, pred) in get_object_ICs(partial_schedule)
        project_ids[get_id(object_id)] = idx
    end
    object_path_dict = Dict{Int,Vector{Vector{Int}}}()
    object_interval_dict = Dict{Int,Vector{Int}}()

    # print_project_schedule(string("schedule",idx,"B"),base_search_env.schedule;mode=:leaf_aligned)
    (solution, _, cost, search_env, optimality_gap), elapsed_time, byte_ct, gc_time, mem_ct = @timed high_level_search!(
        solver, base_search_env, Gurobi.Optimizer;primary_objective=primary_objective)
    # print_project_schedule(string("schedule",idx,"C"),search_env.schedule;mode=:leaf_aligned)
    local_results[idx] = compile_solver_results(solver, solution, cost, search_env, optimality_gap, elapsed_time, t_arrival)
    merge!(final_times[idx], Dict{AbstractID,Int}(OperationID(k)=>t_arrival for k in keys(get_operations(partial_schedule))))
    for i in 1:idx
        for node_id in keys(final_times[i])
            v = get_vtx(search_env.schedule,node_id)
            final_times[i][node_id] = max(final_times[i][node_id], get(search_env.cache.tF, v, -1))
        end
    end

    while idx < length(project_list)
        project_schedule = search_env.schedule
        cache = search_env.cache
        idx += 1
        t_request = arrival_interval * (idx - 1) # time when request reaches command center
        t_arrival = t_request + warning_time # time when objects become available
        # load next project
        def = project_list[idx]
        project_spec, problem_spec, _, _, _ = construct_task_graphs_problem(def, dist_matrix;
            Δt_collect=map(i->get(problem_config,:dt_collect,0), def.s0),
            Δt_deliver=map(i->get(problem_config,:dt_deliver,0), def.s0),
            cost_function=primary_objective,
            task_shapes=def.shapes,
            shape_dict=env_graph.expanded_zones
        )
        next_schedule = construct_partial_project_schedule(project_spec,problem_spec)
        remap_object_ids!(next_schedule,project_schedule)
        print_project_schedule(string("next_schedule",idx),next_schedule;mode=:leaf_aligned)
        # store solution for plotting
        object_path_dict, object_interval_dict = fill_object_path_dicts!(solution,project_schedule,cache,object_path_dict,object_interval_dict)
        for (object_id, pred) in get_object_ICs(next_schedule)
            project_ids[get_id(object_id)] = idx
        end
        # Fallback
        println("Computing Fallback plan at idx = ",idx)
        fallback_search_env = replan(fallback_solver, fallback_model, search_env, env_graph, problem_spec, solution, next_schedule, t_request, t_arrival;commit_threshold=fallback_commit_threshold)
        reset_solver!(fallback_solver)
        (fallback_solution, _, fallback_cost, fallback_search_env, fallback_optimality_gap), fallback_elapsed_time, _, _, _ = @timed high_level_search!(
            fallback_solver, fallback_search_env, Gurobi.Optimizer;primary_objective=primary_objective)

        print_project_schedule(string("schedule",idx,"A"),fallback_search_env;mode=:leaf_aligned)
        # Replanning outer loop
        # t_commit = get_commit_time(replan_model, search_env, t_request, commit_threshold)
        base_search_env = replan(solver, replan_model, search_env, env_graph, problem_spec, solution, next_schedule, t_request, t_arrival; commit_threshold=commit_threshold)
        # base_search_env = replan(solver, replan_model, fallback_search_env, env_graph, problem_spec, fallback_solution, nothing, t_request, t_arrival;commit_threshold=commit_threshold)
        print_project_schedule(string("schedule",idx,"B"),base_search_env;mode=:leaf_aligned)
        # plan for current project
        println("Computing plan at idx = ",idx)
        reset_solver!(solver)
        (solution, _, cost, search_env, optimality_gap), elapsed_time, _, _, _ = @timed high_level_search!(solver, base_search_env, Gurobi.Optimizer;primary_objective=primary_objective)
        print_project_schedule(string("schedule",idx,"C"),search_env;mode=:leaf_aligned)
        # m = maximum(map(p->length(p), get_paths(solution)))
        # @show

        fallback_results = compile_solver_results(fallback_solver, fallback_solution, fallback_cost, fallback_search_env, fallback_optimality_gap, fallback_elapsed_time, t_arrival)
        local_results[idx] = compile_solver_results(solver, solution, cost, search_env, optimality_gap, elapsed_time, t_arrival)
        merge!(local_results[idx],Dict(string("fallback_",k)=>v for (k,v) in fallback_results))
        # Switch to fallback if necessary
        if (elapsed_time > solver.time_limit) || ~local_results[idx]["feasible"]
            if local_results[idx]["feasible"]
                println("TIMEOUT! Using feasible solver output",idx)
            elseif local_results[idx]["fallback_feasible"]
                println("TIMEOUT! Solver failed to find feasible solution. Resorting to fallback plan on ",idx)
                solution = fallback_solution
                search_env = fallback_search_env
                cost = fallback_cost
            else
                println("TIMEOUT! No feasible solution found by fallback solver or regular solver. Terminating...",idx)
                merge!(final_times[idx], Dict{AbstractID,Int}(OperationID(k)=>-1 for k in keys(get_operations(next_schedule))))
                break
            end
        end
        merge!(final_times[idx], Dict{AbstractID,Int}(OperationID(k)=>t_arrival for k in keys(get_operations(next_schedule))))
        for i in 1:idx
            for node_id in keys(final_times[i])
                v = get_vtx(search_env.schedule,node_id)
                final_times[i][node_id] = max(final_times[i][node_id], get(search_env.cache.tF, v, -1))
            end
        end
    end
    results_dict = Dict{String,Any}()
    println("DONE REPLANNING - Aggregating results")
    results_dict["time"]                = map(i->local_results[i]["time"], 1:idx)
    results_dict["optimality_gap"]      = map(i->local_results[i]["optimality_gap"], 1:idx)
    results_dict["optimal"]             = map(i->local_results[i]["optimal"], 1:idx)
    results_dict["feasible"]            = map(i->local_results[i]["feasible"], 1:idx)
    results_dict["cost"]                = map(i->local_results[i]["cost"], 1:idx)
    results_dict["arrival_time"]        = map(i->local_results[i]["arrival_time"], 1:idx)

    results_dict["fallback_feasible"]   = map(i->local_results[i]["fallback_feasible"], 2:idx)

    println("COMPUTING MAKESPANS")
    results_dict["final_time"]         = map(i->maximum(collect(values(final_times[i]))),1:idx)
    results_dict["makespans"]          = [b-a for (a,b) in zip(results_dict["arrival_time"],results_dict["final_time"])]
    @show results_dict["makespans"]

    if cost[1] < Inf
        println("SAVING PATHS")
        object_path_dict, object_interval_dict = fill_object_path_dicts!(solution,search_env.schedule,search_env.cache,object_path_dict,object_interval_dict)
        object_paths, object_intervals = convert_to_path_vectors(object_path_dict, object_interval_dict)

        results_dict["robot_paths"]         = convert_to_vertex_lists(solution)
        results_dict["object_paths"]        = object_paths
        results_dict["object_intervals"]    = object_intervals
        results_dict["project_idxs"]        = map(k->project_ids[k], sort(collect(keys(project_ids))))
    end
    results_dict
end

# Collaborative transport

# GreedyAssignment
# let
#     vtx_grid = initialize_dense_vtx_grid(8,8)
#     # 1   9  17  25  33  41  49  57
#     # 2  10  18  26  34  42  50  58
#     # 3  11  19  27  35  43  51  59
#     # 4  12  20  28  36  44  52  60
#     # 5  13  21  29  37  45  53  61
#     # 6  14  22  30  38  46  54  62
#     # 7  15  23  31  39  47  55  63
#     # 8  16  24  32  40  48  56  64
#     env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
#     # dist_matrix = get_dist_matrix(env_graph)
#     dist_matrix = DistMatrixMap(env_graph.vtx_map,env_graph.vtxs)
#     r0 = [1,25,4,29]
#     # r0 = [1,25,8,28] # check that planning works even when it takes longer for some robots to arrive than others
#     s0 = [10]#,18,11,19]
#     sF = [42] #,50,43,51]
#
#     task_shapes = Dict(1=>(2,2))
#     shape_dict = Dict(
#         10=>Dict((2,2)=>[10,18,11,19]),
#         42=>Dict((2,2)=>[42,50,43,51]),
#         )
#
#     project_spec, robot_ICs = TaskGraphs.initialize_toy_problem(r0,[s0],[sF],(v1,v2)->dist_matrix[v1,v2])
#     add_operation!(project_spec,construct_operation(project_spec,-1,[1],[],0))
#
#     cost_function = MakeSpan
#     project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(
#         project_spec,r0,s0,sF,dist_matrix;cost_function=cost_function,
#         task_shapes=task_shapes,shape_dict=shape_dict)
#
#     solver = PC_TAPF_Solver(
#         DEBUG=true,
#         LIMIT_A_star_iterations=5*nv(env_graph),
#         verbosity=1,
#         l4_verbosity=1
#         );
#
#     env_id = 2
#     env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
#     factory_env = read_env(env_filename)
#     env_graph = factory_env
#     dist_matrix = get_dist_matrix(env_graph)
#     dist_mtx_map = DistMatrixMap(factory_env.vtx_map,factory_env.vtxs)
#
#     problem_filename = "dummy_problem_dir/problem1.toml"
#     problem_def = read_problem_def(problem_filename)
#     project_spec, r0, s0, sF = problem_def.project_spec,problem_def.r0,problem_def.s0,problem_def.sF
#
#     project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
#         project_spec, r0, s0, sF,
#         dist_mtx_map;
#         task_shapes=problem_def.shapes,
#         shape_dict=factory_env.expanded_zones,
#         );
#     # solution, _, cost, env = high_level_search!(SparseAdjacencyMILP(),solver,env_graph,project_spec,problem_spec,robot_ICs,Gurobi.Optimizer)
#     # solution, _, cost, env = high_level_search!(GreedyAssignment(),solver,env_graph,project_spec,problem_spec,robot_ICs,Gurobi.Optimizer)
#
#     project_schedule = construct_partial_project_schedule(project_spec, problem_spec, map(i->robot_ICs[i], 1:problem_spec.N))
#
#     print_project_schedule(project_schedule,"project_schedule1")
#
#     milp_model = formulate_milp(GreedyAssignment(),project_schedule,problem_spec)
#     optimize!(milp_model)
#     X = get_assignment_matrix(milp_model)
#     @test update_project_schedule!(milp_model,project_schedule,problem_spec,X)
#
#     print_project_schedule(project_schedule,"project_schedule2")
# end
