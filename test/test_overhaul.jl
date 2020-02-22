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
        for cost_model in [SumOfMakeSpans, MakeSpan]
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
                @test validate(schedule)
                push!(costs, cost)
                push!(schedules, schedule)
                @test validate(schedule)
                @test cost != Inf

                # Check that it matches low_level_search
                solver = PC_TAPF_Solver(verbosity=0)
                env = construct_search_env(schedule, problem_spec, env_graph;primary_objective=cost_model)
                pc_mapf = PC_MAPF(env)
                constraint_node = initialize_root_node(pc_mapf)
                low_level_search!(solver,pc_mapf,constraint_node)
                # @show i, f, obj_val1, constraint_node.cost
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
                    solver = PC_TAPF_Solver(nbs_model=milp_model,verbosity=0)
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

# Pruning projects - for REPLANNING
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


    # seed = 0
    solver_template = PC_TAPF_Solver(
        nbs_model=SparseAdjacencyMILP(),
        DEBUG=true,
        l1_verbosity=0,
        l2_verbosity=0,
        l3_verbosity=0,
        l4_verbosity=0,
        LIMIT_assignment_iterations=2,
        LIMIT_A_star_iterations=1000
        );
    primary_objective = SumOfMakeSpans

    t = 10
    t_arrival = t
    seed = 0

    # for seed in 0:50
    #     @show seed
    #     let
    # generate random problem sequence
    Random.seed!(seed)
    # seed = seed + 1
    N = 10
    M = 10
    max_parents = 3
    depth_bias = 0.4
    Δt_min = 0
    Δt_max = 0
    task_sizes = (1=>1.0,2=>0.0,4=>0.0) # all single agent tasks for now

    ################################################################################
    ############################## Define Project List #############################
    ################################################################################
    stream_length = 2
    project_list = SimpleProblemDef[]
    for i in 1:stream_length
        r0,s0,sF = get_random_problem_instantiation(N,M,get_pickup_zones(factory_env),get_dropoff_zones(factory_env),
                get_free_zones(factory_env))
        project_spec = construct_random_project_spec(M,s0,sF;max_parents=max_parents,depth_bias=depth_bias,Δt_min=Δt_min,Δt_max=Δt_max)
        shapes = choose_random_object_sizes(M,Dict(task_sizes...))
        push!(project_list, SimpleProblemDef(project_spec,r0,s0,sF,shapes))
    end

    arrival_interval    = 50 # new project requests arrive every `arrival_interval` timesteps
    warning_time        = 20 # the project request arrives in the command center `warning_time` timesteps before the relevant objects become available
    commit_threshold    = 20 # freeze the current plan (route plan and schedule) at `t_arrival` + `commit_threshold`
    greedy_commit_threshold = 5 # a tentative plan (computed with a fast heuristic) may take effect at t = t_arrival + greedy_commit_threshold--just in case the solver fails

    ################################################################################
    ############################## Simulate Replanning #############################
    ################################################################################
    idx = 1
    def = project_list[idx]
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(def, dist_matrix);
    partial_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
    base_search_env = construct_search_env(partial_schedule,problem_spec,env_graph;primary_objective=primary_objective)
    project_ids = Dict{Int,Int}()
    for (object_id, pred) in get_object_ICs(partial_schedule)
        project_ids[get_id(object_id)] = idx
    end
    object_path_dict = Dict{Int,Vector{Vector{Int}}}()
    object_interval_dict = Dict{Int,Vector{Int}}()

    solver = PC_TAPF_Solver(solver_template);
    while idx < length(project_list)
        # plan for current project
        # reset_solver!(solver)
        solver = PC_TAPF_Solver(solver_template);
        (solution, _, cost, search_env), elapsed_time, byte_ct, gc_time, mem_ct = @timed high_level_search!(
            solver, base_search_env, Gurobi.Optimizer;primary_objective=primary_objective)
        project_schedule = search_env.schedule
        cache = search_env.cache
        # plan for current project
        idx += 1
        t_request = arrival_interval * (idx - 1)
        t_arrival = t_request + warning_time
        t_commit = t_request + commit_threshold
        # load next project
        def = project_list[idx]
        project_spec, problem_spec, _, _, _ = construct_task_graphs_problem(def, dist_matrix);
        next_schedule = construct_partial_project_schedule(project_spec,problem_spec)
        remap_object_ids!(next_schedule,project_schedule)
        # store solution for plotting
        object_path_dict, object_interval_dict = fill_object_path_dicts!(solution,project_schedule,cache;
            object_path_dict = object_path_dict,
            object_interval_dict = object_interval_dict
        )
        for (object_id, pred) in get_object_ICs(next_schedule)
            project_ids[get_id(object_id)] = idx
        end
        # Freeze solution and schedule at t_commit
        trimmed_solution = trim_solution(search_env.env, solution, t_commit) # TODO handle the condition where t_commit > length(solution)
        new_schedule, new_cache = prune_project_schedule(project_schedule, problem_spec, cache, t_commit; robot_positions=get_env_snapshot(initial_solution,t_commit))
        # splice projects together!
        splice_schedules!(new_schedule,next_schedule)
        t0 = map(v->get(new_cache.t0, v, t_arrival), vertices(get_graph(new_schedule)))
        tF = map(v->get(new_cache.tF, v, t_arrival), vertices(get_graph(new_schedule)))
        base_search_env = construct_search_env(new_schedule, problem_spec, env_graph;t0=t0,tF=tF)
        base_search_env = SearchEnv(base_search_env, base_solution=trimmed_solution)
        # TODO Plot the output to make sure that the loop has no bugs
    end


    # def1 = project_list[1]
    #
    # project_spec, r0, s0, sF = def1.project_spec,def1.r0,def1.s0,def1.sF
    # project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(
    #         project_spec, r0, s0, sF, dist_matrix);
    #
    # partial_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
    # print_project_schedule("partial_schedule",partial_schedule;mode=:leaf_aligned)
    # base_search_env = construct_search_env(partial_schedule,problem_spec,env_graph;primary_objective=primary_objective)
    #
    # #### solve initial problem
    # solver = PC_TAPF_Solver(solver_template);
    # (initial_solution, assignment, cost, search_env), elapsed_time, byte_ct, gc_time, mem_ct = @timed high_level_search!(
    #     solver, base_search_env, Gurobi.Optimizer;primary_objective=primary_objective)
    #
    # project_schedule = search_env.schedule
    # cache = search_env.cache
    # @test validate(project_schedule)
    #
    # # NOTE for plotting
    # object_path_dict, object_interval_dict = fill_object_path_dicts!(initial_solution,project_schedule,cache)
    # project_ids = Dict(k=>1 for (k,v) in object_path_dict)
    #
    # print_project_schedule("project_schedule",project_schedule,cache;mode=:leaf_aligned)
    #
    # #### Trim existing solution
    # trimmed_solution = trim_solution(search_env.env, initial_solution, t)
    # @test all([length(p) == get_cost(p)[1] for p in get_paths(trimmed_solution)])
    # #### Prune existing schedule
    # new_schedule, new_cache = prune_project_schedule(project_schedule, problem_spec, cache, t; robot_positions=get_env_snapshot(initial_solution,t))
    # # @show new_cache.t0
    #
    # print_project_schedule("new_schedule",new_schedule,new_cache;mode=:leaf_aligned)
    # # for v in topological_sort(new_schedule.graph)
    # #    if get_path_spec(new_schedule, v).fixed
    # #        @show string(get_node_from_vtx(new_schedule,v)), new_cache.t0[v],new_cache.tF[v]
    # #    end
    # # end
    #
    # #### Next Project schedule to splice in:
    # def2 = project_list[2]
    # project_spec2, problem_spec2, _, _, _ = construct_task_graphs_problem(def2, dist_matrix, shape_dict=factory_env.expanded_zones);
    # next_schedule = construct_partial_project_schedule(project_spec2,problem_spec2)
    #
    # print_project_schedule("next_schedule",next_schedule;mode=:leaf_aligned)
    #
    # # remap object ids in incoming project
    # max_obj_id = maximum([get_id(id) for id in get_vtx_ids(project_schedule) if typeof(id) <: ObjectID])
    # remap_object_ids!(next_schedule,max_obj_id)
    # for (object_id, pred) in get_object_ICs(next_schedule)
    #     project_ids[get_id(object_id)] = 2
    # end
    #
    # print_project_schedule("next_schedule_remapped",next_schedule;mode=:leaf_aligned)
    #
    # let
    #     s1 = Set{AbstractID}(new_schedule.vtx_ids)
    #     s2 = Set{AbstractID}(next_schedule.vtx_ids)
    #     @test length(s1) + length(s2) == length(union(s1,s2))
    # end
    #
    # # splice projects together!
    # splice_schedules!(new_schedule,next_schedule)
    # G = get_graph(new_schedule)
    # # do this again because now we have more nodes
    # t0 = map(v->get(new_cache.t0, v, t_arrival), vertices(G))
    # new_cache = initialize_planning_cache(new_schedule;t0=t0)
    #
    # print_project_schedule("spliced_schedule",new_schedule,new_cache;mode=:leaf_aligned)
    #
    # # The information relevant to replanning
    # new_schedule, new_cache, trimmed_solution
    #
    # new_schedule_copy = deepcopy(new_schedule)
    #
    # model = formulate_milp(SparseAdjacencyMILP(),new_schedule,problem_spec;
    #     cost_model=SumOfMakeSpans,
    #     t0_ = Dict{AbstractID,Int}(get_vtx_id(new_schedule, v)=>t0 for (v,t0) in enumerate(new_cache.t0)),
    #     tF_ = Dict{AbstractID,Int}(get_vtx_id(new_schedule, v)=>tF for (v,tF) in enumerate(new_cache.tF))
    #     )
    # optimize!(model)
    # @test termination_status(model) == MOI.OPTIMAL
    # @show Int(round(value(objective_function(model))))
    # adj_matrix = get_assignment_matrix(model)
    # # DEBUG something is wrong here
    # # for v in topological_sort(new_schedule.graph)
    # #    if get_path_spec(new_schedule, v).fixed
    # #        @show string(get_node_from_vtx(new_schedule,v))
    # #    end
    # # end
    # update_project_schedule!(new_schedule,problem_spec,adj_matrix)
    # @test validate(new_schedule)
    # # for v in topological_sort(new_schedule.graph)
    # #    if get_path_spec(new_schedule, v).fixed
    # #        @show string(get_node_from_vtx(new_schedule,v))
    # #    end
    # # end
    #
    # updated_cache = initialize_planning_cache(new_schedule;t0=deepcopy(new_cache.t0),tF=deepcopy(new_cache.tF))
    #
    # # TODO Pass the final solution
    # # reset_solver!(solver)
    # solver = PC_TAPF_Solver(solver_template);
    #
    # base_search_env = construct_search_env(new_schedule_copy, problem_spec, env_graph;t0=deepcopy(new_cache.t0),tF=deepcopy(new_cache.tF))
    # base_search_env = SearchEnv(base_search_env, base_solution=trimmed_solution)
    #
    # print_project_schedule("updated_schedule",new_schedule,initialize_planning_cache(new_schedule;t0=deepcopy(new_cache.t0),tF=deepcopy(new_cache.tF));mode=:leaf_aligned)
    # for v in new_schedule.root_nodes
    #     @show v, base_search_env.cache.tF[v]
    # end
    #
    # solution, assignment, cost, env  = high_level_search!(solver, base_search_env, Gurobi.Optimizer;
    #     cost_model=SumOfMakeSpans,
    # )
    #
    # print_project_schedule("final_schedule",env.schedule,env.cache;mode=:leaf_aligned)
    # @test validate(env.schedule, convert_to_vertex_lists(solution),env.cache.t0,env.cache.tF)
    # # for v in vertices(get_graph(env.schedule))
    # #     t0 = env.cache.t0[v]
    # #     tF = env.cache.tF[v]
    # #     t0_ = base_search_env.cache.t0[v]
    # #     tF_ = base_search_env.cache.tF[v]
    # #     if (t0 - t0_ != 0) || (tF - tF_ != 0)
    # #         node_id = get_vtx_id(env.schedule, v)
    # #         node = get_node_from_id(env.schedule)
    # #         @show node
    # #         @show v, t0-t0_, tF-tF_
    # #     end
    # # end


    # robot_paths = convert_to_vertex_lists(solution)
    # object_path_dict, object_interval_dict = fill_object_path_dicts!(solution,env.schedule,env.cache;
    #         object_path_dict = object_path_dict,
    #         object_interval_dict = object_interval_dict
    #     )
    # object_paths, object_intervals = convert_to_path_vectors(object_path_dict, object_interval_dict)
    # project_idxs = map(k->project_ids[k], sort(collect(keys(project_ids))))
    # color_options = ["black","brown"]
    # active_object_colors = map(k->color_options[project_ids[k]], sort(collect(keys(project_ids))))
    # object_paths, object_intervals, object_ids, path_idxs = get_object_paths(solution,env.schedule,env.cache)
    # for (path,interval,id,idx) in zip(object_paths, object_intervals, object_ids, path_idxs)
    #     if !haskey(object_path_dict,id)
    #         object_path_dict[id] = Vector{Vector{Int}}()
    #     end
    #     if length(object_path_dict[id]) >= idx
    #         object_path_dict[id][idx] = path
    #     else
    #         push!(object_path_dict[id], path)
    #     end
    #     object_interval_dict[id] = interval
    # end
    # object_paths = Vector{Vector{Int}}()
    # object_intervals = Vector{Vector{Int}}()
    # for (id,paths) in object_path_dict
    #     for path in paths
    #         push!(object_intervals, object_interval_dict[id])
    #         push!(object_paths, path)
    #     end
    # end

    #     end
    # end

    # Render video clip
    # tf = maximum(map(p->length(p),robot_paths))
    # set_default_plot_size(24cm,24cm)
    # record_video(joinpath(VIDEO_DIR,string("replanning2.webm")),
    #     t->render_paths(t,factory_env,robot_paths,object_paths;
    #         object_intervals=object_intervals,
    #         colors_vec=map(i->LCHab(60,80,200),1:length(robot_paths)),
    #         project_idxs=project_idxs,
    #         show_paths=false);tf=tf)

end

# Better graph plotting
let
    G = get_graph(project_schedule)
    traversal = topological_sort(G)
    initial_dict = Dict(
        :root_depth => (G,v)->0,
        :degree_in => (G,v)->0,
        :degree_out => (G,v)->0,
        :leaf_depth => (G,v)->0,
        :ancestors => (G,v)->Set([v]),
    )
    forward_dict = Dict(
        :root_depth => (G,v,v2,a,b)->max(a,b+1),
        :degree_in => (G,v,v2,a,b)->a+1,
        :ancestors => (G,v,v2,a,b)->union(a,b),
    )
    backward_dict = Dict(
        :leaf_depth => (G,v,v2,a,b)->max(a,b+1),
        :degree_out => (G,v,v2,a,b)->a+1,
    )
    vtx_vals = map(v->Dict{Symbol,Any}(),traversal)
    for v in traversal
        for (k,f) in initial_dict
            vtx_vals[v][k] = f(G,v)
        end
    end
    for v in traversal
        for v2 in inneighbors(G,v)
            for (k,f) in forward_dict
                vtx_vals[v][k] = f(G,v,v2,get(vtx_vals[v],k,0),get(vtx_vals[v2],k,0))
            end
        end
    end
    for v in reverse(traversal)
        for v2 in outneighbors(G,v)
            for (k,f) in backward_dict
                vtx_vals[v][k] = f(G,v,v2,get(vtx_vals[v],k,0),get(vtx_vals[v2],k,0))
            end
        end
    end

    # A = adjacency_matrix(G)
    # # min_x x * A * x' s.t.
    # model = Model(with_optimizer(Gurobi.Optimizer))
    # @variable(model, y[1:nv(G)])
    # x = ones(nv(G))
    # for v in vertices(G)
    #     @NLconstraint(model, x[v]^2 + y[v]^2 >= 2.0)
    # end
    # @objective(model,Min, y'*A*y)

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
