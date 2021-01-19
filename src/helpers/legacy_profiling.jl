module SolverProfiling

using LightGraphs, MetaGraphs
using DataStructures
using JuMP, MathOptInterface
using Gurobi
using TOML
using Random
using DataFrames
using SparseArrays

using GraphUtils
using CRCBS
using ..TaskGraphs


export
    init_data_frame,
    add_assignment_only_row!,
    add_low_level_search_row!,
    add_full_solver_row!,
    add_config_row!,
    add_assignment_only_row!,
    add_low_level_search_row!,
    add_full_solver_row!,
    add_replanner_row!,
    add_results_row!,
    construct_config_dataframe,
    construct_result_dataframe,
    construct_result_dataframes,

    profile_task_assignment,
    profile_low_level_search_and_repair,
    profile_low_level_search,
    profile_full_solver,
    read_assignment,
    read_sparse_matrix,
    run_profiling,
    compile_solver_results,
    run_replanner_profiling



function init_data_frame(;
        mode = :nothing,
        problem_dir = PROBLEM_DIR,
        results_dir = RESULTS_DIR
        )
    if (mode == :write) || (mode == :config)
        return DataFrame(
            problem_id = Int[],
            min_process_time = Int[],
            max_process_time = Int[],
            max_parents = Int[],
            M = Int[],
            N = Int[],
            depth_bias = Float64[]
            )
    elseif mode == :assignment_only
        return DataFrame(
            problem_id = Int[],
            time=Float64[],
            optimal=Bool[],
            optimality_gap              = Int[],
            feasible                    = Bool[],
            cost=Int[]
            )
    elseif mode == :low_level_search_without_repair
        return DataFrame(
            problem_id = Int[],
            valid_flag = Bool[],
            cost = Int[], # be sure to cast from Float64 to Int
            num_conflicts = Int[], # be sure to cast from Float64 to Int
            time = Float64[]
        )
    elseif mode == :low_level_search_with_repair
        return DataFrame(
            problem_id = Int[],
            valid_flag = Bool[],
            cost = Int[], # be sure to cast from Float64 to Int
            num_conflicts = Int[], # be sure to cast from Float64 to Int
            time = Float64[]
        )
    elseif mode == :full_solver
        return DataFrame(
            problem_id                  = Int[],
            time                        = Float64[],
            optimal                     = Bool[],
            optimality_gap              = Int[],
            feasible                    = Bool[],
            cost                        = Int[],
            # from solver
            total_assignment_iterations = Int[],
            total_CBS_iterations        = Int[],
            total_A_star_iterations     = Int[],
            max_CBS_iterations          = Int[],
            max_A_star_iterations       = Int[]
        )
    elseif mode == :replanning
        return DataFrame(
            problem_id                  = Int[],
            time                        = Vector{Float64}[],
            optimal                     = Vector{Bool}[],
            optimality_gap              = Vector{Int}[],
            feasible                    = Vector{Bool}[],
            fallback_feasible           = Vector{Bool}[],
            # cost                        = Vector{Int}[],
            final_time                  = Vector{Int}[],
            arrival_time                = Vector{Int}[],
            makespans                   = Vector{Int}[],
            # from solver
            # total_assignment_iterations = Int[],
            # total_CBS_iterations        = Int[],
            # total_A_star_iterations     = Int[],
            # max_CBS_iterations          = Int[],
            # max_A_star_iterations       = Int[]
        )
    elseif mode == :replanning_config
        return DataFrame(
            problem_id = Int[],
            min_process_time = Int[],
            max_process_time = Int[],
            max_parents = Int[],
            M = Int[],
            N = Int[],
            num_projects = Int[],
            arrival_interval = Int[],
            depth_bias = Float64[]
            )
    else
        return DataFrame()
    end
end
function add_config_row!(df,toml_dict,problem_id)
    push!(df, Dict(
        :problem_id         => problem_id,
        :min_process_time   => toml_dict["min_process_time"],
        :max_process_time   => toml_dict["max_process_time"],
        :max_parents        => toml_dict["max_parents"],
        :M                  => toml_dict["M"],
        :N                  => toml_dict["N"],
        :depth_bias         => toml_dict["depth_bias"]
    ))
end
function add_replanning_config_row!(df,toml_dict,problem_id)
    push!(df, Dict(
        :problem_id         => problem_id,
        :min_process_time   => toml_dict["min_process_time"],
        :max_process_time   => toml_dict["max_process_time"],
        :max_parents        => toml_dict["max_parents"],
        :arrival_interval   => toml_dict["arrival_interval"],
        :num_projects       => toml_dict["num_projects"],
        :M                  => toml_dict["M"],
        :N                  => toml_dict["N"],
        :depth_bias         => toml_dict["depth_bias"]
    ))
end
function add_assignment_only_row!(df,toml_dict,problem_id)
    push!(df, Dict(
        :problem_id         => problem_id,
        :time               => get(toml_dict, "time", -1),
        :optimal            => get(toml_dict, "optimal", false),
        :optimality_gap     => get(toml_dict, "optimality_gap", 10000),
        :feasible           => get(toml_dict, "feasible", false),
        :cost               => get(toml_dict, "cost", -1),
    ))
end
function add_low_level_search_row!(df,toml_dict,problem_id)
    push!(df, Dict(
        :problem_id     => problem_id,
        :time           => get(toml_dict, "time", -1),
        :cost           => Int(get(toml_dict, "cost", (-1,-1,-1,-1))[1]),
        :valid_flag     => get(toml_dict, "valid_flag", -1),
        :num_conflicts  => get(toml_dict, "num_conflicts", -1)
    ))
end
function add_full_solver_row!(df,toml_dict,problem_id)
    push!(df, Dict(
        :problem_id                 => problem_id,
        :time                       => get(toml_dict, "time", -1),
        :optimal                    => get(toml_dict, "optimal", false),
        :optimality_gap             => get(toml_dict, "optimality_gap", 10000),
        :feasible                   => get(toml_dict, "feasible", false),
        :cost                       => Int(get(toml_dict, "cost", (-1,-1,-1,-1))[1]),
        :total_assignment_iterations=> get(toml_dict, "total_assignment_iterations", -1),
        :total_CBS_iterations       => get(toml_dict, "total_CBS_iterations", -1),
        :total_A_star_iterations    => get(toml_dict, "total_A_star_iterations", -1),
        :max_CBS_iterations         => get(toml_dict, "max_CBS_iterations", -1),
        :max_A_star_iterations      => get(toml_dict, "max_A_star_iterations", -1)
    ))
end
function add_replanner_row!(df,toml_dict,problem_id)
    push!(df, Dict(
        :problem_id                 => problem_id,
        :time                       => get(toml_dict, "time", [-1]),
        :optimal                    => get(toml_dict, "optimal", [false]),
        :optimality_gap             => get(toml_dict, "optimality_gap", [10000]),
        :feasible                   => get(toml_dict, "feasible", [false]),
        :fallback_feasible          => get(toml_dict, "fallback_feasible", [false]),
        :final_time                 => get(toml_dict, "final_time", [1.0]),
        :arrival_time               => get(toml_dict, "arrival_time", [1.0]),
        :makespans                  => get(toml_dict, "makespans", [1.0]),
        # :cost                       => Int(get(toml_dict, "cost", (-1,-1,-1,-1))[1]),
        # :total_assignment_iterations=> get(toml_dict, "total_assignment_iterations", -1),
        # :total_CBS_iterations       => get(toml_dict, "total_CBS_iterations", -1),
        # :total_A_star_iterations    => get(toml_dict, "total_A_star_iterations", -1),
        # :max_CBS_iterations         => get(toml_dict, "max_CBS_iterations", -1),
        # :max_A_star_iterations      => get(toml_dict, "max_A_star_iterations", -1)
    ))
end
function add_results_row!(df,mode,toml_dict,problem_id)
    if mode == :assignment_only
        add_assignment_only_row!(df,toml_dict,problem_id)
    elseif mode == :low_level_search_without_repair
        add_low_level_search_row!(df,toml_dict,problem_id)
    elseif mode == :low_level_search_with_repair
        add_low_level_search_row!(df,toml_dict,problem_id)
    elseif mode == :full_solver
        add_full_solver_row!(df,toml_dict,problem_id)
    elseif mode == :replanning
        add_replanner_row!(df,toml_dict,problem_id)
    end
    df
end
function construct_result_dataframe(mode,problem_dir,results_dir,N_problems,path_function=(dir,mode,id)->joinpath(dir,string(mode),string("results",id,".toml")))
    df = init_data_frame(;mode=mode)
    for problem_id in 1:N_problems
        results_path = path_function(results_dir,mode,problem_id)
        if isfile(results_path)
            results_dict = TOML.parsefile(results_path)
            add_results_row!(df,mode,results_dict,problem_id)
        end
    end
    df
end
function construct_config_dataframe(problem_dir,N_problems,mode=:config,path_function=(dir,id)->joinpath(dir,string("config",id,".toml")))
    df = init_data_frame(;mode=mode)
    for problem_id in 1:N_problems
        config_path  = path_function(problem_dir,problem_id)
        if isfile(config_path)
            config_dict = TOML.parsefile(config_path)
            if mode == :config
                add_config_row!(df,config_dict,problem_id)
            elseif mode == :replanning_config
                add_replanning_config_row!(df,config_dict,problem_id)
            end
        end
    end
    df
end
function construct_result_dataframes(problem_dir,results_dir,N_problems)
    config_df = construct_config_dataframe(problem_dir,N_problems)
    modes = [:assignment_only,:low_level_search_without_repair,:low_level_search_with_repair,:full_solver]
    Dict(
        mode => join(config_df,
            construct_result_dataframe(mode, problem_dir,results_dir,N_problems),
            on = :problem_id, kind = :inner) for mode in modes
    )
end
function construct_replanner_result_dataframe(problem_dir,results_dir,N_problems)
    config_df = construct_config_dataframe(problem_dir,N_problems,:replanning_config,(dir,id)->joinpath(dir,string("stream",id,"/stream_config.toml")))
    modes = [:replanning]
    Dict(
        mode => join(config_df,
            construct_result_dataframe(mode, problem_dir,results_dir,N_problems,(dir,mode,id)->joinpath(dir,string("stream",id,"/results.toml"))),
            on = :problem_id, kind = :inner) for mode in modes
    )
end

function profile_task_assignment(solver, project_spec, problem_spec, robot_ICs, env_graph;
    feats = (RunTime(),SolutionCost(),OptimalityGap(),OptimalFlag(),FeasibleFlag()),
    kwargs...)

    project_schedule = construct_partial_project_schedule(project_spec, problem_spec, robot_ICs)
    base_search_env = construct_search_env(
        solver,
        project_schedule,
        problem_spec,
        env_graph
        )
    prob = PC_TAPF(base_search_env)
    assignment_problem = formulate_assignment_problem(assignment_solver(solver), prob;
        kwargs...)

    (schedule, l_bound), elapsed_time, _, _, _ = @timed solve_assignment_problem!(
                assignment_solver(solver),
                assignment_problem,
                prob;kwargs...)

    results = Dict{String,Any}()
    for feat in feats
        store_feature!(results,feat,extract_feature(solver,feat,schedule,elapsed_time))
    end
    assignment_matrix = get_assignment_matrix(assignment_problem)
    merge!(results, TOML.parse(sparse(assignment_matrix)))
    results
end
function profile_low_level_search_and_repair(solver, project_spec, problem_spec,
        robot_ICs, env_graph,adj_matrix;kwargs...)
    # solver = PC_TAPF_Solver(verbosity=0,LIMIT_A_star_iterations=10*nv(env_graph));
    project_schedule = construct_partial_project_schedule(project_spec, problem_spec, robot_ICs)
    base_search_env = construct_search_env(
        solver,
        project_schedule,
        problem_spec,
        env_graph
        )
    if update_project_schedule!(assignment_solver(solver).milp, project_schedule, problem_spec, adj_matrix)
        pc_mapf = PC_MAPF(env);
        node = initialize_root_node(pc_mapf)
        (search_env, valid_flag), elapsed_time, byte_ct, gc_time, mem_ct = @timed low_level_search!(solver, pc_mapf, node)

        cost = collect(get_cost(node.solution))
        if cost[1] == Inf
            cost = [-1 for c in cost]
        end
        results_dict = TOML.parse(solver)
        results_dict["time"] = elapsed_time
        results_dict["valid_flag"] = valid_flag
        results_dict["cost"] = cost
        results_dict["num_conflicts"] = count_conflicts(detect_conflicts(node.solution))
        return results_dict
    else
        println("update_project_schedule!() failed!")
        results_dict = TOML.parse(solver)
        results_dict["time"] = solver.time_limit
        results_dict["valid_flag"] = false
        results_dict["cost"] = 1000000
        results_dict["num_conflicts"] = -1
        return results_dict
    end
end
function profile_low_level_search(solver, project_spec, problem_spec, robot_ICs, env_graph,dist_matrix,adj_matrix;
    primary_objective=SumOfMakeSpans())

    # solver = PC_TAPF_Solver(verbosity=0,LIMIT_A_star_iterations=10*nv(env_graph));
    project_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
    if update_project_schedule!(solver.nbs_model, project_schedule, problem_spec, adj_matrix)
        env = construct_search_env(solver,project_schedule, problem_spec, env_graph;
            primary_objective=primary_objective); # TODO pass in t0_ here (maybe get it straight from model?)
        pc_mapf = PC_MAPF(env);
        node = initialize_root_node(pc_mapf)
        valid_flag, elapsed_time, byte_ct, gc_time, mem_ct = @timed compute_route_plan!(solver, get_env(pc_mapf), node)

        cost = collect(get_cost(node.solution))
        if cost[1] == Inf
            cost = [-1 for c in cost]
        end
        results_dict = TOML.parse(solver)
        results_dict["time"] = elapsed_time
        results_dict["valid_flag"] = valid_flag
        results_dict["cost"] = cost
        results_dict["num_conflicts"] = count_conflicts(detect_conflicts(node.solution))
        return results_dict
    else
        println("update_project_schedule!() failed!")
        results_dict = TOML.parse(solver)
        results_dict["time"] = solver.time_limit
        results_dict["valid_flag"] = false
        results_dict["cost"] = 1000000
        results_dict["num_conflicts"] = -1
        return results_dict
    end
end
function profile_full_solver(solver, project_spec, problem_spec, robot_ICs, env_graph,dist_matrix;
        primary_objective=SumOfMakeSpans(),save_paths=false,kwargs...)
    # Solve the problem
    # solver = PC_TAPF_Solver(verbosity=0,LIMIT_A_star_iterations=5*nv(env_graph));

    (solution, assignment, cost, search_env, optimality_gap), elapsed_time, byte_ct, gc_time, mem_ct = @timed high_level_search!(
        solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer;
        primary_objective=primary_objective,
        kwargs...);

    optimal = (optimality_gap <= 0)
    feasible = true
    if cost[1] == Inf
        cost = [-1 for c in cost]
        feasible = false
    end
    if optimality_gap == Inf
        optimality_gap = 10000
    end

    results_dict = TOML.parse(solver)
    merge!(results_dict, TOML.parse(sparse(assignment)))
    results_dict["time"] = elapsed_time
    # results_dict["assignment"] = assignment
    results_dict["optimality_gap"] = Int(optimality_gap)
    results_dict["optimal"] = optimal
    results_dict["feasible"] = feasible
    results_dict["cost"] = collect(cost)

    if feasible && save_paths
        robot_paths = convert_to_vertex_lists(solution)
        object_paths, object_intervals = get_object_paths(solution,search_env)
        results_dict["robot_paths"] = robot_paths
        results_dict["object_paths"] = object_paths
        results_dict["object_intervals"] = object_intervals
    end

    results_dict
end
function compile_solver_results(solver, solution, cost, search_env, optimality_gap, elapsed_time, t_arrival)
    optimal = (optimality_gap <= 0)
    feasible = true
    if cost[1] == Inf
        cost = [-1 for c in cost]
        feasible = false
    end
    if optimality_gap == Inf
        optimality_gap = 10000
    end
    dict = TOML.parse(solver)
    dict["time"] = elapsed_time
    dict["arrival_time"] = t_arrival
    dict["optimality_gap"] = Int(optimality_gap)
    dict["optimal"] = optimal
    dict["feasible"] = feasible
    dict["cost"] = collect(cost)
    dict
end
function profile_replanning(replan_model, fallback_model, solver, fallback_solver, project_list, env_graph, solver_config,problem_config;
        primary_objective=SumOfMakeSpans(), kwargs...
    )

    solver_template             = deepcopy(solver)
    fallback_solver_template    = deepcopy(fallback_solver)
    arrival_interval            = problem_config[:arrival_interval] # new project requests arrive every `arrival_interval` timesteps
    warning_time                = problem_config[:warning_time] # the project request arrives in the command center `warning_time` timesteps before the relevant objects become available
    commit_threshold            = problem_config[:commit_threshold] # freeze the current plan (route plan and schedule) at `t_arrival` + `commit_threshold`
    fallback_commit_threshold   = problem_config[:fallback_commit_threshold] # a tentative plan (computed with a fast heuristic) may take effect at t = t_arrival + fallback_commit_threshold--just in case the solver fails

    local_results = map(p->Dict{String,Any}(),project_list)
    final_times = map(p->Dict{AbstractID,Int}(),project_list)

    idx = 1
    t_arrival = 0
    def = project_list[idx]
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(def, env_graph;
        Δt_collect=map(i->get(problem_config,:dt_collect,0), def.s0),
        Δt_deposit=map(i->get(problem_config,:dt_deliver,0), def.s0),
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

    # print_project_schedule(string("schedule",idx,"B"),get_schedule(base_search_env);mode=:leaf_aligned)
    (solution, _, cost, search_env, optimality_gap), elapsed_time, byte_ct, gc_time, mem_ct = @timed high_level_search!(
        solver, base_search_env, Gurobi.Optimizer;primary_objective=primary_objective,kwargs...)
    # print_project_schedule(string("schedule",idx,"C"),get_schedule(search_env);mode=:leaf_aligned)
    local_results[idx] = compile_solver_results(solver, solution, cost, search_env, optimality_gap, elapsed_time, t_arrival)
    merge!(final_times[idx], Dict{AbstractID,Int}(OperationID(k)=>t_arrival for k in keys(get_operations(partial_schedule))))
    for i in 1:idx
        for node_id in keys(final_times[i])
            v = get_vtx(get_schedule(search_env),node_id)
            final_times[i][node_id] = max(final_times[i][node_id], get(get_cache(search_env).tF, v, -1))
        end
    end

    while idx < length(project_list)
        project_schedule = get_schedule(search_env)
        cache = get_cache(search_env)
        idx += 1
        t_request = arrival_interval * (idx - 1) # time when request reaches command center
        t_arrival = t_request + warning_time # time when objects become available
        # load next project
        def = project_list[idx]
        project_spec, problem_spec, _, _, _ = construct_task_graphs_problem(def, dist_matrix;
            Δt_collect=map(i->get(problem_config,:dt_collect,0), def.s0),
            Δt_deposit=map(i->get(problem_config,:dt_deliver,0), def.s0),
            cost_function=primary_objective,
            task_shapes=def.shapes,
            shape_dict=env_graph.expanded_zones
        )
        next_schedule = construct_partial_project_schedule(project_spec,problem_spec)
        remap_object_ids!(next_schedule,project_schedule)
        # print_project_schedule(string("next_schedule",idx),next_schedule;mode=:leaf_aligned)
        # store solution for plotting
        object_path_dict, object_interval_dict = fill_object_path_dicts!(solution,project_schedule,cache,object_path_dict,object_interval_dict)
        for (object_id, pred) in get_object_ICs(next_schedule)
            project_ids[get_id(object_id)] = idx
        end
        # Fallback
        println("Computing Fallback plan at idx = ",idx)
        fallback_solver = PC_TAPF_Solver(fallback_solver_template,start_time=time())
        # reset_solver!(fallback_solver)
        fallback_search_env, fallback_solver = replan!(fallback_solver, fallback_model, search_env, env_graph, problem_spec, solution, next_schedule, t_request, t_arrival;commit_threshold=fallback_commit_threshold)
        (fallback_solution, _, fallback_cost, fallback_search_env, fallback_optimality_gap), fallback_elapsed_time, _, _, _ = @timed high_level_search!(
            fallback_solver, fallback_search_env, Gurobi.Optimizer;primary_objective=primary_objective,kwargs...)

        # print_project_schedule(string("schedule",idx,"A"),fallback_search_env;mode=:leaf_aligned)
        # Replanning outer loop
        println("Computing plan at idx = ",idx)
        solver = PC_TAPF_Solver(solver_template,start_time=time())
        # reset_solver!(solver)
        base_search_env, solver = replan!(solver, replan_model, search_env, env_graph, problem_spec, solution, next_schedule, t_request, t_arrival; commit_threshold=commit_threshold,kwargs...)
        (solution, _, cost, search_env, optimality_gap), elapsed_time, _, _, _ = @timed high_level_search!(solver, base_search_env, Gurobi.Optimizer;primary_objective=primary_objective,kwargs...)
        # print_project_schedule(string("schedule",idx,"C"),search_env;mode=:leaf_aligned)
        # m = maximum(map(p->length(p), get_paths(solution)))
        # @show

        fallback_results = compile_solver_results(fallback_solver, fallback_solution, fallback_cost, fallback_search_env, fallback_optimality_gap, fallback_elapsed_time, t_arrival)
        local_results[idx] = compile_solver_results(solver, solution, cost, search_env, optimality_gap, elapsed_time, t_arrival)
        merge!(local_results[idx],Dict(string("fallback_",k)=>v for (k,v) in fallback_results))
        # Switch to fallback if necessary
        if (elapsed_time > commit_threshold) || ~local_results[idx]["feasible"]
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
                v = get_vtx(get_schedule(search_env),node_id)
                final_times[i][node_id] = max(final_times[i][node_id], get(get_cache(search_env).tF, v, -1))
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

    if cost[1] < Inf && get(problem_config, :save_paths, true)
        println("SAVING PATHS")
        object_path_dict, object_interval_dict = fill_object_path_dicts!(solution,get_schedule(search_env),get_cache(search_env),object_path_dict,object_interval_dict)
        object_paths, object_intervals = convert_to_path_vectors(object_path_dict, object_interval_dict)

        results_dict["robot_paths"]         = convert_to_vertex_lists(solution)
        results_dict["object_paths"]        = object_paths
        results_dict["object_intervals"]    = object_intervals
        results_dict["project_idxs"]        = map(k->project_ids[k], sort(collect(keys(project_ids))))
    end
    results_dict
end

function run_profiling(solver,MODE=:nothing;
        solver_config = Dict(),
        base_problem_dir = get(solver_config,:problem_dir, PROBLEM_DIR),
        base_results_dir = get(solver_config,:results_dir, RESULTS_DIR),
        problem_configs = Vector{Dict}(),
        OutputFlag      = get(solver_config,:OutputFlag,    0),
        Presolve        = get(solver_config,:Presolve,      -1),
        initial_problem_id = 1,
        primary_objective = SumOfMakeSpans(),
        seed = 1,
        optimizer=Gurobi.Optimizer
        )
    if !isdir(base_problem_dir)
        mkdir(base_problem_dir)
    end
    env_ids = Set(map(config->config[:env_id],problem_configs))
    envs = Dict(i=>read_env(joinpath(ENVIRONMENT_DIR,"env_$i.toml")) for i in env_ids)

    problem_id = initial_problem_id-1;
    Random.seed!(seed)
    for problem_config in problem_configs
        num_trials          = get(problem_config,:num_trials,1)
        factory_env = envs[problem_config[:env_id]]
        for trial in num_trials
            # try
            problem_id += 1 # moved this to the beginning so it doesn't get skipped
            problem_filename = joinpath(base_problem_dir,"problem$problem_id.toml")
            config_filename = joinpath(base_problem_dir,"config$problem_id.toml")
            if MODE == :write
                if isfile(problem_filename)
                    println("file ",problem_filename," already exists. Skipping ...")
                    continue # don't overwrite existing files
                end
                problem_def = instantiate_random_pctapf_def(factory_env,problem_config)
                open(problem_filename, "w") do io
                    TOML.print(io, TOML.parse(problem_def))
                end
                open(config_filename, "w") do io
                    TOML.print(io,to_string_dict(problem_config))
                end
            elseif MODE != :nothing
                subdir = joinpath(base_results_dir,string(MODE))
                results_filename = joinpath(subdir,string("results",problem_id,".toml"))
                if !isdir(subdir)
                    mkpath(subdir)
                end
                if isfile(results_filename)
                    println("file ",results_filename," already exists. Skipping ...")
                    continue # don't overwrite existing files
                end
                assignment_filename = joinpath(base_results_dir,string(:assignment_only),string("results",problem_id,".toml"))
                problem_def = read_problem_def(problem_filename)
                project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
                    problem_def,
                    factory_env;
                    Δt_collect=map(i->get(problem_config,:dt_collect,0), problem_def.s0),
                    Δt_deposit=map(i->get(problem_config,:dt_deliver,0), problem_def.s0),
                    cost_function=primary_objective,
                    );

                hard_reset_solver!(solver)
                if MODE == :assignment_only
                    results_dict = profile_task_assignment(solver, project_spec, problem_spec, robot_ICs, factory_env;
                        primary_objective=primary_objective,OutputFlag=OutputFlag,Presolve=Presolve)
                end
                if MODE == :low_level_search_without_repair
                    adj_matrix = read_sparse_matrix(assignment_filename)
                    results_dict = profile_low_level_search(solver, project_spec, problem_spec, robot_ICs,factory_env,dist_matrix,adj_matrix;
                        primary_objective=primary_objective)
                end
                if MODE == :low_level_search_with_repair
                    adj_matrix = read_sparse_matrix(assignment_filename)
                    results_dict = profile_low_level_search_and_repair(solver, project_spec, problem_spec, robot_ICs,factory_env,dist_matrix,adj_matrix;
                        primary_objective=primary_objective)
                end
                if MODE == :full_solver
                    results_dict = profile_full_solver(solver, project_spec, problem_spec, robot_ICs,factory_env,dist_matrix;
                        primary_objective=primary_objective,TimeLimit=TimeLimit,OutputFlag=OutputFlag,Presolve=Presolve)
                end
                println("PROFILER: ",typeof(solver)," --- ",string(MODE), " --- Solved problem ",problem_id," in ",results_dict["RunTime"]," seconds! \n\n")
                open(results_filename, "w") do io
                    TOML.print(io, results_dict)
                end
            end
            # catch e
            #     if typeof(e) <: AssertionError
            #         println(e.msg)
            #     else
            #         throw(e)
            #     end
            # end
        end
    end
end


function run_replanner_profiling(MODE=:nothing;
        solver_config = Dict(),
        base_problem_dir = get(solver_config,:problem_dir, PROBLEM_DIR),
        base_results_dir = get(solver_config,:results_dir, RESULTS_DIR),
        replan_model = get(solver_config,:replan_model,  MergeAndBalance()),
        fallback_model = get(solver_config,:fallback_model,ReassignFreeRobots()),
        env_id = get(solver_config,:env_id,2),
        problem_configs=Vector{Dict}(),
        solver = NBSSolver(), # TODO Remove templates so we just have to call reset! instead of initializing new solvers.
        fallback_solver = NBSSolver(),
        OutputFlag      = get(solver_config,:OutputFlag,    0),
        Presolve        = get(solver_config,:Presolve,      -1),
        initial_problem_id = 1,
        primary_objective = SumOfMakeSpans(),
        time_out_buffer = 1
    )
    # solver profiling
    env_ids = Set(map(config->config[:env_id],problem_configs))
    envs = Dict(i=>read_env(joinpath(ENVIRONMENT_DIR,"env_$i.toml")) for i in env_ids)

    Random.seed!(1)
    folder_id = initial_problem_id-1;
    # run tests and push results into table
    for problem_config in problem_configs
        num_trials          = get(problem_config,:num_trials,1)
        factory_env = envs[problem_config[:env_id]]
        for trial in 1:num_trials
        # try
            folder_id += 1 # moved this to the beginning so it doesn't get skipped
            problem_dir = joinpath(base_problem_dir,string("stream",folder_id))
            if MODE == :write
                if !isdir(problem_dir)
                    mkpath(problem_dir)
                elseif length(readdir(problem_dir)) > 1
                    println("directory ",problem_dir," appears to be full.",
                        " Skipping write to avoid overwriting files.")
                    continue
                end
                config_filename = joinpath(problem_dir,string("stream_config.toml"))
                open(config_filename, "w") do io
                    TOML.print(io,problem_config)
                end
                project_sequence = instantiate_random_pctapf_sequence(factory_env,config)
                for (problem_id,problem_def) in enumerate(project_sequence)
                    problem_filename = joinpath(problem_dir,string("problem",problem_id,".toml"))
                    config_filename = joinpath(problem_dir,string("config",problem_id,".toml"))
                    if isfile(problem_filename)
                        println("file ",problem_filename," already exists. Skipping write for this and all other problem files")
                        already_written=true
                        break
                    end
                    # Save the problem
                    open(problem_filename, "w") do io
                        TOML.print(io, TOML.parse(problem_def))
                    end
                    open(config_filename, "w") do io
                        TOML.print(io,problem_config)
                    end
                end
            elseif mode != nothing
                subdir = joinpath(base_results_dir,string("stream",folder_id))
                results_filename = joinpath(subdir,string("results.toml"))
                if !isdir(subdir)
                    mkpath(subdir)
                end
                if isfile(results_filename)
                    println("file ",results_filename," already exists. Skipping ...")
                    continue # don't overwrite existing files
                end
                fnames = map(i->joinpath(problem_dir,string("problem",i,".toml")))
                project_list = map(fname->read_problem_def(fname),fnames)

                hard_reset_solver!(solver)
                hard_reset_solver!(fallback_solver)

                results_dict = profile_replanning(
                    replan_model,
                    fallback_model,
                    solver,
                    fallback_solver,
                    project_list,
                    factory_env,
                    solver_config,
                    problem_config;
                    primary_objective = primary_objective,
                    OutputFlag = OutputFlag,
                    Presolve = Presolve
                )

                println("PROFILER: ",
                    typeof(replan_model),"-",typeof(fallback_model)," --- ",
                    string(MODE), " --- Solved problem ",folder_id," in ",
                    results_dict["time"]," seconds! \n\n")
                # print the results
                open(results_filename, "w") do io
                    TOML.print(io, results_dict)
                end
            end
        # catch e
        #     if typeof(e) <: AssertionError
        #         println(e.msg)
        #     else
        #         throw(e)
        #     end
        # end
        end
    end
end

export
    get_problem_config_1,
    get_problem_config_2,
    get_replanning_config_1,
    get_replanning_config_2,
    get_replanning_config_3,
    get_replanning_config_4,
    get_replanning_config_5,
    get_replanning_config_6

function get_problem_config_1()
    base_solver_configs = [
        Dict(
        :env_id=>2,
        :OutputFlag => 0,
        :Presolve => -1,
        ),
    ]
    solver_configs = Dict[]
    for dicts in Base.Iterators.product(base_solver_configs)
        push!(solver_configs,merge(dicts...))
    end
    task_configs = map(i->Dict(:num_tasks=>i),[10,20])
    robot_configs = map(i->Dict(:num_tasks=>i),[10,20])
    depth_bias_configs = map(i->Dict(:depth_bias=>i),[0.4])
    base_configs = [
        Dict(
            :num_trials => 1,
            :max_parents => 3,
            :dt_min => 0,
            :dt_max => 0,
            :dt_collect => 0,
            :dt_deliver => 0,
            :task_sizes => (1=>1.0,2=>0.0,4=>0.0),
            :save_paths => false,
            :env_id=>2,
            )
    ]
    problem_configs = Dict[]
    for dicts in Base.Iterators.product(base_configs,task_configs,robot_configs,depth_bias_configs)
        push!(problem_configs,merge(dicts...))
    end

    solvers = [
        NBSSolver(),
    ]

    base_dir            = joinpath("/scratch/task_graphs_experiments")
    base_problem_dir    = joinpath(base_dir,"problem_instances")
    base_results_dir    = joinpath(base_dir,"results")

    base_problem_dir, base_results_dir, solver_configs, problem_configs, solvers
end
function get_problem_config_2()
    base_solver_configs = [
        Dict(
        :env_id=>2,
        :OutputFlag => 0,
        :Presolve => -1,
        ),
    ]
    solver_configs = Dict[]
    for dicts in Base.Iterators.product(base_solver_configs)
        push!(solver_configs,merge(dicts...))
    end
    task_configs = map(i->Dict(:num_tasks=>i),[10,20,30,40,50,60])
    robot_configs = map(i->Dict(:num_tasks=>i),[10,20,30,40])
    depth_bias_configs = map(i->Dict(:depth_bias=>i),[0.1,0.4,0.7,1.0])
    base_configs = [
        Dict(
            :num_trials => 4,
            :max_parents => 3,
            :dt_min => 0,
            :dt_max => 0,
            :dt_collect => 0,
            :dt_deliver => 0,
            :task_sizes => (1=>1.0,2=>0.0,4=>0.0),
            :save_paths => false,
            :env_id=>2,
            )
    ]
    problem_configs = Dict[]
    for dicts in Base.Iterators.product(base_configs,task_configs,robot_configs,depth_bias_configs)
        push!(problem_configs,merge(dicts...))
    end

    solvers = [
        NBSSolver(),
    ]

    base_dir            = joinpath("/scratch/task_graphs_experiments")
    base_problem_dir    = joinpath(base_dir,"problem_instances")
    base_results_dir    = joinpath(base_dir,"results")

    base_problem_dir, base_results_dir, solver_configs, problem_configs, solvers
end
function get_replanning_config_1(base_dir=".")
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
            :save_paths => false,
            :env_id=>2,
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

    base_dir            = joinpath("/scratch/task_graphs_experiments","replanning")
    base_problem_dir    = joinpath(base_dir,"problem_instances")
    base_results_dir    = joinpath(base_dir,"results")

    base_problem_dir, base_results_dir, problem_configs
end
function get_replanning_config_2()
    base_solver_configs = [
        Dict(
        :nbs_time_limit=>8,
        :route_planning_buffer=>2,
        :env_id=>2,
        :OutputFlag => 0,
        :Presolve => -1,
        ),
    ]
    fallback_configs = [
        Dict(:fallback_model=>ReassignFreeRobots(),),
    ]
    replan_configs = [
        Dict(:replan_model=>MergeAndBalance(),),
        Dict(:replan_model=>Oracle(),:time_out_buffer=>-55,:route_planning_buffer=>5),
        Dict(:replan_model=>ReassignFreeRobots(),),
        Dict(:replan_model=>DeferUntilCompletion(),),
    ]
    solver_configs = Dict[]
    for dicts in Base.Iterators.product(base_solver_configs,fallback_configs,replan_configs)
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
            :num_projects => 40,
            :save_paths => false,
            :env_id=>2,
            )
    ]
    stream_configs = [
        Dict(:N=>30, :M=>10,  :arrival_interval=>20, ),
        Dict(:N=>30, :M=>10,  :arrival_interval=>30, ),
        Dict(:N=>30, :M=>10,  :arrival_interval=>40, ),

        Dict(:N=>30, :M=>20,  :arrival_interval=>40, ),
        Dict(:N=>30, :M=>20,  :arrival_interval=>50, ),
        Dict(:N=>30, :M=>20,  :arrival_interval=>60, ),

        Dict(:N=>30, :M=>30,  :arrival_interval=>60, ),
        Dict(:N=>30, :M=>30,  :arrival_interval=>70, ),
        Dict(:N=>30, :M=>30,  :arrival_interval=>80, ),
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

    base_dir            = joinpath("/scratch/task_graphs_experiments","replanning/long_problems")
    base_problem_dir    = joinpath(base_dir,"problem_instances")
    base_results_dir    = joinpath(base_dir,"results")

    base_problem_dir, base_results_dir, solver_configs, problem_configs, solver_template, fallback_solver_template
end
function get_replanning_config_3()
    base_solver_configs = [
        Dict(
        :env_id=>2,
        :OutputFlag => 0,
        :Presolve => -1,
        ),
    ]
    fallback_configs = [
        Dict(:fallback_model=>ReassignFreeRobots(),),
    ]
    replan_configs = [
        Dict(:replan_model=>MergeAndBalance(),),
        Dict(:replan_model=>Oracle(),:time_out_buffer=>-55,:route_planning_buffer=>5),
        Dict(:replan_model=>ReassignFreeRobots(),),
        Dict(:replan_model=>DeferUntilCompletion(),),
    ]
    solver_configs = Dict[]
    for dicts in Base.Iterators.product(base_solver_configs,fallback_configs,replan_configs)
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
            :num_projects => 40,
            :save_paths => false,
            :env_id=>2,
            )
    ]
    stream_configs = [
        Dict(:N=>30, :M=>10,  :arrival_interval=>20, ),
        Dict(:N=>30, :M=>10,  :arrival_interval=>30, ),
        Dict(:N=>30, :M=>10,  :arrival_interval=>40, ),

        Dict(:N=>30, :M=>20,  :arrival_interval=>40, ),
        Dict(:N=>30, :M=>20,  :arrival_interval=>50, ),
        Dict(:N=>30, :M=>20,  :arrival_interval=>60, ),

        Dict(:N=>30, :M=>30,  :arrival_interval=>60, ),
        Dict(:N=>30, :M=>30,  :arrival_interval=>70, ),
        Dict(:N=>30, :M=>30,  :arrival_interval=>80, ),
    ]
    problem_configs = Dict[]
    for dicts in Base.Iterators.product(base_configs,stream_configs)
        push!(problem_configs,merge(dicts...))
    end

    solver_template = PC_TAPF_Solver(
        nbs_model                   = SparseAdjacencyMILP(),
        astar_model                 = AStarPathFinderModel(replan=true),
        DEBUG                       = true,
        l1_verbosity                = 1,
        l2_verbosity                = 1,
        l3_verbosity                = 0,
        l4_verbosity                = 0,
        LIMIT_assignment_iterations = 10,
        LIMIT_A_star_iterations     = 8000,
        best_cost                   = (Inf,Inf,Inf,Inf,Inf),
        );
    fallback_solver_template = PC_TAPF_Solver(
        nbs_model                   = GreedyAssignment(),
        astar_model                 = AStarPathFinderModel(replan=true),
        DEBUG                       = true,
        l1_verbosity                = 1,
        l2_verbosity                = 1,
        l3_verbosity                = 0,
        l4_verbosity                = 0,
        LIMIT_assignment_iterations = 2,
        LIMIT_A_star_iterations     = 8000,
        best_cost                   = (Inf,Inf,Inf,Inf,Inf),
        );

    base_dir            = joinpath("/scratch/task_graphs_experiments","replanning/better_fallback")
    base_problem_dir    = joinpath(base_dir,"problem_instances")
    base_results_dir    = joinpath(base_dir,"results")

    base_problem_dir, base_results_dir, solver_configs, problem_configs, solver_template, fallback_solver_template
end
function get_replanning_config_4()
    base_solver_configs = [
        Dict(
        :env_id=>2,
        :OutputFlag => 0,
        :Presolve => -1,
        ),
    ]
    fallback_configs = [
        Dict(:fallback_model=>ReassignFreeRobots(),),
    ]
    replan_configs = [
        Dict(:replan_model=>MergeAndBalance(),),
        Dict(:replan_model=>Oracle(),:time_out_buffer=>-55,:route_planning_buffer=>5),
        Dict(:replan_model=>ReassignFreeRobots(),),
        Dict(:replan_model=>DeferUntilCompletion(),),
    ]
    solver_configs = Dict[]
    for dicts in Base.Iterators.product(base_solver_configs,fallback_configs,replan_configs)
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
            :num_projects => 40,
            :save_paths => false,
            :env_id=>2,
            )
    ]
    stream_configs = [
        Dict(:N=>30, :M=>10,  :arrival_interval=>20, ),
        Dict(:N=>30, :M=>10,  :arrival_interval=>30, ),
        Dict(:N=>30, :M=>10,  :arrival_interval=>40, ),

        Dict(:N=>30, :M=>20,  :arrival_interval=>40, ),
        Dict(:N=>30, :M=>20,  :arrival_interval=>50, ),
        Dict(:N=>30, :M=>20,  :arrival_interval=>60, ),

        Dict(:N=>30, :M=>30,  :arrival_interval=>60, ),
        Dict(:N=>30, :M=>30,  :arrival_interval=>70, ),
        Dict(:N=>30, :M=>30,  :arrival_interval=>80, ),
    ]
    problem_configs = Dict[]
    for dicts in Base.Iterators.product(base_configs,stream_configs)
        push!(problem_configs,merge(dicts...))
    end

    solver_template = PC_TAPF_Solver(
        nbs_model                   = SparseAdjacencyMILP(),
        astar_model                 = AStarPathFinderModel(replan=true),
        DEBUG                       = true,
        l1_verbosity                = 1,
        l2_verbosity                = 1,
        l3_verbosity                = 0,
        l4_verbosity                = 0,
        LIMIT_assignment_iterations = 10,
        LIMIT_A_star_iterations     = 8000,
        best_cost                   = (Inf,Inf,Inf,Inf,Inf),
        );
    fallback_solver_template = PC_TAPF_Solver(
        nbs_model                   = GreedyAssignment(),
        astar_model                 = PrioritizedAStarModel(replan=true),
        DEBUG                       = true,
        l1_verbosity                = 1,
        l2_verbosity                = 1,
        l3_verbosity                = 0,
        l4_verbosity                = 0,
        LIMIT_assignment_iterations = 2,
        LIMIT_A_star_iterations     = 8000,
        best_cost                   = (Inf,Inf,Inf,Inf,Inf),
        );

    base_dir            = joinpath("/scratch/task_graphs_experiments","replanning/prioritized_planner_fallback")
    base_problem_dir    = joinpath(base_dir,"problem_instances")
    base_results_dir    = joinpath(base_dir,"results")

    base_problem_dir, base_results_dir, solver_configs, problem_configs, solver_template, fallback_solver_template
end
function get_replanning_config_5()
    base_solver_configs = [
        Dict(
        :env_id=>2,
        :OutputFlag => 0,
        :Presolve => -1,
        ),
    ]
    fallback_configs = [
        Dict(:fallback_model=>ReassignFreeRobots(),),
    ]
    replan_configs = [
        Dict(:replan_model=>MergeAndBalance(),),
        Dict(:replan_model=>Oracle(),:time_out_buffer=>-55,:route_planning_buffer=>5),
        Dict(:replan_model=>ReassignFreeRobots(),),
        Dict(:replan_model=>DeferUntilCompletion(),),
    ]
    solver_configs = Dict[]
    for dicts in Base.Iterators.product(base_solver_configs,fallback_configs,replan_configs)
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
            :num_projects => 40,
            :save_paths => false,
            :env_id=>2,
            )
    ]
    stream_configs = [
        Dict(:N=>30, :M=>10,  :arrival_interval=>20, ),
        Dict(:N=>30, :M=>10,  :arrival_interval=>30, ),
        Dict(:N=>30, :M=>10,  :arrival_interval=>40, ),

        Dict(:N=>30, :M=>20,  :arrival_interval=>40, ),
        Dict(:N=>30, :M=>20,  :arrival_interval=>50, ),
        Dict(:N=>30, :M=>20,  :arrival_interval=>60, ),

        Dict(:N=>30, :M=>30,  :arrival_interval=>60, ),
        Dict(:N=>30, :M=>30,  :arrival_interval=>70, ),
        Dict(:N=>30, :M=>30,  :arrival_interval=>80, ),
    ]
    problem_configs = Dict[]
    for dicts in Base.Iterators.product(base_configs,stream_configs)
        push!(problem_configs,merge(dicts...))
    end

    solver_template = PC_TAPF_Solver(
        nbs_model                   = SparseAdjacencyMILP(),
        astar_model                 = AStarPathFinderModel(replan=true),
        DEBUG                       = true,
        l1_verbosity                = 1,
        l2_verbosity                = 1,
        l3_verbosity                = 0,
        l4_verbosity                = 0,
        LIMIT_assignment_iterations = 10,
        LIMIT_A_star_iterations     = 8000,
        );
    fallback_solver_template = PC_TAPF_Solver(
        nbs_model                   = GreedyAssignment(),
        cbs_model                   = PrioritizedDFSPlanner(max_iters=2000),
        astar_model                 = DFS_PathFinder(),
        DEBUG                       = true,
        l1_verbosity                = 1,
        l2_verbosity                = 1,
        l3_verbosity                = 0,
        l4_verbosity                = 0,
        LIMIT_assignment_iterations = 1,
        );

    base_dir            = joinpath("/scratch/task_graphs_experiments","replanning/prioritized_dfs_fallback")
    base_problem_dir    = joinpath(base_dir,"problem_instances")
    base_results_dir    = joinpath(base_dir,"results")

    base_problem_dir, base_results_dir, solver_configs, problem_configs, solver_template, fallback_solver_template
end
function get_replanning_config_6()
    base_solver_configs = [
        Dict(
        :env_id=>2,
        :OutputFlag => 0,
        :Presolve => -1,
        ),
    ]
    fallback_configs = [
        Dict(:fallback_model=>ReassignFreeRobots(),),
    ]
    replan_configs = [
        Dict(:replan_model=>MergeAndBalance(),),
        Dict(:replan_model=>Oracle(),:time_out_buffer=>-55,:route_planning_buffer=>5),
        Dict(:replan_model=>ReassignFreeRobots(),),
        Dict(:replan_model=>DeferUntilCompletion(),),
    ]
    solver_configs = Dict[]
    for dicts in Base.Iterators.product(base_solver_configs,fallback_configs,replan_configs)
        push!(solver_configs,merge(dicts...))
    end
    base_configs = [
        Dict(
            :warning_time=>20,
            :commit_threshold=>20,
            :fallback_commit_threshold=>20,
            :num_trials => 4,
            :max_parents => 3,
            :depth_bias => 0.4,
            :dt_min => 0,
            :dt_max => 0,
            :dt_collect => 0,
            :dt_deliver => 0,
            :task_sizes => (1=>1.0,2=>0.0,4=>0.0),
            :num_projects => 40,
            :save_paths => false,
            :env_id=>2,
            )
    ]
    stream_configs = [
        Dict(:N=>30, :M=>10,  :arrival_interval=>20, ),
        Dict(:N=>30, :M=>10,  :arrival_interval=>30, ),
        Dict(:N=>30, :M=>10,  :arrival_interval=>40, ),

        Dict(:N=>30, :M=>20,  :arrival_interval=>40, ),
        Dict(:N=>30, :M=>20,  :arrival_interval=>50, ),
        Dict(:N=>30, :M=>20,  :arrival_interval=>60, ),

        Dict(:N=>30, :M=>30,  :arrival_interval=>60, ),
        Dict(:N=>30, :M=>30,  :arrival_interval=>70, ),
        Dict(:N=>30, :M=>30,  :arrival_interval=>80, ),
    ]
    problem_configs = Dict[]
    for dicts in Base.Iterators.product(base_configs,stream_configs)
        push!(problem_configs,merge(dicts...))
    end

    solver_template = PC_TAPF_Solver(
        nbs_model                   = SparseAdjacencyMILP(),
        astar_model                 = AStarPathFinderModel(replan=true),
        DEBUG                       = true,
        l1_verbosity                = 1,
        l2_verbosity                = 1,
        l3_verbosity                = 0,
        l4_verbosity                = 0,
        LIMIT_assignment_iterations = 10,
        LIMIT_A_star_iterations     = 8000,
        );
    fallback_solver_template = PC_TAPF_Solver(
        nbs_model                   = GreedyAssignment(),
        astar_model                 = PrioritizedAStarModel(replan=true),
        DEBUG                       = true,
        l1_verbosity                = 1,
        l2_verbosity                = 1,
        l3_verbosity                = 0,
        l4_verbosity                = 0,
        LIMIT_assignment_iterations = 2,
        LIMIT_A_star_iterations     = 8000,
        );

    base_dir            = joinpath("/scratch/task_graphs_experiments","replanning/prioritized_planner_fallback_longer_commit")
    base_problem_dir    = joinpath(base_dir,"problem_instances")
    base_results_dir    = joinpath(base_dir,"results")

    base_problem_dir, base_results_dir, solver_configs, problem_configs, solver_template, fallback_solver_template
end

end
