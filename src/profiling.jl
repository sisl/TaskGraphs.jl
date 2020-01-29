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
    run_profiling


function read_assignment(toml_dict::Dict)
    assignment = toml_dict["assignment"]
end
function read_assignment(io)
    read_assignment(TOML.parsefile(io))
end
function TOML.parse(mat::SparseMatrixCSC{Int64,Int64})
    toml_dict = Dict(
        "edge_list" => map(idx->[idx[1],idx[2]], findall(!iszero, mat)),
        "size_i" => size(mat,1),
        "size_j" => size(mat,2)
        )
end
function read_sparse_matrix(toml_dict::Dict)
    edge_list   = toml_dict["edge_list"]
    size_i      = toml_dict["size_i"]
    size_j      = toml_dict["size_j"]
    mat = sparse(zeros(size_i,size_j))
    for e in edge_list
        mat[e[1],e[2]] = 1
    end
    mat
end
function read_sparse_matrix(io)
    read_sparse_matrix(TOML.parsefile(io))
end

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
            problem_id = Int[],
            time = Float64[],
            optimal = Bool[],
            cost = Int[],
            # from solver
            total_assignment_iterations = Int[],
            total_CBS_iterations        = Int[],
            total_A_star_iterations     = Int[],
            max_CBS_iterations          = Int[],
            max_A_star_iterations       = Int[]
        )
    else
        return DataFrame()
    end
end
function add_config_row!(df,toml_dict,problem_id)
    push!(df, Dict(
        :problem_id => problem_id,
        :min_process_time => toml_dict["min_process_time"],
        :max_process_time => toml_dict["max_process_time"],
        :max_parents => toml_dict["max_parents"],
        :M => toml_dict["M"],
        :N => toml_dict["N"],
        :depth_bias => toml_dict["depth_bias"]
    ))
end
function add_assignment_only_row!(df,toml_dict,problem_id)
    push!(df, Dict(
        :problem_id => problem_id,
        :time       => toml_dict["time"],
        :optimal    => toml_dict["optimal"],
        :cost       => toml_dict["cost"],
    ))
end
function add_low_level_search_row!(df,toml_dict,problem_id)
    push!(df, Dict(
        :problem_id     => problem_id,
        :time           => toml_dict["time"],
        :cost           => Int(toml_dict["cost"][1]),
        :valid_flag     => toml_dict["valid_flag"],
        :num_conflicts  => get(toml_dict["num_conflicts"], 100)
    ))
end
function add_full_solver_row!(df,toml_dict,problem_id)
    push!(df, Dict(
        :problem_id     => problem_id,
        :time           => toml_dict["time"],
        :optimal           => toml_dict["optimal"],
        :cost           => Int(toml_dict["cost"][1]),
        :total_assignment_iterations=> toml_dict["total_assignment_iterations"],
        :total_CBS_iterations       => toml_dict["total_CBS_iterations"],
        :total_A_star_iterations    => toml_dict["total_A_star_iterations"],
        :max_CBS_iterations         => toml_dict["max_CBS_iterations"],
        :max_A_star_iterations      => toml_dict["max_A_star_iterations"]
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
    end
    df
end
function construct_result_dataframe(mode,problem_dir,results_dir,N_problems)
    df = init_data_frame(;mode=mode)
    for problem_id in 1:N_problems
        results_path = joinpath(results_dir,string(mode),string("results",problem_id,".toml"))
        if isfile(results_path)
            results_dict = TOML.parsefile(results_path)
            add_results_row!(df,mode,results_dict,problem_id)
        end
    end
    df
end
function construct_config_dataframe(problem_dir,N_problems)
    df = init_data_frame(;mode=:config)
    for problem_id in 1:N_problems
        config_path  = joinpath(problem_dir,string("config",problem_id,".toml"))
        if isfile(config_path)
            config_dict = TOML.parsefile(config_path)
            add_config_row!(df,config_dict,problem_id)
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

function profile_task_assignment(problem_def::SimpleProblemDef,env_graph,dist_matrix;
    milp_model=AssignmentMILP(),kwargs...)
    project_spec, r0, s0, sF = problem_def.project_spec,problem_def.r0,problem_def.s0,problem_def.sF
    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(
            project_spec, r0, s0, sF, dist_matrix);

    project_schedule = construct_partial_project_schedule(project_spec, problem_spec, robot_ICs)
    model = formulate_milp(milp_model,project_schedule,problem_spec)

    # model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;kwargs...);
    retval, elapsed_time, byte_ct, gc_time, mem_ct = @timed optimize!(model)

    optimal = (termination_status(model) == MathOptInterface.OPTIMAL);
    # assignment_matrix = Matrix{Int}(round.(value.(model[:x])));
    assignment_matrix = get_assignment_matrix(model)
    cost = Int(round(value(objective_function(model))))
    # cost = maximum(Int.(round.(value.(model[:tof]))));
    # assignments = map(j->findfirst(assignment_matrix[:,j] .== 1),1:problem_spec.M);

    results_dict = TOML.parse(sparse(assignment_matrix))
    results_dict["time"] = elapsed_time
    # results_dict["assignment"] = assignments
    results_dict["optimal"] = optimal
    results_dict["cost"] = cost
    results_dict
end
function profile_low_level_search_and_repair(problem_def::SimpleProblemDef,env_graph,dist_matrix,adj_matrix;
    milp_model=AssignmentMILP())
    project_spec, r0, s0, sF = problem_def.project_spec,problem_def.r0,problem_def.s0,problem_def.sF
    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(
            project_spec, r0, s0, sF, dist_matrix);
    # Solve the problem
    solver = PC_TAPF_Solver(verbosity=0,LIMIT_A_star_iterations=10*nv(env_graph));
    project_schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
    @assert update_project_schedule!(milp_model,project_schedule, problem_spec, adj_matrix)
    env, mapf = construct_search_env(project_schedule, problem_spec, env_graph;
        primary_objective=primary_objective); # TODO pass in t0_ here (maybe get it straight from model?)
    pc_mapf = PC_MAPF(env,mapf);
    node = initialize_root_node(pc_mapf)
    valid_flag, elapsed_time, byte_ct, gc_time, mem_ct = @timed low_level_search!(solver, pc_mapf, node)

    cost = collect(get_cost(node.solution))
    if cost[1] == Inf
        cost = [-1 for c in cost]
    end
    results_dict = TOML.parse(solver)
    results_dict["time"] = elapsed_time
    results_dict["valid_flag"] = valid_flag
    results_dict["cost"] = cost
    results_dict["num_conflicts"] = count_conflicts(detect_conflicts(node.solution))
    results_dict
end
function profile_low_level_search(problem_def::SimpleProblemDef,env_graph,dist_matrix,adj_matrix;
    milp_model=AssignmentMILP())
    project_spec, r0, s0, sF = problem_def.project_spec,problem_def.r0,problem_def.s0,problem_def.sF
    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(
            project_spec, r0, s0, sF, dist_matrix);
    # Solve the problem
    solver = PC_TAPF_Solver(verbosity=0,LIMIT_A_star_iterations=10*nv(env_graph));
    project_schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
    @assert update_project_schedule!(milp_model, project_schedule, problem_spec, adj_matrix)
    env, mapf = construct_search_env(project_schedule, problem_spec, env_graph;
        primary_objective=primary_objective); # TODO pass in t0_ here (maybe get it straight from model?)
    pc_mapf = PC_MAPF(env,mapf);
    node = initialize_root_node(pc_mapf)
    valid_flag, elapsed_time, byte_ct, gc_time, mem_ct = @timed low_level_search!(solver, pc_mapf.env, pc_mapf.mapf, node)

    cost = collect(get_cost(node.solution))
    if cost[1] == Inf
        cost = [-1 for c in cost]
    end
    results_dict = TOML.parse(solver)
    results_dict["time"] = elapsed_time
    results_dict["valid_flag"] = valid_flag
    results_dict["cost"] = cost
    results_dict["num_conflicts"] = count_conflicts(detect_conflicts(node.solution))
    results_dict
end
function profile_full_solver(problem_def::SimpleProblemDef,env_graph,dist_matrix;milp_model=AssignmentMILP(),kwargs...)
    project_spec, r0, s0, sF = problem_def.project_spec,problem_def.r0,problem_def.s0,problem_def.sF
    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(
            project_spec, r0, s0, sF, dist_matrix);
    # Solve the problem
    solver = PC_TAPF_Solver(verbosity=0,LIMIT_A_star_iterations=5*nv(env_graph));

    (solution, assignment, cost, search_env), elapsed_time, byte_ct, gc_time, mem_ct = @timed high_level_search!(
        milp_model, solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer;
        kwargs...);

    robot_paths = convert_to_vertex_lists(solution)
    object_paths = get_object_paths(solution,search_env)
    optimal = true
    if cost[1] == Inf
        cost = [-1 for c in cost]
        optimal = false
    end

    results_dict = TOML.parse(solver)
    merge!(results_dict, TOML.parse(assignment))
    results_dict["time"] = elapsed_time
    # results_dict["assignment"] = assignment
    results_dict["robot_paths"] = robot_paths
    results_dict["object_paths"] = object_paths
    results_dict["optimal"] = optimal
    results_dict["cost"] = collect(cost)
    results_dict
end


function run_profiling(MODE=:nothing;
    num_tasks = [10,20,30,40,50,60],
    num_robots = [10,20,30,40],
    depth_biases = [0.1,0.4,0.7,1.0],
    max_parent_settings = [3],
    num_trials = 4,
    env_id = 2,
    initial_problem_id = 1,
    problem_dir = PROBLEM_DIR,
    results_dir = RESULTS_DIR,
    TimeLimit=50,
    OutputFlag=0,
    milp_model=AssignmentMILP()
    )
    # solver profiling
    env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
    factory_env = read_env(env_filename)
    env_graph = factory_env.graph
    dist_matrix = get_dist_matrix(env_graph)

    Random.seed!(1)
    problem_id = initial_problem_id-1;
    # run tests and push results into table
    for M in num_tasks
        for N in num_robots
            for max_parents in max_parent_settings
                for depth_bias in depth_biases
                    for trial in 1:num_trials
                        try
                            problem_id += 1 # moved this to the beginning so it doesn't get skipped
                            problem_filename = joinpath(problem_dir,string("problem",problem_id,".toml"))
                            config_filename = joinpath(problem_dir,string("config",problem_id,".toml"))
                            if MODE == :write
                                # initialize a random problem
                                if !isdir(problem_dir)
                                    mkdir(problem_dir)
                                end
                                if isfile(problem_filename)
                                    println("file ",problem_filename," already exists. Skipping ...")
                                    continue # don't overwrite existing files
                                end
                                Δt_min=0
                                Δt_max=0
                                r0,s0,sF = get_random_problem_instantiation(N,M,get_pickup_zones(factory_env),get_dropoff_zones(factory_env),
                                        get_free_zones(factory_env))
                                project_spec = construct_random_project_spec(M,s0,sF;max_parents=max_parents,depth_bias=depth_bias,Δt_min=0,Δt_max=0)
                                problem_def = SimpleProblemDef(project_spec,r0,s0,sF)
                                # Save the problem
                                open(problem_filename, "w") do io
                                    TOML.print(io, TOML.parse(problem_def))
                                end
                                open(config_filename, "w") do io
                                    TOML.print(io, Dict(
                                        "N"=>N,
                                        "M"=>M,
                                        "max_parents"=>max_parents,
                                        "depth_bias"=>depth_bias,
                                        "min_process_time"=>Δt_min,
                                        "max_process_time"=>Δt_max,
                                        "env_id"=>env_id
                                        )
                                    )
                                end
                            elseif MODE != :nothing
                                subdir = joinpath(results_dir,string(MODE))
                                results_filename = joinpath(subdir,string("results",problem_id,".toml"))
                                if !isdir(subdir)
                                    mkpath(subdir)
                                end
                                if isfile(results_filename)
                                    println("file ",results_filename," already exists. Skipping ...")
                                    continue # don't overwrite existing files
                                end
                                assignment_filename = joinpath(results_dir,string(:assignment_only),string("results",problem_id,".toml"))
                                # Load the problem
                                problem_def = read_problem_def(problem_filename)
                                if MODE == :assignment_only
                                    results_dict = profile_task_assignment(problem_def,env_graph,dist_matrix;
                                        milp_model=milp_model,TimeLimit=TimeLimit,OutputFlag=OutputFlag)
                                end
                                if MODE == :low_level_search_without_repair
                                    # assignments = read_assignment(assignment_filename)
                                    adj_matrix = read_sparse_matrix(assignment_filename)
                                    results_dict = profile_low_level_search(problem_def,env_graph,dist_matrix,adj_matrix;
                                        milp_model=milp_model)
                                end
                                if MODE == :low_level_search_with_repair
                                    # assignments = read_assignment(assignment_filename)
                                    adj_matrix = read_sparse_matrix(assignment_filename)
                                    results_dict = profile_low_level_search_and_repair(problem_def,env_graph,dist_matrix,adj_matrix;
                                        milp_model=milp_model)
                                end
                                if MODE == :full_solver
                                    results_dict = profile_full_solver(problem_def,env_graph,dist_matrix;
                                        milp_model=milp_model,TimeLimit=TimeLimit,OutputFlag=OutputFlag)
                                end
                                println("Solved problem ",problem_id," in ",results_dict["time"]," seconds! MODE = ",string(MODE))
                                # print the results
                                open(results_filename, "w") do io
                                    TOML.print(io, results_dict)
                                end
                            end
                        catch e
                            if typeof(e) <: AssertionError
                                println(e.msg)
                            else
                            end
                        end
                    end
                end
            end
        end
    end
end



end
