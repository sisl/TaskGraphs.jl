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

function profile_task_assignment(solver, project_spec, problem_spec, robot_ICs, env_graph, dist_matrix;
    milp_model=AssignmentMILP(),primary_objective=SumOfMakeSpans,kwargs...)

    project_schedule = construct_partial_project_schedule(project_spec, problem_spec, robot_ICs)
    model = formulate_milp(milp_model,project_schedule,problem_spec)

    retval, elapsed_time, byte_ct, gc_time, mem_ct = @timed optimize!(model)

    optimal = (termination_status(model) == MOI.OPTIMAL);
    feasible = (primal_status(model) == MOI.FEASIBLE_POINT)
    assignment_matrix = get_assignment_matrix(model)
    cost = Int(round(value(objective_function(model))))
    optimality_gap = cost - Int(round(objective_bound(model)))

    results_dict = TOML.parse(sparse(assignment_matrix))
    results_dict["time"] = elapsed_time
    results_dict["optimal"] = optimal
    results_dict["optimality_gap"] = Int(optimality_gap)
    results_dict["feasible"] = feasible
    results_dict["cost"] = cost
    results_dict
end
function profile_low_level_search_and_repair(solver, project_spec, problem_spec, robot_ICs, env_graph,dist_matrix,adj_matrix;
    milp_model=AssignmentMILP(),primary_objective=SumOfMakeSpans)

    # solver = PC_TAPF_Solver(verbosity=0,LIMIT_A_star_iterations=10*nv(env_graph));
    project_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
    if update_project_schedule!(milp_model,project_schedule, problem_spec, adj_matrix)
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
    milp_model=AssignmentMILP(),primary_objective=SumOfMakeSpans)

    # solver = PC_TAPF_Solver(verbosity=0,LIMIT_A_star_iterations=10*nv(env_graph));
    project_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
    if update_project_schedule!(milp_model, project_schedule, problem_spec, adj_matrix)
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
        milp_model=AssignmentMILP(),primary_objective=SumOfMakeSpans,kwargs...)
    # Solve the problem
    # solver = PC_TAPF_Solver(verbosity=0,LIMIT_A_star_iterations=5*nv(env_graph));

    (solution, assignment, cost, search_env, optimality_gap), elapsed_time, byte_ct, gc_time, mem_ct = @timed high_level_search!(
        milp_model, solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer;
        primary_objective=primary_objective,
        kwargs...);

    robot_paths = convert_to_vertex_lists(solution)
    object_paths, object_intervals = get_object_paths(solution,search_env)
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
    results_dict["robot_paths"] = robot_paths
    results_dict["object_paths"] = object_paths
    results_dict["object_intervals"] = object_intervals
    results_dict["optimality_gap"] = Int(optimality_gap)
    results_dict["optimal"] = optimal
    results_dict["feasible"] = feasible
    results_dict["cost"] = collect(cost)
    results_dict
end


function run_profiling(MODE=:nothing;
    num_tasks = [10,20,30,40,50,60],
    num_robots = [10,20,30,40],
    depth_biases = [0.1,0.4,0.7,1.0],
    max_parent_settings = [3],
    task_size_distributions = [
        (1=>1.0,2=>0.0,4=>0.0)
        ],
    num_trials = 4,
    env_id = 2,
    initial_problem_id = 1,
    problem_dir = PROBLEM_DIR,
    results_dir = RESULTS_DIR,
    Δt_min=0,
    Δt_max=0,
    Δt_collect=0,
    Δt_deliver=0,
    solver_template=PC_TAPF_Solver(),
    TimeLimit=solver_template.time_limit,
    OutputFlag=0,
    Presolve = -1,
    # verbosity=0,
    # LIMIT_A_star_iterations=10000,
    milp_model=AssignmentMILP(),
    primary_objective=SumOfMakeSpans
    )
    # solver profiling
    env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
    factory_env = read_env(env_filename)
    env_graph = factory_env
    dist_matrix = get_dist_matrix(env_graph)
    dist_mtx_map = DistMatrixMap(factory_env.vtx_map,factory_env.vtxs)

    Random.seed!(1)
    problem_id = initial_problem_id-1;
    # run tests and push results into table
    for (M,N,task_sizes,max_parents,depth_bias,trial) in Base.Iterators.product(
        num_tasks,num_robots,task_size_distributions,max_parent_settings,depth_biases,1:num_trials
        )
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
                r0,s0,sF = get_random_problem_instantiation(N,M,get_pickup_zones(factory_env),get_dropoff_zones(factory_env),
                        get_free_zones(factory_env))
                project_spec = construct_random_project_spec(M,s0,sF;max_parents=max_parents,depth_bias=depth_bias,Δt_min=Δt_min,Δt_max=Δt_max)
                shapes = choose_random_object_sizes(M,Dict(task_sizes...))
                problem_def = SimpleProblemDef(project_spec,r0,s0,sF,shapes)
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
                project_spec, r0, s0, sF = problem_def.project_spec,problem_def.r0,problem_def.s0,problem_def.sF
                # @show problem_def.shapes
                # @show map(o->factory_env.expanded_zones[s0[o]][problem_def.shapes[o]], 1:M)
                project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
                    project_spec, r0, s0, sF,
                    dist_mtx_map;
                    Δt_collect=map(i->Δt_collect, s0),
                    Δt_deliver=map(i->Δt_deliver, s0),
                    # dist_matrix;
                    cost_function=primary_objective,
                    task_shapes=problem_def.shapes,
                    shape_dict=factory_env.expanded_zones,
                    );
                # problem_spec = ProblemSpec(problem_spec, D=dist_mtx_map)

                solver = PC_TAPF_Solver(solver_template,start_time=time());
                if MODE == :assignment_only
                    results_dict = profile_task_assignment(solver, project_spec, problem_spec, robot_ICs, env_graph, dist_matrix;
                        milp_model=milp_model,primary_objective=primary_objective,TimeLimit=TimeLimit,OutputFlag=OutputFlag,Presolve=Presolve)
                end
                if MODE == :low_level_search_without_repair
                    # assignments = read_assignment(assignment_filename)
                    adj_matrix = read_sparse_matrix(assignment_filename)
                    results_dict = profile_low_level_search(solver, project_spec, problem_spec, robot_ICs,env_graph,dist_matrix,adj_matrix;
                        milp_model=milp_model,primary_objective=primary_objective)
                end
                if MODE == :low_level_search_with_repair
                    # assignments = read_assignment(assignment_filename)
                    adj_matrix = read_sparse_matrix(assignment_filename)
                    results_dict = profile_low_level_search_and_repair(solver, project_spec, problem_spec, robot_ICs,env_graph,dist_matrix,adj_matrix;
                        milp_model=milp_model,primary_objective=primary_objective)
                end
                if MODE == :full_solver
                    results_dict = profile_full_solver(solver, project_spec, problem_spec, robot_ICs,env_graph,dist_matrix;
                        milp_model=milp_model,primary_objective=primary_objective,TimeLimit=TimeLimit,OutputFlag=OutputFlag,Presolve=Presolve)
                end
                println("PROFILER: ",typeof(milp_model)," --- ",string(MODE), " --- Solved problem ",problem_id," in ",results_dict["time"]," seconds! \n\n")
                # print the results
                open(results_filename, "w") do io
                    TOML.print(io, results_dict)
                end
            end
        catch e
            if typeof(e) <: AssertionError
                println(e.msg)
            else
                throw(e)
            end
        end
    end
end



end
