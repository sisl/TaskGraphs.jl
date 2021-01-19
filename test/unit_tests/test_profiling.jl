let
    solver = NBSSolver()
    solution = SearchEnv()
    solve_time = 0.01

    extract_feature(solver,RunTime(),solution,solve_time)
    extract_feature(solver,SolutionCost(),solution,solve_time)
    extract_feature(solver,OptimalityGap(),solution,solve_time)
    extract_feature(solver,OptimalFlag(),solution,solve_time)
    extract_feature(solver,FeasibleFlag(),solution,solve_time)
    extract_feature(solver,NumConflicts(),solution,solve_time)

    feats = extract_feature(solver,(RunTime(),SolutionCost(),OptimalityGap()),solution,solve_time)
    @test isa(feats,NTuple{3,Float64})

    feature_dict = Dict{String,Float64}()
    store_feature!(feature_dict,RunTime(),0.01)
    load_feature(feature_dict,RunTime())

    filename = "/tmp/dummy.toml"
    open(filename,"w") do io
        TOML.print(io, feature_dict)
    end
    dict = TOML.parsefile(filename)
    load_feature(dict,RunTime())

    # df = DataFrame()
    # for feat in (RunTime(),SolutionCost(),OptimalityGap())
    #     df[!,Symbol(typeof(feat))] = feature_type(feat)[]
    # end
    # df
end
let

    println("WARMING UP")

    dummy_problem_dir = "dummy_problem_dir"
    dummy_results_dir = "dummy_results_dir"
    run(pipeline(`rm -rf $dummy_problem_dir`, stdout=devnull, stderr=devnull))
    run(pipeline(`rm -rf $dummy_results_dir`, stdout=devnull, stderr=devnull))
    modes = [
        :write,
        :assignment_only,
        # :low_level_search_without_repair,
        :low_level_search_with_repair,
        # :full_solver
        ]
    problem_dir, results_dir, solver_configs, problem_configs, solvers = get_problem_config_1()

    for solver in solvers
        for mode in modes
            run_profiling(solver,mode;
                problem_configs = problem_configs,
                primary_objective = MakeSpan(),
                base_problem_dir = dummy_problem_dir,
                base_results_dir = joinpath(
                    dummy_results_dir,
                    string(typeof(assignment_solver(solver).milp))),
                )
        end
        # run(pipeline(`rm -rf $dummy_problem_dir`, stdout=devnull, stderr=devnull))
        # run(pipeline(`rm -rf $dummy_results_dir`, stdout=devnull, stderr=devnull))
    end
end
let

    println("RUNNING PROFILING TESTS")

    modes = [
        # :assignment_only,
        # :low_level_search_without_repair,
        # :low_level_search_with_repair,
        :full_solver
        ]
    results_dirs = [
        joinpath(EXPERIMENT_DIR,"assignment_solver/cbs_fix_results")
        # joinpath(EXPERIMENT_DIR,"assignment_solver/cache_fix_results")
        # joinpath(EXPERIMENT_DIR,"adjacency_solver/results")
        # joinpath(EXPERIMENT_DIR,"sparse_adjacency_solver/results")
    ]
    milp_models = [
        AssignmentMILP(),
        # AdjacencyMILP(),
        # SparseAdjacencyMILP()
    ]
    problem_dir = PROBLEM_DIR
    for (milp_model, results_dir) in zip(milp_models, results_dirs)
        for mode in modes
            run_profiling(mode;
                num_tasks = [10,20,30,40,50,60],
                num_robots = [10,20,30,40],
                depth_biases = [0.1,0.4,0.7,1.0],
                max_parent_settings = [3],
                num_trials = 4,
                env_id = 2,
                initial_problem_id = 1,
                problem_dir = problem_dir,
                results_dir = results_dir,
                # milp_model = milp_model,
                primary_objective=MakeSpan,
                OutputFlag = 0,
                Presolve = -1, # automatic setting (-1), off (0), conservative (1), or aggressive (2)
                # TimeLimit = 100,
                solver_template = PC_TAPF_Solver(
                    nbs_model=milp_model,
                    verbosity=0,
                    l1_verbosity=2,
                    l2_verbosity=2,
                    l3_verbosity=1,
                    l4_verbosity=0,
                    LIMIT_A_star_iterations=8000,
                    time_limit=120,
                    nbs_time_limit=100
                    )
                )
        end
    end
end
let

    println("RUNNING PROFILING TESTS")

    modes = [
        # :write,
        # :assignment_only,
        :low_level_search_without_repair,
        :low_level_search_with_repair,
        :full_solver
        ]
    problem_dir = joinpath(PROBLEM_DIR,"collaborative_transport/final")
    results_dirs = [
        # joinpath(EXPERIMENT_DIR,"assignment_solver/results")
        # joinpath(EXPERIMENT_DIR,"adjacency_solver/results")
        joinpath(EXPERIMENT_DIR,"sparse_adjacency_solver/final/meta_env_repaired/results")
        # joinpath(EXPERIMENT_DIR,"greedy_assignment/final/results")
    ]
    milp_models = [
        # AssignmentMILP(),
        # AdjacencyMILP(),
        SparseAdjacencyMILP(),
        # GreedyAssignment()
    ]
    for (milp_model, results_dir) in zip(milp_models, results_dirs)
        for mode in modes
            run_profiling(mode;
                num_tasks = [12,18,24],
                num_robots = [24],
                depth_biases=[0.1],
                task_size_distributions = [
                    ( 1=>1.0, 2=>0.0, 4=>0.0 ),
                    ( 1=>1.0, 2=>1.0, 4=>0.0 ),
                    ( 1=>1.0, 2=>1.0, 4=>1.0 ),
                    ( 1=>0.0, 2=>1.0, 4=>1.0 ),
                    ],
                num_trials=16,
                Δt_collect=0,
                Δt_deposit=0,
                # milp_model = milp_model,
                problem_dir = problem_dir,
                results_dir = results_dir,
                OutputFlag=0,
                Presolve = -1, # automatic setting (-1), off (0), conservative (1), or aggressive (2)
                TimeLimit = 100, # 50
                solver_template = PC_TAPF_Solver(
                    nbs_model=milp_model,
                    verbosity=1,
                    l1_verbosity=2,
                    l2_verbosity=2,
                    l3_verbosity=0,
                    l4_verbosity=0,
                    LIMIT_assignment_iterations = isa(milp_model, GreedyAssignment) ? 1 : 50,
                    LIMIT_A_star_iterations=8000,
                    LIMIT_CBS_iterations=400,
                    time_limit=200 # 60
                    )
                )
        end
    end
end

let
    # results_dir = joinpath(EXPERIMENT_DIR,"sparse_adjacency_solver/collaborative_transport_dist_maps/results")
    results_dir = joinpath(EXPERIMENT_DIR,"sparse_adjacency_solver/final/results/full_solver")
    for (root, dirs, files) in walkdir(results_dir)
        for file in files
            if splitext(file)[end] == ".toml"
                toml_dict = TOML.parsefile(joinpath(root, file))
                if get(toml_dict, "optimal", true) == false
                    println(joinpath(root, file)) # path to files
                    # rm(joinpath(root,file))
                end
            end
        end
    end

end

# REPLANNING
let

    # base_problem_dir, base_results_dir, solver_configs, problem_configs, solver_template, fallback_solver_template = get_replanning_config_1()
    # base_problem_dir, base_results_dir, solver_configs, problem_configs, solver_template, fallback_solver_template = get_replanning_config_2()
    # base_problem_dir, base_results_dir, solver_configs, problem_configs, solver_template, fallback_solver_template = get_replanning_config_3()
    # base_problem_dir, base_results_dir, solver_configs, problem_configs, solver_template, fallback_solver_template = get_replanning_config_4()
    base_problem_dir, base_results_dir, solver_configs, problem_configs, solver_template, fallback_solver_template = get_replanning_config_5()
    # base_problem_dir, base_results_dir, solver_configs, problem_configs, solver_template, fallback_solver_template = get_replanning_config_6()

    # solver_template = PC_TAPF_Solver(solver_template,l2_verbosity=2)

    # fallback_solver_template.l2_verbosity=4
    # fallback_solver_template.cbs_model = PrioritizedDFSPlanner(max_iters=200)

    reset_debug_file_id!()
    reset_operation_id_counter!()
    reset_action_id_counter!()
    run_replanner_profiling(:write;
        problem_configs=problem_configs,
        base_problem_dir=base_problem_dir,
        )
    modes = [:solve]
    for solver_config in solver_configs
        replan_model = get(solver_config,:replan_model,  MergeAndBalance())
        fallback_model  = get(solver_config,:fallback_model,ReassignFreeRobots())
        results_dir     = get(solver_config,:results_dir,   joinpath(
            base_results_dir, string(typeof(replan_model),"-",typeof(fallback_model))))
        for mode in modes
            reset_operation_id_counter!()
            reset_action_id_counter!()
            run_replanner_profiling(mode;
                solver_config=solver_config,
                problem_configs=problem_configs,
                base_problem_dir=base_problem_dir,
                base_results_dir=results_dir,
                solver_template=solver_template,
                fallback_solver_template=fallback_solver_template,
                primary_objective = SumOfMakeSpans,
                )
        end
    end

end
