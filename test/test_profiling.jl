let

    println("WARMING UP")

    dummy_problem_dir = "dummy_problem_dir"
    dummy_results_dir = "dummy_results_dir"
    modes = [
        :write,
        # :assignment_only,
        # :low_level_search_without_repair,
        # :low_level_search_with_repair,
        :full_solver
        ]
    milp_models = [
        # AssignmentMILP(),
        # AdjacencyMILP(),
        SparseAdjacencyMILP()
        # GreedyAssignment()
    ]
    for milp_model in milp_models
        for mode in modes
            run_profiling(mode;
                num_tasks=[4],
                num_robots=[4],
                depth_biases=[0.1],
                task_size_distributions = [
                    # ( 1=>1.0, 2=>0.0, 4=>0.0 ),
                    # ( 1=>1.0, 2=>1.0, 4=>0.0 ),
                    ( 1=>1.0, 2=>1.0, 4=>1.0 ),
                    # ( 1=>0.0, 2=>1.0, 4=>1.0 ),
                    ],
                num_trials=4,
                Δt_collect=1,
                Δt_deliver=1,
                primary_objective=MakeSpan,
                problem_dir = dummy_problem_dir,
                results_dir = joinpath(dummy_results_dir, string(typeof(milp_model))),
                # milp_model = milp_model,
                OutputFlag=0,
                Presolve = -1, # automatic setting (-1), off (0), conservative (1), or aggressive (2)
                TimeLimit = 20,
                solver_template = PC_TAPF_Solver(
                    nbs_model=milp_model,
                    verbosity=0,
                    l1_verbosity=2,
                    l2_verbosity=2,
                    l3_verbosity=1,
                    l4_verbosity=0,
                    LIMIT_A_star_iterations=8000,
                    time_limit=25
                    )
                )
        end
        run(pipeline(`rm -rf $dummy_problem_dir`, stdout=devnull, stderr=devnull))
        run(pipeline(`rm -rf $dummy_results_dir`, stdout=devnull, stderr=devnull))
    end
end
let

    println("RUNNING PROFILING TESTS")

    modes = [
        :assignment_only,
        # :low_level_search_without_repair,
        # :low_level_search_with_repair,
        # :full_solver
        ]
    results_dirs = [
        # joinpath(EXPERIMENT_DIR,"assignment_solver/results")
        # joinpath(EXPERIMENT_DIR,"adjacency_solver/results")
        joinpath(EXPERIMENT_DIR,"sparse_adjacency_solver/results")
    ]
    milp_models = [
        # AssignmentMILP(),
        # AdjacencyMILP(),
        SparseAdjacencyMILP()
    ]
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
                problem_dir = PROBLEM_DIR,
                results_dir = results_dir,
                # milp_model = milp_model,
                OutputFlag=0,
                Presolve = -1, # automatic setting (-1), off (0), conservative (1), or aggressive (2)
                TimeLimit = 100,
                solver_template = PC_TAPF_Solver(
                    nbs_model=milp_model,
                    verbosity=0,
                    l2_verbosity=2,
                    l3_verbosity=0,
                    l4_verbosity=0,
                    LIMIT_A_star_iterations=8000,
                    time_limit=100
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
                Δt_deliver=0,
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

    base_solver_configs = [
        Dict(
        :nbs_time_limit=>8,
        :env_id=>2,
        :OutputFlag => 0,
        :Presolve => -1,
        ),
    ]
    replan_configs = [
        Dict(:replan_model=>MergeAndBalance(),),
        Dict(:replan_model=>Oracle(),),
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
            :fallback_commit_threshold=>5,
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

    # run_replanner_profiling(:write;
    #     problem_configs=problem_configs,
    #     base_problem_dir=base_problem_dir,
    #     )
    modes = [:solve]
    for solver_config in solver_configs
        replanner_model = get(solver_config,:replan_model,  MergeAndBalance())
        fallback_model  = get(solver_config,:fallback_model,ReassignFreeRobots())
        results_dir     = get(solver_config,:results_dir,   joinpath(
            base_results_dir, string(typeof(replanner_model),"-",typeof(fallback_model))))
        for mode in modes
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
