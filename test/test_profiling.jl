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
    ]
    for milp_model in milp_models
        for mode in modes
            run_profiling(mode;
                num_tasks=[1],
                num_robots=[4],
                depth_biases=[0.1],
                task_size_distributions = [
                    ( 1=>0.0, 2=>0.0, 4=>1.0 )
                    ],
                num_trials=1,
                problem_dir = dummy_problem_dir,
                results_dir = joinpath(dummy_results_dir, string(typeof(milp_model))),
                milp_model = milp_model,
                TimeLimit=20,
                OutputFlag=0,
                Presolve = -1, # automatic setting (-1), off (0), conservative (1), or aggressive (2)
                verbosity = 4,
                LIMIT_A_star_iterations=8000,
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
        :low_level_search_without_repair,
        :low_level_search_with_repair,
        :full_solver
        ]
    results_dirs = [
        # joinpath(EXPERIMENT_DIR,"assignment_solver/results")
        # joinpath(EXPERIMENT_DIR,"adjacency_solver/results")
        joinpath(EXPERIMENT_DIR,"sparse_adjacency_solver_collab/results")
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
                TimeLimit=100,
                OutputFlag=0,
                Presolve = -1, # automatic setting (-1), off (0), conservative (1), or aggressive (2)
                milp_model = milp_model
                )
        end
    end
end
