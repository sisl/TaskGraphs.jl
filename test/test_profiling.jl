# let
#     println("WARMING UP")
#
#     dummy_problem_dir = "dummy_problem_dir"
#     dummy_results_dir = "dummy_results_dir"
#     modes = [
#         :write,
#         :assignment_only,
#         :low_level_search_without_repair,
#         :low_level_search_with_repair,
#         :full_solver
#         ]
#     for mode in modes
#         run_profiling(mode;
#             num_tasks=[10],
#             num_robots=[10],
#             depth_biases=[0.1],
#             num_trials=1,
#             problem_dir = dummy_problem_dir,
#             results_dir = dummy_results_dir
#             )
#     end
#     run(pipeline(`rm -rf $dummy_problem_dir`, stdout=devnull, stderr=devnull))
#     run(pipeline(`rm -rf $dummy_results_dir`, stdout=devnull, stderr=devnull))
# end
# let
#     println("RUNNING PROFILING TESTS")
#
#     modes = [
#         :write,
#         :assignment_only,
#         :low_level_search_without_repair,
#         :low_level_search_with_repair,
#         :full_solver
#         ]
#     for mode in modes
#         run_profiling(mode;
#             num_tasks = [10,20,30,40,50,60],
#             num_robots = [10,20,30,40],
#             depth_biases = [0.1,0.4,0.7,1.0],
#             max_parent_settings = [3],
#             num_trials = 4,
#             env_id = 2,
#             initial_problem_id = 1,
#             problem_dir = PROBLEM_DIR,
#             results_dir = RESULTS_DIR,
#             TimeLimit=100,
#             OutputFlag=0
#             )
#     end
# end
