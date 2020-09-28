# Problem Generation
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

# env_ids = Set(map(config->config[:env_id],problem_configs))
# envs = Dict(i=>read_env(joinpath(ENVIRONMENT_DIR,"env_$i.toml")) for i in env_ids)


solvers = [
    NBSSolver(),
    NBSSolver(path_planner = PIBTPlanner{NTuple{3,Float64}}()),
]


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
    Dict(:fallback_replan_model=>ReassignFreeRobots(),),
]
replan_configs = [
    Dict(:replan_model=>MergeAndBalance(),),
    Dict(:replan_model=>Oracle(),:time_out_buffer=>-105,:route_planning_buffer=>5),
    Dict(:replan_model=>ReassignFreeRobots(),),
    Dict(:replan_model=>DeferUntilCompletion(),),
]
solver_configs = Dict[]
for dicts in Base.Iterators.product(base_solver_configs,fallback_configs,replan_configs)
    push!(solver_configs,merge(dicts...))
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


base_dir            = joinpath("/scratch/task_graphs_experiments","replanning")
base_problem_dir    = joinpath(base_dir,"problem_instances")
base_results_dir    = joinpath(base_dir,"results")
