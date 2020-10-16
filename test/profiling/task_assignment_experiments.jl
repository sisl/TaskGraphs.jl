base_dir = joinpath("/scratch/task_graphs_experiments","assignment_experiments")
problem_dir = joinpath(base_dir,"problem_instances")
base_results_path = joinpath(base_dir,"results")

feats = [
    RunTime(),
    FeasibleFlag(),
    OptimalFlag(),
    OptimalityGap(),
    SolutionCost(),
    PrimaryCost(),
]
solver_configs = [
    (
        solver = TaskGraphsMILPSolver(FastSparseAdjacencyMILP()),
        results_path = joinpath(base_results_path,"FastSparseAdjacencyMILP"),
        feats = feats,
        objective = MakeSpan(),
    ),
    (
        solver = TaskGraphsMILPSolver(SparseAdjacencyMILP()),
        results_path = joinpath(base_results_path,"SparseAdjacencyMILP"),
        feats = feats,
        objective = MakeSpan(),
    ),
    (
        solver = TaskGraphsMILPSolver(AssignmentMILP()),
        results_path = joinpath(base_results_path,"AssignmentMILP"),
        feats = feats,
        objective = MakeSpan(),
    ),
]

base_config = Dict(
    :env_id => "env_2",
    :num_trials => 16,
    :max_parents => 3,
    :depth_bias => 0.4,
    :dt_min => 0,
    :dt_max => 0,
    :dt_collect => 0,
    :dt_deliver => 0,
    # :task_sizes => (1=>1.0,2=>0.0,4=>0.0)),
)
size_configs = [Dict(:M => m, :N => n) for (n,m) in Base.Iterators.product(
    [10,20,30,40],[10,20,30,40,50,60]
)][:]
problem_configs = map(d->merge(d,base_config), size_configs)

loader = PCTA_Loader()
add_env!(loader,"env_2",init_env_2())
# write_problems!(loader,problem_configs,problem_dir)
#
# for solver_config in solver_configs
#     set_runtime_limit!(solver_config.solver,100)
#     warmup(loader,solver_config,problem_dir)
#     run_profiling(loader,solver_config,problem_dir)
# end

# plot results
config_template = problem_configs[1]
config_df = construct_config_dataframe(loader,problem_dir,config_template)
for solver_config in solver_configs
    results_df = construct_results_dataframe(loader,solver_config,config_template)
    df = innerjoin(config_df,results_df,on=:problem_name)
    include(joinpath(pathof(TaskGraphs),"..","helpers/render_tools.jl"))
    # plt = get_box_plot_group_plot(df;obj=:RunTime,title=split(solver_config.results_path,"/")[end])
    plt = get_runtime_box_plot(df;scale=1.0,obj=:RunTime,title=split(solver_config.results_path,"/")[end])
    display(plt);
end
