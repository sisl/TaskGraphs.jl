using TaskGraphs, CRCBS, TOML

# initialize loader
base_dir = joinpath("/scratch/task_graphs_experiments","replanning3")
base_problem_dir    = joinpath(base_dir,"problem_instances")
# base_results_dir    = joinpath(base_dir,"results")
base_results_dir    = joinpath(base_dir,"results_debug")

loader = ReplanningProblemLoader()
add_env!(loader,"env_2",init_env_2())

reset_task_id_counter!()
reset_operation_id_counter!()
# get probem config
problem_configs = replanning_config_3()
# # write problems
# Random.seed!(0)
# write_problems!(loader,problem_configs,base_problem_dir)

feats = [
    RunTime(),IterationCount(),LowLevelIterationCount(),
    TimeOutStatus(),IterationMaxOutStatus(),
    SolutionCost(),OptimalityGap(),OptimalFlag(),FeasibleFlag(),NumConflicts(),
    ObjectPathSummaries(),
    ]
final_feats = [SolutionCost(),NumConflicts(),RobotPaths()]
planner_configs = []
for (primary_replanner, backup_replanner) in [
        (MergeAndBalance(),MergeAndBalance()),
        (ReassignFreeRobots(),ReassignFreeRobots()),
        (DeferUntilCompletion(),DeferUntilCompletion()),
    ]
    # Primary planner
    path_finder = DefaultAStarSC()
    set_iteration_limit!(path_finder,5000)
    primary_route_planner = CBSSolver(ISPS(path_finder))
    set_iteration_limit!(primary_route_planner,1000)
    primary_planner = FullReplanner(
        solver = NBSSolver(path_planner=primary_route_planner),
        replanner = primary_replanner,
        cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
        )
    # Backup planner
    backup_planner = FullReplanner(
        solver = NBSSolver(
            assignment_model = TaskGraphsMILPSolver(GreedyAssignment()),
            path_planner = PIBTPlanner{NTuple{3,Float64}}()
            ),
        replanner = backup_replanner,
        cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
        )
    set_iteration_limit!(backup_planner,1)
    set_iteration_limit!(route_planner(backup_planner.solver),5000)
    set_verbosity!(route_planner(backup_planner.solver),3)
    set_debug!(backup_planner,true)
    # set_debug!(route_planner(backup_planner.solver),true)
    # Full solver
    planner = ReplannerWithBackup(primary_planner,backup_planner)
    planner_config = (
        planner=planner,
        results_path=joinpath(base_results_dir,string(typeof(primary_replanner))),
        objective = SumOfMakeSpans()
    )
    push!(planner_configs,planner_config)
end

for planner_config in planner_configs
    # warm up to precompile replanning code
    warmup(planner_config.planner,loader)
    # profile
    profile_replanner!(loader,
        planner_config.planner,
        base_problem_dir,
        planner_config.results_path)
end

# if results show ''"CRCBS.Feature" = val', use the following line to convert:
# sed -i 's/\"CRCBS\.\([A-Za-z]*\)\"/\1/g' **/**/*.toml

include(joinpath(pathof(TaskGraphs),"..","helpers/render_tools.jl"))

# # plotting results
config_df = construct_config_dataframe(loader,base_problem_dir,problem_configs[1])
plotting_config = (
    feats = Dict(
            :makespans => Int[],
            :arrival_times => Int[],
            :backup_flags => Bool[],
            :runtime_gaps => Float64[],
            :primary_runtimes => Float64[],
            :backup_runtimes => Float64[],
    ),
    results_path = joinpath(base_results_dir,"MergeAndBalance"),
    problem_path = base_problem_dir,
)
results_df = TaskGraphs.construct_replanning_results_dataframe(loader,plotting_config,plotting_config.feats)
df = innerjoin(config_df,results_df;on=:problem_name)

df_list = [df,]

# plot_histories_pgf(df_list;
#     y_key=:runtime_gaps,
#     include_keys = [:num_projects=>30,:M=>30,:arrival_interval=>80],
#     xlabel = "time",
#     ylabel = "makespans",
#     ytick_show = true,
#     ymode="linear",
#     lines=false
# )

plt = PGFPlots.GroupPlot(3,5)
for (n_vals, m_vals) in [
    ([10,],[20,30,40,]),
    ([15,],[30,40,50,]),
    ([20,],[40,50,60,]),
    ([25,],[50,60,70,]),
    ([30,],[60,70,80,]),
    ]
    gp = group_history_plot(df_list;
        y_key = :primary_runtimes,
        x_key=:none,
        n_key = :M,
        n_vals = n_vals,
        m_key = :arrival_interval,
        m_vals = m_vals,
        ymode="linear",
        lines=false,
    )
    gp2 = group_history_plot(df_list;
        y_key = :backup_runtimes,
        x_key=:none,
        n_key = :M,
        n_vals = n_vals,
        m_key = :arrival_interval,
        m_vals = m_vals,
        ymode="linear",
        lines=false,
        colors=["blue"],
    )
    for (ax,ax2) in zip(gp.axes,gp2.axes)
        append!(ax.plots,ax2.plots)
    end

    append!(plt.axes,gp.axes)
end
plt
save("replanning_plots.pdf",plt)
