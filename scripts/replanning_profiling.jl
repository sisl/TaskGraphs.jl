using TaskGraphs, CRCBS, TOML, Random

# initialize loader
loader = ReplanningProblemLoader()
add_env!(loader,"env_2",init_env_2())

# base_dir = joinpath("/scratch/task_graphs_experiments","replanning3")
# base_problem_dir    = joinpath(base_dir,"problem_instances")
# base_results_dir    = joinpath(base_dir,"results_no_fail")
# problem_configs = replanning_config_3()
# MAX_TIME_LIMIT = 50
# COMMIT_THRESHOLD = 10
# ASTAR_ITERATION_LIMIT = 5000
# PIBT_ITERATION_LIMIT = 5000
# CBS_ITERATION_LIMIT = 1000

# New experiments to stress the planner
# base_dir = joinpath("/scratch/task_graphs_experiments","replanning4")
# base_problem_dir    = joinpath(base_dir,"problem_instances")
# base_results_dir    = joinpath(base_dir,"results_extended")
# problem_configs = replanning_config_4()
# MAX_TIME_LIMIT = 50
# COMMIT_THRESHOLD = 10
# ASTAR_ITERATION_LIMIT = 5000
# PIBT_ITERATION_LIMIT = 5000
# CBS_ITERATION_LIMIT = 1000

# Final set of experiments
base_dir = joinpath("/scratch/task_graphs_experiments","replanning")
base_problem_dir    = joinpath(base_dir,"problem_instances")
base_results_dir    = joinpath(base_dir,"results")
problem_configs = replanning_config_5()
MAX_TIME_LIMIT = 40
COMMIT_THRESHOLD = 10
ASTAR_ITERATION_LIMIT = 5000
PIBT_ITERATION_LIMIT = 5000
CBS_ITERATION_LIMIT = 1000

reset_task_id_counter!()
reset_operation_id_counter!()
# # write problems
Random.seed!(0)
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
        (ConstrainedMergeAndBalance(max_problem_size=300),     DeferUntilCompletion()),
        (NullReplanner(),       DeferUntilCompletion()),
        (MergeAndBalance(),     DeferUntilCompletion()),
        (ReassignFreeRobots(),  DeferUntilCompletion()),
        (DeferUntilCompletion(),DeferUntilCompletion()),
    ]
    # Primary planner
    path_finder = DefaultAStarSC()
    set_iteration_limit!(path_finder,ASTAR_ITERATION_LIMIT)
    primary_route_planner = CBSSolver(ISPS(path_finder))
    set_iteration_limit!(primary_route_planner,CBS_ITERATION_LIMIT)
    primary_planner = FullReplanner(
        solver = NBSSolver(path_planner=primary_route_planner),
        replanner = primary_replanner,
        cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
        )
    set_max_time_limit!(primary_planner,MAX_TIME_LIMIT)
    set_commit_threshold!(primary_planner,COMMIT_THRESHOLD)
    # Backup planner
    backup_planner = FullReplanner(
        solver = NBSSolver(
            assignment_model = TaskGraphsMILPSolver(GreedyAssignment()),
            path_planner = PIBTPlanner{NTuple{3,Float64}}(partial=true)
            ),
        replanner = backup_replanner,
        cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
        )
    set_iteration_limit!(backup_planner,1)
    set_iteration_limit!(route_planner(backup_planner.solver),PIBT_ITERATION_LIMIT)
    set_commit_threshold!(backup_planner,COMMIT_THRESHOLD)
    set_debug!(backup_planner,true)
    # set_debug!(route_planner(backup_planner.solver),true)
    # Full solver
    planner = ReplannerWithBackup(primary_planner,backup_planner)
    planner_config = (
        planner=planner,
        planner_name=string(typeof(primary_replanner)),
        results_path=joinpath(base_results_dir,string(typeof(primary_replanner))),
        objective = SumOfMakeSpans(),
    )
    push!(planner_configs,planner_config)
end

# for planner_config in planner_configs
#     # warm up to precompile replanning code
#     warmup(planner_config.planner,loader)
#     # profile
#     profile_replanner!(loader,
#         planner_config.planner,
#         base_problem_dir,
#         planner_config.results_path)
# end

# if results show ''"CRCBS.Feature" = val', use the following line to convert:
# sed -i 's/\"CRCBS\.\([A-Za-z]*\)\"/\1/g' **/**/*.toml

# problem_configs = [replanning_config_3()..., replanning_config_4()...]
# copy problems and results folder4
# n = 180
# paths = [base_problem_dir, map(c->c.results_path, planner_configs)...]
# for p in paths
#     for f in collect(readdir(p;join=true))
#         id = parse(Int,f[end-3:end])
#         new_name = TaskGraphs.padded_problem_name(id+n,"problem","")
#         new_f = joinpath(p,new_name)
#         # @show id, new_name
#         # @show f, new_f
#         # @info mv(f,new_f)
#     end
# end
# source_prob_dir = "/scratch/task_graphs_experiments/replanning3/problem_instances/"
# source_results_dir = "/scratch/task_graphs_experiments/replanning3/results_no_fail/"
# for (src,dst) in [
#     (source_prob_dir,base_problem_dir),
#     map(c->(joinpath(source_results_dir,c.planner_name),
#         joinpath(base_results_dir,c.planner_name)), planner_configs)...
#     ]
#     @show src, dst
#
# end

Revise.includet(joinpath(pathof(TaskGraphs),"..","helpers/render_tools.jl"))

# # plotting results

df_dict = Dict{String,DataFrame}()
for planner_config in planner_configs
    config_df = construct_config_dataframe(loader,base_problem_dir,problem_configs[1])
    plotting_config = (
        feats = Dict(
                :makespans => Int[],
                :arrival_times => Int[],
                :completion_times => Int[],
                :backup_flags => Bool[],
                :runtime_gaps => Float64[],
                :primary_runtimes => Float64[],
                :backup_runtimes => Float64[],
        ),
        results_path = joinpath(base_results_dir,planner_config.planner_name),
        problem_path = base_problem_dir,
    )
    results_df = TaskGraphs.construct_replanning_results_dataframe(loader,plotting_config,plotting_config.feats)
    df = innerjoin(config_df,results_df;on=:problem_name)
    df.projects_per_second = df.num_projects ./ map(maximum,df.completion_times)
    df.backlog_factor = (df.num_projects .+ 1) .* df.arrival_interval ./ df.projects_per_second
    df.tasks_per_second = df.projects_per_second .* df.M
    df_dict[planner_config.planner_name] = df
end
df_list = Vector{DataFrame}([df_dict[config.planner_name] for config in planner_configs])
min_makespans = minimum(map(df->map(minimum, df.makespans), df_list))
for (k,df) in df_dict
    # df.backlog_factor = map(maximum, df.completion_times) ./ ((df.num_projects .+ 1) .* df.arrival_interval) # .+ map(minimum, df.makespans))
    # df.backlog_factor = map(maximum, df.completion_times) ./ ((df.num_projects .+ 0) .* df.arrival_interval .+ map(minimum, df.makespans))
    df.backlog_factor = map(maximum, df.completion_times) ./ ((df.num_projects .+ 0) .* df.arrival_interval .+ min_makespans)
    df.avg_makespan = map(sum, df.makespans) ./ df.num_projects
    df.tasks_per_minute = 60.0*df.tasks_per_second
    df.fallback_count = map(sum, df.backup_flags)
    df.fallback_rate = df.fallback_count ./ df.num_projects
end
plot_configs = Dict(
    "MergeAndBalance"=>Dict(
        :legend_entry   => "\\mergeAndBalance-∞",
        :color          => "lime",
        :mark           => "star",
        ),
    "ConstrainedMergeAndBalance"=>Dict(
        :legend_entry   => "\\mergeAndBalance",
        :color          => "red",
        :mark           => "diamond",
        ),
    "ReassignFreeRobots"=>Dict(
        :legend_entry   => "\\reassignFree",
        :color          => "blue",
        :mark           => "x",
        ),
    "DeferUntilCompletion"=>Dict(
        :legend_entry   => "\\deferUntilCompletion",
        :color          => "black",
        :mark           => "triangle",
        ),
    "NullReplanner"=>Dict(
        :legend_entry   => "\\backupOnly",
        :color          => "brown",
        :mark           => "+",
        ),
)

planner_ordering = ["ConstrainedMergeAndBalance","ReassignFreeRobots","DeferUntilCompletion","NullReplanner"]
# legend_entries=["\\mergeAndBalance","\\reassignFree","\\deferUntilCompletion","\\backupOnly"]
# colors=["red","blue","black","brown"]

# table of rates
# tab = build_table(df_dict["MergeAndBalance"])
# table_product([build_table(df_dict["MergeAndBalance"]),build_table(df_dict["DeferUntilCompletion"])])
xkey = :M
ykey = :arrival_interval
f = vals -> sum(vals)/length(vals)
table_base_dir = joinpath(pwd(),"tables")
mkpath(table_base_dir)
for (obj,prec,comp_func,rprec,op) in [
    (:tasks_per_minute, 1,  argmax, 2, arr->arr[1]/maximum(arr[2:end])),
    (:backlog_factor,   2,  argmin, 2, arr->minimum(arr[2:end])/arr[1]),
    (:avg_makespan,     0,  argmin, 2, arr->minimum(arr[2:end])/arr[1]),
    (:fallback_rate,    2,  argmin, 2, arr->arr[1]/maximum(arr[2:end])),
    ]
    tables = []
    for k in planner_ordering
        df = df_dict[k]
        tab = build_table(df;obj=obj,xkey=xkey,ykey=ykey,aggregator=f)
        push!(tables,tab)
        # write_tex_table(joinpath(table_base_dir,"$(string(obj))_$(k).tex"),tab)
    end
    composite_table = table_product(tables)
    write_tex_table(joinpath(table_base_dir,"$(string(obj))_composite.tex"),
        composite_table;
        print_func=(io,vals)->print_multi_value_real(io,vals;precision=prec,comp_func=comp_func),
        header_printer=print_latex_header,
        row_start_printer=print_latex_row_start,
        )
    reduced_table = table_reduce(composite_table,op)
    write_tex_table(joinpath(table_base_dir,"$(string(obj))_reduced.tex"),
        reduced_table;
        print_func=(io,vals)->print_real(io,vals;precision=rprec),
        header_printer=print_latex_header,
        row_start_printer=print_latex_row_start,
        )
end

for (include_keys, filename) in [
    ([:num_projects=>30,:M=>10,:arrival_interval=>40],"makespans-10-40.tex"),
    ([:num_projects=>30,:M=>30,:arrival_interval=>80],"makespans-30-80.tex"),
    ]
    plt = plot_histories_pgf(df_dict,planner_ordering;
        y_key=:makespans,
        x_key=:none,
        include_keys = include_keys,
        xlabel = "stage",
        ylabel = "makespans",
        ytick_show = true,
        ymode="linear",
        lines=false,
        legend_entries=map(k->plot_configs[k][:legend_entry],planner_ordering),
        colors = map(k->plot_configs[k][:color],planner_ordering),
        opt_marks=map(k->plot_configs[k][:mark],planner_ordering),
    )
    plt.legendPos="north west"
    # display(plt)
    save(filename,plt)
end

plt = PGFPlots.GroupPlot(7,5)
plt.groupStyle="horizontal sep = 8pt, vertical sep = 8pt"
for (idx,(n_vals, m_vals)) in enumerate([
    ([10,],[20,30,40,50,60,70,80]),
    ([15,],[20,30,40,50,60,70,80]),
    ([20,],[20,30,40,50,60,70,80]),
    ([25,],[20,30,40,50,60,70,80]),
    ([30,],[20,30,40,50,60,70,80]),
    ])
    gp = group_history_plot(df_dict,planner_ordering;
        y_key = :makespans,
        use_y_lims=true,
        # y_min=0,
        # y_max=2500,
        include_keys = [:num_projects=>30],
        ymode="linear",
        x_key=:none,
        n_key = :M,
        n_vals = n_vals,
        m_key = :arrival_interval,
        m_vals = m_vals,
        lines=false,
        colors=map(k->plot_configs[k][:color],planner_ordering),
        opt_marks=map(k->plot_configs[k][:mark],planner_ordering),
    )
    if idx < 5
        for ax in gp.axes
            ax.xlabel = nothing
            if ax.style === nothing
                ax.style="xticklabels=none"
            else
                ax.style=string(ax.style,", ","xticklabels=none")
            end
        end
    end
    append!(plt.axes,gp.axes)
end
display(plt)
save("makespan_trends.pdf",plt)


planner_ordering2 = ["MergeAndBalance","ConstrainedMergeAndBalance","ReassignFreeRobots","DeferUntilCompletion","NullReplanner"]
plt = PGFPlots.GroupPlot(3,2)
for (idx,(n_vals, m_vals)) in enumerate([
    ([15,],[50,30,20]),
    ([30,],[50,30,20]),
    ])
    gp = group_history_plot(df_dict,planner_ordering2;
        y_key = :makespans,
        use_y_lims=true,
        y_min=-100,
        # y_max=2500,
        # legend_entries=legend_entries,
        legend_pos = "north west",
        legend_flag = idx == 1,
        include_keys = [:num_projects=>30],
        ymode="linear",
        x_key=:none,
        # x_key=:arrival_times,
        n_key = :M,
        n_vals = n_vals,
        m_key = :arrival_interval,
        m_vals = m_vals,
        lines=false,
        legend_entries=map(k->plot_configs[k][:legend_entry][2:end],planner_ordering2),
        colors=map(k->plot_configs[k][:color],planner_ordering2),
        opt_marks=["diamond","x","triangle","+","star"],
    )
    append!(plt.axes,gp.axes)
end
for (i,ax) in enumerate(plt.axes)
    if i in Set([4,5,6])
        # ax.ylabel = "makespan"
    else
        ax.xlabel = nothing
        if ax.style === nothing
            ax.style="xticklabels=none"
        else
            ax.style=string(ax.style,", ","xticklabels=none")
        end
    end
end
plt.groupStyle="horizontal sep = 8pt, vertical sep = 8pt"
display(plt)
save("makespan_comparisons.tex",plt)
save("makespan_comparisons.pdf",plt)


name = "ConstrainedMergeAndBalance"
plt = plot_history_layers(df_dict[name],[:primary_runtimes,:backup_runtimes];
    legend_entries=["primary","backup"],
    x_key=:none,
    n_key = :M,
    n_vals = [25,],
    m_key = :arrival_interval,
    m_vals = [50,],
    include_keys = [:num_projects=>30],
    xlabel="",
    ylabel="",
    ymode="linear",
    lines=false,
    opt_marks=["diamond","+"],
)
plt.legendPos = "north east"
display(plt)
save("planner_runtimes-25-50.tex",plt)

for (df,planner_config) in zip(df_list,planner_configs)
    plt = PGFPlots.GroupPlot(3,5)
    for (n_vals, m_vals) in [
        ([10,],[20,30,40,]),
        ([15,],[30,40,50,]),
        ([20,],[40,50,60,]),
        ([25,],[50,60,70,]),
        ([30,],[60,70,80,]),
        ]
        gp = group_history_plot([df];
            y_key = :primary_runtimes,
            x_key=:none,
            n_key = :M,
            n_vals = n_vals,
            m_key = :arrival_interval,
            m_vals = m_vals,
            ymode="log", #"linear"
            lines=false,
        )
        gp2 = group_history_plot([df];
            y_key = :backup_runtimes,
            x_key=:none,
            n_key = :M,
            n_vals = n_vals,
            m_key = :arrival_interval,
            m_vals = m_vals,
            ymode="log",
            lines=false,
            colors=["blue"],
        )
        for (ax,ax2) in zip(gp.axes,gp2.axes)
            append!(ax.plots,ax2.plots)
        end

        append!(plt.axes,gp.axes)
    end
    display(plt)
    save("/home/kylebrown/Desktop/$(planner_config.planner_name)_runtimes.pdf",plt)
end
