using TaskGraphs, CRCBS, TOML, Random, Gurobi
set_default_milp_optimizer!(Gurobi.Optimizer)

# initialize loader
loader = ReplanningProblemLoader()
add_env!(loader,"env_2",init_env_2())

# Final set of experiments
base_dir = joinpath("/scratch/task_graphs_experiments","replanning","case2021")
base_problem_dir    = joinpath(base_dir,"problem_instances")
base_results_dir    = joinpath(base_dir,"results")

problem_configs = TaskGraphs.replanning_config_6()
reset_all_id_counters!()
# # write problems
Random.seed!(0)
# write_problems!(loader,problem_configs,base_problem_dir)

# Define solvers
MAX_TIME_LIMIT = 20
MAX_BACKUP_TIME_LIMIT = 20
COMMIT_THRESHOLD = 10
ASTAR_ITERATION_LIMIT = 5000
PIBT_ITERATION_LIMIT = 5000
CBS_ITERATION_LIMIT = 1000
NBS_ITERATION_LIMIT = 10

feats = [
    RunTime(),IterationCount(),LowLevelIterationCount(),
    TimeOutStatus(),IterationMaxOutStatus(),
    SolutionCost(),OptimalityGap(),OptimalFlag(),FeasibleFlag(),NumConflicts(),
    ObjectPathSummaries(),
    ]
final_feats = [SolutionCost(),NumConflicts(),RobotPaths()]
planner_configs = []
# planner A
path_finder = DefaultAStarSC()
set_iteration_limit!(path_finder,ASTAR_ITERATION_LIMIT)
primary_route_planner = CBSSolver(ISPS(path_finder))
set_iteration_limit!(primary_route_planner,CBS_ITERATION_LIMIT)
primary_assignment_solver = TaskGraphsMILPSolver(ExtendedAssignmentMILP())
set_iteration_limit!(primary_assignment_solver,NBS_ITERATION_LIMIT)

solverA = NBSSolver(
    assignment_model=primary_assignment_solver,
    path_planner=primary_route_planner,
    return_first_feasible = true, # return if a feasible solution has been found
    )
# Backup planner
solverB = NBSSolver(
    assignment_model = TaskGraphsMILPSolver(GreedyAssignment(
        greedy_cost = GreedyFinalTimeCost(),
    )),
    path_planner = PIBTPlanner{NTuple{3,Float64}}(partial=true) # allow partial solutions
    )
set_iteration_limit!(solverB,1)
set_iteration_limit!(route_planner(solverB),PIBT_ITERATION_LIMIT)
set_debug!(solverB,true)

solver_configs = [
    (primary_solver=solverA, backup_solver=solverB, solver_name="Optimal",),
    (primary_solver=solverB, backup_solver=solverB, solver_name="Greedy",),
]
replanner_configs = [
    (
        # primary_replanner=ConstrainedMergeAndBalance(max_problem_size=300),
        primary_replanner=ConstrainedMergeAndBalance(max_tasks=60,constrain_tasks=true,constrain_nodes=false),
        backup_replanner=DeferUntilCompletion(),
        replanner_name="ConstrainedMergeAndBalance",
    ),
    (
        primary_replanner=ReassignFreeRobots(),
        backup_replanner=DeferUntilCompletion(),
        replanner_name="ReassignFreeRobots",
    ),
    (
        primary_replanner=DeferUntilCompletion(),
        backup_replanner=DeferUntilCompletion(),
        replanner_name="DeferUntilCompletion",
    ),
]

for (solver_config,replanner_config) in Base.Iterators.product(solver_configs,replanner_configs)
    plannerA = FullReplanner(
        solver=solver_config.primary_solver,
        replanner = replanner_config.primary_replanner,
        cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
    )
    set_max_time_limit!(plannerA,MAX_TIME_LIMIT)
    set_commit_threshold!(plannerA,COMMIT_THRESHOLD)
    # Backup planner
    plannerB = FullReplanner(
        solver = solver_config.backup_solver,
        replanner = replanner_config.backup_replanner,
        cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
        )
    set_max_time_limit!(plannerB,MAX_BACKUP_TIME_LIMIT)
    set_commit_threshold!(plannerB,COMMIT_THRESHOLD)
    set_debug!(plannerB,true)

    # Full planner
    planner_name=string(solver_config.solver_name,"-",replanner_config.replanner_name)
    planner_config = (
        planner=ReplannerWithBackup(deepcopy(plannerA),deepcopy(plannerB)),
        planner_name=planner_name,
        results_path=joinpath(base_results_dir,planner_name),
        objective = SumOfMakeSpans(),
    )
    push!(planner_configs,planner_config)
end

# for planner_config in planner_configs
#     # warm up to precompile replanning code
#     @info "Warming up $(planner_config.planner_name)"
#     warmup(planner_config.planner,loader)
#     @info "Profiling $(planner_config.planner_name)"
#     # profile
#     profile_replanner!(loader,
#         planner_config.planner,
#         base_problem_dir,
#         planner_config.results_path)
# end

Revise.includet(joinpath(pathof(TaskGraphs),"..","helpers/render_tools.jl"))

# # plotting results
planner_configs = filter(c->c.planner_name in Set([
    "Greedy-ConstrainedMergeAndBalance",
    "Optimal-ConstrainedMergeAndBalance",
    "Optimal-ReassignFreeRobots",
]),planner_configs)

df_dict = Dict{String,DataFrame}()
for planner_config in planner_configs
    config_df = construct_config_dataframe(loader,base_problem_dir,problem_configs[1])
    plotting_config = (
        feats = Dict(
                :makespans => Int[],
                :arrival_times => Int[],
                :completion_times => Int[],
                :backup_flags => Bool[],
                # :runtime_gaps => Float64[],
                :primary_runtimes => Float64[],
                :backup_runtimes => Float64[],
        ),
        # results_path = joinpath(base_results_dir,planner_config.planner_name),
        results_path = planner_config.results_path,
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
    "Optimal-ConstrainedMergeAndBalance"=>Dict(
        # :legend_entry   => "\$\\mergeAndBalance^\\infty\$",
        :legend_entry   => "Optimal-ConstrainedMergeAndBalance",
        :color          => "green!60!black",
        :mark           => "star",
        ),
    "Greedy-ConstrainedMergeAndBalance"=>Dict(
        # :legend_entry   => "\$\\mergeAndBalance\$",
        :legend_entry   => "Greedy-ConstrainedMergeAndBalance",
        :color          => "red",
        :mark           => "diamond",
        ),
    "Optimal-ReassignFreeRobots"=>Dict(
        # :legend_entry   => "\$\\reassignFree\$",
        :legend_entry   => "Greedy-ReassignFreeRobots",
        :color          => "blue",
        :mark           => "x",
        ),
    # # "DeferUntilCompletion"=>Dict(
    # "DeferUntilCompletion-DeferUntilCompletion"=>Dict(
    #     :legend_entry   => "\$\\deferUntilCompletion\$",
    #     :color          => "black",
    #     :mark           => "triangle",
    #     ),
    # # "NullReplanner"=>Dict(
    # "NullReplanner-DeferUntilCompletion"=>Dict(
    #     :legend_entry   => "\$\\backupOnly\$",
    #     :color          => "brown",
    #     :mark           => "+",
    #     ),
    # "NullReplanner-ConstrainedMergeAndBalance"=>Dict(
    #     :legend_entry   => "\$\\backupOnly-\\mergeAndBalance\$",
    #     :color          => "purple",
    #     :mark           => "+",
    #     ),
)

# planner_ordering = ["ConstrainedMergeAndBalance","ReassignFreeRobots","DeferUntilCompletion","NullReplanner"]
planner_ordering = [
    "Optimal-ConstrainedMergeAndBalance",
    "Greedy-ConstrainedMergeAndBalance",
    "Optimal-ReassignFreeRobots",
    ]
# legend_entries=["\\mergeAndBalance","\\reassignFree","\\deferUntilCompletion","\\backupOnly"]
# colors=["red","blue","black","brown"]

# table of rates
xkey = :M
ykey = :arrival_interval
base_table_dir = joinpath(pwd(),"case_tables")
include_keys=[:num_unique_projects=>1,]

"""
    write_tabular_results(base_table_dir,df_dict,planner_ordering;

Generate and save a table that compares results in df_dict, sorted by 
planner_ordering, on objective `obj` along axes `xkey` and `ykey`.
"""
function write_tabular_results(base_table_dir,df_dict,planner_ordering;
    xkey=:M,
    ykey=:arrival_interval,
    include_keys=[],
    exclude_keys=[],
    obj = :tasks_per_minute,
    prec = 1,
    comp_func=argmax,
    rprec=2,
    op=arr->arr[1]/maximum(arr[2:end]),
    aggregator=vals -> sum(vals)/length(vals),
    )
    tables = []
    for k in planner_ordering
        df = df_dict[k]
        tab = build_table(df;obj=obj,xkey=xkey,ykey=ykey,
            include_keys=include_keys,
            exclude_keys=exclude_keys,
            aggregator=aggregator)
        push!(tables,tab)
        # write_tex_table(joinpath(base_table_dir,"$(string(obj))_$(k).tex"),tab)
    end
    composite_table = table_product(tables)
    write_tex_table(joinpath(base_table_dir,"$(string(obj))_composite.tex"),
        composite_table;
        print_func=(io,vals)->print_multi_value_real(io,vals;precision=prec,comp_func=comp_func),
        header_printer=print_latex_header,
        row_start_printer=print_latex_row_start,
        )
    reduced_table = table_reduce(composite_table,op)
    write_tex_table(joinpath(base_table_dir,"$(string(obj))_reduced.tex"),
        reduced_table;
        print_func=(io,vals)->print_real(io,vals;precision=rprec),
        header_printer=print_latex_header,
        row_start_printer=print_latex_row_start,
        )
end

for (obj,prec,comp_func,rprec,op) in [
    (:tasks_per_minute, 1,  argmax, 2, arr->arr[1]/maximum(arr[2:end])),
    (:backlog_factor,   2,  argmin, 2, arr->minimum(arr[2:end])/arr[1]),
    (:avg_makespan,     0,  argmin, 2, arr->minimum(arr[2:end])/arr[1]),
    (:fallback_rate,    2,  argmin, 2, arr->arr[1]/maximum(arr[2:end])),
    ]
    write_tabular_results("case_tables",df_dict,planner_ordering;
        xkey=:M,
        ykey=:arrival_interval,
        include_keys=[:num_unique_projects=>1,],
        obj=obj,prec=prec,comp_func=comp_func,rprec=rprec,op,
    )
    write_tabular_results("smaller_projects",df_dict,planner_ordering;
        xkey=:M,
        ykey=:num_unique_projects,
        exclude_keys=[:num_unique_projects=>1,],
        obj=obj,prec=prec,comp_func=comp_func,rprec=rprec,op,
    )
end