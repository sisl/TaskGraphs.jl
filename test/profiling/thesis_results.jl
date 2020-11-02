using TaskGraphs, CRCBS, TOML, DataFrames
Revise.includet(joinpath(pathof(TaskGraphs),"..","helpers/render_tools.jl"))

## ICRA experiments
base_dir = joinpath("/scratch/task_graphs_experiments")
problem_dir = joinpath(base_dir,"pctapf_problem_instances")
base_results_path = joinpath(base_dir,"pcta_results")

feats = [
    RunTime(),
    FeasibleFlag(),
    OptimalFlag(),
    OptimalityGap(),
    SolutionCost(),
    PrimaryCost(),
    SolutionAdjacencyMatrix(),
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
write_problems!(loader,problem_configs,problem_dir)
for solver_config in solver_configs
    set_runtime_limit!(solver_config.solver,100)
    warmup(loader,solver_config,problem_dir)
    run_profiling(loader,solver_config,problem_dir)
end

# plot results
config_template = problem_configs[1]
config_df = construct_config_dataframe(loader,problem_dir,config_template)
for solver_config in solver_configs
    results_df = construct_results_dataframe(loader,solver_config,config_template)
    df = innerjoin(config_df,results_df,on=:problem_name)
    include(joinpath(pathof(TaskGraphs),"..","helpers/render_tools.jl"))
    # plt = get_box_plot_group_plot(df;obj=:RunTime,title=split(solver_config.results_path,"/")[end])
    plt = get_titled_group_box_plot(df;scale=1.0,obj=:RunTime,title=split(solver_config.results_path,"/")[end])
    display(plt);
end



################################################################################
#################################### Videos ####################################
################################################################################

# [ route plan ] [ schedule ]
rstack = RenderStack(
    (
    object_model = ObjectPositions(3),
    robot_model = RobotPositions(2),
    floor_model = Floor()
    )
)
rstack.models.robot_model.rsize = 8pt
rstack.models.robot_model.show_paths = false
rstack.models.robot_model.label_robots = true
rstack.models.robot_model.colors_vec .= ROBOT_COLOR
rstack.models.object_model.osize = 4pt
rstack.models.object_model.label_objects = true
rstack.models.object_model.inactive_object_colors .= colorant"Black"
rstack.models.object_model.completed_object_colors .= colorant"Black"
rstack.models.floor_model.pickup_color = colorant"LightGray"
rstack.models.floor_model.dropoff_color = colorant"LightGray"

# tiny PC_TAPF
solver = NBSSolver()
prob = pctapf_problem_1(solver;Δt_op=1,Δt_collect=[1,1,1],Δt_deliver=[1,1,1])
env,cost = solve!(solver,prob)
summary = SolutionSummary(env)
rstack.models.robot_model.colors_vec = map(r->ROBOT_COLOR,1:num_agents(env))

t = 2.5
plts = Vector{Union{Gadfly.Plot,Compose.Context}}()
sched_plot = plot_schedule_snapshot(env,t)
env_plot = render_env(rstack,get_graph(env),summary,t)
push!(plts,env_plot)
push!(plts,sched_plot)
Compose.set_default_graphic_size(
    Compose.default_graphic_width,
    Compose.default_graphic_height*2,
    )
Compose.vstack(plts)


outfile = "pibt_debug.mp4"
t0 = minimum(map(p->findlast(p .!= p[end]), summary.robot_paths))
tF = maximum(map(p->findlast(p .!= p[end]), summary.robot_paths)) + 1

Compose.set_default_graphic_size(20cm,20cm)
env_pic = render_env(rstack,env,summary,t0+143.0;
    point_label_font_size=8pt,
)
env_pic |> SVG("env_pic.svg",8inch,8inch)

record_video(outfile,t->render_env(rstack,env,summary,t;point_label_font_size=8pt);
    t0 = t0,
    tf = tF,
    dt = 0.25,
    fps = 10,
    s = (20cm,20cm),
)


# large PC_TAPF
