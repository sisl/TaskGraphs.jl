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


## Videos
rstack = RenderStack(
    (
    object_model = ObjectPositions(60),
    robot_model = RobotPositions(60),
    floor_model = Floor()
    )
)
rstack.models.robot_model.rsize = 8pt
rstack.models.robot_model.show_paths = false
rstack.models.robot_model.label_robots = false
rstack.models.robot_model.colors_vec .= ROBOT_COLOR
rstack.models.object_model.osize = 4pt
rstack.models.object_model.label_objects = false
rstack.models.object_model.inactive_object_colors .= colorant"Black"
rstack.models.object_model.completed_object_colors .= colorant"Gray"
rstack.models.floor_model.pickup_color = colorant"LightGray"
rstack.models.floor_model.dropoff_color = colorant"LightGray"

prob_list = []
solver = NBSSolver()
# tiny PC_TAPF
prob = pctapf_problem_1(solver;Δt_op=1,Δt_collect=[1,1,1],Δt_deliver=[1,1,1])
push!(prob_list,("small_pctapf.mp4",prob))
prob = C_PC_TAPF(pctapf_problem_11(solver;Δt_op=1,Δt_collect=[1,1,1],Δt_deliver=[1,1,1]))
push!(prob_list,("small_cpctapf.mp4",prob));
for (outpath,prob) in prob_list
    reset_solver!(solver)
    env,cost = solve!(solver,prob)
    env_summary = SolutionSummary(env)
    o_keys = collect(keys(get_object_ICs(get_schedule(env))))
    rstack.models.object_model.object_sizes = map(k->get_node_from_id(get_schedule(env),k).shape, sort(o_keys))

    t = 3
    render_env_and_schedule("output.png",rstack,env,t;verbose=false,pix_width=1024)

    save_function = (t,path,s)->render_env_and_schedule(path,rstack,env,t;pix_width = 1024)
    record_video(outpath,nothing,save_function;
        t0 = 0,
        tf = min(maximum(map(length,env_summary.robot_paths)),50),
        dt = 0.25,
        fps = 10,
        s = (4cm,4cm),
    )
end

# large PC_TAPF
env_graph = init_env_2()
dx = get_graph(env).x_dim
dy = get_graph(env).y_dim
Compose.set_default_graphic_size(dx*1cm,dy*1cm)

prob_list = []
def = read_problem_def("/scratch/task_graphs_experiments/problem_instances/problem384.toml")
prob = PC_TAPF(solver,def,env_graph)
push!(prob_list, ("large_pctapf.mp4", prob))

# def = read_problem_def("/scratch/task_graphs_experiments/collaborative_problem_instances/problem33.toml")
# def = read_problem_def("/scratch/task_graphs_experiments/collaborative_problem_instances/problem96.toml")
def = read_problem_def("/scratch/task_graphs_experiments/collaborative_problem_instances/problem189.toml")
prob = C_PC_TAPF(PC_TAPF(solver,def,env_graph).env);
push!(prob_list, ("large_cpctapf.mp4", prob))

for (outpath,prob) in prob_list
    reset_solver!(solver)
    env,cost = solve!(solver,prob);

    M = length(get_object_ICs(get_schedule(env)))
    env_summary = SolutionSummary(env)
    o_keys = collect(keys(get_object_ICs(get_schedule(env))))
    rstack.models.object_model.object_sizes = map(k->get_node_from_id(get_schedule(env),k).shape, sort(o_keys))
    rstack.models.robot_model.colors_vec = map(r->ROBOT_COLOR,1:num_agents(env))
    rstack.models.object_model.active_object_colors = map(r->colorant"black",1:M)
    rstack.models.object_model.completed_object_colors = map(r->colorant"black",1:M)
    rstack.models.object_model.inactive_object_colors = map(r->colorant"black",1:M)

    record_video(prob,t->render_env(rstack,get_graph(env),env_summary,t);
        t0 = 0,
        tf = min(maximum(map(length,env_summary.robot_paths)),50),
        dt = 0.25,
        fps = 10,
        s = (20cm,20cm),
    )
end

## Clean Up Bot video
# prob = pctapf_problem_1(solver)
prob = pctapf_problem_1(solver;Δt_op=1,Δt_collect=[1,1,1],Δt_deliver=[1,1,1])
s_prob = TaskGraphs.stochastic_problem(
    PC_TAPF,
    solver,
    prob,
    [CUB_AT(3,6)],
    DisturbanceSequence([2=>DroppedObject(1)])
)
t_drop = 3
reset_solver!(solver)
base_env, cost = solve!(solver,s_prob.prob)
new_env = deepcopy(base_env)
base_env = handle_disturbance!(solver,s_prob,base_env,DroppedObject(1),t_drop)
reset_solver!(solver)
env, cost = solve!(solver,PC_TAPF(base_env))

dx = get_graph(env).x_dim
dy = get_graph(env).y_dim
Compose.set_default_graphic_size(dx*1cm,dy*1cm)

plot_project_schedule(env)

env_summary = SolutionSummary(env)
o_keys = collect(keys(get_object_ICs(get_schedule(env))))
rstack.models.object_model.object_sizes = map(k->get_node_from_id(get_schedule(env),k).shape, sort(o_keys))
rstack.models.robot_model.colors_vec[3] = colorant"red" # CleanUpBot

t = 1
render_env_and_schedule("output.png",rstack,new_env,t;verbose=false,pix_width=1024)
render_env_and_schedule("output.png",rstack,env,t;verbose=false,pix_width=1024)


save_function = (t,path,s)->render_env_and_schedule(path,rstack,new_env,t;pix_width = 1024)
record_video("tiny_stochastic_pre.mp4",nothing,save_function;
    t0 = 0,
    tf = t_drop,
    dt = 0.25,
    fps = 10,
    s = (4cm,4cm),
)
save_function = (t,path,s)->render_env_and_schedule(path,rstack,env,t;pix_width = 1024)
record_video("tiny_stochastic_post.mp4",nothing,save_function;
    t0 = t_drop,
    tf = min(maximum(map(length,env_summary.robot_paths)),50),
    dt = 0.25,
    fps = 10,
)

## Replanning
prob = replanning_problem_1(solver;spacing=4)
planner = FullReplanner(solver=solver,replanner=MergeAndBalance())
set_commit_threshold!(planner,2)
set_real_time_flag!(planner.replanner,false) # turn off real time constraints
set_deadline!(solver,Inf)
set_runtime_limit!(solver,100)

dx = get_graph(prob.env).x_dim
dy = get_graph(prob.env).y_dim
Compose.set_default_graphic_size(dx*1cm,dy*1cm)
# request = prob.requests[1]
# env = get_env(prob)
# reset_solver!(planner.solver)
# remap_object_ids!(request.schedule,get_schedule(env))
# base_env = replan!(planner,env,request)
# env, cost = solve!(planner.solver,construct_routing_problem(prob,base_env))
function record_tiny_replanner(outpath,rstack,planner,prob)
    reset_cache!(planner)
    env = get_env(prob)
    t0 = 0
    for (stage,request) in enumerate(prob.requests)
        if stage > 1
            if stage == length(prob.requests)
                tf = maximum(map(length,get_paths(get_route_plan(env))))
            else
                tf = request.t_request
            end
            M = length(get_object_ICs(get_schedule(env)))
            env_summary = SolutionSummary(env)
            o_keys = collect(keys(get_object_ICs(get_schedule(env))))
            rstack.models.object_model.object_sizes = map(k->get_node_from_id(get_schedule(env),k).shape, sort(o_keys))
            rstack.models.robot_model.colors_vec = map(r->ROBOT_COLOR,1:num_agents(env))
            rstack.models.object_model.active_object_colors = map(r->colorant"black",1:M)
            rstack.models.object_model.completed_object_colors = map(r->colorant"black",1:M)
            rstack.models.object_model.inactive_object_colors = map(r->colorant"black",1:M)
            save_function = (t,path,s)->render_env_and_schedule(path,rstack,env,t;pix_width = 1024)
            record_video(joinpath(outpath,"tiny_replanning_$(stage-1).mp4"),nothing,save_function;
                t0 = t0,
                tf = tf,
                dt = 0.25,
                fps = 10,
            )
            t0 = tf
        end
        reset_solver!(planner.solver)
        remap_object_ids!(request.schedule,get_schedule(env))
        base_env = replan!(planner,env,request)
        env, cost = solve!(planner.solver,construct_routing_problem(prob,base_env))
    end
end
record_tiny_replanner("",rstack,planner,prob)

## Large replanning videos

function record_large_replanner(outpath,rstack,planner,prob)
    reset_cache!(planner)
    env = get_env(prob)
    t0 = 0
    for (stage,request) in enumerate(prob.requests)
        if stage > 1
            if stage == length(prob.requests)
                tf = maximum(map(length,get_paths(get_route_plan(env))))
            else
                tf = request.t_request
            end
            M = length(get_object_ICs(get_schedule(env)))
            env_summary = SolutionSummary(env)
            o_keys = collect(keys(get_object_ICs(get_schedule(env))))
            rstack.models.object_model.object_sizes = map(k->get_node_from_id(get_schedule(env),k).shape, sort(o_keys))
            rstack.models.robot_model.colors_vec = map(r->ROBOT_COLOR,1:num_agents(env))
            rstack.models.object_model.active_object_colors = map(r->colorant"black",1:M)
            rstack.models.object_model.completed_object_colors = map(r->colorant"black",1:M)
            rstack.models.object_model.inactive_object_colors = map(r->colorant"black",1:M)

            record_video(joinpath(outpath,"large_replanning_$(stage-1).mp4"),t->render_env(rstack,get_graph(env),env_summary,t);
                t0 = t0,
                tf = tf,
                dt = 0.25,
                fps = 10,
                s = (20cm,20cm),
            )
            t0 = tf
        end
        reset_solver!(planner.solver)
        remap_object_ids!(request.schedule,get_schedule(env))
        base_env = replan!(planner,env,request)
        env, cost = solve!(planner.solver,construct_routing_problem(prob,base_env))
        @show cost
    end
end

path_finder = DefaultAStarSC()
set_iteration_limit!(path_finder,5000)
primary_route_planner = CBSSolver(ISPS(path_finder))
set_iteration_limit!(primary_route_planner,1000)
primary_planner = FullReplanner(
    solver = NBSSolver(path_planner=primary_route_planner),
    replanner = MergeAndBalance(),
    )
set_max_time_limit!(primary_planner,50)
set_commit_threshold!(primary_planner,10)
# Backup planner
backup_planner = FullReplanner(
    solver = NBSSolver(
        assignment_model = TaskGraphsMILPSolver(GreedyAssignment()),
        path_planner = PIBTPlanner{NTuple{3,Float64}}(partial=true)
        ),
    replanner = DeferUntilCompletion(),
    )
set_iteration_limit!(backup_planner,1)
set_iteration_limit!(route_planner(backup_planner.solver),5000)
set_commit_threshold!(backup_planner,10)
set_debug!(backup_planner,true)
# set_debug!(route_planner(backup_planner.solver),true)
# Full solver
planner = ReplannerWithBackup(primary_planner,backup_planner)

loader = ReplanningProblemLoader()
add_env!(loader,"env_2",init_env_2())
prob_file = "/scratch/task_graphs_experiments/replanning3/problem_instances/problem0083"
simple_prob_def = read_simple_repeated_problem_def(prob_file)
prob = RepeatedPC_TAPF(simple_prob_def,planner.primary_planner.solver,loader)

deleteat!(prob.requests,5:30)
dt = 16
global t = 10
for i in 1:length(prob.requests)
    prob.requests[i] = ProjectRequest(prob.requests[i],t_request=t,t_arrival=t)
    global t += dt
end
prob.requests[3]
length(prob.requests)

dx = get_graph(prob.env).x_dim
dy = get_graph(prob.env).y_dim
Compose.set_default_graphic_size(dx*1cm,dy*1cm)

record_large_replanner("",rstack,planner.primary_planner,prob)
