results_path = joinpath(planner_configs[1].results_path,"problem0150")
results = load_results(loader,results_path)

stage = 15
stage_results = results[:backup_planner][stage]
stage_results[:RobotPaths]

include(joinpath(pathof(TaskGraphs),"..","helpers/render_tools.jl"))

env = get_env!(loader,"env_2")
# visualize_env(env,0.5;robot_paths=stage_results[:RobotPaths])

object_paths, object_intervals = TaskGraphs.reconstruct_object_paths(
    stage_results[:RobotPaths],stage_results[:ObjectPathSummaries]
)
summary = SolutionSummary(
    robot_paths = stage_results[:RobotPaths],
    object_paths = object_paths,
    object_intervals = object_intervals
)

rstack = RenderStack(
    (
    object_model = ObjectPositions(length(summary.object_paths)),
    robot_model = RobotPositions(length(summary.robot_paths)),
    floor_model = Floor()
    )
)
rstack.models.robot_model.rsize = 8pt
rstack.models.robot_model.show_paths = true
rstack.models.object_model.osize = 4pt
rstack.models.object_model.label_objects = true

Compose.set_default_graphic_size(20cm,20cm)
render_env(rstack,env,summary,0.5;
    point_label_font_size=8pt,
)

outfile = "pibt_debug.mp4"
record_video(outfile,t->render_env(rstack,env,summary,t;point_label_font_size=8pt);
    t0 = 0.0,
    tf = 71.0,
    dt = 0.25,
    s = (20cm,20cm),
)

########## Debugging main solver

planner = planner_configs[2].planner
problem_name = "problem0090"
f = joinpath(base_problem_dir,problem_name)
outpath = joinpath(base_results_dir,problem_name)
simple_prob_def = read_simple_repeated_problem_def(f)
prob = RepeatedPC_TAPF(simple_prob_def,planner.primary_planner.solver,loader)
# search_env, planner = profile_replanner!(planner,prob)

reset_cache!(planner)
plannerA = planner.primary_planner
plannerB = planner.backup_planner
hard_reset_solver!(planner.primary_planner.solver)
hard_reset_solver!(planner.backup_planner.solver)
env = prob.env

stage = 0

stage += 1
request = prob.requests[stage];

remap_object_ids!(request.schedule,env.schedule);

base_envB = replan!(plannerB,env,request);
envB, resultsB = profile_solver!(plannerB.solver,construct_routing_problem(prob,base_envB));
# compile_replanning_results!(plannerB.cache,plannerB.solver,envB,
#     resultsB,prob,stage,request)

base_envA = replan!(plannerA,env,request);
# pctapf = construct_routing_problem(prob,base_envA);
# envA, cost = solve!(plannerA.solver,pctapf);
envA, resultsA = profile_solver!(plannerA.solver,pctapf);
# compile_replanning_results!(plannerA.cache,plannerA.solver,envA,
#     resultsA,prob,stage,request)

# if failed_status(plannerA) == false
if feasible_status(plannerA)
    @log_info(-1,0,"REPLANNING: ",
    "Primary planner succeeded at stage $stage.")
    env = envA;
elseif feasible_status(plannerB)
    @log_info(-1,0,"REPLANNING: ",
    "Primary planner failed at stage $stage. Proceeding with backup plan.")
    env = envB;
else
    @log_info(-1,0,"REPLANNING:",
        "Both primary and backup planners failed at stage $stage.",
        " Returning early.")
end;
