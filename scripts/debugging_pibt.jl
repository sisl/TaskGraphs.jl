problem_name = "problem0001"
problem_file = joinpath(base_problem_dir,problem_name)
results_path = joinpath(planner_configs[4].results_path,problem_name)
results = load_results(loader,results_path)
config = CRCBS.load_config(loader,problem_file)
TaskGraphs.post_process_replanning_results!(results,config)


stage = 1
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
rstack.models.robot_model.show_paths = false
rstack.models.robot_model.label_robots = false
rstack.models.robot_model.colors_vec .= ROBOT_COLOR
rstack.models.object_model.osize = 4pt
rstack.models.object_model.label_objects = false
rstack.models.object_model.inactive_object_colors .= colorant"Black"
rstack.models.object_model.completed_object_colors .= colorant"Black"
rstack.models.floor_model.pickup_color = colorant"LightGray"
rstack.models.floor_model.dropoff_color = colorant"LightGray"


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
