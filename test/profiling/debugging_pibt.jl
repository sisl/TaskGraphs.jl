results_path = joinpath(planner_configs[1].results_path,"problem0001")
results = load_results(loader,results_path)

stage = 3
stage_results = results[:backup_planner][stage]
stage_results[:RobotPaths]

include(joinpath(pathof(TaskGraphs),"..","helpers/render_tools.jl"))

env = get_env!(loader,"env_2")
visualize_env(env,0.5;robot_paths=stage_results[:RobotPaths])

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
