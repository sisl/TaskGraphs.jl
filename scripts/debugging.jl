using TaskGraphs
using CRCBS
using LightGraphs, MetaGraphs, GraphUtils
using ImageFiltering
using Gurobi
using JuMP, MathOptInterface
using TOML
# using JLD2, FileIO
using Random
using Test
using GraphPlottingBFS
using Compose
# load rendering tools
include(joinpath(pathof(TaskGraphs),"..","helpers/render_tools.jl"))
# for f in *.svg; do inkscape -z $f -e $f.png; done

# Visualizing A star
let
    env_id = 2
    env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
    factory_env = read_env(env_filename)

    validate_edge_cache(factory_env,factory_env.vtxs,factory_env.edge_cache)

    file_name = joinpath(DEBUG_PATH,"A_star_dump_54.jld2")
    dict = load(file_name)
    agent_id = dict["agent_id"]
    history = dict["history"]
    start   = dict["start"]
    goal    = dict["goal"]
    robot_paths = map(p->p.nzval, dict["paths"])
    state_constraints = dict["state_constraints"]
    action_constraints = dict["action_constraints"]

    # findall(map(p->p[212],robot_paths) .== 550)
    # robot_paths[24][170:end]

    t0 = start[2]
    history = history[1:400]
    empty!(robot_paths[agent_id])
    start,goal

    # Render video clip
    set_default_plot_size(24cm,24cm)
    record_video(joinpath(VIDEO_DIR,string("A_star_debug.webm")),
        k->render_search(history[k][2],factory_env;
            robot_paths=robot_paths,
            search_patterns=[history],
            goals=[goal[1]],
            search_idx=k,
            show_search_paths=true,
            # search_size=1.0pt,
            # goal_size=1.0pt,
            colors_vec=map(i->LCHab(60,80,200),1:length(robot_paths)),
            show_paths=false,
            );
            t_history=1:length(history)
            )

end
