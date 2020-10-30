import Cairo #, Fontconfig
using GraphPlottingBFS
using Compose
using Colors
using Gadfly
using DataFrames
using GraphUtils

using PGFPlotsX
latexengine!(PGFPlotsX.PDFLATEX)
using PGFPlots
using Printf
using Parameters
using Measures
using MetaGraphs

ROBOT_COLOR = RGB(0.1,0.5,0.9)

interpolate(a,b,t) = (1 - t)*a + t*b

function interp_position(path::Vector{Int},vtxs,t)
    t1 = Int(floor(t))+1
    idx1 = path[max(1,min(t1,length(path)))]
    idx2 = path[max(1,min(t1+1,length(path)))]
    x=interpolate(vtxs[idx1][1],vtxs[idx2][1],t-(t1-1))
    y=interpolate(vtxs[idx1][2],vtxs[idx2][2],t-(t1-1))
    return x,y
end
function interp_position(paths::Vector{Vector{Int}},vtxs,t)
    coords = map(p->interp_position(p,vtxs,t),paths)
end
function interp_path(path::Vector{Int},vtxs,t)
    x,y = interp_position(path,vtxs,t)
    t1 = Int(floor(t))+1
    idx1 = max(1,min(t1,length(path)))
    idx2 = max(1,min(t1+1,length(path)))
    xpath = [x, map(v->v[1],vtxs[path[idx2:end]])...]
    ypath = [y, map(v->v[2],vtxs[path[idx2:end]])...]
    return xpath,ypath
end

function gen_object_colors(n)
  cs = distinguishable_colors(n,
      [colorant"#FE4365", colorant"#eca25c"], # seed colors
      lchoices=Float64[58, 45, 72.5, 90],     # lightness choices
      transform=c -> deuteranopic(c, 0.1),    # color transform
      cchoices=Float64[20,40],                # chroma choices
      hchoices=[75,51,35,120,180,210,270,310] # hue choices
  )

  convert(Vector{Color}, cs)
end

function record_video(outfile_name,render_function;
        t0 = 0,
        tf = 10,
        dt = 0.25,
        t_history=t0:dt:tf,
        fps = 10,
        ext = "png",
        s = (4inch, 4inch),
        res = "1080x1080"
    )
    tmpdir = mktempdir()
    for (i,t) in enumerate(t_history)
        filename = joinpath(tmpdir,string(i,".",ext))
        render_function(t) |> PNG(filename, s[1], s[2])
    end
    outfile = outfile_name
    run(pipeline(`ffmpeg -y -r $fps -f image2 -i $tmpdir/%d.$ext -crf 20 $outfile`, stdout=devnull, stderr=devnull))
    run(pipeline(`rm -rf $tmpdir`, stdout=devnull, stderr=devnull))
end

function get_grid_world_layer(vtxs,color,cw)
    if length(vtxs) > 0
        return layer(
            xmin=map(vec->vec[1], vtxs) .- cw,
            ymin=map(vec->vec[2], vtxs) .- cw,
            xmax=map(vec->vec[1], vtxs) .+ cw,
            ymax=map(vec->vec[2], vtxs) .+ cw,
            Geom.rect, Theme(default_color=color) )
    else
        return layer(x=[],y=[])
    end
end

@with_kw struct SolutionSummary
    robot_paths::Vector{Vector{Int}} = Vector{Vector{Int}}()
    object_paths::Dict{Int,Vector{Int}} = Dict{Int,Vector{Int}}()
    object_intervals::Dict{Int,Tuple{Int,Int}} = Dict{Int,Tuple{Int,Int}}()
end

abstract type RenderModel end
struct RenderStack{T}
    models::T
end
@with_kw mutable struct RobotPositions <: RenderModel
    label_robots    ::Bool              = true
    colors_vec      ::Vector{Color}     = Color[]
    rsize           ::Measures.Length   = 5pt
    line_width      ::Measures.Length   = 2pt
    show_paths      ::Bool              = false
end
function RobotPositions(n::Int)
    color_scale = Scale.color_discrete_hue()
    RobotPositions(colors_vec = color_scale.f(n))
end
@with_kw mutable struct ObjectPositions <: RenderModel
    label_objects           ::Bool          = true
    show_inactive_objects   ::Bool          = true
    # show_completed_objects   ::Bool          = true
    osize                   ::Measures.Length = 5pt
    inactive_object_colors  ::Vector{Color} = Color[]
    completed_object_colors ::Vector{Color} = Color[]
    active_object_colors    ::Vector{Color} = Color[]
end
function ObjectPositions(n::Int)
    ObjectPositions(
        active_object_colors = map(i->colorant"Black",1:n),
        inactive_object_colors = map(i->colorant"Gray",1:n),
        completed_object_colors = map(i->colorant"Gray",1:n),
    )
end
@with_kw mutable struct Floor <: RenderModel
    cell_width      ::Float64   = 0.9
    floor_color     ::Color     = colorant"LightGray"
    pickup_color    ::Color     = colorant"LightBlue"
    dropoff_color   ::Color     = colorant"Pink"
end
function render_layer(model::RobotPositions,env::GridFactoryEnvironment,summary,t,theme=Theme())
    robot_paths = summary.robot_paths
    vtxs = get_vtxs(env)
    robot_layers = []
    path_layers = []
    for (i,p) in enumerate(robot_paths)
        if length(p) > 0
            x,y = interp_position(p,vtxs,t)
            label = model.label_robots ? string("R",i) : ""
            push!(robot_layers,
                layer(
                    x=[x],
                    y=[y],
                    label=[label],
                    Geom.label(position=:centered,hide_overlaps=false),
                    Geom.point,
                    size=[model.rsize],
                    Theme(
                        theme,
                        default_color=model.colors_vec[i],
                        # highlight_width=0pt,
                        # point_label_font=point_label_font,
                        # point_label_color=point_label_color,
                        # point_label_font_size=point_label_font_size
                        )
                    )
            )
            if model.show_paths
                xpath,ypath = interp_path(p,vtxs,t)
                push!(path_layers,
                    layer(x=xpath,y=ypath,Geom.path,
                        Theme(theme,line_width=model.line_width,default_color=model.colors_vec[i])
                    )
                )
            end
        end
    end
    return [robot_layers..., path_layers...]
end
function render_layer(model::ObjectPositions,env::GridFactoryEnvironment,summary,t,theme=Theme())
    object_paths = summary.object_paths
    object_intervals = summary.object_intervals
    object_layers = []
    for (i,k) in enumerate(sort(collect(keys(object_paths)))) # dictionary
        p = object_paths[k]
        interval = object_intervals[k]
        if interval[1] > t
            object_color = model.inactive_object_colors[i]
        elseif interval[2] < t
            object_color = model.completed_object_colors[i]
        else
            object_color = model.active_object_colors[i]
        end
        if length(p) > 0 && interval[2] > t
            if (interval[1] <= t) #|| model.show_inactive_objects
                x,y = interp_position(p,get_vtxs(env),t)
                label = model.label_objects ? string("O",k) : ""
                push!(object_layers,
                    layer(
                        x=[x],
                        y=[y],
                        label=[label],
                        Geom.label(position=:centered,hide_overlaps=false),
                        Geom.point,
                        size=[model.osize],
                        Theme(
                            theme,
                            default_color=object_color,
                            )
                        )
                )
            end
        end
    end
    return object_layers
end
function render_layer(model::Floor,env::GridFactoryEnvironment,summary,t,theme=Theme())
    map(args->get_grid_world_layer(args...,model.cell_width/2.0),
        [
            (get_pickup_vtxs(env),  model.pickup_color),
            (get_dropoff_vtxs(env), model.dropoff_color),
            (get_vtxs(env),         model.floor_color),
        ]
    )
end

function render_env(rstack::RenderStack,env,summary,t;
        cell_width=1.0,
        plot_padding=[1mm],
        background_color="Gray",
        highlight_width=0pt,
        point_label_font="arial",
        point_label_color="White",
        point_label_font_size=10pt,
        theme = Theme(
            plot_padding=plot_padding,
            background_color=background_color,
            highlight_width=highlight_width,
            point_label_font=point_label_font,
            point_label_color=point_label_color,
            point_label_font_size=point_label_font_size,
        )
    )
    layers = vcat(map(m->render_layer(m,env,summary,t,theme),rstack.models)...)
    xpts = map(v->v[1],get_vtxs(env))
    ypts = map(v->v[2],get_vtxs(env))
    Gadfly.plot(
        layers...,
        Coord.cartesian(fixed=true,
            xmin=minimum(xpts) - cell_width/2,
            ymin=minimum(ypts) - cell_width/2,
            xmax=maximum(xpts) + cell_width/2,
            ymax=maximum(ypts) + cell_width/2),
        Guide.xticks(;ticks=nothing),
        Guide.yticks(;ticks=nothing),
        Guide.xlabel(nothing),
        Guide.ylabel(nothing),
        theme
    )
end

"""
    `visualize_env`

    Displays the current agent and object locations as well as the current
    planned path segment of each agent.
"""
function visualize_env(search_env::S,vtxs,pickup_vtxs,dropoff_vtxs,t=0;
        robot_vtxs=[],
        object_vtxs=[],
        robot_paths=[],
        object_paths=[],
        search_patterns=[],
        goals=[],
        vpad = 0.05,
        n=80,
        rsize=5pt,
        osize=3pt,
        cell_width=1.0,
        floor_color = "LightGray",
        pickup_color= "LightBlue",
        dropoff_color="Pink",
        line_width=2pt) where {S<:SearchEnv}

    cache = get_cache(search_env)
    project_schedule = get_schedule(search_env)

    color_scale = Scale.color_discrete_hue()
    colors_vec = color_scale.f(n)

    cw = cell_width/2 - vpad
    xpts = map(vtx->cell_width*vtx[1], vtxs)
    ypts = map(vtx->cell_width*vtx[2], vtxs)

    t1 = Int(floor(t))+1
    path_layers = []
    for v in 1:get_num_vtxs(project_schedule)
        vtx_id = get_vtx_id(project_schedule, v)
        if typeof(vtx_id) <: ActionID
            node = get_node_from_id(project_schedule,vtx_id)
            if (cache.t0[v] <= t) && (cache.tF[v] >= t)
                spec = get_path_spec(project_schedule, v)
                agent_id = get_id(spec.agent_id)
                agent_path = robot_paths[agent_id]
                p = agent_path[max(1,min(t1+1,cache.tF[v]+1)):cache.tF[v]+1]
                if length(p) > 0
                    idx1 = agent_path[max(1,min(t1,length(agent_path)))]
                    idx2 = agent_path[max(1,min(t1+1,length(agent_path)))]
                    x0=interpolate(vtxs[idx1][1],vtxs[idx2][1],t-(t1-1))
                    y0=interpolate(vtxs[idx1][2],vtxs[idx2][2],t-(t1-1))
                    push!(path_layers,
                        layer(
                            x=[x0, map(v->vtxs[v][1],p)...],
                            y=[y0, map(v->vtxs[v][2],p)...],Geom.path,
                            Theme(line_width=line_width,default_color=colors_vec[agent_id]))
                    )
                end
            end
            # end
        end
    end
    robot_layers = []
    for (i,p) in enumerate(robot_paths)
        if length(p) > 0
            idx1 = p[max(1,min(t1,length(p)))]
            idx2 = p[max(1,min(t1+1,length(p)))]
            push!(robot_layers,
                layer(
                    x=[interpolate(vtxs[idx1][1],vtxs[idx2][1],t-(t1-1))],
                    y=[interpolate(vtxs[idx1][2],vtxs[idx2][2],t-(t1-1))],
                    Geom.point,size=[rsize],Theme(default_color=colors_vec[i]))
            )
        end
    end
    search_pattern_layers = []
    for (i,p) in enumerate(search_patterns)
        df = DataFrame(x=map(s->vtxs[s[1]][1],p),y=map(s->vtxs[s[1]][2],p),t=map(s->s[2],p))
        push!(path_layers, layer(df,x="x",y="y",color="t",Geom.point,size=[3pt]))
    end
    goal_layers = []
    for (i,s) in enumerate(goals)
        push!(goal_layers,
            layer(x=[vtxs[s[1]][1]],y=[vtxs[s[1]][2]],Geom.point,size=[vsize],shape=[Shape.diamond],Theme(
                    default_color=colors_vec[i]))
        )
    end
    object_layers = []
    for (i,p) in enumerate(object_paths)
        if length(p) > 0
            interpolate(p[min(t1,length(p))],p[min(t1+1,length(p))],t1-(t+1))
            idx1 = p[max(1,min(t1,length(p)))]
            idx2 = p[max(1,min(t1+1,length(p)))]
            push!(object_layers,
                layer(
                    x=[interpolate(vtxs[idx1][1],vtxs[idx2][1],t-(t1-1))],
                    y=[interpolate(vtxs[idx1][2],vtxs[idx2][2],t-(t1-1))],
                    Geom.point,size=[osize],Theme(default_color="Black"))
            )
        end
    end
    # object_layer = layer( x=map(v->vtxs[v][1], object_vtxs), y=map(v->vtxs[v][2], object_vtxs),
    # size=[3pt], Geom.point, Theme(default_color="Black") )

    plot(
        object_layers...,
        # object_layer,
        robot_layers...,
        path_layers...,
        # search_pattern_layers...,
        get_grid_world_layer(pickup_vtxs,pickup_color,cw),
        get_grid_world_layer(dropoff_vtxs,dropoff_color,cw),
        get_grid_world_layer(vtxs,floor_color,cw),
        Coord.cartesian(fixed=true,
            xmin=minimum(xpts) - cell_width/2,
            ymin=minimum(ypts) - cell_width/2,
            xmax=maximum(xpts) + cell_width/2,
            ymax=maximum(ypts) + cell_width/2),
        Guide.xticks(;ticks=nothing),
        Guide.yticks(;ticks=nothing),
        Guide.xlabel(nothing),
        Guide.ylabel(nothing),
        Theme(
            plot_padding=[1mm],
            # default_color="LightGray",
            background_color="Gray"
        )
    )
end
function visualize_env(search_env::S,env::E,t=0;kwargs...) where {S<:SearchEnv,E<:GridFactoryEnvironment}
    visualize_env(search_env,get_vtxs(env),get_pickup_vtxs(env),get_dropoff_vtxs(env),t;kwargs...)
end
function visualize_env(vtxs,pickup_vtxs,dropoff_vtxs,t=0;
        robot_vtxs=[],
        object_vtxs=[],
        robot_paths=[],
        object_paths=[],
        object_intervals=map(p->[0,length(p)], object_paths),
        search_idx=1,
        show_search_paths=false,
        search_patterns=[],
        goals=[],
        vpad = 0.05,
        n=80,
        rsize=4pt,
        osize=3pt,
        search_size=1.5pt,
        goal_size=2.0pt,
        cell_width=1.0,
        line_width=2pt,
        color_scale = Scale.color_discrete_hue(),
        colors_vec = color_scale.f(n),
        floor_color = "LightGray",
        pickup_color= "LightBlue",
        dropoff_color="Pink",
        active_object_color="Black",
        project_idxs=map(i->1,object_paths),
        proj_colors_vec = gen_object_colors(maximum([1,project_idxs...])),
        active_object_colors=map(idx->proj_colors_vec[idx],project_idxs),
        label_objects=false,
        label_robots=false,
        point_label_font="arial",
        point_label_color="White",
        point_label_font_size=10pt,
        inactive_object_color="Gray",
        inactive_object_colors=map(idx->inactive_object_color,project_idxs),
        completed_object_color="Gray",
        completed_object_colors=map(idx->completed_object_color,project_idxs),
        show_inactive_objects=true,
        show_paths=true
    )

    cw = cell_width/2 - vpad
    vtxs = map(vtx->(cell_width*vtx[1],cell_width*vtx[2]),vtxs)
    xpts = map(vtx->vtx[1], vtxs)
    ypts = map(vtx->vtx[2], vtxs)

    theme = Theme(
        plot_padding=[1mm],
        background_color="Gray",
        highlight_width=0pt,
        point_label_font=point_label_font,
        point_label_color=point_label_color,
        point_label_font_size=point_label_font_size
    )

    t1 = Int(floor(t))+1
    path_layers = []
    if show_paths
        for (i,p) in enumerate(robot_paths)
            if length(p) > 0
                push!(path_layers,
                    layer(x=map(v->vtxs[v][1],p),y=map(v->vtxs[v][2],p),Geom.path,
                        Theme(theme,line_width=line_width,default_color=colors_vec[i]))
                )
            end
        end
    end
    robot_layers = []
    for (i,p) in enumerate(robot_paths)
        if length(p) > 0
            # interpolate(p[min(t1,length(p))],p[min(t1+1,length(p))],t1-(t+1))
            idx1 = p[max(1,min(t1,length(p)))]
            idx2 = p[max(1,min(t1+1,length(p)))]
            label = label_robots ? string("R",i) : ""
            push!(robot_layers,
                layer(
                    x=[interpolate(vtxs[idx1][1],vtxs[idx2][1],t-(t1-1))],
                    y=[interpolate(vtxs[idx1][2],vtxs[idx2][2],t-(t1-1))],
                    label=[label],
                    Geom.label(position=:centered,hide_overlaps=false),
                    Geom.point,
                    size=[rsize],
                    Theme(
                        theme,
                        default_color=colors_vec[i],
                        )
                    )
            )
        end
    end
    search_pattern_layers = []
    if show_search_paths
        for (i,p) in enumerate(search_patterns)
            if length(p) > 0
                idx = max(1,min(search_idx,length(p)))
                t_search = p[idx][2] # time step
                df = DataFrame(x=map(s->vtxs[s[1]][1],p[1:idx]),y=map(s->vtxs[s[1]][2],p[1:idx]),t=map(s->s[2],p[1:idx]))
                push!(search_pattern_layers,
                    layer(x=[df.x[end]],y=[df.y[end]],Geom.point,size=[search_size],
                        Theme(default_color="Black",highlight_width=0pt))
                    )
                push!(search_pattern_layers,
                    layer(df,x="x",y="y",Geom.point,size=[search_size],
                        Theme(default_color="Gray",highlight_width=0pt))
                    )
            end
        end
    end
    goal_layers = []
    for (i,s) in enumerate(goals)
        push!(goal_layers,
            layer(x=[vtxs[s[1]][1]],y=[vtxs[s[1]][2]],Geom.point,size=[goal_size],
                Theme(default_color="red",highlight_width=0pt)
                # Theme(default_color=colors_vec[i])
            )
        )
    end
    object_layers = []
    for (i,(p,interval)) in enumerate(zip(object_paths,object_intervals))
        if interval[1] > t
            object_color = inactive_object_colors[i]
        elseif interval[2] < t
            object_color = completed_object_colors[i]
        else
            object_color = active_object_colors[i]
        end
        if length(p) > 0 && interval[2] > t
            if (interval[1] <= t) || show_inactive_objects
                interpolate(p[min(t1,length(p))],p[min(t1+1,length(p))],t1-(t+1))
                idx1 = p[max(1,min(t1,length(p)))]
                idx2 = p[max(1,min(t1+1,length(p)))]
                label = label_objects ? string("O",i) : ""
                push!(object_layers,
                    layer(
                        x=[interpolate(vtxs[idx1][1],vtxs[idx2][1],t-(t1-1))],
                        y=[interpolate(vtxs[idx1][2],vtxs[idx2][2],t-(t1-1))],
                        label=[label],
                        Geom.label(position=:centered,hide_overlaps=false),
                        Geom.point,
                        size=[osize],
                        Theme(
                            theme,
                            default_color=object_color,
                            )
                        )
                )
            end
        end
    end
    if length(object_vtxs) > 0
        push!(object_layers,layer( x=map(v->vtxs[v][1], object_vtxs), y=map(v->vtxs[v][2], object_vtxs),
            size=[3pt], Geom.point, Theme(default_color="Black")))
    end
    for (i,idx) in enumerate(robot_vtxs)
        push!(robot_layers,
            layer(
                x=[vtxs[idx][1]],
                y=[vtxs[idx][2]],
                Geom.point,size=[rsize],Theme(default_color=colors_vec[i]))
        )
    end

    Gadfly.plot(
        search_pattern_layers...,
        goal_layers...,
        object_layers...,
        robot_layers...,
        path_layers...,
        get_grid_world_layer(pickup_vtxs,pickup_color,cw),
        get_grid_world_layer(dropoff_vtxs,dropoff_color,cw),
        get_grid_world_layer(vtxs,floor_color,cw),
        Coord.cartesian(fixed=true,
            xmin=minimum(xpts) - cell_width/2,
            ymin=minimum(ypts) - cell_width/2,
            xmax=maximum(xpts) + cell_width/2,
            ymax=maximum(ypts) + cell_width/2),
        Guide.xticks(;ticks=nothing),
        Guide.yticks(;ticks=nothing),
        Guide.xlabel(nothing),
        Guide.ylabel(nothing),
        theme
    )
end
function visualize_env(env::GridFactoryEnvironment,t=0;kwargs...)
    visualize_env(get_vtxs(env),get_pickup_vtxs(env),get_dropoff_vtxs(env),t;kwargs...)
end


render_env(t) = visualize_env(factory_env,t;robot_vtxs=r0,object_vtxs=s0,paths=paths,search_patterns=[],goals=[])
render_paths(t,factory_env,robot_paths,object_paths=[];kwargs...) = visualize_env(factory_env,t;robot_paths=robot_paths,object_paths=object_paths,kwargs...)
render_both(t,paths1,paths2) = hstack(render_paths(t,paths1),render_paths(t,paths2))
render_search(t_start,factory_env;kwargs...) = visualize_env(factory_env,t_start;show_search_paths=true,kwargs...)



# Plotting results
function preprocess_results!(df)
    if nrow(df) > 0
        if :depth_bias in names(df)
            begin df[!,:depth_bias_string] = string.(df.depth_bias)
                df
            end
        end
        if :N in names(df)
            begin df[!,:N_string] = string.(df.N)
                df
            end
        end
        sort!(df, (:M,:N))
    end
    df
end
function preprocess_results!(df_dict::Dict)
    for (k,df) in df_dict
        preprocess_results!(df)
    end
    df_dict
end

function robots_vs_task_vs_time_box_plot(df;
        title="Solution time by Number of Robots (N) and Number of Tasks (M)",
        yticks=[-1,0,1,2],
        ymin=-1.5,
        ymax=2.3,
        y_bounds=[0.01,100.0],
        big_font=14pt,
        small_font=12pt,
        y=:time,
        x=:N_string,
        xgroup=:M,
        color=:N_string,
        ylabel="computation time (s)",
        scale_y = Scale.y_log10(minvalue=y_bounds[1],maxvalue=y_bounds[2])
    )
    latex_fonts = Theme(major_label_font="CMU Serif", major_label_font_size=big_font,
                    minor_label_font="CMU Serif", minor_label_font_size=small_font,
                    key_title_font="CMU Serif", key_title_font_size=small_font,
                    key_label_font="CMU Serif", key_label_font_size=small_font)

    plot(df, xgroup=xgroup, x=x, y=y, color=color,
        Geom.subplot_grid(
            Geom.boxplot(;suppress_outliers=false),
            Coord.cartesian(; ymin=ymin, ymax=ymax),
            Guide.yticks(;ticks=yticks),
            Guide.xticks(;label=false),
            ),
        Guide.title(title),
        Guide.colorkey(title="number of robots", labels=["10","20","30","40"], pos=[0.1w,-0.32h]),
        Scale.group_discrete(labels=M->string(M," Tasks"),levels=[10,20,30,40,60]),
        Guide.xlabel("number of tasks"),
        Guide.ylabel(ylabel),
        scale_y,
        latex_fonts
    )
end


function get_box_plot_group_plot(df;
        obj=:time,
        outer_key=:M,
        inner_key=:N,
        outer_range=10:10:60,
        inner_range=10:10:40,
        xmin=0,
        xmax=length(inner_range)+1,
        ymin=0.007,
        ymax=120,
        xtick=[10,20,30,40],
        xticklabels=[10,20,30,40],
        tickpos="left",
        ytick=[0.1,1,10,100],
        ylabel_shift="0pt",
        title="",
        title_shift=[3.3,2.55],
        inner_sym="n",
        outer_sym="m",
        xlabels=map(m->string(outer_sym," = ",m), outer_range),
        ylabels=map(n->string(inner_sym," = ",n), inner_range),
        ylabel="time (s)",
        draw_labels=true,
        ymode="log",
        width="3.25cm",
        height="6cm",
        legend_draw="black",
        legend_fill="white",
        legend_x_shift="2pt",
        mark="*",
    )
    @pgf gp = GroupPlot({group_style = {
                "group name"="myPlots",
                "group size"=string(length(outer_range)," by 1"),
                "xlabels at"="edge bottom",
                "xticklabels at"="edge bottom",
                "vertical sep"="0pt",
                "horizontal sep"="2pt"
            },
            boxplot,
            "boxplot/draw direction"="y",
            ymode=ymode,
            footnotesize,
            width=width,
            height=height,
            xmin=xmin,
            xmax=xmax,
            ymin=ymin,
            ymax=ymax,
            xtick=xtick,
            xticklabels=xtick,
            tickpos=tickpos,
            ytick=ytick,
            yticklabels=[],
            # "set layers"="standard",
            "ylabel shift"=ylabel_shift,
            "ytick align"="outside",
            "xtick align"="outside",
            # "legend entries"={map(j->@sprintf("\$%s\$",ylabels[j]),1:length(inner_range))...},
            # "legend cell align"="left",
            # "legend to name"="grouplegend",
            # "legend style"={
            #     draw=legend_draw,
            #     fill=legend_fill,
            #     xshift=legend_x_shift,
            #     "inner sep"="1pt",
            #     yshift="0pt",
            #     font="\\scriptsize"
            # },
            # "legend pos"="north west"
        }

        );

    @pgf for (i,m) in enumerate(outer_range)
        if i == 1 && draw_labels
            # push!(gp,
            #     """
            #     \\coordinate (leg) at (rel axis cs:0,1);
            #     """
            # )
            push!(gp,
                {xlabel=@sprintf("\$%s\$",xlabels[i]),
                ylabel=ylabel,
                yticklabels=ytick,
                "legend style"={
                    draw=legend_draw,
                    fill=legend_fill,
                    xshift=legend_x_shift,
                    yshift="0pt"},
                "legend pos"="north west"
                },
                map(j->LegendEntry({},@sprintf("\$%s\$",ylabels[j]),false),1:length(inner_range))...,
                """
                \\addlegendimage{no markers,blue}
                \\addlegendimage{no markers,red}
                \\addlegendimage{no markers,brown}
                \\addlegendimage{no markers,black}
                """,
                map(n->PGFPlotsX.PlotInc({boxplot,mark=mark},Table(
                            {"y index"=0},
                            [:data=>df[(df[:,outer_key] .== m) .& (df[:,inner_key] .== n),obj]])),inner_range)...)
        else
            push!(gp, {
                    xlabel=@sprintf("\$%s\$",xlabels[i]),
                    ymajorticks="false",
                    yminorticks="false"
                },
                map(n->PGFPlotsX.PlotInc({boxplot,mark=mark},Table(
                            {"y index"=0},
                            [:data=>df[(df[:,outer_key] .== m) .& (df[:,inner_key] .== n),obj]])),inner_range)...)
        end
    end;
    gp
end
function get_titled_group_box_plot(df;
        title="",
        title_shift=[3.3,2.55],
        scale=0.7,
        kwargs...)
    gp = get_box_plot_group_plot(df;kwargs...)
    if title != ""
        tikzpic = @pgf TikzPicture({scale=scale},
            """
            \\centering
            \\pgfplotsset{set layers=standard,cell picture=true}
            """,
            gp,
            @sprintf("""
            \\node (title) at (\$(myPlots c1r1.center)!0.5!(myPlots c2r1.center)+(%2.2fcm,%2.2fcm)\$) {\\textbf{%s}};
            """,title_shift...,title),
            # """
            # \\node[anchor= north west] (leg) at (myPlots c1r1.north west){\\pgfplotslegendfromname{grouplegend}};
            # """
        )
    else
        tikzpic = @pgf TikzPicture({scale=scale},
            """
            \\centering
            \\pgfplotsset{set layers=standard,cell picture=true}
            """,
            gp,
            # """
            # \\node[anchor= north west] (leg) at (myPlots c1r1.north west){\\pgfplotslegendfromname{grouplegend}};
            # """
        )
    end
    return tikzpic
end

# print_tex(gp)

function preprocess_collab_results!(df_dict)
    num_tasks = [12,18,24]
    num_robots = [24]
    depth_biases=[0.1]
    task_size_distributions = [1,2,3,4]
    num_trials=16
    task_ratios = Int[]
    for (M,N,task_sizes,trial) in Base.Iterators.product(
            num_tasks,num_robots,task_size_distributions,1:num_trials
            )
        push!(task_ratios, task_sizes)
    end

    for (k,df) in df_dict
        if nrow(df) > 0
            begin df[!,:depth_bias_string] = string.(df.depth_bias)
                df
            end
            begin df[!,:N_string] = string.(df.N)
                df
            end
            begin df[!,:M_string] = string.(df.M)
                df
            end
            begin df[!,:TaskRatio] = task_ratios[df.problem_id]
                df
            end
        end
    end
    df_dict
end
function plot_collab_runtimes(df;
        title="Solution time by Number of Tasks (M) and ratio of tasks (task ratio )",
        yticks=[-1,0,1,2],
        ymin=-1.5,
        ymax=2.3,
        y_bounds=[0.01,100.0],
        big_font=14pt,
        small_font=12pt,
        xgroup=:TaskRatio,
        x=:M_string,
        y=:time,
        suppress_outliers=true,
        scale_y = Scale.y_log10(minvalue=y_bounds[1],maxvalue=y_bounds[2])
    )
    latex_fonts = Theme(major_label_font="CMU Serif", major_label_font_size=big_font,
                    minor_label_font="CMU Serif", minor_label_font_size=small_font,
                    key_title_font="CMU Serif", key_title_font_size=small_font,
                    key_label_font="CMU Serif", key_label_font_size=small_font)

    plot(df, xgroup=xgroup, x=x, y=y, color=x,
        Geom.subplot_grid(
            Geom.boxplot(;suppress_outliers=suppress_outliers),
            Coord.cartesian(; ymin=ymin, ymax=ymax),
            Guide.yticks(;ticks=yticks),
            Guide.xticks(;label=false),
            ),
        Guide.title(title),
        Guide.colorkey(title="num tasks", labels=["12","18","24"], pos=[0.1w,-0.32h]),
        # Scale.group_discrete(labels=M->string(M," Tasks"),levels=[1,2,3,4]),
        Guide.xlabel("task ratio"),
        Guide.ylabel("computation time (s)"),
        scale_y,
        latex_fonts
    )
end
function plot_collab_counts(df;
        title="# failures vs. # Tasks (M) x Task Ratio",
        yticks=[0,4,8,12,16],
        ymin=0,
        ymax=16,
        y_bounds=[0.01,100.0],
        big_font=14pt,
        small_font=12pt,
        task_ratios = [
            ( 1=>1.0, 2=>0.0, 4=>0.0 ),
            ( 1=>1.0, 2=>1.0, 4=>0.0 ),
            ( 1=>1.0, 2=>1.0, 4=>1.0 ),
            ( 1=>0.0, 2=>1.0, 4=>1.0 ),
        ],
        key=df.optimal
    )


    opt_df = DataFrame( M = Int[], task_ratio = Int[], num_no = Int[], num_yes = Int[] )

    for (i,ratio) in enumerate(task_ratios)
        for m in Set(collect(df.M))
        push!(
            opt_df,
            Dict(
                :M =>m,
                :task_ratio => i,
                :num_no => nrow(df[(df.M .== m) .* (df.TaskRatio .== i) .* (key .== false),:]),
                :num_yes => nrow(df[(df.M .== m) .* (df.TaskRatio .== i) .* (key .== true),:]),
            )
        )
        end
    end
    begin opt_df[!,:M_string] = string.(opt_df.M)
        opt_df
    end

    latex_fonts = Theme(major_label_font="CMU Serif", major_label_font_size=big_font,
                    minor_label_font="CMU Serif", minor_label_font_size=small_font,
                    key_title_font="CMU Serif", key_title_font_size=small_font,
                    key_label_font="CMU Serif", key_label_font_size=small_font)

    plot(opt_df, xgroup=:task_ratio, x=:M_string, y=:num_no, color=:M_string,
        Geom.subplot_grid(
            Geom.bar,
            Coord.cartesian(; ymin=ymin, ymax=ymax),
            Guide.yticks(;ticks=yticks),
            Guide.xticks(;label=false),
            ),
        Guide.title(title),
        Guide.colorkey(title="num tasks", labels=["12","18","24"], pos=[0.1w,-0.32h]),
        Scale.group_discrete(labels=M->string(M," Tasks"),levels=[1,2,3,4]),
        Guide.xlabel("task ratio"),
        Guide.ylabel("computation time (s)"),
        latex_fonts
    )
end

function plot_histories_pgf(df_list::Vector,ax=PGFPlots.Axis();
        y_key=:makespans,
        x_key=:arrival_times,
        m_key=:M,
        m_vals=sort(intersect([unique(df[m_key]) for df in df_list]...)),
        n_key=:arrival_interval,
        n_vals=sort(intersect([unique(df[n_key]) for df in df_list]...)),
        include_keys=[],
        exclude_keys=[],
        opt_key=:backup_flags,
        # ymin=minimum(df[y_key]),
        # ymax=maximum(df[y_key]),
        xlabel=string("\$",string(m_key)," = ",m_vals...,"\$"),
        ylabel=string("\$",string(n_key)," = ",n_vals...,"\$"),
        # ylabel=string("M = ",m_vals...),
        colors=["red","blue","black","brown"],
        ytick_show=false,
        ymode="log",
        lines=true,
    )

    ax.ymode    =   ymode
    ax.xlabel   =   xlabel
    ax.ylabel   =   ytick_show ? ylabel : ""

    for (i,m_val) in enumerate(m_vals)
        for (j,n_val) in enumerate(n_vals)
            for (df,color) in zip(df_list,colors)
                idxs = .&(
                    (df[!,m_key] .== m_val),
                    (df[!,n_key] .== n_val),
                    )
                if !isempty(include_keys)
                    idxs = .&(idxs, [(df[!,k] .== v) for (k,v) in include_keys]...)
                end
                if !isempty(exclude_keys)
                    idxs = .&(idxs, [(df[!,k] .!= v) for (k,v) in exclude_keys]...)
                end
                df_cut = df[idxs,:]
                style=string(color,",solid, mark options={solid,fill=",color,"}")
                for k in 1:nrow(df_cut)
                    y_arr = df_cut[y_key][k]
                    x_arr = x_key == :none ? collect(1:length(y_arr)) : df_cut[x_key][k]
                    # x_arr = df_cut[x_key][k]
                    opt_vals = df_cut[opt_key][k] # tracks whether the fall back was employed

                    # idx = findfirst(y_arr .< 0)
                    # idx = idx == nothing ? length(y_arr) : idx-1
                    idx = minimum([length(x_arr),length(y_arr),length(opt_vals)])
                    # x_arr = x_arr[1:idx]
                    # y_arr = y_arr[1:idx]
                    # @show x_arr, y_arr, opt_vals
                    # Overall trend
                    if lines
                        push!(ax,
                            Plots.Linear(x_arr[1:idx], y_arr[1:idx], style=style, mark="none")
                        )
                    end
                    # Timeouts
                    if 0 < idx < length(y_arr)
                        # End points
                        push!(ax,
                            Plots.Linear([x_arr[idx]], [y_arr[idx]], mark="o", style=style, onlyMarks=true, )
                        )
                        idx -= 1 # decrement so that the final point is not covered twice
                    end

                    tmidxs=findall(.~opt_vals[1:idx])
                    if length(tmidxs) > 0
                        push!(ax,
                            Plots.Linear(x_arr[tmidxs], y_arr[tmidxs], style=style, onlyMarks=true, mark="x")
                        )
                    end
                end
            end
        end
    end
    return ax
end

function group_history_plot(df_list::Vector;
    m_key=:M,
    m_vals=sort(intersect([unique(df[m_key]) for df in df_list]...)),
    n_key=:arrival_interval,
    n_vals=sort(intersect([unique(df[n_key]) for df in df_list]...)),
    kwargs...
    )
    g = PGFPlots.GroupPlot(length(m_vals),length(n_vals))
    for (i,m_val) in enumerate(m_vals)
        ytick_show = (i == 1)
        for (j,n_val) in enumerate(n_vals)
            plt = plot_histories_pgf(df_list;
                m_key=m_key,
                m_vals=[m_val,],
                n_key=n_key,
                n_vals=[n_val,],
                ytick_show = ytick_show,
                kwargs...)
            push!(g,plt)
        end
    end
    return g
end

# For rendering the factory floor as a custom ImageAppearance in Webots
function caution_tape(d,n;yellow_color=RGB(0.8,0.7,0.0),black_color=RGB(0.0,0.0,0.0))
    compose(context(),map(i->(context(),
                (context(),polygon([
                            (0,(i-1)/n),
                            (d,(2*i-1)/(2*n)),
                            (d,i/n),
                            (0,(2*i-1)/(2*n))]),fill(yellow_color)),
            ), 1:n)...,
            (context(),polygon([(0,0),(0,1),(d,1),(d,0)]),fill(black_color))
    )
end
function taped_square(d=0.1;n=10,yellow_color=RGB(0.8,0.7,0.0),black_color=RGB(0.0,0.0,0.0))
    compose(
        context(),
        map(i->(context(rotation=Rotation(i*π/2,0.5,0.5)),
                caution_tape(d,n;yellow_color=yellow_color,black_color=black_color)), 0:3)...
    )
end
function render_factory_floor(env::GridFactoryEnvironment;
        d=0.1,n=10,yellow_color=RGB(0.8,0.7,0.0),black_color=RGB(0.0,0.0,0.0),
        floor_color=RGB(0.6,0.6,0.6)
    )
    x_dim = get_x_dim(env)
    y_dim = get_y_dim(env)
    compose(
        context(units=UnitBox()),
        map(vtx->(context((vtx[1]-1)/x_dim,(vtx[2]-1)/y_dim,1/x_dim,1/y_dim),
                taped_square(d;n=n,yellow_color=yellow_color,black_color=black_color)), get_pickup_vtxs(env))...,
        map(vtx->(context((vtx[1]-1)/x_dim,(vtx[2]-1)/y_dim,1/x_dim,1/y_dim),
                taped_square(d;n=n,yellow_color=yellow_color,black_color=black_color)), get_dropoff_vtxs(env))...,
        (context(), rectangle(),fill(floor_color))
    )
end

function get_node_shape(search_env::SearchEnv,graph,v,x,y,r)
    project_schedule = get_schedule(search_env)
    cache = get_cache(search_env)
    node_id = get_vtx_id(project_schedule,get_prop(graph,v,:vtx_id))
    if typeof(node_id) <: ActionID
        return Compose.circle(x,y,r)
    elseif typeof(node_id) <: RobotID
        return Compose.ngon(x,y,r,4)
    elseif typeof(node_id) <: ObjectID
        return Compose.ngon(x,y,r,3)
    elseif typeof(node_id) <: OperationID
        return Compose.circle(x,y,r)
    end
end
function get_node_color(search_env::SearchEnv,graph,v,x,y,r)
    project_schedule = get_schedule(search_env)
    cache = get_cache(search_env)
    node_id = get_vtx_id(project_schedule,get_prop(graph,v,:vtx_id))
    if typeof(node_id) <: ActionID
        return "cyan"
    elseif typeof(node_id) <: CleanUpBotID
        return "purple"
    elseif typeof(node_id) <: BotID
        return "lime"
    elseif typeof(node_id) <: ObjectID
        return "orange"
    elseif typeof(node_id) <: OperationID
        return "red"
    end
end
function get_node_text(search_env::SearchEnv,graph,v,x,y,r)
    project_schedule = get_schedule(search_env)
    cache = get_cache(search_env)
    v_ = get_prop(graph,v,:vtx_id)
    string(cache.t0[v_]," - ",cache.tF[v_],"\n",cache.local_slack[v_]," - ",cache.slack[v_])
end

function show_times(sched::OperatingSchedule,v)
    arr = process_schedule(sched)
    return string(map(a->string(a[v],","), arr[1:2])...)
end
function show_times(cache::PlanningCache,v)
    slack = minimum(cache.slack[v])
    slack_string = slack == Inf ? string(slack) : string(Int(slack))
    return string(cache.t0[v],",",cache.tF[v],",",slack_string)
end

function plot_project_schedule(
        project_schedule::OperatingSchedule,
        cache=initialize_planning_cache(project_schedule),
        ;
        mode=:root_aligned,
        verbose=true,
        shape_function = (G,v,x,y,r)->Compose.circle(x,y,r),
        color_function = (G,v,x,y,r)->get_prop(G,v,:color),
        text_function = (G,v,x,y,r)->string(
            title_string(
                get_node_from_id(project_schedule,
                    get_vtx_id(project_schedule, v)),
                verbose),
            "\n",show_times(cache,v)
            )
        )
    rg = get_display_metagraph(project_schedule;
        f=(v,p)->string(v,",",get_id(get_path_spec(project_schedule,v).agent_id)))
    plot_graph_bfs(rg;
        mode=mode,
        shape_function=shape_function,
        color_function=color_function,
        text_function=text_function
        # shape_function = (G,v,x,y,r)->Compose.circle(x,y,r),
        # color_function = (G,v,x,y,r)->get_prop(G,v,:color),
        # text_function = (G,v,x,y,r)->string(
        #     title_string(get_node_from_id(project_schedule, get_vtx_id(project_schedule, v)),verbose),
        #     "\n",show_times(cache,v)
        #     )
    )
    # `inkscape -z project_schedule1.svg -e project_schedule1.png`
    # OR: `for f in *.svg; do inkscape -z $f -e $f.png; done`
end
plot_project_schedule(search_env::SearchEnv;kwargs...) = plot_project_schedule(get_schedule(search_env),get_cache(search_env);kwargs...)
function print_project_schedule(filename::String,args...;kwargs...)
    plot_project_schedule(args...;kwargs...) |> Compose.SVG(string(filename,".svg"))
end
