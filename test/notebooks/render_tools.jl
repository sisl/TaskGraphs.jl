import Cairo #, Fontconfig
using GraphPlottingBFS
using Compose
using Colors
using Gadfly
using DataFrames

using PGFPlotsX
latexengine!(PGFPlotsX.PDFLATEX)
using Printf

interpolate(a,b,t) = (1 - t)*a + t*b

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
        line_width=2pt) where {S<:SearchEnv}

    cache = search_env.cache
    project_schedule = search_env.schedule

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
            # if typeof(node) <: Union{GO,CARRY}
            if (cache.t0[v] <= t) && (cache.tF[v] >= t)
                spec = get_path_spec(project_schedule, v)
                agent_id = spec.agent_id
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
                    Geom.point,size=[osize],Theme(default_color="black"))
            )
        end
    end
    # object_layer = layer( x=map(v->vtxs[v][1], object_vtxs), y=map(v->vtxs[v][2], object_vtxs),
    # size=[3pt], Geom.point, Theme(default_color="black") )

    plot(
        object_layers...,
        # object_layer,
        robot_layers...,
        path_layers...,
        # search_pattern_layers...,
        layer(
            xmin=map(vec->vec[1], pickup_vtxs) .- cw,
            ymin=map(vec->vec[2], pickup_vtxs) .- cw,
            xmax=map(vec->vec[1], pickup_vtxs) .+ cw,
            ymax=map(vec->vec[2], pickup_vtxs) .+ cw,
            Geom.rect, Theme(default_color="light blue") ),
        layer(
            xmin=map(vec->vec[1], dropoff_vtxs) .- cw,
            ymin=map(vec->vec[2], dropoff_vtxs) .- cw,
            xmax=map(vec->vec[1], dropoff_vtxs) .+ cw,
            ymax=map(vec->vec[2], dropoff_vtxs) .+ cw,
            Geom.rect, Theme(default_color="pink") ),
        layer(
            xmin=collect(xpts .- cw),
            xmax=collect(xpts .+ cw),
            ymin=collect(ypts .- cw),
            ymax=collect(ypts .+ cw),
            Geom.rect, Theme(default_color="light gray") ),
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
            # default_color="light gray",
            background_color="gray"
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
        search_patterns=[],
        goals=[],
        vpad = 0.05,
        n=80,
        rsize=5pt,
        osize=3pt,
        cell_width=1.0,
        line_width=2pt,
        color_scale = Scale.color_discrete_hue(),
        colors_vec = color_scale.f(n),
        floor_color = "light gray",
        pickup_color= "light blue",
        dropoff_color="pink"
    )


    cw = cell_width/2 - vpad
    xpts = map(vtx->cell_width*vtx[1], vtxs)
    ypts = map(vtx->cell_width*vtx[2], vtxs)

    t1 = Int(floor(t))+1
    path_layers = []
    for (i,p) in enumerate(robot_paths)
        if length(p) > 0
            push!(path_layers,
                layer(x=map(v->vtxs[v][1],p),y=map(v->vtxs[v][2],p),Geom.path,
                    Theme(line_width=line_width,default_color=colors_vec[i]))
            )
        end
    end
    robot_layers = []
    for (i,p) in enumerate(robot_paths)
        if length(p) > 0
            interpolate(p[min(t1,length(p))],p[min(t1+1,length(p))],t1-(t+1))
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
                    Geom.point,size=[osize],Theme(default_color="black"))
            )
        end
    end
    if length(object_vtxs) > 0
        push!(object_layers,layer( x=map(v->vtxs[v][1], object_vtxs), y=map(v->vtxs[v][2], object_vtxs),
            size=[3pt], Geom.point, Theme(default_color="black")))
    end
    for (i,idx) in enumerate(robot_vtxs)
        push!(robot_layers,
            layer(
                x=[vtxs[idx][1]],
                y=[vtxs[idx][2]],
                Geom.point,size=[rsize],Theme(default_color=colors_vec[i]))
        )
    end

    plot(
        object_layers...,
        robot_layers...,
        path_layers...,
        search_pattern_layers...,
        layer(
            xmin=map(vec->vec[1], pickup_vtxs) .- cw,
            ymin=map(vec->vec[2], pickup_vtxs) .- cw,
            xmax=map(vec->vec[1], pickup_vtxs) .+ cw,
            ymax=map(vec->vec[2], pickup_vtxs) .+ cw,
            Geom.rect, Theme(default_color=pickup_color) ),
        layer(
            xmin=map(vec->vec[1], dropoff_vtxs) .- cw,
            ymin=map(vec->vec[2], dropoff_vtxs) .- cw,
            xmax=map(vec->vec[1], dropoff_vtxs) .+ cw,
            ymax=map(vec->vec[2], dropoff_vtxs) .+ cw,
            Geom.rect, Theme(default_color=dropoff_color) ),
        layer(
            xmin=collect(xpts .- cw),
            xmax=collect(xpts .+ cw),
            ymin=collect(ypts .- cw),
            ymax=collect(ypts .+ cw),
            Geom.rect, Theme(default_color=floor_color) ),
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
            # default_color="light gray",
            background_color="gray"
        )
    )
end
function visualize_env(env::GridFactoryEnvironment,t=0;kwargs...)
    visualize_env(get_vtxs(env),get_pickup_vtxs(env),get_dropoff_vtxs(env),t;kwargs...)
end
function record_video(outfile_name,render_function;
        dt = 0.25,
        tf = 10,
        fps = 10,
        ext = "png",
        s = (4inch, 4inch),
        res = "1080x1080"
    )
    tmpdir = mktempdir()
    for (i,t) in enumerate(0.0:dt:tf)
        filename = joinpath(tmpdir,string(i,".",ext))
        render_function(t) |> PNG(filename, s[1], s[2])
    end
    outfile = outfile_name
    run(pipeline(`ffmpeg -y -r $fps -f image2 -i $tmpdir/%d.$ext -crf 20 $outfile`, stdout=devnull, stderr=devnull))
    run(pipeline(`rm -rf $tmpdir`, stdout=devnull, stderr=devnull))
end

render_env(t) = visualize_env(factory_env,t;robot_vtxs=r0,object_vtxs=s0,paths=paths,search_patterns=[],goals=[])
render_paths(t,robot_paths,object_paths=[]) = visualize_env(factory_env,t;robot_paths=robot_paths,object_paths=object_paths)
render_both(t,paths1,paths2) = hstack(render_paths(t,paths1),render_paths(t,paths2))


function get_runtime_box_plot(df;obj=:time,m_range=10:10:60,n_range=10:10:40,ymin=0.007,title="",nsym="n",msym="m",)
    @pgf gp = GroupPlot({group_style = {
                "group name"="myPlots",
                "group size"="6 by 1",
                "xlabels at"="edge bottom",
                "xticklabels at"="edge bottom",
                "vertical sep"="0pt",
                "horizontal sep"="2pt"
            },
            boxplot,
            "boxplot/draw direction"="y",
            # axis lines=left,
            # hide axis,
            ymode="log",
            footnotesize,
            width="3.25cm",
            height="6cm",
            xmin=0,
            xmax=5,
            ymin=ymin,
            ymax=120,
            xtick=[10,20,30,40],
            xticklabels=[10,20,30,40],
            tickpos="left",
            ytick=[0.1,1,10,100],
            yticklabels=[],
            "ylabel shift"="0pt",
            "ytick align"="outside",
            "xtick align"="outside"});

    @pgf for (i,m) in enumerate(m_range)
        if i == 1
            push!(gp,
                {xlabel=@sprintf("\$%s=%i\$",msym,m),
                ylabel="time (s)",
                yticklabels=[0.1,1,10,100],
                "legend style"="{draw=none,xshift=2pt}",
                "legend pos"="north west"},
                map(n->LegendEntry({},@sprintf("\$%s=%i\$",nsym,n),false),n_range)...,
                """
                \\addlegendimage{no markers,blue}
                \\addlegendimage{no markers,red}
                \\addlegendimage{no markers,brown}
                \\addlegendimage{no markers,black}
                """,
                map(n->PGFPlotsX.PlotInc({boxplot},Table(
                            {"y index"=0},
                            [:data=>df[(df.M .== m) .& (df.N .== n),obj]])),n_range)...)
        else
            push!(gp, {xlabel=@sprintf("\$%s=%i\$",msym,m)},
                map(n->PGFPlotsX.PlotInc({boxplot},Table(
                            {"y index"=0},
                            [:data=>df[(df.M .== m) .& (df.N .== n),obj]])),n_range)...)
        end
    end;
    if title != ""
        tikzpic = @pgf TikzPicture({scale=0.7},
            """
            \\centering
            """,
            gp,
            @sprintf("""
            \\node (title) at (\$(myPlots c1r1.center)!0.5!(myPlots c2r1.center)+(3.3cm,2.55cm)\$) {\\textbf{%s}};
            """,title)
        )
    else
        tikzpic = @pgf TikzPicture({scale=0.7},
            """
            \\centering
            """,
            gp
        )
    end
    return tikzpic
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
    project_schedule = search_env.schedule
    cache = search_env.cache
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
    project_schedule = search_env.schedule
    cache = search_env.cache
    node_id = get_vtx_id(project_schedule,get_prop(graph,v,:vtx_id))
    if typeof(node_id) <: ActionID
        return "cyan"
    elseif typeof(node_id) <: RobotID
        return "lime"
    elseif typeof(node_id) <: ObjectID
        return "orange"
    elseif typeof(node_id) <: OperationID
        return "red"
    end
end
function get_node_text(search_env::SearchEnv,graph,v,x,y,r)
    project_schedule = search_env.schedule
    cache = search_env.cache
    v_ = get_prop(graph,v,:vtx_id)
    string(cache.t0[v_]," - ",cache.tF[v_],"\n",cache.local_slack[v_]," - ",cache.slack[v_])
end
