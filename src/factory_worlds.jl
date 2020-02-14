module FactoryWorlds

using Parameters
using LightGraphs, MetaGraphs, GraphUtils
using SparseArrays
using ImageFiltering
using Printf
using TOML

export
    GridFactoryEnvironment,
    get_x_dim,
    get_y_dim,
    get_cell_width,
    get_transition_time,
    get_vtxs,
    get_pickup_zones,
    get_dropoff_zones,
    get_obstacles,
    get_pickup_vtxs,
    get_dropoff_vtxs,
    get_obstacle_vtxs,
    get_num_free_vtxs,
    get_free_zones


function construct_vtx_map(vtxs,dims)
    vtx_map = zeros(Int,dims)
    for (i,vtx) in enumerate(vtxs)
        vtx_map[vtx[1],vtx[2]] = i
    end
    vtx_map
end

function construct_edge_cache(vtxs,vtx_map)
    edge_cache = Vector{Set{Tuple{Int,Int}}}()
    for (v,pt) in enumerate(vtxs)
        edge_list = Set{Tuple{Int,Int}}()
        for d in [(0,0),(-1,0),(0,1),(1,0),(0,-1)]
            if get(vtx_map, (pt[1]+d[1], pt[2]+d[2]), 0) >= 1
                push!(edge_list, d)
            end
        end
        push!(edge_cache, edge_list)
    end
    edge_cache
end

export
    construct_expanded_zones

"""
    returns a dictionary mapping vertices to a dict of shape=>vtxs for expanded
    ssize delivery zones.

    WARNING - Can lead to failure when constructing expanded dropoff zones.
"""
function construct_expanded_zones(vtxs,vtx_map,pickup_zones,dropoff_zones;shapes=[(1,1),(1,2),(2,1),(2,2)])
    expanded_zones = Dict{Int,Dict{Tuple{Int,Int},Vector{Int}}}()
    R = [ 0 -1 ; 1 0 ]
    for v in vcat(pickup_zones, dropoff_zones)
        expanded_zones[v] = Dict{Tuple{Int,Int},Vector{Int}}()
        vtx = [vtxs[v]...]
        d = [0,1]
        k = 0
        while get(vtx_map,tuple((vtx .- d)...),0) > 0 && k < 4
            d = R*d # rotate d until it points at obstacle
            k += 1
        end
        anchor = (vtx .- d)
        d2 = R*d
        if get(vtx_map,tuple((anchor .- d2)...),0) > get(vtx_map,tuple((anchor .+ d2)...),0)
            d2 = -d2
        end
        d = d + d2
        for shape in shapes
            vtx_list = Int[]
            for dx in sort([0,d[1]][1:shape[1]])
                for dy in sort([0,d[2]][1:shape[2]])
                    push!(vtx_list, vtx_map[vtx[1]+dx, vtx[2]+dy])
                end
            end
            expanded_zones[v][shape] = vtx_list
        end
    end
    expanded_zones
end



export
    DistMatrixMap,
    get_distance

"""
    maps team size to the effective distance (computed by Djikstra) between
    leader (top left) vtxs.
    A DistMatrixMap is constructed by starting with a base environment grid
    graph, which is represented as a binary occupancy grid. The occupancy grid
    is then convolved with kernels of various sizes (which represent
    configurations of robots moving as a team). The output of each convolution
    represents a new occupancy grid corresponding to the workspace of the robot
    team.
    It is assumed that the ``lead'' robot is always in the top left of the
    configuration. If a team of robots wishes to query the distance to a
    particular target configuration, they pass the leader's current vtx,
    the leader's target vtx, and the team configuration (shape) to the
    DistMatrixMap, which returns the correct distance.
"""
struct DistMatrixMap
    dist_mtxs::Dict{Tuple{Int,Int},Dict{Int,Function}}
end
function get_distance(mtx_map::DistMatrixMap,v1::Int,v2::Int,shape::Tuple{Int,Int}=(1,1),config_idx=1)
    D = mtx_map.dist_mtxs[shape][config_idx](v1,v2)
end
function get_distance(mtx::Matrix,v1::Int,v2::Int,args...)
    return get(mtx,(v1,v2),0)
end
function DistMatrixMap(base_vtx_map::Matrix{Int},base_vtxs::Vector{Tuple{Int,Int}};shapes=[(1,1),(1,2),(2,1),(2,2)])
    G_ = initialize_grid_graph_from_vtx_grid(base_vtx_map)
    D_ = get_dist_matrix(G_)
    # shape_vtx_maps = Dict{Tuple{Int,Int},Vector{Int}}()
    dist_mtxs = Dict{Tuple{Int,Int},Dict{Int,Function}}(s=>Dict{Int,Function}() for s in shapes)
    grid_map = Int.(base_vtx_map .== 0)
    for s in shapes
        # s = (2,2)
        filtered_grid = imfilter(grid_map,centered(ones(s)))
        filtered_grid = Int.(filtered_grid .> 0)
        vtx_map = initialize_vtx_grid_from_indicator_grid(filtered_grid)
        graph = initialize_grid_graph_from_vtx_grid(vtx_map)
        D = get_dist_matrix(graph)
        dist_mtx = zeros(Int,nv(G_),nv(G_))
        for (v1,v1_) in zip(base_vtx_map,vtx_map)
            if !(v1 > 0 && v1_ > 0)
                continue
            end
            for (v2,v2_) in zip(base_vtx_map,vtx_map)
                if !(v2 > 0 && v2_ > 0)
                    continue
                end
                dist_mtx[v1,v2] = D[v1_,v2_]
            end
        end
        config = 0
        for i in 1:s[1]
            for j in 1:s[2]
                config += 1
                # dist_mtxs[s][config] = dist_mtx
                dist_mtxs[s][config] = (v1,v2) -> get(
                    dist_mtx,(
                    get(base_vtx_map, tuple(get(base_vtxs, v1, (-s[1],-s[2])) .+ [1-i, 1-j]...), -1),
                    get(base_vtx_map, tuple(get(base_vtxs, v2, (-s[1],-s[2])) .+ [1-i, 1-j]...), -1) ),
                    0)
            end
        end
    end
    DistMatrixMap(
        # shape_vtx_maps,
        dist_mtxs
    )
end
Base.getindex(d::DistMatrixMap,v1::Int,v2::Int) = get_distance(d,v1,v2,(1,1),1)



"""
    `GridFactoryEnvironment`
"""
@with_kw struct GridFactoryEnvironment{G} <: AbstractGraph{Int}
    graph               ::G                         = MetaDiGraph()
    x_dim               ::Int                       = 20
    y_dim               ::Int                       = 20
    cell_width          ::Float64                   = 0.5
    transition_time     ::Float64                   = 2.0
    vtxs                ::Vector{Tuple{Int,Int}}    = Vector{Tuple{Int,Int}}()
    pickup_zones        ::Vector{Int}               = Vector{Int}()
    dropoff_zones       ::Vector{Int}               = Vector{Int}()
    obstacles           ::Vector{Tuple{Int,Int}}    = Vector{Tuple{Int,Int}}()
    vtx_map             ::Matrix{Int}               = construct_vtx_map(vtxs,(x_dim,y_dim))
    edge_cache          ::Vector{Set{Tuple{Int,Int}}} = construct_edge_cache(vtxs,vtx_map)
    expanded_zones      ::Dict{Int,Dict{Tuple{Int,Int},Vector{Int}}}     = construct_expanded_zones(vtxs,vtx_map,pickup_zones,dropoff_zones)
    dist_function       ::DistMatrixMap             = DistMatrixMap(vtx_map,vtxs)
end
# get_graph(env::E) where {E<:GridFactoryEnvironment} = env.graph
get_x_dim(env::E) where {E<:GridFactoryEnvironment} = env.x_dim
get_y_dim(env::E) where {E<:GridFactoryEnvironment} = env.y_dim
get_cell_width(env::E) where {E<:GridFactoryEnvironment} = env.cell_width
get_transition_time(env::E) where {E<:GridFactoryEnvironment} = env.transition_time
get_vtxs(env::E) where {E<:GridFactoryEnvironment} = env.vtxs
get_pickup_zones(env::E) where {E<:GridFactoryEnvironment} = env.pickup_zones
get_dropoff_zones(env::E) where {E<:GridFactoryEnvironment} = env.dropoff_zones
get_obstacles(env::E) where {E<:GridFactoryEnvironment} = env.obstacles
get_pickup_vtxs(env::E) where {E<:GridFactoryEnvironment} = get_vtxs(env)[get_pickup_zones(env)]
get_dropoff_vtxs(env::E) where {E<:GridFactoryEnvironment} = get_vtxs(env)[get_dropoff_zones(env)]
get_obstacle_vtxs(env::E) where {E<:GridFactoryEnvironment} = get_obstacles(env)
function GridFactoryEnvironment(env::E,graph::G) where {E<:GridFactoryEnvironment,G<:AbstractGraph}
    GridFactoryEnvironment(
        graph           = graph,
        x_dim           = env.x_dim,
        y_dim           = env.y_dim,
        cell_width      = env.cell_width,
        transition_time = env.transition_time,
        vtxs            = env.vtxs,
        pickup_zones    = env.pickup_zones,
        dropoff_zones   = env.dropoff_zones,
        obstacles       = env.obstacles,
        vtx_map         = env.vtx_map
    )
end

Base.zero(env::GridFactoryEnvironment{G}) where {G} = GridFactoryEnvironment(env,graph=G())
LightGraphs.edges(env::GridFactoryEnvironment) = edges(env.graph)
LightGraphs.edgetype(env::GridFactoryEnvironment{G}, args...) where {G} = edgetype(env.graph, args...)
LightGraphs.has_edge(env::GridFactoryEnvironment{G}, args...) where {G} = has_edge(env.graph, args...)
LightGraphs.has_vertex(env::GridFactoryEnvironment{G}, args...) where {G} = has_vertex(env.graph, args...)
LightGraphs.inneighbors(env::GridFactoryEnvironment{G}, args...) where {G} = inneighbors(env.graph, args...)
LightGraphs.is_directed(env::GridFactoryEnvironment{G}, args...) where {G} = is_directed(env.graph, args...)
LightGraphs.ne(env::GridFactoryEnvironment{G}, args...) where {G} = ne(env.graph, args...)
LightGraphs.nv(env::GridFactoryEnvironment{G}, args...) where {G} = nv(env.graph, args...)
LightGraphs.outneighbors(env::GridFactoryEnvironment{G}, args...) where {G} = outneighbors(env.graph, args...)
LightGraphs.vertices(env::GridFactoryEnvironment{G}, args...) where {G} = vertices(env.graph, args...)

get_x(env::E,v::Int) where {E<:GridFactoryEnvironment} = get_vtxs(env)[v][1]
get_y(env::E,v::Int) where {E<:GridFactoryEnvironment} = get_vtxs(env)[v][2]
get_θ(env::E,v::Int) where {E<:GridFactoryEnvironment} = 0.0
get_num_free_vtxs(env::E) where {E<:GridFactoryEnvironment} = length(get_vtxs(env)) - length(get_pickup_zones(env)) - length(get_dropoff_zones(env))
function get_free_zones(env::E) where {E<:GridFactoryEnvironment}
    idxs = collect(1:length(get_vtxs(env)))
    setdiff!(idxs, get_pickup_zones(env))
    setdiff!(idxs, get_dropoff_zones(env))
    idxs
end

################################################################################
################################ READ AND WRITE ################################
################################################################################
export
    read_env

function TOML.parse(env::E) where {E<:GridFactoryEnvironment}
    toml_dict = Dict()
    toml_dict["title"]              = "GridFactoryEnvironment"
    toml_dict["x_dim"]              = get_x_dim(env)
    toml_dict["y_dim"]              = get_y_dim(env)
    toml_dict["cell_width"]         = get_cell_width(env)
    toml_dict["transition_time"]    = get_transition_time(env)
    toml_dict["vtxs"]               = map(tup->[tup[1],tup[2]],get_vtxs(env))
    toml_dict["pickup_zones"]       = get_pickup_zones(env)
    toml_dict["dropoff_zones"]      = get_dropoff_zones(env)
    toml_dict["obstacles"]          = map(tup->[tup[1],tup[2]],get_obstacles(env))
    return toml_dict
end
function TOML.print(io,env::E) where {E<:GridFactoryEnvironment}
    TOML.print(io,TOML.parse(env))
end
function read_env(io)
    toml_dict = TOML.parsefile(io)
    x_dim                   = toml_dict["x_dim"]
    y_dim                   = toml_dict["y_dim"]
    vtxs                    = map(arr->(arr[1],arr[2]), toml_dict["vtxs"])
    vtx_grid                = construct_vtx_grid(x_dim,y_dim,vtxs)
    graph                   = initialize_grid_graph_from_vtx_grid(vtx_grid)
    env = GridFactoryEnvironment(
        graph                   = graph,
        x_dim                   = toml_dict["x_dim"],
        y_dim                   = toml_dict["y_dim"],
        cell_width              = toml_dict["cell_width"],
        transition_time         = toml_dict["transition_time"],
        vtxs                    = map(arr->(arr[1],arr[2]), toml_dict["vtxs"]),
        pickup_zones            = toml_dict["pickup_zones"],
        dropoff_zones           = toml_dict["dropoff_zones"],
        obstacles               = map(arr->(arr[1],arr[2]), toml_dict["obstacles"]),
    )
    GridFactoryEnvironment(env, initialize_factory_graph(env))
end

################################################################################
################################ INITIALIZATION ################################
################################################################################

export
    initialize_factory_graph,
    construct_regular_factory_world,
    construct_random_factory_world,
    construct_factory_env_from_vtx_grid,
    sample_random_robot_locations

"""

    `initialize_factory_graph()`

    Returns a grid graph that represents a 2D environment with obstacles,
    dropoff_zones, pickup_zones and robot_locations selected randomly.
"""
function initialize_factory_graph(env::GridFactoryEnvironment)
    vtx_grid = construct_vtx_grid(get_x_dim(env),get_y_dim(env),get_vtxs(env))
    G = initialize_grid_graph_from_vtx_grid(vtx_grid)
    for i in 1:get_x_dim(env)
        for j in 1:get_y_dim(env)
            v = vtx_grid[i,j]
            if v > 0
                set_prop!(G,v,:x,i)
                set_prop!(G,v,:y,j)
            end
        end
    end
    G
end


function construct_regular_factory_world(;
    n_obstacles_x=2,
    n_obstacles_y=2,
    obs_width = [2;2],
    obs_offset = [1;1],
    env_pad = [1;1],
    env_offset = [1,1],
    env_scale = 0.5,
    transition_time=2.0
    )
    PICKUP_FLAG = -1
    DROPOFF_FLAG = -2
    OBSTACLE_FLAG = 0
    NORMAL_VTX_FLAG = 1
    # generate occupancy grid representing the environment
    o = OBSTACLE_FLAG*ones(Int,obs_width[1],obs_width[2]) # obstacle region
    op = pad_matrix(o,(1,1),NORMAL_VTX_FLAG) # padded obstacles region
    flag = PICKUP_FLAG
    for i = 1:size(o,1)
        op[i+1,1] = flag
        flag = flag == PICKUP_FLAG ? DROPOFF_FLAG : PICKUP_FLAG
    end
    for j = 1:size(o,2)
        op[end,j+1] = flag
        flag = flag == PICKUP_FLAG ? DROPOFF_FLAG : PICKUP_FLAG
    end
    for i = reverse(collect(1:size(o,1)))
        op[i+1,end] = flag
        flag = flag == PICKUP_FLAG ? DROPOFF_FLAG : PICKUP_FLAG
    end
    for j = reverse(collect(1:size(o,2)))
        op[1,j+1] = flag
        flag = flag == PICKUP_FLAG ? DROPOFF_FLAG : PICKUP_FLAG
    end
    op = pad_matrix(op,(obs_offset[1]-1,obs_offset[2]-1),NORMAL_VTX_FLAG) # padded obstacles region
    A = repeat(op,n_obstacles_x,n_obstacles_y)
    Ap = pad_matrix(A,(env_pad[1],env_pad[2]),NORMAL_VTX_FLAG) # padded occupancy grid
    K = zeros(Int,size(Ap))

    # pickup_zones    = Vector{Tuple{Int,Int}}()
    pickup_zones    = Vector{Int}()
    #[K[i] for (i,val) in enumerate(Ap[:]) if val == PICKUP_FLAG]
    # dropoff_zones   = Vector{Tuple{Int,Int}}()
    dropoff_zones   = Vector{Int}()
    #[K[i] for (i,val) in enumerate(Ap[:]) if val == DROPOFF_FLAG]
    # obstacles       = Vector{Tuple{Int,Int}}()
    obstacles       = Vector{Tuple{Int,Int}}()
    #[K[i] for (i,val) in enumerate(Ap[:]) if val == OBSTACLE_FLAG]
    vtxs            = Vector{Tuple{Int,Int}}()
    k = 0
    for i in 1:size(Ap,1)
        for j in 1:size(Ap,2)
            if Ap[i,j] == OBSTACLE_FLAG
                push!(obstacles, (i,j))
            else
                k += 1
                K[i,j] = k
                push!(vtxs, (i,j))
                if Ap[i,j] == PICKUP_FLAG
                    push!(pickup_zones, k)
                elseif Ap[i,j] == DROPOFF_FLAG
                    push!(dropoff_zones, k)
                end
            end
        end
    end

    graph = initialize_grid_graph_from_vtx_grid(K)

    env = GridFactoryEnvironment(
        graph               = graph,
        x_dim               = size(Ap,1),
        y_dim               = size(Ap,2),
        cell_width          = env_scale,
        transition_time     = transition_time,
        vtxs                = vtxs,
        pickup_zones        = pickup_zones,
        dropoff_zones       = dropoff_zones,
        obstacles           = obstacles
    )

    return env
end

function construct_factory_env_from_vtx_grid(vtx_grid;
        cell_width=1.0,
        transition_time=1.0,
        pickup_zones = Int64[],
        dropoff_zones = Int64[]
    )

    graph = initialize_grid_graph_from_vtx_grid(vtx_grid)
    vtxs = Vector{Tuple{Int,Int}}()
    for i in 1:size(vtx_grid,1)
        for j in 1:size(vtx_grid,2)
            if vtx_grid[i,j] > 0
                push!(vtxs, (i,j))
            end
        end
    end

    env = GridFactoryEnvironment(
        graph               = graph,
        x_dim               = size(vtx_grid,1),
        y_dim               = size(vtx_grid,2),
        cell_width          = cell_width,
        transition_time     = transition_time,
        vtxs                = vtxs,
        pickup_zones        = pickup_zones,
        dropoff_zones       = dropoff_zones,
    )
end

function construct_random_factory_world(;
    x_dim=20,
    y_dim=20,
    cell_width=0.5,
    transition_time=2.0,
    n_pickup_zones=20,
    n_dropoff_zones=20,
    n_obstacles=20
    )
    N = x_dim*y_dim
    @assert n_pickup_zones + n_dropoff_zones + n_obstacles < N "not enough grid cells"
    vtxs = [(i,j) for i in 1:x_dim for j in 1:y_dim]
    # randomize
    idxs = sortperm(rand(N))
    pickup_zones    = map(i->pop!(idxs), 1:n_pickup_zones)
    dropoff_zones   = map(i->pop!(idxs), 1:n_dropoff_zones)
    obstacles       = map(i->vtxs[pop!(idxs)], 1:n_obstacles)
    env = GridFactoryEnvironment(
        x_dim           = x_dim,
        y_dim           = y_dim,
        cell_width      = cell_width,
        transition_time = transition_time,
        vtxs            = vtxs,
        pickup_zones    = pickup_zones,
        dropoff_zones   = dropoff_zones,
        obstacles       = obstacles
        )

    env = GridFactoryEnvironment(env, initialize_factory_graph(env))
    return env #, robot_locations
end

function sample_random_robot_locations(env::E,N::Int) where {E<:GridFactoryEnvironment}
    idxs = get_free_zones(env)
    @assert N <= length(idxs)
    robot_locations = idxs[sortperm(rand(length(idxs)))][1:N]
end

end # module
