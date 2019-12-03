module FactoryWorlds

using Parameters
using LightGraphs, MetaGraphs, GraphUtils
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

"""
    `GridFactoryEnvironment`
"""
@with_kw struct GridFactoryEnvironment{G}
    graph               ::G                         = MetaDiGraph()
    x_dim               ::Int                       = 20
    y_dim               ::Int                       = 20
    cell_width          ::Float64                   = 0.5
    transition_time     ::Float64                   = 2.0
    vtxs                ::Vector{Tuple{Int,Int}}    = Vector{Tuple{Int,Int}}()
    pickup_zones        ::Vector{Int}               = Vector{Int}()
    dropoff_zones       ::Vector{Int}               = Vector{Int}()
    obstacles           ::Vector{Tuple{Int,Int}}    = Vector{Tuple{Int,Int}}()
    # robot_locations     ::Vector{Tuple{Int,Int}}        = Vector{Tuple{Int,Int}}()
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
        obstacles       = env.obstacles
    )
end

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
        dropoff_zones       = dropoff_zones
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
