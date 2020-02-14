let
    env = construct_regular_factory_world()
    filename = "env.toml"
    open(filename,"w") do io
        TOML.print(io, TOML.parse(env))
    end
    env2 = read_env(filename)

    @test get_vtxs(env) == get_vtxs(env2)
    @test nv(env.graph) == nv(env2.graph)
    @test ne(env.graph) == ne(env2.graph)
end
let
    N = 4
    M = 6
    env = construct_random_factory_world(
        n_pickup_zones=M*2,
        n_dropoff_zones=M*2
        )
    robot_locations = map(v->collect(v), get_vtxs(env)[sample_random_robot_locations(env,N)])
    robot_headings = map(i->[1,0],1:N)
end
let
    N = 4
    M = 6
    env = construct_random_factory_world(n_pickup_zones=M*2,n_dropoff_zones=M*2)

    r0,s0,sF = get_random_problem_instantiation(N,M,
        get_pickup_zones(env),get_dropoff_zones(env),get_free_zones(env))

    dist_matrix = get_dist_matrix(env.graph)
    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])

    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    project_spec = construct_random_project_spec(M,s0,sF;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
end
let
    xdim = 4
    ydim = 4
    vtx_grid = initialize_dense_vtx_grid(xdim,ydim)
    env_graph = initialize_grid_graph_from_vtx_grid(vtx_grid)
    vtxs = [(i,j) for i in 1:xdim for j in 1:ydim]
    factory_env = GridFactoryEnvironment(
        graph = env_graph,
        x_dim=xdim,
        y_dim=ydim,
        vtxs=vtxs
    )
end
# test custom envs for different-sized robot sizes
let
    factory_env = construct_regular_factory_world(;n_obstacles_x=1,n_obstacles_y=1)
    # 1   2   3   4   5   6
    # 7   8   9  10  11  12
    # 13  14   0   0  15  16
    # 17  18   0   0  19  20
    # 21  22  23  24  25  26
    # 27  28  29  30  31  32
    dist_matrix = get_dist_matrix(factory_env)
    dist_mtx_map = factory_env.dist_function

    v1 = 13
    v2 = 15
    @test get_distance(dist_mtx_map,v1,v2) == dist_matrix[v1,v2]
    @test get_distance(dist_mtx_map,v1,v2,(1,1)) == dist_matrix[v1,v2]
    @test get_distance(dist_mtx_map,v1,v2,(1,2)) == dist_matrix[v1,v2]
    @test get_distance(dist_mtx_map,v1,v2,(2,1)) == dist_matrix[v1,v2] + 2
    @test get_distance(dist_mtx_map,v1,v2,(2,2)) == dist_matrix[v1,v2] + 2

    v3 = 18
    v4 = 20
    config = 4
    @test get_distance(dist_mtx_map,v3,v4,(2,2),config) == dist_matrix[v1,v2] + 2

end
