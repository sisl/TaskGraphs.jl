let
    s = PCCBS.State()
    a = PCCBS.Action()
    env = PCCBS.LowLevelEnv()
    get_next_state(s,a)
    get_next_state(env,s,a)

    @test PCCBS.State(1,2) == PCCBS.State(1,2)
    @test PCCBS.State() == PCCBS.State()
    @test states_match(PCCBS.State(),PCCBS.State())
    @test states_match(env,PCCBS.State(),PCCBS.State())
    CRCBS.wait(s)
    CRCBS.wait(env,s)
end
let
    env = PCCBS.LowLevelEnv(graph=construct_regular_factory_world(;n_obstacles_x=1,n_obstacles_y=1))
    get_cost_model(env)
    get_heuristic_model(env)
    @test CRCBS.is_valid(env,PCCBS.State()) == false
    @test CRCBS.is_valid(env,PCCBS.Action()) == false
    @test CRCBS.is_valid(env,PCCBS.State(1,2))
    @test CRCBS.is_valid(env,PCCBS.Action(Edge(1,2),1))

    get_heuristic_cost(env,PCCBS.State(1,2))
end
let
    env_graph = construct_regular_factory_world(;n_obstacles_x=1,n_obstacles_y=1)
    env = PCCBS.LowLevelEnv(
        graph=env_graph,
        heuristic = PerfectHeuristic(get_dist_matrix(env_graph))
        )
    get_heuristic_cost(env,PCCBS.State(1,2))

    env = PCCBS.LowLevelEnv(
        graph=env_graph,
        heuristic = HardConflictHeuristic(env_graph,100,1)
        )
    get_heuristic_cost(env,PCCBS.State(1,2))
end
let
    env = PCCBS.LowLevelEnv(goal=PCCBS.State(1,2))
    @test is_goal(env,PCCBS.State(1,0)) == false
    @test is_goal(env,PCCBS.State(2,3)) == false
    @test is_goal(env,PCCBS.State(1,3))
end
let
    env = PCCBS.LowLevelEnv(graph=construct_regular_factory_world(;n_obstacles_x=1,n_obstacles_y=1))
    s = PCCBS.State(1,0)
    get_possible_actions(env,s)
    @test length(get_possible_actions(env,PCCBS.State(-1,-1))) == 0
end
let
    env = PCCBS.LowLevelEnv(
        graph=construct_regular_factory_world(;n_obstacles_x=1,n_obstacles_y=1),
        schedule_node=COLLECT(1,2,3)
        )
    s = PCCBS.State(1,0)
    @test length(get_possible_actions(env,PCCBS.State(1,0))) == 1
    @test length(get_possible_actions(env,PCCBS.State(-1,-1))) == 0
end
