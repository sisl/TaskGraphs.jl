using TaskGraphs
using Gurobi
set_default_milp_optimizer!(Gurobi.Optimizer)
using Test
using LightGraphs

let
    solver = NBSSolver()
    prob = pctapf_problem_1(NBSSolver())
    solution,cost = solve!(solver,prob)
    # 
    pcta = PC_TA(prob.env)
    sched, cost = solve!(solver.assignment_model,pcta)
    search_env = construct_search_env(solver,sched,get_env(prob))
    env_graph = get_graph(search_env)
    cost_model, _ = construct_cost_model(solver,search_env,EnvDeadlineCost())

    # init pc_mapf
    pc_mapf = construct_routing_problem(prob,search_env)

    # build MPCCBSEnv for robot 1
    agent_id = RobotID(1)
    itineraries = Dict(id=>TaskGraphs.extract_robot_itinerary(sched,id) for id in keys(get_robot_ICs(sched)))
    vtx_sequences = Dict(get_id(k)=>map(n->get_vtx(TaskGraphs.construct_goal(n)),v) for (k,v) in itineraries)
    cost_to_go = CRCBS.construct_multi_stage_env_distance_heuristic(get_graph(search_env),vtx_sequences)
    heuristic = construct_composite_heuristic(
        cost_to_go,
        NullHeuristic(),
        cost_to_go,
        cost_to_go,
        NullHeuristic(),
    )
    env = TaskGraphs.MPCCBSEnv(
        search_env = get_env(pc_mapf),
        agent_id = agent_id,
        itinerary = TaskGraphs.extract_robot_itinerary(sched,agent_id),
        cost_model = cost_model,
        heuristic = heuristic 
        )
    # construct first goal
    s0 = get_final_state(get_paths(get_route_plan(search_env))[get_id(agent_id)])
    s = TaskGraphs.MState(
        vtx=s0.vtx,
        t=s0.t,
        stage=1,
        node=get_node(sched,env.itinerary[1]),
    )
    TaskGraphs.construct_goal(env,s)
    TaskGraphs.check_stage_goal(env,s)
    # Test cost and heuristic_model
    c = get_initial_cost(env)
    @test c[1] == 0.0
    h = CRCBS.compute_heuristic_cost(env,c,s)
    @test h[1] == 0.0

    a = CRCBS.wait(env,s)
    sp = get_next_state(env,s,a)
    c = get_transition_cost(env,s,a,sp)
    @test c[1] == 1
    h = CRCBS.compute_heuristic_cost(env,c,sp)
    @test h[1] == 1

    a = GraphAction(Edge(s.vtx,5),1)
    sp = get_next_state(env,s,a)
    c = get_transition_cost(env,s,a,sp)
    @test c[1] == 1
    h = CRCBS.compute_heuristic_cost(env,c,sp)
    @test h[1] == 0

    solver = AStar{NTuple{5,Float64}}()
    set_iteration_limit!(solver,10)
    set_verbosity!(solver,5)
    path, cost = CRCBS.a_star!(solver,env,s)

end