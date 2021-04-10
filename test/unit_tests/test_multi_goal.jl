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
    pc_mapf = construct_routing_problem(prob,construct_search_env(solver,sched,get_env(prob)))

    base_env = TaskGraphs.build_base_multi_goal_env(solver,pc_mapf)

    # build MPCCBSEnv for robot 1
    agent_id = RobotID(1)
    env = TaskGraphs.MPCCBSEnv(base_env,agent_id = agent_id)

    s = get_initial_state(get_paths(get_route_plan(env.search_env))[get_id(agent_id)])
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