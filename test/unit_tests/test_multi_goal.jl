using TaskGraphs
using Gurobi
set_default_milp_optimizer!(Gurobi.Optimizer)
using Test
using LightGraphs

let
    for f in [
            pctapf_problem_1,
            pctapf_problem_2,
            pctapf_problem_3,
            pctapf_problem_4,
            pctapf_problem_5,
            pctapf_problem_6,
            pctapf_problem_7,
            pctapf_problem_8,
            pctapf_problem_9,
            pctapf_problem_10,
        ]
        @show f
        solver = NBSSolver()
        prob = f(solver)
        solution,cost = solve!(solver,prob)
        pcta = PC_TA(prob.env)
        sched, cost = solve!(solver.assignment_model,pcta)
        pc_mapf = construct_routing_problem(prob,construct_search_env(solver,sched,get_env(prob)))

        solver = AStarSC{NTuple{5,Float64}}()
        env = TaskGraphs.solve_with_multi_goal_solver!(solver,pc_mapf)
        env.search_env
    end

end

let
    solver = NBSSolver()
    prob = pctapf_problem_2(NBSSolver())
    solution,cost = solve!(solver,prob)
    pcta = PC_TA(prob.env)
    sched, cost = solve!(solver.assignment_model,pcta)
    pc_mapf = construct_routing_problem(prob,construct_search_env(solver,sched,get_env(prob)))
    solver = AStarSC{NTuple{5,Float64}}()
    env = TaskGraphs.solve_with_multi_goal_solver!(solver,pc_mapf);

    for (id,itinerary) in env.itineraries
        for n in get_paths(env)[get_id(id)].path_nodes
            @show string(n.s), string(n.sp)
        end
    end

    env.search_env

    base_env = TaskGraphs.build_base_multi_goal_env(solver,pc_mapf) 

    agent_id = RobotID(1)
    env = TaskGraphs.MPCCBSEnv(base_env,agent_id = agent_id,agent_idx=get_id(agent_id))
    s = get_start(env,agent_id)

    @test TaskGraphs.check_stage_goal(env,s)
    @test TaskGraphs.check_stage_goal(env,s)

    c = get_initial_cost(env)
    @test c[1] == 0.0
    h = CRCBS.compute_heuristic_cost(env,c,s)
    @test h[1] == 0.0

    a = CRCBS.wait(env,s)
    sp = get_next_state(env,s,a)
    @test !TaskGraphs.check_stage_goal(env,sp)
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
end