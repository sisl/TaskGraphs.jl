using TaskGraphs
using Gurobi
set_default_milp_optimizer!(Gurobi.Optimizer)

let
    solver = NBSSolver()
    prob = pctapf_problem_1(NBSSolver())
    pcta = PC_TA(prob.env)
    sched, cost = solve!(solver.assignment_model,pcta)
    search_env = construct_search_env(solver,sched,get_env(prob))
    # init pc_mapf
    pc_mapf = construct_routing_problem(prob,search_env)

    # build MPCCBSEnv for robot 1
    agent_id = RobotID(1)
    goal_sequence = TaskGraphs.extract_robot_itinerary(sched,agent_id)
    env = TaskGraphs.MPCCBSEnv(
        search_env = get_env(pc_mapf),
        agent_id = agent_id,
        goal_sequence = goal_sequence,
        )
    # construct first goal
    s0 = get_final_state(get_paths(get_route_plan(search_env))[get_id(agent_id)])
    s = TaskGraphs.MState(
        vtx=s0.vtx,
        t=s0.t,
        stage=1,
        node=get_node(sched,env.goal_sequence[1]),
    )
    TaskGraphs.construct_goal(env,s)
    TaskGraphs.check_stage_goal(env,s)
    a = CRCBS.wait(env,s)
    sp = get_next_state(env,s,a)

end