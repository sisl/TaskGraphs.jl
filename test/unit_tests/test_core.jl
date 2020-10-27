let
    reset_debug_file_id!()
    @test get_debug_file_id() == 1
    @test get_debug_file_id() == 2
    reset_debug_file_id!()
end
let
    spec = ProblemSpec()
    get_distance(spec,1,1)
end
let
    spec = ProjectSpec()
    get_initial_nodes(spec)

    TaskGraphs.set_initial_condition!(spec,1,OBJECT_AT(1,1))
    @test TaskGraphs.get_initial_condition(spec,1) == OBJECT_AT(1,1)
    TaskGraphs.set_initial_condition!(spec,1,OBJECT_AT(1,1))
    @test TaskGraphs.get_initial_condition(spec,1) == OBJECT_AT(1,1)
    TaskGraphs.set_initial_condition!(spec,2,OBJECT_AT(2,1))
    @test TaskGraphs.get_initial_condition(spec,2) == OBJECT_AT(2,1)
    TaskGraphs.set_final_condition!(spec,1,OBJECT_AT(1,2))
    @test TaskGraphs.get_final_condition(spec,1) == OBJECT_AT(1,2)
    TaskGraphs.set_final_condition!(spec,1,OBJECT_AT(1,3))
    @test TaskGraphs.get_final_condition(spec,1) == OBJECT_AT(1,3)
    TaskGraphs.set_final_condition!(spec,2,OBJECT_AT(2,3))
    @test TaskGraphs.get_final_condition(spec,2) == OBJECT_AT(2,3)

    # add operations to a ProjectSpec and test that the dependencies are
    # stored correctly
    op1 = construct_operation(spec,-1,[],[1],0.0)
    op2 = construct_operation(spec,-1,[1],[2],0.0)
    op3 = construct_operation(spec,-1,[2],[],0.0)
    add_operation!(spec,op1)
    add_operation!(spec,op2)
    add_operation!(spec,op3)
    @test has_edge(spec.graph, spec.op_id_to_vtx[get_id(op1)], spec.op_id_to_vtx[get_id(op2)])
    @test has_edge(spec.graph, spec.op_id_to_vtx[get_id(op2)], spec.op_id_to_vtx[get_id(op3)])

    # now reset the spec to empty and see if the operations can be safely added
    spec = ProjectSpec()
    add_operation!(spec,op1)
    add_operation!(spec,op2)
    add_operation!(spec,op3)
    @test has_edge(spec.graph, spec.op_id_to_vtx[get_id(op1)], spec.op_id_to_vtx[get_id(op2)])
    @test has_edge(spec.graph, spec.op_id_to_vtx[get_id(op2)], spec.op_id_to_vtx[get_id(op3)])

end
let
    def = SimpleProblemDef(
        project_spec = ProjectSpec(),
        r0 = [1,2,3],
        s0 = [1,2,3],
        sF = [1,2,3],
    )
    @test length(def.shapes) == length(def.s0)
end
# ProjectSpec
let
    M = 3
    object_ICs = Vector{OBJECT_AT}([
        OBJECT_AT(1,1),
        OBJECT_AT(2,2),
        OBJECT_AT(3,3)
        ])
    object_FCs = Vector{OBJECT_AT}([
        OBJECT_AT(1,4),
        OBJECT_AT(2,5),
        OBJECT_AT(3,6)
        ])
    robot_ICs = [
        ROBOT_AT(1,7),
        ROBOT_AT(2,8),
        ROBOT_AT(3,9)
        ]
    # Testing root nodes
    let
        project_spec = ProjectSpec(object_ICs,object_FCs)
        add_operation!(project_spec,construct_operation(project_spec, 3, [1,2], [3], 1.0))
        @test project_spec.terminal_vtxs == Set{Int}([1])
        add_operation!(project_spec,construct_operation(project_spec, 6, [3], [], 0.0))
        @test project_spec.terminal_vtxs == Set{Int}([2])
    end
    let
        project_spec = ProjectSpec(object_ICs,object_FCs)
        add_operation!(project_spec,construct_operation(project_spec, 3, [1,2], [], 1.0))
        @test project_spec.terminal_vtxs == Set{Int}([1])
        add_operation!(project_spec,construct_operation(project_spec, 6, [3], [], 0.0))
        @test project_spec.terminal_vtxs == Set{Int}([1,2])
    end
    let
        project_spec = ProjectSpec(object_ICs,object_FCs)
        add_operation!(project_spec,construct_operation(project_spec, 3, [1,2], [3], 1.0))
        add_operation!(project_spec,construct_operation(project_spec, 6, [3], [], 0.0))
        delivery_graph = construct_delivery_graph(project_spec,M)
    end
end
let
    PathSpec()
end
let
    sched = OperatingSchedule()
    node = OBJECT_AT(1,1)
    add_to_schedule!(sched,node,get_object_id(node))
    replace_in_schedule!(sched,node,get_object_id(node))
    get_path_spec(sched,1)
    set_path_spec!(sched,1,get_path_spec(sched,1))
    for pred in [
            OBJECT_AT(1,2),
            ROBOT_AT(1,2),
            GO(),
            COLLECT(),
            CARRY(),
            DEPOSIT(),
            TEAM_CARRY(instructions=[CARRY()])
            ]
        generate_path_spec(sched,pred)
    end

    get_graph(sched)
    get_vtx_ids(sched)

    id = get_object_id(node)
    get_node_from_id(sched,id)
    get_vtx(sched,id)
    v = 1
    get_vtx_id(sched,v)
    get_node_from_vtx(sched,v)

    get_object_ICs(sched)
    get_robot_ICs(sched)
    get_actions(sched)
    get_operations(sched)

    zero(sched)
    LightGraphs.edges(sched)
    LightGraphs.edgetype(sched)
    LightGraphs.has_edge(sched,1,1)
    LightGraphs.has_vertex(sched,1)
    LightGraphs.inneighbors(sched,1)
    LightGraphs.outneighbors(sched,1)
    LightGraphs.is_directed(sched)
    LightGraphs.ne(sched)
    LightGraphs.nv(sched)
    LightGraphs.vertices(sched)
end
let
    # test node deletion
    sched = OperatingSchedule()
    nodes = [OBJECT_AT(1,1),OBJECT_AT(2,2),OBJECT_AT(3,3)]
    for node in nodes
        add_to_schedule!(sched,node,get_object_id(node))
    end
    delete_nodes!(sched,[1,2])
    @test get_node_from_vtx(sched,1) == OBJECT_AT(3,3)
end
# Test process_schedule
let
    solver = TaskGraphsMILPSolver(AssignmentMILP())
    project_spec, problem_spec, robot_ICs, env_graph, _ = pctapf_problem_1()

    project_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
    model = formulate_milp(solver,project_schedule,problem_spec;cost_model=MakeSpan())
    optimize!(model)
    @test termination_status(model) == MOI.OPTIMAL
    cost = Int(round(value(objective_function(model))))
    update_project_schedule!(solver,model,project_schedule,problem_spec,get_assignment_matrix(model))

    t0,tF,slack,local_slack = process_schedule(project_schedule)
    @test t0[get_vtx(project_schedule,RobotID(1))] == 0
    @test t0[get_vtx(project_schedule,RobotID(2))] == 0
    # try with perturbed start times
    t0[get_vtx(project_schedule,RobotID(2))] = 1
    t0,tF,slack,local_slack = process_schedule(project_schedule,t0)
    @test t0[get_vtx(project_schedule,RobotID(1))] == 0
    @test t0[get_vtx(project_schedule,RobotID(2))] == 1

    @test length(get_vtx_ids(project_schedule)) == nv(get_graph(project_schedule))
    for (v,id) in enumerate(project_schedule.vtx_ids)
        @test get_vtx(project_schedule, id) == v
    end
end
let
    project_spec, problem_spec, robot_ICs, env_graph, _ = pctapf_problem_1()

    filename = "/tmp/project_spec.toml"
    open(filename, "w") do io
        TOML.print(io, TOML.parse(project_spec))
    end
    project_spec_mod = read_project_spec(filename)
    # run(`rm $filename`)

    r0 = [get_id(get_initial_location_id(robot_ICs[k])) for k in sort(collect(keys(robot_ICs)))]
    s0 = map(pred->get_id(get_initial_location_id(pred)),project_spec.initial_conditions)
    sF = map(pred->get_id(get_initial_location_id(pred)),project_spec.final_conditions)
    problem_def = SimpleProblemDef(project_spec,r0,s0,sF)
    filename = "/tmp/problem_def.toml"
    open(filename, "w") do io
        TOML.print(io, TOML.parse(problem_def))
    end
    problem_def = read_problem_def(filename)
    # run(`rm $filename`)
end
let
    solver = NBSSolver()
    prob = pctapf_problem_1(solver)
    model = formulate_assignment_problem(assignment_solver(solver),prob)
    sched,_ = solve_assignment_problem!(assignment_solver(solver),model,prob)
    tips = robot_tip_map(sched)
    [k=>string(get_node_from_id(sched,id)) for (k,id) in tips]
end
let
    sched = OperatingSchedule()
    add_to_schedule!(sched,ROBOT_AT(1,1),RobotID(1))
    add_to_schedule!(sched,ROBOT_AT(2,5),RobotID(2))
    add_to_schedule!(sched,GO(1,1,2),ActionID(1))
    add_to_schedule!(sched,OBJECT_AT(1,2),ObjectID(1))
    add_to_schedule!(sched,COLLECT(1,1,2),ActionID(2))
    add_to_schedule!(sched,CARRY(1,1,2,3),ActionID(3))
    add_edge!(sched,RobotID(1),ActionID(1))
    add_edge!(sched,ActionID(1),ActionID(2))
    add_edge!(sched,ObjectID(1),ActionID(2))
    add_edge!(sched,ActionID(2),ActionID(3))
    for (id,vtxs) in [
        (ActionID(3),[get_vtx(sched,ActionID(2))]),
        (ActionID(2),[get_vtx(sched,ActionID(1))]),
        (ActionID(1),[get_vtx(sched,RobotID(1))]),
        (ObjectID(1),Int[]),
        ]
        v = get_vtx(sched,id)
        @test vtxs == backtrack_node(sched,v)
    end
    tips = robot_tip_map(sched)
    @test tips[RobotID(1)] == ActionID(3)
    @test tips[RobotID(2)] == RobotID(2)
end
let
    sched = OperatingSchedule()
    add_to_schedule!(sched,ROBOT_AT(1,1),RobotID(1))
    add_to_schedule!(sched,ROBOT_AT(2,2),RobotID(2))
    add_to_schedule!(sched,OBJECT_AT(1,2),ObjectID(1))
    add_to_schedule!(sched,GO(1,1,2),ActionID(1))
    add_to_schedule!(sched,GO(2,2,3),ActionID(2))
    add_to_schedule!(sched,TEAM_COLLECT(instructions=[COLLECT(1,1,2),COLLECT(2,1,3),]),ActionID(3))
    add_edge!(sched,RobotID(1),ActionID(1))
    add_edge!(sched,RobotID(2),ActionID(2))
    add_edge!(sched,RobotID(2),ActionID(2))
    add_edge!(sched,ActionID(1),ActionID(3))
    add_edge!(sched,ActionID(2),ActionID(3))
    tips = robot_tip_map(sched)
    @test tips[RobotID(1)] == ActionID(3)
    @test tips[RobotID(2)] == ActionID(3)
end
let
    sched = OperatingSchedule()
    for (robot_ids, node, node_id) in [
        ([1],ROBOT_AT(1,1),RobotID(1)),
        ([1],GO(1,1,2),ActionID(1)),
        ([-1],GO(-1,1,2),ActionID(2)),
        ([],OBJECT_AT(1,2),ObjectID(1)),
        ([1,2],TEAM_CARRY(instructions=[CARRY(1,1,2,3),CARRY(2,1,3,4)]),ActionID(3))
        ]
        add_to_schedule!(sched,node,node_id)
        @test get_robot_ids(sched,node_id) == map(i->RobotID(i),robot_ids)
    end
end
