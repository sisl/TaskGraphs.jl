let
    reset_action_id_counter!()
    reset_operation_id_counter!()
    reset_task_id_counter!()

    @test get_unique_task_id() == 1
    @test get_unique_operation_id() == 1
    @test get_unique_action_id() == 1
end
let
    ObjectID()
    RobotID()
    LocationID()
    ActionID()
    OperationID()

    get_id(ObjectID())

    @test ObjectID(1) < ObjectID(2)
    @test ObjectID(2) > ObjectID(1)
    @test ObjectID(1) + 1 == ObjectID(2)
    @test ObjectID(1) == ObjectID(2) - 1
end
let
    ROBOT_AT(1,2)
    get_robot_id(ROBOT_AT(1,2))
end
let
    OBJECT_AT(1,2)
    OBJECT_AT(1,[2])
    OBJECT_AT(1,LocationID(2))
    OBJECT_AT(1,[LocationID(2)])

    get_object_id(OBJECT_AT(1,2))
    get_location_id(OBJECT_AT(1,2))
    get_location_ids(OBJECT_AT(1,2))
end
let
    op = Operation()
    get_location_id(op)
    get_operation_id(op)
    get_input_ids(op)
    get_output_ids(op)
    duration(op)
    preconditions(op)
    postconditions(op)
    add_conditions(op)
    delete_conditions(op)
    get_id(op)
end
let
    GO()
    node = GO(1,2,3)
    get_initial_location_id(node)
    get_destination_location_id(node)
    get_robot_id(node)
    # GO does not have a single location, so this method should fail
    @test_throws MethodError get_location_id(node)
    @test_throws MethodError get_object_id(node)
end
let
    CARRY()
    node = CARRY(1,2,3,4)
    get_initial_location_id(node)
    get_destination_location_id(node)
    get_object_id(node)
    get_robot_id(node)
    # CARRY does not have a single location, so this method should fail
    @test_throws MethodError get_location_id(node)
end
let
    COLLECT()
    node = COLLECT(1,2,3)
    get_initial_location_id(node)
    get_destination_location_id(node)
    get_location_id(node)
    get_object_id(node)
    get_robot_id(node)
end
let
    DEPOSIT()
    node = DEPOSIT(1,2,3)
    get_initial_location_id(node)
    get_destination_location_id(node)
    get_location_id(node)
    get_object_id(node)
    get_robot_id(node)
end
let
    for node in [OBJECT_AT(1,2),ROBOT_AT(1,2),GO(),COLLECT(),CARRY(),DEPOSIT(),Operation()]
        required_predecessors(node)
        required_successors(node)
        eligible_predecessors(node)
        eligible_successors(node)
        resources_reserved(node)
        for node2 in [OBJECT_AT(1,2),ROBOT_AT(1,2),GO(),COLLECT(),CARRY(),DEPOSIT(),Operation()]
            matches_template(typeof(node),node2)
            matches_template((typeof(node),typeof(node2)),node2)
        end
    end
end
let
    n1 = GO(1,2,3)
    n2 = GO(-1,-1,4)
    n = align_with_predecessor(n2,n1)
    @test n.r == n1.r
    @test n.x1 == n1.x2
    @test n.x2 == n2.x2
end
let
    n1 = ROBOT_AT(1,2)
    n2 = GO(-1,-1,3)
    n = align_with_predecessor(n2,n1)
    @test n.r == n1.r
    @test n.x1 == n1.x
end
let
    n1 = DEPOSIT(1,2,3)
    n2 = GO(-1,3,4)
    n = align_with_predecessor(n2,n1)
    @test n.r == n1.r
    @test n.x1 == n1.x
end
let
    n1 = GO(1,2,4)
    n2 = COLLECT(-1,3,4)
    n = align_with_predecessor(n2,n1)
    @test n.r == n1.r
    @test n.x == n1.x2
end
let
    n1 = OBJECT_AT(3,4)
    n2 = COLLECT(-1,3,4)
    n = align_with_predecessor(n2,n1)
    @test n.o == n1.o
    @test n.x == n1.x[1]
end
let
    n1 = COLLECT(1,2,3)
    n2 = CARRY(-1,2,3,4)
    n = align_with_predecessor(n2,n1)
    @test n.o == n1.o
    @test n.x1 == n1.x
    @test n.r == n1.r
end
let
    n1 = CARRY(1,2,3,4)
    n2 = DEPOSIT(-1,2,4)
    n = align_with_predecessor(n2,n1)
    @test n.o == n1.o
    @test n.x == n1.x2
    @test n.r == n1.r
end
let
    TEAM_ACTION()
    TEAM_ACTION(instructions=[CARRY(1,2,3,4),CARRY(2,3,4,4)],shape=(2,1))

    sequence = [
        TEAM_ACTION(instructions=[COLLECT(1,2,3),COLLECT(2,3,4)],shape=(2,1)),
        TEAM_ACTION(instructions=[CARRY(-1,2,3,4),CARRY(-1,3,4,5)],shape=(2,1)),
        TEAM_ACTION(instructions=[DEPOSIT(-1,2,4),DEPOSIT(-1,3,5)],shape=(2,1)),
    ]
    @test all(map(p->get_id(get_robot_id(p)) == -1, sequence[2].instructions))
    sequence[2] = align_with_predecessor(sequence[2],sequence[1])
    @test all(map(p->get_id(get_robot_id(p)) != -1, sequence[2].instructions))

    @test all(map(p->get_id(get_robot_id(p)) == -1, sequence[3].instructions))
    sequence[3] = align_with_predecessor(sequence[3],sequence[2])
    @test all(map(p->get_id(get_robot_id(p)) != -1, sequence[3].instructions))
end
let
    @test validate_edge(ROBOT_AT(1,2),ROBOT_AT(1,2)) == false
    @test validate_edge(ROBOT_AT(1,2),GO(1,2,3))
    @test validate_edge(GO(1,2,3),GO(1,3,3))
    @test validate_edge(GO(1,2,3),COLLECT(1,4,3))
    @test validate_edge(COLLECT(1,2,3),CARRY(1,2,3,4))
    @test validate_edge(CARRY(1,2,3,4),COLLECT(1,2,3)) == false
    @test validate_edge(CARRY(1,2,3,4),DEPOSIT(1,2,4))
    @test validate_edge(DEPOSIT(1,2,4),CARRY(1,2,3,4)) == false
    @test validate_edge(DEPOSIT(1,2,4),GO(1,4,3))
    @test validate_edge(DEPOSIT(1,2,4),DEPOSIT(1,2,4))
    @test validate_edge(COLLECT(1,2,4),DEPOSIT(1,2,4))
    @test validate_edge(COLLECT(1,2,4),COLLECT(1,2,4))
end
