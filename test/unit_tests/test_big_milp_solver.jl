# Big MILP solver
let
    prob = pctapf_problem_2(NBSSolver())
    model = Model(default_milp_optimizer())
    milp_model = TaskGraphs.formulate_big_milp(prob,8,model)
    optimize!(model)
    @test termination_status(model) == MOI.OPTIMAL
    robot_paths = TaskGraphs.extract_robot_paths(prob,milp_model)
    @test robot_paths[RobotID(1)][1:7] == [1, 5, 9, 13, 17, 21, 25]
    @test robot_paths[RobotID(2)][1:7] == [4, 8, 12, 16, 20, 24, 28]

    object_paths = TaskGraphs.extract_object_paths(prob,milp_model)
    @test object_paths[ObjectID(1)][1:7] == [9, 9, 9, 13, 17, 17, 17,] 
    @test object_paths[ObjectID(2)][1:7] == [12, 12, 12, 16, 20, 24, 28,]
    @test object_paths[ObjectID(3)][1:7] == [21, 21, 21, 21, 21, 21, 25,]
end