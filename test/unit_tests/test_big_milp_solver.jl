# Big MILP solver
let
    prob = pctapf_problem_2(NBSSolver())
    solver = TaskGraphs.BigMILPSolver(EXTRA_T=2)
    solve!(solver,prob)

    solve!(NBSSolver(),prob)

    milp = TaskGraphs.formulate_big_milp(prob,solver.EXTRA_T)
    optimize!(milp)
    @test termination_status(milp) == MOI.OPTIMAL
    robot_paths = TaskGraphs.extract_robot_paths(prob,milp)
    @test robot_paths[RobotID(1)][1:7] == [1, 5, 9, 13, 17, 21, 25]
    @test robot_paths[RobotID(2)][1:7] == [4, 8, 12, 16, 20, 24, 28]

    object_paths = TaskGraphs.extract_object_paths(prob,milp)
    @test object_paths[ObjectID(1)][1:7] == [9, 9, 9, 13, 17, 17, 17,] 
    @test object_paths[ObjectID(2)][1:7] == [12, 12, 12, 16, 20, 24, 28,]
    @test object_paths[ObjectID(3)][1:7] == [21, 21, 21, 21, 21, 21, 25,]

    # robot_path = robot_paths[RobotID(1)]
    # object_path = object_paths[ObjectID(1)]
    # TaskGraphs.extract_solution(prob,milp)
end