# module Helpers
#
# using LightGraphs
# using MetaGraphs
# using GraphUtils
# using CRCBS
#
# # using ..PlanningPredicates
# # using ..TaskGraphsCore
# # using ..TaskGraphsUtils
# using ..TaskGraphs

export
    init_env_1,
    init_env_2,
    init_env_3

init_env_1() = construct_regular_factory_world(;
    n_obstacles_x=2,
    n_obstacles_y=2,
    obs_width = [4;4], # obs_w = 8/n
    obs_offset = [4;4],
    env_pad = [1;1],
    env_offset = [1,1],
    env_scale = 1
)
init_env_2() = construct_regular_factory_world(;
    n_obstacles_x=4,
    n_obstacles_y=4,
    obs_width = [2;2],
    obs_offset = [2;2],
    env_pad = [1;1],
    env_offset = [1,1],
    env_scale = 1
)
init_env_3() = construct_regular_factory_world(;
    n_obstacles_x=8,
    n_obstacles_y=8,
    obs_width = [1;1],
    obs_offset = [1;1],
    env_pad = [1;1],
    env_offset = [1,1],
    env_scale = 1
)

export
    pctapf_problem,
    pctapf_problem_1,
    pctapf_problem_2,
    pctapf_problem_3,
    pctapf_problem_4,
    pctapf_problem_5,
    pctapf_problem_6,
    pctapf_problem_7,
    pctapf_problem_8

function get_zero_initial_conditions(G,N)
    # set initial conditions
    to0_ = Dict{Int,Float64}()
    for v in vertices(G)
        if is_root_node(G,v)
            to0_[v] = 0.0
        end
    end
    tr0_ = Dict{Int,Float64}()
    for i in 1:N
        tr0_[i] = 0.0
    end
    return to0_, tr0_
end

function print_toy_problem_specs(prob_name,vtx_grid,r0,s0,sF,project_spec,delivery_graph=nothing)
    println(prob_name)
    display(vtx_grid)
    print("\n\n")
    @show r0
    @show s0
    @show sF
    display(project_spec.operations)
    print("\n\n")
    display(delivery_graph.tasks)
    print("\n\n")
end

function pctapf_problem(r0,s0,sF)
    N = length(r0)
    M = length(s0)
    object_ICs = Vector{OBJECT_AT}([OBJECT_AT(o,s0[o]) for o in 1:M]) # initial_conditions
    object_FCs = Vector{OBJECT_AT}([OBJECT_AT(o,sF[o]) for o in 1:M]) # final conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    for r in N+1:N+M
        robot_ICs[r] = ROBOT_AT(r,sF[r-N][1])
    end
    # Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,dist_function)
    project_spec = ProjectSpec(
        M=M,
        initial_conditions=object_ICs,
        final_conditions=object_FCs,
        )
    project_spec, robot_ICs
end

function pctapf_problem(
        solver,
        project_spec::ProjectSpec,
        problem_spec::ProblemSpec,
        robot_ICs,
        env_graph,
        args...
    )
    project_schedule = construct_partial_project_schedule(
        project_spec,
        problem_spec,
        robot_ICs,
        )
    env = construct_search_env(
        solver,
        project_schedule,
        problem_spec,
        env_graph
        )
    PC_TAPF(env)
end

# This is a place to put reusable problem initializers for testing
"""
    pctapf_problem_1

Optimal MakeSpan = 5
Optimal SumOfMakeSpans = 5
"""
function pctapf_problem_1(;cost_function=SumOfMakeSpans(),verbose=false)
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    r0 = [1,4]
    s0 = [5,8,14]
    sF = [13,12,15]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )
    project_spec, robot_ICs = pctapf_problem(r0,s0,sF)

    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],0))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], 0))
    assignment_dict = Dict(1=>[1,3],2=>[2])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function)
    # if verbose
    #     problem_description = """
    #     #### TOY PROBLEM 1 ####
    #     r0 = [1,4]
    #     s0 = [5,8,14]
    #     sF = [13,12,15]
    #     assignment_dict = Dict(1=>[1,3],2=>[2])
    #     """
    #     print_toy_problem_specs(problem_description,vtx_grid,r0,s0,sF,project_spec)
    # end
    return project_spec, problem_spec, robot_ICs, env_graph, assignment_dict
end

"""
    In this problem robot 1 will first do [1-5-9], then [9-13-17]
    robot 2 will do [4-8-32]. The key thing is that robot 1 will need to wait
    until robot 2 is finished before robot 1 can do its second task.

    Optimal paths:
    Optimal MakeSpan = 8
    Optimal SumOfMakeSpans = 8
"""
function pctapf_problem_2(;cost_function=SumOfMakeSpans(),verbose=false)
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(8,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    # 17  18  19  20
    # 21  22  23  24
    # 25  26  27  28
    # 29  30  31  32
    r0 = [1,4]
    s0 = [5,8,13]
    sF = [9,32,17]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )
    project_spec, robot_ICs = pctapf_problem(r0,s0,sF)

    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],0))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], 0))
    assignment_dict = Dict(1=>[1,3],2=>[2])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function)
    if verbose
        problem_description = """
            TOY PROBLEM 2

            In this problem robot 1 will need to wait while robot 2 finishes.
            First operation:
                robot 1 does [1-5-9]
                robot 2 does [4-8-32]
            Second operation:
                robot 1 does [9-13-17]

        """
        print_toy_problem_specs(problem_description,vtx_grid,r0,s0,sF,project_spec)
    end
    return project_spec, problem_spec, robot_ICs, env_graph, assignment_dict
end

"""
    #### TOY PROBLEM 3 ####

    In this problem robot 2 will need to yield to let robot 1 through.
    First operation:
        robot 1 does [2-2-30]
    Second operation:
        robot 1 does [30-30-32]
        robot 2 does [5-7-8]
    Third operation:
        robot 2 does [8-12-16]
"""
function pctapf_problem_3(;cost_function=SumOfMakeSpans(),verbose=false,Δt_op=0,Δt_collect=[0,0,0,0],Δt_deliver=[0,0,0,0])
    N = 2                  # num robots
    M = 4                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(8,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    # 17  18  19  20
    # 21  22  23  24
    # 25  26  27  28
    # 29  30  31  32
    r0 = [2,5]
    s0 = [2,30,7,12]
    sF = [30,32,8,16]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )
    project_spec, robot_ICs = pctapf_problem(r0,s0,sF)

    add_operation!(project_spec,construct_operation(project_spec,-1,[1],[2],0.0))
    add_operation!(project_spec,construct_operation(project_spec,-1,[2,3],[4],0.0))
    add_operation!(project_spec,construct_operation(project_spec,-1,[4],  [], 0.0))
    assignment_dict = Dict(1=>[1,2],2=>[3,4])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function,Δt_collect=Δt_collect,Δt_deliver=Δt_deliver)

    if verbose
        problem_description = """

        #### TOY PROBLEM 3 ####

        In this problem robot 2 will need to yield to let robot 1 through.
        First operation:
            robot 1 does [2-2-30]
        Second operation:
            robot 1 does [30-30-32]
            robot 2 does [5-7-8]
        Third operation:
            robot 2 does [8-12-16]
        """
        print_toy_problem_specs(problem_description,vtx_grid,r0,s0,sF,project_spec)
    end
    return project_spec, problem_spec, robot_ICs, env_graph, assignment_dict
end


"""
    #### TOY PROBLEM 4 ####

    In this problem the cost of the task assignment problem is lower than the
    true cost (which requires that one of the robots is delayed by a single time
    step)
    First operation:
        robot 1 does [2-2-8]
        robot 2 does [4-4-6]
"""
function pctapf_problem_4(;cost_function=SumOfMakeSpans(),verbose=false)
    N = 2                  # num robots
    M = 2                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(3,3)
    # 1  2  3
    # 4  5  6
    # 7  8  9
    r0 = [2,4]
    s0 = [2,4]
    sF = [8,6]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )

    project_spec, robot_ICs = pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[],0))
    assignment_dict = Dict(1=>[1],2=>[2])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function)

    if verbose
        problem_description = """

        #### TOY PROBLEM 4 ####

        In this problem the cost of the task assignment problem is lower than the
        true cost (which requires that one of the robots is delayed by a single time
        step)
        First operation:
            robot 1 does [2-2-8]
            robot 2 does [4-4-6]

        As a result, the MILP solver will try switching assignment, see that the
        cost of the switch is higher than the cost of the path planning for the
        first assignment, and return with an optimality gap of -1.0.

        """
        print_toy_problem_specs(problem_description,vtx_grid,r0,s0,sF,project_spec)
    end
    return project_spec, problem_spec, robot_ICs, env_graph, assignment_dict
end


"""
    #### TOY PROBLEM 5 ####

    In this problem the robots try to pass through each other in such a way that
    an edge conflict is generated.

    First operation:
        robot 1 does [3-11]
        robot 2 does [15-7]
"""
function pctapf_problem_5(;cost_function=SumOfMakeSpans(),verbose=false)
    N = 2                  # num robots
    M = 2                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,4)
    # 1   2   3   4
    # 5   6   7   8
    # 9  10  11  12
    # 13  14  15  16
    r0 = [3,15]
    s0 = [3,15]
    sF = [11,7]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )

    project_spec, robot_ICs = pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[],0))
    assignment_dict = Dict(1=>[1],2=>[2])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function)

    if verbose
        problem_description =
        """

        #### TOY PROBLEM 5 ####

        In this problem the robots try to pass through each other in such a way that
        an edge conflict is generated.

        First operation:
            robot 1 does [3-11]
            robot 2 does [15-7]

        """
        print_toy_problem_specs(problem_description,vtx_grid,r0,s0,sF,project_spec)
    end
    return project_spec, problem_spec, robot_ICs, env_graph, assignment_dict
end

"""
    Identical to problem 2, but process time is non-zero.
    In this problem robot 1 will first do [1-5-9], then [9-13-17]
    robot 2 will do [4-8-32]. The key thing is that robot 1 will need to wait
    until robot 2 is finished before robot 1 can do its second task
"""
function pctapf_problem_6(;cost_function=SumOfMakeSpans(),verbose=false,Δt_op=1,Δt_collect=[0,0,0],Δt_deliver=[0,0,0])
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(8,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    # 17  18  19  20
    # 21  22  23  24
    # 25  26  27  28
    # 29  30  31  32
    r0 = [1,4]
    s0 = [5,12,13]
    sF = [9,32,17]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )

    project_spec, robot_ICs = pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], Δt_op))
    assignment_dict = Dict(1=>[1,3],2=>[2])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function,Δt_collect=Δt_collect,Δt_deliver=Δt_deliver)

    if verbose
        print_toy_problem_specs("TOY PROBLEM 6",vtx_grid,r0,s0,sF,project_spec)
    end
    return project_spec, problem_spec, robot_ICs, env_graph, assignment_dict
end


"""
    Robot 2 will have to sit and wait at the pickup station, meaning that robot 1 will have to go around
    if robot 2 is on the critical path
"""
function pctapf_problem_7(;cost_function=SumOfMakeSpans(),verbose=false,Δt_op=0,Δt_collect=[0,4,0],Δt_deliver=[0,0,0])
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    r0 = [2,9]
    s0 = [6,10,15]
    sF = [14,12,16]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )

    project_spec, robot_ICs = pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], Δt_op))
    assignment_dict = Dict(1=>[1,3],2=>[2])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function,Δt_collect=Δt_collect,Δt_deliver=Δt_deliver)

    if verbose
        print_toy_problem_specs("TOY PROBLEM 7",vtx_grid,r0,s0,sF,project_spec)
    end
    return project_spec, problem_spec, robot_ICs, env_graph, assignment_dict
end

"""
    two-headed project. Robot 1 does the first half of the first head, and
    robot 2 handles the first half of the second head, and then they swap.
    Optimal MakeSpan = 8
    Optimal SumOfMakeSpans = 16
"""
function pctapf_problem_8(;cost_function=SumOfMakeSpans(),verbose=false,Δt_op=0,Δt_collect=[0,0,0,0],Δt_deliver=[0,0,0,0])
    N = 2                  # num robots
    M = 4                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(8,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    # 17  18  19  20
    # 21  22  23  24
    # 25  26  27  28
    # 29  30  31  32
    r0 = [1,29]
    s0 = [5,25,12,24]
    sF = [8,28,9,21]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )

    project_spec, robot_ICs = pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1],[4],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[2],[3],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[4],[],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],[],Δt_op))
    assignment_dict = Dict(1=>[1,3],2=>[2,4])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function,Δt_collect=Δt_collect,Δt_deliver=Δt_deliver)


    if verbose
        print_toy_problem_specs("TOY PROBLEM 8",vtx_grid,r0,s0,sF,project_spec)
    end
    return project_spec, problem_spec, robot_ICs, env_graph, assignment_dict
end

export
    pctapf_problem_9
"""
    Project with station-sharing. Station 5 needs to accessed by both robots for picking up their objects.
"""
function pctapf_problem_9(;cost_function=SumOfMakeSpans(),verbose=false,Δt_op=0,Δt_collect=[0,0],Δt_deliver=[0,0])
    N = 2                  # num robots
    M = 2                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,4)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    r0 = [1,13]
    s0 = [5,5]
    sF = [8,6]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )

    project_spec, robot_ICs = pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1],[],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[2],[],Δt_op))
    assignment_dict = Dict(1=>[1],2=>[2])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function,Δt_collect=Δt_collect,Δt_deliver=Δt_deliver)

    if verbose
        print_toy_problem_specs("""
            TOY PROBLEM 9

            Project with station-sharing. Station 5 needs to accessed by both robots for picking up their objects.
            """,vtx_grid,r0,s0,sF,project_spec,problem_spec.graph)
    end
    return project_spec, problem_spec, robot_ICs, env_graph, assignment_dict
end


export
    pctapf_problem_10

"""
    Motivation for backtracking in ISPS

    The makespan optimal solution is T = 8. However, the optimistic schedule
    will always prioritize task route planning for tasks 1,2, and 3 before 4.
    This leads to a double delay that will not be caught without backtracking
    in ISPS. Hence, the solver will return a solution with T = 9.
"""
function pctapf_problem_10(;cost_function=MakeSpan(),verbose=false,Δt_op=0,Δt_collect=[0,0,0,0,0,0],Δt_deliver=[0,0,0,0,0,0])
    N = 4                  # num robots
    M = 4                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(13,11)

    #   1    2    3    4    5    6    7    8    9   10   11
    #  12   13   14   15   16   17   18   19   20   21   22
    #  23   24   25   26   27   28   29   30   31   32   33
    #  34   35   36   37   38   39   40   41   42   43   44
    #  45   46   47   48   49   50   51   52   53   54   55
    #  56   57   58   59   60   61   62   63   64   65   66
    #  67   68   69   70   71   72   73   74   75   76   77
    #  78   79   80   81   82   83   84   85   86   87   88
    #  89   90   91   92   93   94   95   96   97   98   99
    # 100  101  102  103  104  105  106  107  108  109  110
    # 111  112  113  114  115  116  117  118  119  120  121
    # 122  123  124  125  126  127  128  129  130  131  132
    # 133  134  135  136  137  138  139  140  141  142  143

    #            (2)
    #
    #
    #     (1)
    # (4)    [4]     [4] [3] [4]                         [3]
    #
    #
    #
    #            [2]
    #
    #     [1]

    r0 = [15,34,137, 5       ]
    s0 = [15,34,137, 5, 27,49 ]
    sF = [22,42,60,  27,49,71]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )

    project_spec, robot_ICs = pctapf_problem(r0,s0,sF)
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2,3,6],[],Δt_op))
    # add_operation!(project_spec,construct_operation(project_spec,-1,[2],[],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[4],[5],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[5],[6],Δt_op))
    # add_operation!(project_spec,construct_operation(project_spec,-1,[4],[],Δt_op))
    assignment_dict = Dict(1=>[1],2=>[2],3=>[3],4=>[4,5,6])

    def = SimpleProblemDef(project_spec,r0,s0,sF)
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function,Δt_collect=Δt_collect,Δt_deliver=Δt_deliver)

    if verbose
        print_toy_problem_specs("""
            TOY PROBLEM 10

            The makespan optimal solution is T = 8. However, the optimistic schedule
            will always prioritize task route planning for tasks 1,2, and 3 before 4.
            This leads to a double delay that will not be caught without backtracking
            in ISPS. Hence, the solver will return a solution with T = 9.
            """,vtx_grid,r0,s0,sF,project_spec,problem_spec.graph)
    end
    return project_spec, problem_spec, robot_ICs, env_graph, assignment_dict
end

export
    pctapf_problem_11

"""
    #### TOY PROBLEM 11 ####

    Requires collaborative transport: Robots 1 and 2 transport object 1 while
    robot 3 transports object 2. Robot 3 will need to move over to let the other
    robots pass.

"""
function pctapf_problem_11(;
        cost_function = SumOfMakeSpans(),
        verbose = false,
    )
    N = 3                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4, 4)
    # 1   2   3   4
    # 5   6   7   8
    # 9  10  11  12
    # 13  14  15  16
    r0 = [2, 4, 11]
    s0 = [2, 7, 13]
    sF = [14, 3, 9]
    env_graph = construct_factory_env_from_vtx_grid(
        vtx_grid;
    )
    shapes = [(1, 2), (1, 1), (1, 1)]

    project_spec, robot_ICs = pctapf_problem(r0, s0, sF)
    add_operation!(
        project_spec,
        construct_operation(project_spec, -1, [1, 2], [3], 0),
    )
    add_operation!(
        project_spec,
        construct_operation(project_spec, -1, [3], [], 0),
    )
    assignment_dict = Dict(1 => [1, 3], 2 => [1], 3 => [2])

    def = SimpleProblemDef(project_spec,r0,s0,sF,shapes)
    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
        def,env_graph;cost_function=cost_function)

    return project_spec, problem_spec, robot_ICs, env_graph, assignment_dict
end

for op in [
        :pctapf_problem_1,
        :pctapf_problem_2,
        :pctapf_problem_3,
        :pctapf_problem_4,
        :pctapf_problem_5,
        :pctapf_problem_6,
        :pctapf_problem_7,
        :pctapf_problem_8,
        :pctapf_problem_9,
        :pctapf_problem_10,
        :pctapf_problem_11,
    ]
    @eval $op(solver,args...;kwargs...) = pctapf_problem(solver,$op(args...;kwargs...)...)
end

export
    replanning_problem,
    replanning_problem_1,
    replanning_problem_2,
    replanning_problem_3,
    replanning_test_problems

"""
    replanning_problem

Constructs a replanning problem, consisting of robot initial conditions, an
environment, and a sequence of project requests scheduled to arrive in the
factory at regular intervals.
Args:
- r0: list of integers specifying the start locations of the robots
- defs: a list of tuples, where each tuple is of the form

    `([start_1=>goal_1, ...], [([inputs],[outputs]),...])`

    where the `start=>goal` pairs define the start and end points for each
    object to be delivered, and the `([inputs],[outputs])` pairs define the
    objects that go in and out of each operation.
- env_graph: the environment (presumably a GridFactoryEnvironment)
Outputs:
- requests: a sequence of `ProjectRequest`s
- problem_spec: a `ProblemSpec`
- robot_ICs: Robot initial conditions `ROBOT_AT`
- env_graph: the environment
"""
function replanning_problem(solver,r0,defs,env_graph;
        cost_function=SumOfMakeSpans(),
        spacing=8,
        t0=0,
        Δt_op=0,
        )
    requests = Vector{ProjectRequest}()
    problem_spec = nothing
    robot_ICs = nothing
    for (i,def) in enumerate(defs)
        s0 = map(i->i.first,def.tasks)
        sF = map(i->i.second,def.tasks)
        spec, _ = pctapf_problem(r0,s0,sF)
        for op in def.ops
            add_operation!(spec,construct_operation(spec,-1,op.inputs,op.outputs,Δt_op))
        end
        if i == 1
            problem_def = SimpleProblemDef(spec,r0,s0,sF)
            _, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
                problem_def,env_graph;
                cost_function=cost_function)
            # project_schedule = construct_partial_project_schedule(
            #     spec,
            #     problem_spec,
            #     robot_ICs,
            #     )
        # else
        #     project_schedule = construct_partial_project_schedule(
        #         spec,
        #         problem_spec,
        #         )
        end
        project_schedule = construct_partial_project_schedule(
            spec,
            problem_spec,
            )
        t = t0+spacing*(i-1)
        push!(requests,ProjectRequest(project_schedule,t,t))
    end

    base_schedule = construct_partial_project_schedule(
        ProjectSpec(),
        problem_spec,
        robot_ICs
        )
    # base_schedule = requests[1].schedule
    base_env = construct_search_env(solver,base_schedule,problem_spec,env_graph)
    return RepeatedPC_TAPF(base_env,requests)

    # return requests, problem_spec, robot_ICs, env_graph
    # env = construct_search_env(solver,OperatingSchedule(),problem_spec,env_graph)

end

function replanning_problem_1(solver;kwargs...)
    N = 2                  # num robots
    vtx_grid = initialize_dense_vtx_grid(4,4)
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    r0 = [1,13]
    defs = [
        ( tasks=[1=>4,8=>6],            ops=[ (inputs=[1],outputs=[]), (inputs=[2],outputs=[]) ] ),
        ( tasks=[13=>1,16=>15],         ops=[ (inputs=[1],outputs=[2]), (inputs=[2],outputs=[]) ] ),
        ( tasks=[9=>11,8=>11,7=>16],    ops=[ (inputs=[1,2],outputs=[3]), (inputs=[3],outputs=[]) ] ),
        ( tasks=[2=>14,3=>15,11=>4],    ops=[ (inputs=[1,2],outputs=[3]), (inputs=[3],outputs=[]) ] ),
        ]
    return replanning_problem(solver,r0,defs,env_graph;kwargs...)
end

function replanning_problem_2(solver;kwargs...)
    N = 2                  # num robots
    vtx_grid = initialize_dense_vtx_grid(4,4)
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    r0 = [1,13]
    defs = [
        ( tasks=[13=>1,16=>15],         ops=[ (inputs=[1],outputs=[2]), (inputs=[2],outputs=[]) ] ),
        ( tasks=[1=>4,8=>6],            ops=[ (inputs=[1],outputs=[]), (inputs=[2],outputs=[]) ] ),
        ( tasks=[2=>14,3=>15,11=>4],    ops=[ (inputs=[1,2],outputs=[3]), (inputs=[3],outputs=[]) ] ),
        ( tasks=[9=>11,8=>11,7=>16],    ops=[ (inputs=[1,2],outputs=[3]), (inputs=[3],outputs=[]) ] ),
        ]
    return replanning_problem(solver,r0,defs,env_graph;kwargs...)
end

function replanning_problem_3(solver;kwargs...)
    N = 1                  # num robots
    vtx_grid = initialize_dense_vtx_grid(4,4)
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    #  1   2   3   4
    #  5   6   7   8
    #  9  10  11  12
    # 13  14  15  16
    r0 = [1]
    defs = [
        ( tasks=[2=>4,8=>5,9=>12],
            ops=[
                (inputs=[1],outputs=[2]),
                (inputs=[2],outputs=[3]),
                (inputs=[3],outputs=[])
                ] ),
        ( tasks=[13=>16],
            ops=[
                (inputs=[1],outputs=[]),
                ] ),
        ]
    return replanning_problem(solver,r0,defs,env_graph;kwargs...)
end
replanning_test_problems() = [
    replanning_problem_1,
    replanning_problem_2,
    replanning_problem_3,
]

export
    pctapf_test_problems,
    collaborative_pctapf_test_problems

pctapf_test_problems() = [
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
collaborative_pctapf_test_problems() = [
    pctapf_problem_11,
]


# end
