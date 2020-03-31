module Helpers

using LightGraphs
using MetaGraphs
using GraphUtils
using CRCBS

# using ..PlanningPredicates
# using ..TaskGraphsCore
# using ..TaskGraphsUtils
using ..TaskGraphs

export
    initialize_toy_problem,
    initialize_toy_problem_1,
    initialize_toy_problem_2,
    initialize_toy_problem_3,
    initialize_toy_problem_4,
    initialize_toy_problem_5,
    initialize_toy_problem_6,
    initialize_toy_problem_7,
    initialize_toy_problem_8

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

function initialize_toy_problem(r0,s0,sF,dist_function)
    N = length(r0)
    M = length(s0)
    object_ICs = Vector{OBJECT_AT}([OBJECT_AT(o,s0[o]) for o in 1:M]) # initial_conditions
    object_FCs = Vector{OBJECT_AT}([OBJECT_AT(o,sF[o]) for o in 1:M]) # final conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    for r in N+1:N+M
        robot_ICs[r] = ROBOT_AT(r,sF[r-N][1])
    end
    # Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,dist_function)
    project_spec = ProjectSpec( M=M, initial_conditions=object_ICs, final_conditions=object_FCs)
    project_spec, robot_ICs
end

# This is a place to put reusable problem initializers for testing
function initialize_toy_problem_1(;cost_function=SumOfMakeSpans,verbose=false)
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,4)
    # 1  5   9  13
    # 2  6  10  14
    # 3  7  11  15
    # 4  8  12  16
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [1,4]
    s0 = [5,8,14]
    sF = [13,12,15]

    project_spec, robot_ICs = initialize_toy_problem(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],0))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], 0))
    assignment_dict = Dict(1=>[1,3],2=>[2])
    assignments = [1,2,1]
    # assignments = [1,2,3]

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(
        project_spec,r0,s0,sF,dist_matrix;cost_function=cost_function)
    if verbose
        problem_description = """
        #### TOY PROBLEM 1 ####
        r0 = [1,4]
        s0 = [5,8,14]
        sF = [13,12,15]
        assignment_dict = Dict(1=>[1,3],2=>[2])
        """
        print_toy_problem_specs(problem_description,vtx_grid,r0,s0,sF,project_spec)
    end
    return project_spec, problem_spec, robot_ICs, assignments, env_graph
end

"""
    In this problem robot 1 will first do [1-5-9], then [9-13-17]
    robot 2 will do [4-8-32]. The key thing is that robot 1 will need to wait
    until robot 2 is finished before robot 1 can do its second task
"""
function initialize_toy_problem_2(;cost_function=SumOfMakeSpans,verbose=false)
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,8)
    # 1  5   9  13  17  21  25  29
    # 2  6  10  14  18  22  26  30
    # 3  7  11  15  19  23  27  31
    # 4  8  12  16  20  24  28  32
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [1,4]
    s0 = [5,8,13]
    sF = [9,32,17]

    project_spec, robot_ICs = initialize_toy_problem(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],0))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], 0))
    # assignments = [1,2,3]
    assignment_dict = Dict(1=>[1,3],2=>[2])
    assignments = [1,2,1]

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(
        project_spec,r0,s0,sF,dist_matrix;cost_function=cost_function)
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
    return project_spec, problem_spec, robot_ICs, assignments, env_graph
end

"""
    #### TOY PROBLEM 3 ####

    In this problem robot 1 will need to yield to let robot 2 through.
    First operation:
        robot 1 does [5-7-8]
        robot 2 does [2-30-32]
    Second operation:
        robot 1 does [8-12-16]
"""
function initialize_toy_problem_3(;cost_function=SumOfMakeSpans,verbose=false,Δt_op=0,Δt_collect=[0,0,0],Δt_deliver=[0,0,0])
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,8)
    # 1  5   9  13  17  21  25  29
    # 2  6  10  14  18  22  26  30
    # 3  7  11  15  19  23  27  31
    # 4  8  12  16  20  24  28  32
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [5,2]
    s0 = [7,30,12]
    sF = [8,32,16]

    project_spec, robot_ICs = initialize_toy_problem(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],0.0))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], 0.0))
    # assignments = [1,2,3]
    assignment_dict = Dict(1=>[1,3],2=>[2])
    assignments = [1,2,1]

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(
        project_spec,r0,s0,sF,dist_matrix;Δt_collect=Δt_collect,Δt_deliver=Δt_deliver,cost_function=cost_function)
    if verbose
        problem_description = """

        #### TOY PROBLEM 3 ####

        In this problem robot 1 will need to yield to let robot 2 through.
        First operation:
            robot 1 does [5-7-8]
            robot 2 does [2-30-32]
        Second operation:
            robot 1 does [8-12-16]

        """
        print_toy_problem_specs(problem_description,vtx_grid,r0,s0,sF,project_spec)
    end
    return project_spec, problem_spec, robot_ICs, assignments, env_graph
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
function initialize_toy_problem_4(;cost_function=SumOfMakeSpans,verbose=false)
    N = 2                  # num robots
    M = 2                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(3,3)
    # 1  4  7
    # 2  5  8
    # 3  6  9
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [2,4]
    s0 = [2,4]
    sF = [8,6]

    project_spec, robot_ICs = initialize_toy_problem(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[],0))
    # assignments = [1,2]
    assignment_dict = Dict(1=>[1],2=>[2])
    assignments = [1,2]

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix;cost_function=cost_function)
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
    return project_spec, problem_spec, robot_ICs, assignments, env_graph
end


"""
    #### TOY PROBLEM 5 ####

    In this problem the robots try to pass through each other in such a way that
    an edge conflict is generated.

    First operation:
        robot 1 does [3-11]
        robot 2 does [15-7]
"""
function initialize_toy_problem_5(;cost_function=SumOfMakeSpans,verbose=false)
    N = 2                  # num robots
    M = 2                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,4)
    #  1  5   9  13
    #  2  6  10  14
    #  3  7  11  15
    #  4  8  12  16
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [3,15]
    s0 = [3,15]
    sF = [11,7]

    project_spec, robot_ICs = initialize_toy_problem(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[],0))
    # assignments = [1,2]
    assignment_dict = Dict(1=>[1],2=>[2])
    assignments = [1,2]

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix;cost_function=cost_function)
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
    return project_spec, problem_spec, robot_ICs, assignments, env_graph
end

"""
    Identical to problem 2, but process time is non-zero.
    In this problem robot 1 will first do [1-5-9], then [9-13-17]
    robot 2 will do [4-8-32]. The key thing is that robot 1 will need to wait
    until robot 2 is finished before robot 1 can do its second task
"""
function initialize_toy_problem_6(;cost_function=SumOfMakeSpans,verbose=false,Δt_op=1,Δt_collect=[0,0,0],Δt_deliver=[0,0,0])
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,8)
    # 1  5   9  13  17  21  25  29
    # 2  6  10  14  18  22  26  30
    # 3  7  11  15  19  23  27  31
    # 4  8  12  16  20  24  28  32
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [1,4]
    s0 = [5,12,13]
    sF = [9,32,17]

    project_spec, robot_ICs = initialize_toy_problem(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], Δt_op))
    # assignments = [1,2,3]
    assignment_dict = Dict(1=>[1,3],2=>[2])
    assignments = [1,2,1]

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix;Δt_collect=Δt_collect,Δt_deliver=Δt_deliver,cost_function=cost_function)
    if verbose
        print_toy_problem_specs("TOY PROBLEM 6",vtx_grid,r0,s0,sF,project_spec)
    end
    return project_spec, problem_spec, robot_ICs, assignments, env_graph
end


"""
    Robot 2 will have to sit and wait at the pickup station, meaning that robot 1 will have to go around
    if robot 2 is on the critical path
"""
function initialize_toy_problem_7(;cost_function=SumOfMakeSpans,verbose=false,Δt_op=0,Δt_collect=[0,4,0],Δt_deliver=[0,0,0])
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,4)
    # 1  5   9  13
    # 2  6  10  14
    # 3  7  11  15
    # 4  8  12  16
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [2,9]
    s0 = [6,10,15]
    sF = [14,12,16]

    project_spec, robot_ICs = initialize_toy_problem(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], Δt_op))
    # assignments = [1,2,3]
    assignment_dict = Dict(1=>[1,3],2=>[2])
    assignments = [1,2,1]

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix;Δt_collect=Δt_collect,Δt_deliver=Δt_deliver,cost_function=cost_function)
    if verbose
        print_toy_problem_specs("TOY PROBLEM 7",vtx_grid,r0,s0,sF,project_spec)
    end
    return project_spec, problem_spec, robot_ICs, assignments, env_graph
end

"""
    two-headed project. Robot 1 does the first half of the first head, and
    robot 2 handles the first half of the second head, and then they swap.
"""
function initialize_toy_problem_8(;cost_function=SumOfMakeSpans,verbose=false,Δt_op=0,Δt_collect=[0,0,0,0],Δt_deliver=[0,0,0,0])
    N = 2                  # num robots
    M = 4                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,8)
    # 1  5   9  13  17  21  25  29
    # 2  6  10  14  18  22  26  30
    # 3  7  11  15  19  23  27  31
    # 4  8  12  16  20  24  28  32
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [1,29]
    s0 = [5,25,12,24]
    sF = [8,28,9,21]

    project_spec, robot_ICs = initialize_toy_problem(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    add_operation!(project_spec,construct_operation(project_spec,-1,[1],[4],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[2],[3],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[4],[],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],[],Δt_op))
    # assignments = [1,2,3,4]
    assignment_dict = Dict(1=>[1,3],2=>[2,4])
    assignments = [1,2,1,2]

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix;Δt_collect=Δt_collect,Δt_deliver=Δt_deliver,cost_function=cost_function)

    if verbose
        print_toy_problem_specs("TOY PROBLEM 8",vtx_grid,r0,s0,sF,project_spec)
    end
    return project_spec, problem_spec, robot_ICs, assignments, env_graph
end

export
    initialize_toy_problem_9
"""
    Project with station-sharing. Station 5 needs to accessed by both robots for picking up their objects.
"""
function initialize_toy_problem_9(;cost_function=SumOfMakeSpans,verbose=false,Δt_op=0,Δt_collect=[0,0],Δt_deliver=[0,0])
    N = 2                  # num robots
    M = 2                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,4)
    # 1  5   9  13
    # 2  6  10  14
    # 3  7  11  15
    # 4  8  12  16
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [1,13]
    s0 = [5,5]
    sF = [8,6]

    project_spec, robot_ICs = initialize_toy_problem(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    add_operation!(project_spec,construct_operation(project_spec,-1,[1],[],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[2],[],Δt_op))
    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix;Δt_collect=Δt_collect,Δt_deliver=Δt_deliver,cost_function=cost_function)
    # assignments = [1,2]
    assignment_dict = Dict(1=>[1],2=>[2])
    assignments = [1,2]

    if verbose
        print_toy_problem_specs("""
            TOY PROBLEM 9

            Project with station-sharing. Station 5 needs to accessed by both robots for picking up their objects.
            """,vtx_grid,r0,s0,sF,project_spec,problem_spec.graph)
    end
    return project_spec, problem_spec, robot_ICs, assignments, env_graph
end


export
    initialize_toy_problem_10

"""
    Motivation for backtracking in ISPS

    The makespan optimal solution is T = 8. However, the optimistic schedule
    will always prioritize task route planning for tasks 1,2, and 3 before 4.
    This leads to a double delay that will not be caught without backtracking
    in ISPS. Hence, the solver will return a solution with T = 9.
"""
function initialize_toy_problem_10(;cost_function=MakeSpan,verbose=false,Δt_op=0,Δt_collect=[0,0,0,0,0,0],Δt_deliver=[0,0,0,0,0,0])
    N = 4                  # num robots
    M = 4                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(11,13)

    #  1  12  23  34  45  56  67  78  89  100  111  122  133
    #  2  13  24  35  46  57  68  79  90  101  112  123  134
    #  3  14  25  36  47  58  69  80  91  102  113  124  135
    #  4  15  26  37  48  59  70  81  92  103  114  125  136
    #  5  16  27  38  49  60  71  82  93  104  115  126  137
    #  6  17  28  39  50  61  72  83  94  105  116  127  138
    #  7  18  29  40  51  62  73  84  95  106  117  128  139
    #  8  19  30  41  52  63  74  85  96  107  118  129  140
    #  9  20  31  42  53  64  75  86  97  108  119  130  141
    # 10  21  32  43  54  65  76  87  98  109  120  131  142
    # 11  22  33  44  55  66  77  88  99  110  121  132  143

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

    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [15,34,137, 5       ]
    s0 = [15,34,137, 5, 27,49 ]
    sF = [22,42,60,  27,49,71]

    project_spec, robot_ICs = initialize_toy_problem(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2,3,6],[],Δt_op))
    # add_operation!(project_spec,construct_operation(project_spec,-1,[2],[],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[4],[5],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[5],[6],Δt_op))
    # add_operation!(project_spec,construct_operation(project_spec,-1,[4],[],Δt_op))
    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix;Δt_collect=Δt_collect,Δt_deliver=Δt_deliver,cost_function=cost_function)
    # assignments = [1,2]
    assignment_dict = Dict(1=>[1],2=>[2],3=>[3],4=>[4,5,6])
    assignments = [1,2,3,4,4,4]

    if verbose
        print_toy_problem_specs("""
            TOY PROBLEM 10

            The makespan optimal solution is T = 8. However, the optimistic schedule
            will always prioritize task route planning for tasks 1,2, and 3 before 4.
            This leads to a double delay that will not be caught without backtracking
            in ISPS. Hence, the solver will return a solution with T = 9.
            """,vtx_grid,r0,s0,sF,project_spec,problem_spec.graph)
    end
    return project_spec, problem_spec, robot_ICs, assignments, env_graph
end

end
