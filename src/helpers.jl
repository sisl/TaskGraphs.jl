module Helpers

using LightGraphs
using MetaGraphs
using GraphUtils

using ..PlanningPredicates
using ..TaskGraphsCore
using ..TaskGraphsUtils

export
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
        if is_leaf_node(G,v)
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
#     display(delivery_graph.tasks)
#     print("\n\n")
end

# This is a place to put reusable problem initializers for testing
function initialize_toy_problem_1(;verbose=false)
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,4)
    env_graph = initialize_grid_graph_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [1,4]
    s0 = [5,8,14]
    sF = [13,12,15]

    object_ICs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,s0[o]) for o in 1:M) # initial_conditions
    object_FCs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,sF[o]) for o in 1:M) # final conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    for r in N+1:N+M
        robot_ICs[r] = ROBOT_AT(r,sF[r-N])
    end
    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    project_spec = ProjectSpec( M=M, initial_conditions=object_ICs, final_conditions=object_FCs )
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],0))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], 0))
    assignments = [1,2,3]

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix)
    if verbose
        problem_description = """
        #### TOY PROBLEM 1 ####
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
function initialize_toy_problem_2(;verbose=false)
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,8)
    # 1  5   9  13  17  21  25  29
    # 2  6  10  14  18  22  26  30
    # 3  7  11  15  19  23  27  31
    # 4  8  12  16  20  24  28  32
    env_graph = initialize_grid_graph_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [1,4]
    s0 = [5,8,13]
    sF = [9,32,17]

    object_ICs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,s0[o]) for o in 1:M) # initial_conditions
    object_FCs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,sF[o]) for o in 1:M) # final conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    for r in N+1:N+M
        robot_ICs[r] = ROBOT_AT(r,sF[r-N])
    end
    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    project_spec = ProjectSpec( M=M, initial_conditions=object_ICs, final_conditions=object_FCs )
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],0))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], 0))
    assignments = [1,2,3]

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix)
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
function initialize_toy_problem_3(;verbose=false)
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,8)
    # 1  5   9  13  17  21  25  29
    # 2  6  10  14  18  22  26  30
    # 3  7  11  15  19  23  27  31
    # 4  8  12  16  20  24  28  32
    env_graph = initialize_grid_graph_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [5,2]
    s0 = [7,30,12]
    sF = [8,32,16]

    object_ICs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,s0[o]) for o in 1:M) # initial_conditions
    object_FCs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,sF[o]) for o in 1:M) # final conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    for r in N+1:N+M
        robot_ICs[r] = ROBOT_AT(r,sF[r-N])
    end
    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    project_spec = ProjectSpec( M=M, initial_conditions=object_ICs, final_conditions=object_FCs )
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],0.0))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], 0.0))
    assignments = [1,2,3]

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix,Δt_collect,Δt_deliver)
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
function initialize_toy_problem_4(;verbose=false)
    N = 2                  # num robots
    M = 2                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(3,3)
    # 1  4  7
    # 2  5  8
    # 3  6  9
    env_graph = initialize_grid_graph_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [2,4]
    s0 = [2,4]
    sF = [8,6]

    object_ICs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,s0[o]) for o in 1:M) # initial_conditions
    object_FCs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,sF[o]) for o in 1:M) # final conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    for r in N+1:N+M
        robot_ICs[r] = ROBOT_AT(r,sF[r-N])
    end
    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    project_spec = ProjectSpec( M=M, initial_conditions=object_ICs, final_conditions=object_FCs )
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[],0))
    assignments = [1,2]

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix)
    if verbose
        problem_description = """

        #### TOY PROBLEM 4 ####

        In this problem the cost of the task assignment problem is lower than the
        true cost (which requires that one of the robots is delayed by a single time
        step)
        First operation:
            robot 1 does [2-2-8]
            robot 2 does [4-4-6]

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
function initialize_toy_problem_5(;verbose=false)
    N = 2                  # num robots
    M = 2                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,4)
    #  1  5   9  13
    #  2  6  10  14
    #  3  7  11  15
    #  4  8  12  16
    env_graph = initialize_grid_graph_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [3,15]
    s0 = [3,15]
    sF = [11,7]

    object_ICs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,s0[o]) for o in 1:M) # initial_conditions
    object_FCs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,sF[o]) for o in 1:M) # final conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    for r in N+1:N+M
        robot_ICs[r] = ROBOT_AT(r,sF[r-N])
    end
    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    project_spec = ProjectSpec( M=M, initial_conditions=object_ICs, final_conditions=object_FCs )
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[],0))
    assignments = [1,2]

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix)
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
function initialize_toy_problem_6(;verbose=false,Δt_op=1,Δt_collect=[0,0,0],Δt_deliver=[0,0,0])
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,8)
    # 1  5   9  13  17  21  25  29
    # 2  6  10  14  18  22  26  30
    # 3  7  11  15  19  23  27  31
    # 4  8  12  16  20  24  28  32
    env_graph = initialize_grid_graph_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [1,4]
    s0 = [5,8,13]
    sF = [9,32,17]

    object_ICs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,s0[o]) for o in 1:M) # initial_conditions
    object_FCs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,sF[o]) for o in 1:M) # final conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    for r in N+1:N+M
        robot_ICs[r] = ROBOT_AT(r,sF[r-N])
    end
    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    project_spec = ProjectSpec( M=M, initial_conditions=object_ICs, final_conditions=object_FCs )
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], Δt_op))
    assignments = [1,2,3]

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix,Δt_collect,Δt_deliver)
    if verbose
        print_toy_problem_specs("TOY PROBLEM 6",vtx_grid,r0,s0,sF,project_spec)
    end
    return project_spec, problem_spec, robot_ICs, assignments, env_graph
end


"""
    Robot 2 will have to sit and wait at the pickup station, meaning that robot 1 will have to go around
    if robot 2 is on the critical path
"""
function initialize_toy_problem_7(;verbose=false,Δt_op=0,Δt_collect=[0,4,0],Δt_deliver=[0,0,0])
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,4)
    # 1  5   9  13
    # 2  6  10  14
    # 3  7  11  15
    # 4  8  12  16
    env_graph = initialize_grid_graph_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [2,9]
    s0 = [6,10,15]
    sF = [14,12,16]

    object_ICs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,s0[o]) for o in 1:M) # initial_conditions
    object_FCs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,sF[o]) for o in 1:M) # final conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    for r in N+1:N+M
        robot_ICs[r] = ROBOT_AT(r,sF[r-N])
    end
    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    project_spec = ProjectSpec( M=M, initial_conditions=object_ICs, final_conditions=object_FCs )
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], Δt_op))
    assignments = [1,2,3]

    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix,Δt_collect,Δt_deliver)
    if verbose
        print_toy_problem_specs("TOY PROBLEM 7",vtx_grid,r0,s0,sF,project_spec)
    end
    return project_spec, problem_spec, robot_ICs, assignments, env_graph
end

"""
    two-headed project. Robot 1 does the first half of the first head, and
    robot 2 handles the first half of the second head, and then they swap.
"""
function initialize_toy_problem_8(;verbose=false,Δt_op=0,Δt_collect=[0,0,0,0],Δt_deliver=[0,0,0,0])
    N = 2                  # num robots
    M = 4                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,8)
    # 1  5   9  13  17  21  25  29
    # 2  6  10  14  18  22  26  30
    # 3  7  11  15  19  23  27  31
    # 4  8  12  16  20  24  28  32
    env_graph = initialize_grid_graph_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [1,29]
    s0 = [5,25,12,24]
    sF = [8,28,9,21]

    object_ICs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,s0[o]) for o in 1:M) # initial_conditions
    object_FCs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,sF[o]) for o in 1:M) # final conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    for r in N+1:N+M
        robot_ICs[r] = ROBOT_AT(r,sF[r-N])
    end
    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    project_spec = ProjectSpec( M=M, initial_conditions=object_ICs, final_conditions=object_FCs )
    add_operation!(project_spec,construct_operation(project_spec,-1,[1],[4],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[2],[3],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[4],[],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],[],Δt_op))
    assignments = [1,2,3,4]
    
    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix,Δt_collect,Δt_deliver)

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
function initialize_toy_problem_9(;verbose=false,Δt_op=0,Δt_collect=[0,0],Δt_deliver=[0,0])
    N = 2                  # num robots
    M = 2                  # num delivery tasks
    vtx_grid = initialize_dense_vtx_grid(4,4)
    # 1  5   9  13
    # 2  6  10  14
    # 3  7  11  15
    # 4  8  12  16
    env_graph = initialize_grid_graph_from_vtx_grid(vtx_grid)
    dist_matrix = get_dist_matrix(env_graph)
    r0 = [1,13]
    s0 = [5,5]
    sF = [8,6]

    object_ICs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,s0[o]) for o in 1:M) # initial_conditions
    object_FCs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,sF[o]) for o in 1:M) # final conditions
    robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r0[r]) for r in 1:N)
    for r in N+1:N+M
        robot_ICs[r] = ROBOT_AT(r,sF[r-N])
    end
    Drs, Dss = cached_pickup_and_delivery_distances(r0,s0,sF,(v1,v2)->dist_matrix[v1,v2])
    project_spec = ProjectSpec( M=M, initial_conditions=object_ICs, final_conditions=object_FCs )
    add_operation!(project_spec,construct_operation(project_spec,-1,[1],[],Δt_op))
    add_operation!(project_spec,construct_operation(project_spec,-1,[2],[],Δt_op))
    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec,r0,s0,sF,dist_matrix,Δt_collect,Δt_deliver)
    assignments = [1,2]

    if verbose
        print_toy_problem_specs("""
            TOY PROBLEM 9
            
            Project with station-sharing. Station 5 needs to accessed by both robots for picking up their objects.
            """,vtx_grid,r0,s0,sF,project_spec,problem_spec.graph)
    end
    return project_spec, problem_spec, robot_ICs, assignments, env_graph
end

end
