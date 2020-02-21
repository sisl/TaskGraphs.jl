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
    # display(delivery_graph.tasks)
    # print("\n\n")
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
    get_object_paths

function get_object_paths(solution,schedule,cache)
    robot_paths = convert_to_vertex_lists(solution)
    tF = maximum(map(length, robot_paths))
    object_paths = Vector{Vector{Int}}()
    object_intervals = Vector{Vector{Int}}()
    object_ids = Int[]
    path_idxs = Int[]
    for v in vertices(schedule.graph)
        node = get_node_from_id(schedule,get_vtx_id(schedule,v))
        if isa(node, Union{CARRY,TEAM_ACTION{CARRY}})
            if isa(node, CARRY)
                object_id = get_object_id(node)
                agent_id_list = [get_id(get_robot_id(node))]
                s0_list = [get_id(get_initial_location_id(node))]
                sF_list = [get_id(get_destination_location_id(node))]
            else
                object_id = get_object_id(node.instructions[1])
                agent_id_list = map(n->get_id(get_robot_id(n)), node.instructions)
                s0_list = map(n->get_id(get_initial_location_id(n)), node.instructions)
                sF_list = map(n->get_id(get_destination_location_id(n)), node.instructions)
            end
            object_vtx = get_vtx(schedule,object_id)
            for (idx,(agent_id,s0,sF)) in enumerate(zip(agent_id_list,s0_list,sF_list))
                if get_id(agent_id) != -1
                    push!(object_paths,[
                        map(t->s0,0:cache.t0[v]-1)...,
                        map(t->robot_paths[agent_id][t],min(cache.t0[v]+1,tF):min(cache.tF[v]+1,tF,length(robot_paths[agent_id])))...,
                        map(t->sF,min(cache.tF[v]+1,tF):tF)...
                    ])
                    push!(object_intervals,[cache.t0[object_vtx],cache.tF[v]+1])
                    push!(object_ids, get_id(object_id))
                    push!(path_idxs, idx)
                end
            end
        end
    end
    object_paths, object_intervals, object_ids, path_idxs
end
function get_object_paths(solution,env)
    get_object_paths(solution,env.schedule,env.cache)
end

export
    fill_object_path_dicts!,
    convert_to_path_vectors

function fill_object_path_dicts!(solution,project_schedule,cache;
        object_path_dict = Dict{Int,Vector{Vector{Int}}}(),
        object_interval_dict = Dict{Int,Vector{Int}}()
    )
    object_paths, object_intervals, object_ids, path_idxs = get_object_paths(solution,project_schedule,cache)
    for (path,interval,id,idx) in zip(object_paths, object_intervals, object_ids, path_idxs)
        if !haskey(object_path_dict,id)
            object_path_dict[id] = Vector{Vector{Int}}()
        end
        if length(object_path_dict[id]) >= idx
            object_path_dict[id][idx] = path
        else
            push!(object_path_dict[id], path)
        end
        object_interval_dict[id] = interval
    end
    object_path_dict, object_interval_dict
end

function convert_to_path_vectors(object_path_dict, object_interval_dict)
    object_paths = Vector{Vector{Int}}()
    object_intervals = Vector{Vector{Int}}()
    for (id,paths) in object_path_dict
        for path in paths
            push!(object_intervals, object_interval_dict[id])
            push!(object_paths, path)
        end
    end
    object_paths, object_intervals
end

end
