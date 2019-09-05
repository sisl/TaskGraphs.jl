module Helpers

using LightGraphs
using MetaGraphs
using GraphUtils

using ..PlanningPredicates
using ..TaskGraphsCore
using ..TaskGraphsUtils

export
    initialize_toy_problem_1

# This is a place to put reusable problem initializers for testing
function initialize_toy_problem_1(;verbose=false)
    N = 2                  # num robots
    M = 3                  # num delivery tasks
    env_graph = initialize_grid_graph_from_vtx_grid(initialize_dense_vtx_grid(4,4))
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
    add_operation!(project_spec,construct_operation(project_spec,-1,[1,2],[3],0.0))
    add_operation!(project_spec,construct_operation(project_spec,-1,[3],  [], 0.0))


    delivery_graph = construct_delivery_graph(project_spec,M)
    G = delivery_graph.graph
    Δt = get_duration_vector(project_spec) # initialize vector of operation times
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
    problem_spec = TaskGraphProblemSpec(N,M,G,dist_matrix,Drs,Dss,Δt,tr0_,to0_)

    if verbose
        display(initialize_dense_vtx_grid(4,4))
        print("\n\n")
        @show r0
        @show s0
        @show sF
        display(project_spec.operations)
        print("\n\n")
        display(delivery_graph.tasks)
    end
    assignments = [1,2,3]
    return project_spec, problem_spec, robot_ICs, assignments
end

end
