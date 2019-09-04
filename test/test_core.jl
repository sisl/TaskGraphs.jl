let
    id = ActionID(1)
    id += 1
end
let
    M = 3
    object_ICs = Dict{Int,OBJECT_AT}(
        1=>OBJECT_AT(1,1),
        2=>OBJECT_AT(2,2),
        3=>OBJECT_AT(3,3)
        )
    object_FCs = Dict{Int,OBJECT_AT}(
        1=>OBJECT_AT(1,4),
        2=>OBJECT_AT(2,5),
        3=>OBJECT_AT(3,6)
        )
    robot_ICs = Dict{Int,ROBOT_AT}(
        1=>ROBOT_AT(1,7),
        2=>ROBOT_AT(2,8),
        3=>ROBOT_AT(3,9)
        )
    project_spec = ProjectSpec(initial_conditions=object_ICs,final_conditions=object_FCs)
    add_operation!(project_spec,construct_operation(project_spec, 3, [1,2], [3], 1.0))
    add_operation!(project_spec,construct_operation(project_spec, 6, [3], [], 0.0))
    delivery_graph = construct_delivery_graph(project_spec,M)
    @show delivery_graph.tasks
    assignments = [1,2,3]
    project_schedule = construct_project_schedule(project_spec, object_ICs, object_FCs, robot_ICs, assignments)
    o_keys = Set(collect(keys(get_object_ICs(project_schedule))))
    input_ids = union([get_input_ids(op) for (k,op) in get_operations(project_schedule)]...)
    @test o_keys == input_ids
    output_ids = union([get_output_ids(op) for (k,op) in get_operations(project_schedule)]...)
    rg = get_display_metagraph(project_schedule)
end
# let
#     G = DiGraph(3)
#     add_edge!(G,1,2)
#     @test get_all_root_nodes(G) == Set([2,3])

#     N = 4                  # num robots
#     M = 6                  # num delivery tasks
#     r₀,s₀,sₜ,pts = initialize_random_2D_task_graph_env(N,M;d=[10,10])

#     # N = 40                  # num robots
#     # M = 60                  # num delivery tasks
#     # r₀,s₀,sₜ,pts = initialize_random_2D_task_graph_env(N,M;d=[100,100])

#     object_ICs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,s₀[o]) for o in 1:M) # initial_conditions
#     object_FCs = Dict{Int,OBJECT_AT}(o => OBJECT_AT(o,sₜ[o]) for o in 1:M) # final conditions
#     robot_ICs = Dict{Int,ROBOT_AT}(r => ROBOT_AT(r,r₀[r]) for r in 1:N)
#     Drs, Dss = cached_pickup_and_delivery_distances(pts[r₀],pts[s₀],pts[sₜ])
#     project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
#     project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
#     project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
#     project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
#     project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
#     project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
#     project_spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
#     object_ICs1 = Dict{Int,OBJECT_AT}(o => object_ICs[o] for o in 1:Int(M/2))
#     object_FCs1 = Dict{Int,OBJECT_AT}(o => object_FCs[o] for o in 1:Int(M/2))
#     project_spec1 = construct_random_project_spec(Int(M/2),object_ICs1,object_FCs1;max_parents=3,depth_bias=0.25,Δt_min=0,Δt_max=0)
#     object_ICs2 = Dict{Int,OBJECT_AT}(o => object_ICs[o] for o in Int(M/2):M)
#     object_FCs2 = Dict{Int,OBJECT_AT}(o => object_FCs[o] for o in Int(M/2):M)
#     project_spec2 = construct_random_project_spec(Int(M/2),object_ICs2,object_FCs2;max_parents=3,depth_bias=0.25,Δt_min=0,Δt_max=0)
#     project_spec = combine_project_specs([project_spec1, project_spec2])
    
#     filename = "project_spec.toml"
#     open(filename, "w") do io
#         TOML.print(io, TOML.parse(project_spec))
#     end
#     project_spec = read_project_spec(filename)
    
#     delivery_graph = construct_delivery_graph(project_spec,M)
#     G = delivery_graph.graph
#     # initialize vector of operation times
#     Δt = zeros(nv(G)) # Δt[j] = wait time for object j to appear once all inputs have been satisfied
#     for op in project_spec.operations
#         for id in get_output_ids(op)
#             Δt[id] = duration(op)
#         end
#     end
    
#     # set initial conditions
#     to0_ = Dict{Int,Float64}()
#     for v in vertices(G)
#         if is_leaf_node(G,v)
#             to0_[v] = 0.0
#         end
#     end
#     tr0_ = Dict{Int,Float64}()
#     for i in 1:N
#         tr0_[i] = 0.0
#     end
    
#     model = formulate_JuMP_optimization_problem(G,Drs,Dss,Δt,to0_,tr0_,Gurobi.Optimizer;OutputFlag=0);
#     spec = TaskGraphProblemSpec(N,M,G,Drs,Dss,Δt,tr0_,to0_)
    
#     optimize!(model)
#     optimal = (termination_status(model) == MathOptInterface.TerminationStatusCode(1))
#     @show optimal;
#     assignment = Matrix{Int}(value.(model[:x]));
    
#     spec = TaskGraphProblemSpec(N,M,G,Drs,Dss,Δt,tr0_,to0_)
#     cache = SearchCache(N,M,to0_,tr0_)
#     for j in 1:M
#         i = findfirst(assignment[:,j] .== 1)
#         cache.x[i,j] = 1
#     end
#     solution_graph = construct_solution_graph(delivery_graph.graph,assignment)
#     cache = process_solution(model,cache,spec);
    
#     assignments = map(j->findfirst(cache.x[:,j] .== 1),1:M)
    
#     for r in N+1:N+M
#         robot_ICs[r] = ROBOT_AT(r,sₜ[r-N])
#     end
#     project_schedule = construct_project_schedule(project_spec, object_ICs, object_FCs, robot_ICs, assignments);
    
#     o_keys = Set(collect(keys(get_object_ICs(project_schedule))))
#     input_ids = union([get_input_ids(op) for (k,op) in get_operations(project_schedule)]...)
#     @test o_keys == input_ids
    
# end
