# using Pkg
# Pkg.activate("/home/kylebrown/.julia/dev/TaskGraphs")
using Test
using LightGraphs, MetaGraphs, GraphUtils
using TaskGraphs
using Gurobi
using JuMP, MathOptInterface
using TOML
using Random
using Compose
using GraphPlottingBFS

# function formulate_generic_optimization_problem(
#     project_schedule::P,
#     problem_spec::S,
#     ;
#     cost_model=:MakeSpan,
#     optimizer=Gurobi.Optimizer,
#     TimeLimit=100,
#     OutputFlag=0
#     ) where {P<:ProjectSchedule,S<:TaskGraphProblemSpec}

#     # Formulate MILP problem
#     G = get_graph(project_schedule);
#     t0_ = Dict{AbstractID,Float64}();
#     # nR = ones(M); # number of robots required for each task
#     assignments = [];
#     Δt = map(v->get_path_spec(project_schedule, v).min_path_duration, vertices(G))

#     optimizer = Gurobi.Optimizer;
#     TimeLimit=100;
#     OutputFlag=0;
#     model = Model(with_optimizer(optimizer,
#         TimeLimit=TimeLimit,
#         OutputFlag=OutputFlag
#         ));

#     @variable(model, t0[1:nv(G)] >= 0.0); # initial times for all nodes
#     @variable(model, tF[1:nv(G)] >= 0.0); # final times for all nodes
#     @variable(model, X[1:nv(G),1:nv(G)], binary = true); # Adjacency Matrix
#     @constraint(model, X .+ X' .<= 1) # no bidirectional edges (also guarantees that diagonal is zero)
#     for (id,t) in t0_
#         v = get_vtx(project_schedule, id)
#         @constraint(model, t0[v] == t)
#     end
#     # other constraints
#     Mm = 10000 # for big-M constraints
#     for v in vertices(G)
#         # @constraint(model, X[v,v] == 0) # no self edges (alreadt taken care of above)
#         @constraint(model, tF[v] >= t0[v] + Δt[v])
#         for v2 in outneighbors(G,v)
#             @constraint(model, X[v,v2] == 1)
#             @constraint(model, t0[v2] >= tF[v])
#         end
#     end
#     # what edges to add?
#     missing_successors      = Dict{Int,Dict}()
#     missing_predecessors    = Dict{Int,Dict}()
#     n_eligible_successors   = zeros(Int,nv(G))
#     n_eligible_predecessors = zeros(Int,nv(G))
#     n_required_successors   = zeros(Int,nv(G))
#     n_required_predecessors = zeros(Int,nv(G))
#     for v in vertices(G)
#         node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
#         for (key,val) in required_successors(node)
#             n_required_successors[v] += val
#         end
#         for (key,val) in required_predecessors(node)
#             n_required_predecessors[v] += val
#         end
#         for (key,val) in eligible_successors(node)
#             n_eligible_successors[v] += val
#         end
#         for (key,val) in eligible_predecessors(node)
#             n_eligible_predecessors[v] += val
#         end
#         missing_successors[v] = eligible_successors(node)
#         for v2 in outneighbors(G,v)
#             id2 = get_vtx_id(project_schedule, v2)
#             node2 = get_node_from_id(project_schedule, id2)
#             for key in collect(keys(missing_successors[v]))
#                 if matches_template(key,typeof(node2))
#                     missing_successors[v][key] -= 1
#                     break
#                 end
#             end
#         end
#         missing_predecessors[v] = eligible_predecessors(node)
#         for v2 in inneighbors(G,v)
#             id2 = get_vtx_id(project_schedule, v2)
#             node2 = get_node_from_id(project_schedule, id2)
#             for key in collect(keys(missing_predecessors[v]))
#                 if matches_template(key,typeof(node2))
#                     missing_predecessors[v][key] -= 1
#                     break
#                 end
#             end
#         end
#     end
#     @assert(!any(n_eligible_predecessors .< n_required_predecessors))
#     @assert(!any(n_eligible_successors .< n_required_successors))
#     nodes, edge_list, n_eligible_predecessors, n_required_predecessors, n_eligible_successors, n_required_successors

#     # @constraint(model, X * ones(nv(G)) .<= n_eligible_successors);
#     @constraint(model, X * ones(nv(G)) .>= n_required_successors);
#     # @constraint(model, X' * ones(nv(G)) .<= n_eligible_predecessors);
#     @constraint(model, X' * ones(nv(G)) .>= n_required_predecessors);
#     nodes, edge_list, missing_predecessors, missing_successors

#     for v in vertices(G)
#         upstream_vertices = [v, map(e->e.dst,collect(edges(bfs_tree(G,v;dir=:in))))...]
#         for v2 in upstream_vertices
#             @constraint(model, X[v,v2] == 0)
#         end
#         node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
#         for (template, val) in missing_successors[v]
#             for v2 in vertices(G)
#                 node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
#                 if (v2 in upstream_vertices) || !matches_template(template, typeof(node2))
#                     # @constraint(model, X[v,v2] == 0)
#                     continue
#                 end
#                 potential_match = false
#                 for (template2, val2) in missing_predecessors[v2]
#                     if matches_template(template2,typeof(node)) # possible to add and edge
#                         potential_match = true
#                         if !(val > 0 && val2 > 0)
#                             continue
#                         end
#                         @show new_node = align_with_predecessor(node2,node)
#                         @show dt_min = generate_path_spec(project_schedule,problem_spec,new_node).min_path_duration
#                         @constraint(model, tF[v2] - (t0[v2] + dt_min) >= -Mm*(1 - X[v,v2]))
#                     end
#                 end
#                 if potential_match == false
#                     @constraint(model, X[v,v2] == 0)
#                 end
#             end
#         end
#     end

#     # "Job-shop" constraints specifying that no station may be double-booked. A station
#     # can only support a single COLLECT or DEPOSIT operation at a time, meaning that all
#     # the windows for these operations cannot overlap. In the constraints below, t1 and t2
#     # represent the intervals for the COLLECT or DEPOSIT operations of tasks j and j2,
#     # respectively. If eny of the operations for these two tasks require use of the same
#     # station, we introduce a 2D binary variable y. if y = [1,0], the operation for task
#     # j must occur before the operation for task j2. The opposite is true for y == [0,1].
#     # We use the big M method here as well to tightly enforce the binary constraints.
#     job_shop_variables = Dict{Tuple{Int,Int},JuMP.VariableRef}();
#     for v in 1:nv(G)
#         node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
#         for v2 in v+1:nv(G)
#             node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
#             common_resources = intersect(resources_reserved(node),resources_reserved(node2))
#             if length(common_resources) > 0
#                 @show common_resources
#                 tmax = @variable(model)
#                 tmin = @variable(model)
#                 y = @variable(model, binary=true)
#                 job_shop_variables[(v,v2)] = y
#                 @constraint(model, tmax >= t0[v])
#                 @constraint(model, tmax >= t0[v2])
#                 @constraint(model, tmin <= tF[v])
#                 @constraint(model, tmin <= tF[v2])

#                 @constraint(model, tmax - t0[v2] <= (1 - y)*Mm)
#                 @constraint(model, tmax - t0[v] <= y*Mm)
#                 @constraint(model, tmin - tF[v] >= (1 - y)*-Mm)
#                 @constraint(model, tmin - tF[v2] >= y*-Mm)
#                 @constraint(model, tmin + 1 <= tmax)
#             end
#         end
#     end

#     # Formulate Objective
#     # cost_model = :MakeSpan
#     cost_model = :SumOfMakeSpans
#     if cost_model == :SumOfMakeSpans
#         root_nodes = project_schedule.root_nodes
#         @variable(model, T[1:length(root_nodes)])
#         for (i,project_head) in enumerate(root_nodes)
#             for v in project_head
#                 @show v
#                 @constraint(model, T[i] >= tF[v])
#             end
#         end
#         cost1 = @expression(model, sum(map(v->tF[v]*get(project_schedule.weights,v,0.0), root_nodes)))
#     elseif cost_model == :MakeSpan
#         @variable(model, T)
#         @constraint(model, T .>= tF)
#         cost1 = @expression(model, T)
#     end
#     cost2 = @expression(model, (0.5/(nv(G)^2))*sum(X)) # cost term to encourage sparse X
#     @objective(model, Min, cost1 + cost2)
#     model, job_shop_variables
# end

# function init_sparse_random_project_schedule(
#     N,
#     M,
#     ;
#     dx=8,
#     dy=8,
#     max_parents = 3,
#     seed = 0
#     )
#     Random.seed!(seed);

#     # Define Environment
#     vtx_grid = initialize_dense_vtx_grid(dx,dy);
#     env_graph = initialize_grid_graph_from_vtx_grid(vtx_grid);
#     dist_matrix = get_dist_matrix(env_graph);

#     # Define project
#     object_ICs = [OBJECT_AT(j,j) for j in 1:M];
#     object_FCs = [OBJECT_AT(j,j+M) for j in 1:M];
#     robot_ICs = [ROBOT_AT(i,i) for i in 1:N];
#     spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=max_parents);
#     operations = spec.operations;
#     root_ops = map(op->op.id, spec.operations[collect(spec.root_nodes)])
#     problem_spec = TaskGraphProblemSpec(N=N,M=M,D=dist_matrix);

#     # Construct Partial Project Schedule
#     project_schedule = ProjectSchedule();
#     for pred in object_ICs
#        add_to_schedule!(project_schedule, problem_spec, pred, get_object_id(pred))
#     end
#     for pred in robot_ICs
#        add_to_schedule!(project_schedule, problem_spec, pred, get_robot_id(pred))
#     end
#     for op in operations
#        operation_id = op.id
#        add_to_schedule!(project_schedule, problem_spec, op, operation_id)
#     end
#     # add root nodes
#     for operation_id in root_ops
#         v = get_vtx(project_schedule, operation_id)
#         push!(project_schedule.root_nodes, v)
#         project_schedule.weights[v] = 1.0
#     end
#     # Fill in gaps in project schedule (except for GO assignments)
#     for op in operations
#         operation_id = op.id
#         for object_id in get_input_ids(op)
#             # add action sequence
#             object_ic = get_object_ICs(project_schedule)[object_id]
#             pickup_station_id = get_id(get_location_id(object_ic))
#             object_fc = object_FCs[object_id]
#             dropoff_station_id = get_id(get_location_id(object_fc))
#             # TODO Handle collaborative tasks
#             # if is_single_robot_task(project_spec, object_id)
#             robot_id = -1
#             add_single_robot_delivery_task!(project_schedule,problem_spec,robot_id,
#                 object_id,pickup_station_id,dropoff_station_id)
#             # elseif is_collaborative_robot_task(project_spec, object_id)
#             # end
#             action_id = ActionID(get_num_actions(project_schedule))
#             add_edge!(project_schedule, action_id, operation_id)
#         end
#         for object_id in get_output_ids(op)
#             add_edge!(project_schedule, operation_id, ObjectID(object_id))
#         end
#     end
#     project_schedule, problem_spec
# end

# function test_generic_formulation(
#         N,
#         M,
#         ;
#         kwargs...
#         )
#         N = 2; M = 4;
#         project_schedule, problem_spec = init_sparse_random_project_schedule(N,M)

#         edge_list = collect(edges(project_schedule.graph))
#         nodes = map(id->get_node_from_id(project_schedule, id), project_schedule.vtx_ids)
#         nodes, edge_list

#         model, job_shop_variables = formulate_generic_optimization_problem(project_schedule,problem_spec;kwargs...)

#         # Optimize!
#         optimize!(model)
#         status = termination_status(model)
#         obj_val = Int(round(value(objective_function(model))))
#         adj_matrix = Int.(round.(value.(model[:X])))

#         rg = get_display_metagraph(project_schedule;
#             f=(v,p)->string(v,",",get_path_spec(project_schedule,v).path_id,",",get_path_spec(project_schedule,v).agent_id))
#         plot_graph_bfs(rg;
#             shape_function = (G,v,x,y,r)->Compose.circle(x,y,r),
#             color_function = (G,v,x,y,r)->get_prop(G,v,:color),
#             text_function = (G,v,x,y,r)->title_string(get_node_from_id(project_schedule, get_vtx_id(project_schedule, v)),false)
#         ) |> Compose.SVG("project_schedule1.svg")
#         # inkscape -z project_schedule1.svg -e project_schedule1.png

#         # Update Project Graph
#         G = get_graph(project_schedule)
#         @show is_cyclic(G)
#         for v in vertices(G)
#             for v2 in vertices(G)
#                 if adj_matrix[v,v2] == 1
#                     add_edge!(G,v,v2)
#                     if is_cyclic(G)
#                         node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
#                         node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
#                         @show v, v2, typeof(node), typeof(node2), is_cyclic(G)
#                         rem_edge!(G,v,v2)
#                     end
#                 end
#             end
#         end

#         rg = get_display_metagraph(project_schedule;
#             f=(v,p)->string(v,",",get_path_spec(project_schedule,v).path_id,",",get_path_spec(project_schedule,v).agent_id))
#         plot_graph_bfs(rg;
#             shape_function = (G,v,x,y,r)->Compose.circle(x,y,r),
#             color_function = (G,v,x,y,r)->get_prop(G,v,:color),
#             text_function = (G,v,x,y,r)->title_string(get_node_from_id(project_schedule, get_vtx_id(project_schedule, v)),false)
#         ) |> Compose.SVG("project_schedule2.svg")

#         # for v in topological_sort(G)
#         #     node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
#         #     for v2 in outneighbors(G,v)
#         #         id2 = get_vtx_id(project_schedule, v2)
#         #         node2 = get_node_from_id(project_schedule, id2)
#         #         @show new_node = align_with_predecessor(node2,node)
#         #         replace_in_schedule!(project_schedule, spec, new_node, id2)
#         #         # @show path_spec = generate_path_spec(project_schedule,problem_spec,new_node)
#         #     end
#         # end
#         edge_list = collect(edges(G))
#         model, status, obj_val, adj_matrix, nodes, edge_list, project_schedule
# end

# let
#     N = 2; M = 4;
#     model, status, obj_val, adj_matrix, nodes, edge_list, project_schedule = test_generic_formulation(N,M)
# end

function print_project_schedule(project_schedule,filename="project_schedule1")
    rg = get_display_metagraph(project_schedule;
        f=(v,p)->string(v,",",get_path_spec(project_schedule,v).path_id,",",get_path_spec(project_schedule,v).agent_id))
    plot_graph_bfs(rg;
        mode=:root_aligned,
        shape_function = (G,v,x,y,r)->Compose.circle(x,y,r),
        color_function = (G,v,x,y,r)->get_prop(G,v,:color),
        text_function = (G,v,x,y,r)->title_string(get_node_from_id(project_schedule, get_vtx_id(project_schedule, v)),true)
    ) |> Compose.SVG(string(filename,".svg"))
    # `inkscape -z project_schedule1.svg -e project_schedule1.png`
    # OR: `for f in *.svg; do inkscape -z $f -e $f.png; done`
end

let

    Random.seed!(0);
    # Define Environment
    vtx_grid = initialize_dense_vtx_grid(8,8);
    env_graph = initialize_grid_graph_from_vtx_grid(vtx_grid);
    dist_matrix = get_dist_matrix(env_graph);
    # Define project
    N = 1; M = 2;
    object_ICs = [OBJECT_AT(j,j) for j in 1:M];
    object_FCs = [OBJECT_AT(j,j+M) for j in 1:M];
    robot_ICs = [ROBOT_AT(i,i) for i in 1:N];
    spec = construct_random_project_spec(M,object_ICs,object_FCs;max_parents=3);
    operations = spec.operations;
    root_ops = map(op->op.id, spec.operations[collect(spec.root_nodes)])
    problem_spec = TaskGraphProblemSpec(N=N,M=M,D=dist_matrix);

    # Construct Partial Project Schedule
    project_schedule = construct_partial_project_schedule(spec,problem_spec,robot_ICs)

    # edge_list = collect(edges(project_schedule.graph))
    # nodes = map(id->get_node_from_id(project_schedule, id), project_schedule.vtx_ids)
    # nodes, edge_list

    # Formulate MILP problem
    model, job_shop_variables = formulate_schedule_milp(project_schedule,problem_spec)

    # Optimize!
    optimize!(model)
    @show status = termination_status(model)
    obj_val = Int(round(value(objective_function(model))))
    @show adj_matrix = Int.(round.(value.(model[:X])))

    print_project_schedule(project_schedule,"project_schedule1")

    # Update Project Graph by adding all edges encoded by the optimized adjacency graph
    update_project_schedule!(project_schedule,problem_spec,adj_matrix)

    print_project_schedule(project_schedule,"project_schedule2")

    job_shop_variables

end
let

    filename = "/home/kylebrown/.julia/dev/WebotsSim/experiments/problem_instances/problem2.toml"
    env_file = "/home/kylebrown/.julia/dev/TaskGraphs/test/env_graph.lg"
    problem_def = read_problem_def(filename)
    env_graph = loadgraph(env_file)
    dist_matrix = get_dist_matrix(env_graph)
    project_spec, r0, s0, sF = problem_def.project_spec,problem_def.r0,problem_def.s0,problem_def.sF
    project_spec, problem_spec, object_ICs, object_FCs, _ = construct_task_graphs_problem(project_spec, r0, s0, sF, dist_matrix)
    robot_ICs = [ROBOT_AT(i,x) for (i,x) in enumerate(r0)] # remove dummy robots
    project_schedule = construct_partial_project_schedule(project_spec,problem_spec,robot_ICs)
    model, job_shop_variables = formulate_schedule_milp(project_schedule,problem_spec)
    optimize!(model)
    @show status = termination_status(model)
    @show obj_val = Int(round(value(objective_function(model))))
    adj_matrix = Int.(round.(value.(model[:X])))

    print_project_schedule(project_schedule,"project_schedule1")
    update_project_schedule!(project_schedule,problem_spec,adj_matrix)
    print_project_schedule(project_schedule,"project_schedule2")

end

function TaskGraphs.formulate_schedule_milp(project_schedule::ProjectSchedule,problem_spec::TaskGraphProblemSpec;
        optimizer = Gurobi.Optimizer,
        TimeLimit=100,
        OutputFlag=0,
        t0_ = Dict{AbstractID,Float64}(), # dictionary of initial times. Default is empty
        Mm = 10000, # for big M constraints
        cost_model = :SumOfMakeSpans,
    )
    G = get_graph(project_schedule);
    assignments = [];
    Δt = map(v->get_path_spec(project_schedule, v).min_path_duration, vertices(G))

    model = Model(with_optimizer(optimizer,
        TimeLimit=TimeLimit,
        OutputFlag=OutputFlag
        ));
    @variable(model, t0[1:nv(G)] >= 0.0); # initial times for all nodes
    @variable(model, tF[1:nv(G)] >= 0.0); # final times for all nodes
    @variable(model, X[1:nv(G),1:nv(G)], binary = true); # Adjacency Matrix
    @constraint(model, X .+ X' .<= 1) # no bidirectional or self edges
    # set all initial times that are provided
    for (id,t) in t0_
        v = get_vtx(project_schedule, id)
        @constraint(model, t0[v] == t)
    end
    # Precedence constraints and duration constraints for existing nodes and edges
    for v in vertices(G)
        @constraint(model, tF[v] >= t0[v] + Δt[v])
        for v2 in outneighbors(G,v)
            @constraint(model, X[v,v2] == 1)
            @constraint(model, t0[v2] >= tF[v])
        end
    end
    # Identify required and eligible edges
    missing_successors      = Dict{Int,Dict}()
    missing_predecessors    = Dict{Int,Dict}()
    n_eligible_successors   = zeros(Int,nv(G))
    n_eligible_predecessors = zeros(Int,nv(G))
    n_required_successors   = zeros(Int,nv(G))
    n_required_predecessors = zeros(Int,nv(G))
    for v in vertices(G)
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        for (key,val) in required_successors(node)
            n_required_successors[v] += val
        end
        for (key,val) in required_predecessors(node)
            n_required_predecessors[v] += val
        end
        for (key,val) in eligible_successors(node)
            n_eligible_successors[v] += val
        end
        for (key,val) in eligible_predecessors(node)
            n_eligible_predecessors[v] += val
        end
        missing_successors[v] = eligible_successors(node)
        for v2 in outneighbors(G,v)
            id2 = get_vtx_id(project_schedule, v2)
            node2 = get_node_from_id(project_schedule, id2)
            for key in collect(keys(missing_successors[v]))
                if matches_template(key,typeof(node2))
                    missing_successors[v][key] -= 1
                    break
                end
            end
        end
        missing_predecessors[v] = eligible_predecessors(node)
        for v2 in inneighbors(G,v)
            id2 = get_vtx_id(project_schedule, v2)
            node2 = get_node_from_id(project_schedule, id2)
            for key in collect(keys(missing_predecessors[v]))
                if matches_template(key,typeof(node2))
                    missing_predecessors[v][key] -= 1
                    break
                end
            end
        end
    end
    @assert(!any(n_eligible_predecessors .< n_required_predecessors))
    @assert(!any(n_eligible_successors .< n_required_successors))
    @constraint(model, X * ones(nv(G)) .<= n_eligible_successors);
    @constraint(model, X * ones(nv(G)) .>= n_required_successors);
    @constraint(model, X' * ones(nv(G)) .<= n_eligible_predecessors);
    @constraint(model, X' * ones(nv(G)) .>= n_required_predecessors);

    for v in vertices(G)
        upstream_vertices = [v, map(e->e.dst,collect(edges(bfs_tree(G,v;dir=:in))))...]
        for v2 in upstream_vertices
            @constraint(model, X[v,v2] == 0)
        end
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        for (template, val) in missing_successors[v]
            for v2 in vertices(G)
                node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
                if (v2 in upstream_vertices) || !matches_template(template, typeof(node2))
                    # @constraint(model, X[v,v2] == 0)
                    continue
                end
                potential_match = false
                for (template2, val2) in missing_predecessors[v2]
                    if matches_template(template2,typeof(node)) # possible to add and edge
                        potential_match = true
                        if !(val > 0 && val2 > 0)
                            continue
                        end
                        new_node = align_with_predecessor(node2,node)
                        dt_min = generate_path_spec(project_schedule,problem_spec,new_node).min_path_duration
                        @constraint(model, tF[v2] - (t0[v2] + dt_min) >= -Mm*(1 - X[v,v2]))
                    end
                end
                if potential_match == false
                    @constraint(model, X[v,v2] == 0)
                end
            end
        end
    end

    # "Job-shop" constraints specifying that no station may be double-booked. A station
    # can only support a single COLLECT or DEPOSIT operation at a time, meaning that all
    # the windows for these operations cannot overlap. In the constraints below, t1 and t2
    # represent the intervals for the COLLECT or DEPOSIT operations of tasks j and j2,
    # respectively. If eny of the operations for these two tasks require use of the same
    # station, we introduce a 2D binary variable y. if y = [1,0], the operation for task
    # j must occur before the operation for task j2. The opposite is true for y == [0,1].
    # We use the big M method here as well to tightly enforce the binary constraints.
    job_shop_variables = Dict{Tuple{Int,Int},JuMP.VariableRef}();
    for v in 1:nv(G)
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        for v2 in v+1:nv(G)
            node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
            common_resources = intersect(resources_reserved(node),resources_reserved(node2))
            if length(common_resources) > 0
                # @show common_resources
                tmax = @variable(model)
                tmin = @variable(model)
                y = @variable(model, binary=true)
                job_shop_variables[(v,v2)] = y
                @constraint(model, tmax >= t0[v])
                @constraint(model, tmax >= t0[v2])
                @constraint(model, tmin <= tF[v])
                @constraint(model, tmin <= tF[v2])

                @constraint(model, tmax - t0[v2] <= (1 - y)*Mm)
                @constraint(model, tmax - t0[v] <= y*Mm)
                @constraint(model, tmin - tF[v] >= (1 - y)*-Mm)
                @constraint(model, tmin - tF[v2] >= y*-Mm)
                @constraint(model, tmin + 1 <= tmax)
            end
        end
    end

    # Formulate Objective
    if cost_model == :SumOfMakeSpans
        root_nodes = project_schedule.root_nodes
        @variable(model, T[1:length(root_nodes)])
        for (i,project_head) in enumerate(root_nodes)
            for v in project_head
                @constraint(model, T[i] >= tF[v])
            end
        end
        cost1 = @expression(model, sum(map(v->tF[v]*get(project_schedule.weights,v,0.0), root_nodes)))
    elseif cost_model == :MakeSpan
        @variable(model, T)
        @constraint(model, T .>= tF)
        cost1 = @expression(model, T)
    end
    sparsity_cost = @expression(model, (0.5/(nv(G)^2))*sum(X)) # cost term to encourage sparse X. Otherwise the solver may add pointless edges
    @objective(model, Min, cost1 + sparsity_cost)
    # @objective(model, Min, cost1 )
    model, job_shop_variables
end
