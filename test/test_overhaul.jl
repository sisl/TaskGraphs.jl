# using Pkg
# Pkg.activate("/home/kylebrown/.julia/dev/TaskGraphs")
using TaskGraphs
using CRCBS
using LightGraphs, MetaGraphs, GraphUtils
using ImageFiltering
using Gurobi
using JuMP, MathOptInterface
using TOML
using Random
using Test
using GraphPlottingBFS
using Compose

# load rendering tools
include(joinpath(pathof(TaskGraphs),"../..","test/notebooks/render_tools.jl"))

function show_times(sched,v)
    arr = process_schedule(sched)
    return string(map(a->string(a[v],","), arr[1:2])...)
end
function print_project_schedule(project_schedule,filename;mode=:root_aligned,verbose=true)
    rg = get_display_metagraph(project_schedule;
        f=(v,p)->string(v,",",get_path_spec(project_schedule,v).agent_id))
    plot_graph_bfs(rg;
        mode=mode,
        shape_function = (G,v,x,y,r)->Compose.circle(x,y,r),
        color_function = (G,v,x,y,r)->get_prop(G,v,:color),
        text_function = (G,v,x,y,r)->string(
            title_string(get_node_from_id(project_schedule, get_vtx_id(project_schedule, v)),verbose),
            "\n",show_times(project_schedule,v)
            )
    ) |> Compose.SVG(string(filename,".svg"))
    # `inkscape -z project_schedule1.svg -e project_schedule1.png`
    # OR: `for f in *.svg; do inkscape -z $f -e $f.png; done`
end
function print_project_schedule(project_schedule,model,filename;mode=:root_aligned,verbose=true)
    rg = get_display_metagraph(project_schedule;
        f=(v,p)->string(v,",",get_path_spec(project_schedule,v).agent_id))
    plot_graph_bfs(rg;
        mode=mode,
        shape_function = (G,v,x,y,r)->Compose.circle(x,y,r),
        color_function = (G,v,x,y,r)->get_prop(G,v,:color),
        text_function = (G,v,x,y,r)->string(
            title_string(get_node_from_id(project_schedule, get_vtx_id(project_schedule, v)),verbose),
            "\n",show_times(project_schedule,v),
            "-",Int(round(value(model[:t0][v]))),",",Int(round(value(model[:tF][v])))
            )
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
    problem_spec = ProblemSpec(N=N,M=M,D=dist_matrix);

    # Construct Partial Project Schedule
    project_schedule = construct_partial_project_schedule(spec,problem_spec,robot_ICs)

    # edge_list = collect(edges(project_schedule.graph))
    # nodes = map(id->get_node_from_id(project_schedule, id), project_schedule.vtx_ids)
    # nodes, edge_list

    # Formulate MILP problem
    model = formulate_schedule_milp(project_schedule,problem_spec)

    # Optimize!
    optimize!(model)
    @show status = termination_status(model)
    obj_val = Int(round(value(objective_function(model))))
    @show adj_matrix = Int.(round.(value.(model[:X])))

    print_project_schedule(project_schedule,"project_schedule1";mode=:leaf_aligned)

    # Update Project Graph by adding all edges encoded by the optimized adjacency graph
    update_project_schedule!(project_schedule,problem_spec,adj_matrix)
    print_project_schedule(project_schedule,"project_schedule2")

    @test validate(project_schedule)
end
# let

# Verify that old method (assignment milp) and new method (adjacency matrix
    # milp) yield the same costs
let

    for (i, f) in enumerate([
        initialize_toy_problem_1,
        initialize_toy_problem_2,
        initialize_toy_problem_3,
        initialize_toy_problem_4,
        initialize_toy_problem_5,
        initialize_toy_problem_6,
        initialize_toy_problem_7,
        initialize_toy_problem_8,
        ])
        for cost_model in [SumOfMakeSpans, MakeSpan]
            costs = Float64[]
            schedules = ProjectSchedule[]
            project_spec, problem_spec, robot_ICs, assignments, env_graph = f(;verbose=false);
            for milp_model in [AssignmentMILP(),AdjacencyMILP(),SparseAdjacencyMILP()]
                # MILP formulations alone
                schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
                model = formulate_milp(milp_model,schedule,problem_spec;cost_model=cost_model)
                optimize!(model)
                @test termination_status(model) == MOI.OPTIMAL
                cost = Int(round(value(objective_function(model))))
                adj_matrix = get_assignment_matrix(model)
                update_project_schedule!(milp_model,schedule,problem_spec,adj_matrix)
                @test validate(schedule)
                push!(costs, cost)
                push!(schedules, schedule)
                @test validate(schedule)
                @test cost != Inf

                # Check that it matches low_level_search
                solver = PC_TAPF_Solver(verbosity=0)
                env, mapf = construct_search_env(schedule, problem_spec, env_graph;primary_objective=cost_model)
                pc_mapf = PC_MAPF(env,mapf)
                constraint_node = initialize_root_node(pc_mapf)
                low_level_search!(solver,pc_mapf,constraint_node)
                # @show i, f, obj_val1, constraint_node.cost
                @test constraint_node.cost[1] == cost
                @test validate(env.schedule)
            end
            if !(costs[1] == costs[2])
                print_project_schedule(schedules[1],string("project_schedule1_",i))
                print_project_schedule(schedules[2],string("project_schedule2_",i))
            end
            if !(costs[1] == costs[3])
                print_project_schedule(schedules[1],string("project_schedule1_",i))
                print_project_schedule(schedules[3],string("project_schedule3_",i))
            end
            @test all(costs .== costs[1])
        end
    end
end

# Test that the full planning stack works with the new model and returns the same final cost
let

    for (i, f) in enumerate([
        initialize_toy_problem_1,
        initialize_toy_problem_2,
        initialize_toy_problem_3,
        initialize_toy_problem_4,
        initialize_toy_problem_5,
        initialize_toy_problem_6,
        initialize_toy_problem_7,
        initialize_toy_problem_8,
        ])
        for cost_model in [SumOfMakeSpans, MakeSpan]
            let
                costs = Float64[]
                project_spec, problem_spec, robot_ICs, assignments, env_graph = f(;verbose=false);
                for milp_model in [AssignmentMILP(),AdjacencyMILP(),SparseAdjacencyMILP()]
                    solver = PC_TAPF_Solver(verbosity=0)
                    solution, assignment, cost, env = high_level_search!(
                        milp_model,
                        solver,
                        env_graph,
                        project_spec,
                        problem_spec,
                        robot_ICs,
                        Gurobi.Optimizer;
                        primary_objective=cost_model,
                        )
                    push!(costs, cost[1])
                    @test validate(env.schedule)
                    @show convert_to_vertex_lists(solution)
                    @test validate(env.schedule, convert_to_vertex_lists(solution), env.cache.t0, env.cache.tF)
                    @test cost[1] != Inf
                end
                @test all(costs .== costs[1])
            end
        end
    end

end

# end

let
    env_id = 2
    env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
    factory_env = read_env(env_filename)
    env_graph = factory_env.graph
    dist_matrix = get_dist_matrix(env_graph)
    logfile = "log.txt"

    TimeLimit = 40
    OutputFlag = 0
    problem_dir = PROBLEM_DIR

    problematic_ids = [
        43, # one of the solvers gets stuck after A* returns an infeasible path (twice)
        146, # A* infeasible, again.
        197, # can't remember why I put this on here
        255, # more A* infeasible. These always seem to terminate with "bounds error"
        267, # just pausing here--nothing necessarily wrong.
        146, # TODO why does this fail for SparseAdjacencyMILP?
        ]

    ##
    # for problem_id in problematic_ids[end]+1:384
    for problem_id in 1:10
        problem_filename = joinpath(problem_dir,string("problem",problem_id,".toml"))
        problem_def = read_problem_def(problem_filename)
        project_spec, r0, s0, sF = problem_def.project_spec,problem_def.r0,problem_def.s0,problem_def.sF
        project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(project_spec, r0, s0, sF, dist_matrix)
        println("PROBLEM ID: ", problem_id)
        for cost_model in [SumOfMakeSpans, MakeSpan]
            costs = Float64[]
            for milp_model in [AdjacencyMILP(),SparseAdjacencyMILP()]
                try
                    solver = PC_TAPF_Solver(verbosity=0)
                    solution, assignment, cost, env = high_level_search!(
                        milp_model,
                        solver,
                        env_graph,
                        project_spec,
                        problem_spec,
                        robot_ICs,
                        Gurobi.Optimizer;
                        primary_objective=cost_model,
                        TimeLimit=TimeLimit
                        )
                    push!(costs, cost[1])
                    @assert validate(env.schedule)
                    @assert cost[1] != Inf
                catch e
                    open(logfile, "a") do io
                        write(io, string("PROBLEM ", problem_id, " - ",
                            "cost model: ", cost_model, " - ",
                            typeof(milp_model), " - ", e.msg, "\n"))
                    end
                end
            end
            try
                @assert all(costs .== costs[1])
            catch e
                open(logfile, "a") do io
                    write(io, string("PROBLEM ", problem_id, " - ",
                        "cost model: ", cost_model, " - ",
                         e.msg, " costs: ", costs, "\n"))
                end
            end
        end
    end
    ##
end

# Pruning projects
let
    env_id = 2
    env_filename = string(ENVIRONMENT_DIR,"/env_",env_id,".toml")
    factory_env = read_env(env_filename)
    # env_graph = factory_env.graph
    env_graph = factory_env
    dist_matrix = get_dist_matrix(env_graph)

    # problem_def = read_problem_def(joinpath(PROBLEM_DIR,"problem223.toml"))
    problem_id = 25
    problem_def = read_problem_def(joinpath(PROBLEM_DIR,string("problem",problem_id,".toml")))
    # length(problem_def.r0), length(problem_def.s0)
    project_spec, r0, s0, sF = problem_def.project_spec,problem_def.r0,problem_def.s0,problem_def.sF
    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(
            project_spec, r0, s0, sF, dist_matrix);

    solver = PC_TAPF_Solver(DEBUG=true,verbosity=1,LIMIT_A_star_iterations=5*nv(env_graph));
    (solution, assignment, cost, search_env), elapsed_time, byte_ct, gc_time, mem_ct = @timed high_level_search!(
        SparseAdjacencyMILP(), solver, env_graph, project_spec, problem_spec, robot_ICs, Gurobi.Optimizer;)

    meta_env = MetaAgentCBS.construct_meta_env([search_env.env], get_cost_model(search_env.env))
    s = MetaAgentCBS.State([PCCBS.State(1,0)])
    for a in get_possible_actions(meta_env, s)
        @show get_next_state(meta_env, s, a)
    end

    # project_schedule = construct_partial_project_schedule(project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
    # model = formulate_milp(SparseAdjacencyMILP(),project_schedule,problem_spec;Presolve=1,TimeLimit=0.5)
    # exclude_solutions!(model)
    # retval, elapsed_time, byte_ct, gc_time, mem_ct = @timed optimize!(model)
    # @show elapsed_time
    # @show primal_status(model)
    # @show dual_status(model)
    # @show objective_bound(model), value(objective_function(model))
    # @test termination_status(model) == MathOptInterface.OPTIMAL
    # assignment_matrix = get_assignment_matrix(model);
    # update_project_schedule!(project_schedule,problem_spec,assignment_matrix)
    # @test validate(project_schedule)
    # @test project_schedule.graph == DiGraph(assignment_matrix)
    # cache = initialize_planning_cache(project_schedule)

    # env, mapf = construct_search_env(project_schedule, problem_spec, env_graph);
    # pc_mapf = PC_MAPF(env,mapf);

    project_schedule = search_env.schedule
    cache = search_env.cache
    @test validate(project_schedule)

    t = 30
    trimmed_solution = trim_solution(search_env.env, solution, t)
    @test all([length(p) == get_cost(p)[1] for p in get_paths(trimmed_solution)])

    new_schedule = prune_project_schedule(project_schedule, cache, t; robot_positions=get_env_snapshot(solution,t))
    set_leaf_operation_nodes!(new_schedule)
    G = get_graph(new_schedule)
    # init planning cache with the existing solution
    t0 = map(v->get(cache.t0, get_vtx(project_schedule, get_vtx_id(new_schedule, v)), 0.0), vertices(G))
    new_cache = initialize_planning_cache(new_schedule;t0=t0)
    # identify active and fixed nodes
    active_vtxs = Set{Int}()
    fixed_vtxs = Set{Int}()
    for v in vertices(G)
        if new_cache.tF[v] < t
            push!(fixed_vtxs, v)
        elseif new_cache.t0[v] <= t <= new_cache.tF[v] # test if vertex is eligible to be dropped
            node_id = get_vtx_id(new_schedule,v)
            push!(active_vtxs, v)
            for e in edges(bfs_tree(G,v;dir=:in))
                push!(fixed_vtxs, e.dst)
            end
        end
    end
    # set all fixed_vtxs to plan_path=false
    for v in fixed_vtxs
        set_path_spec!(new_schedule,v,PathSpec(get_path_spec(new_schedule,v), plan_path=false, fixed=true))
    end
    # verify that all vertices following active_vtxs have a start time > 0
    let
        s1 = map(v->new_cache.t0[v], collect(fixed_vtxs))
        s2 = map(v->new_cache.t0[v], collect(active_vtxs))
        s3 = map(v->new_cache.t0[v], collect(setdiff(collect(vertices(G)),union(fixed_vtxs,active_vtxs))))
        @test all(s3 .> 0)
    end


    #### New Project schedule to splice in:
    problem_id2 = problem_id + 1
    def2 = read_problem_def(joinpath(PROBLEM_DIR,string("problem",problem_id2,".toml")))
    project_spec2, problem_spec2, _, _, _ = construct_task_graphs_problem(def2.project_spec, def2.r0, def2.s0, def2.sF, dist_matrix);
    next_schedule = construct_partial_project_schedule(project_spec2,problem_spec2)

    # remap object ids
    max_obj_id = maximum([get_id(id) for id in get_vtx_ids(project_schedule) if typeof(id) <: ObjectID])
    remap_object_ids!(next_schedule,max_obj_id)

    let
        s1 = Set{AbstractID}(new_schedule.vtx_ids)
        s2 = Set{AbstractID}(next_schedule.vtx_ids)
        @test length(s1) + length(s2) == length(union(s1,s2))
    end

    # splice projects together!
    for v in vertices(get_graph(next_schedule))
        node_id = get_vtx_id(next_schedule, v)
        add_to_schedule!(new_schedule, get_node_from_id(next_schedule, node_id), node_id)
    end
    for e in edges(get_graph(next_schedule))
        node_id1 = get_vtx_id(next_schedule, e.src)
        node_id2 = get_vtx_id(next_schedule, e.dst)
        add_edge!(new_schedule, node_id1, node_id2)
    end
    set_leaf_operation_nodes!(new_schedule)
    # do this again because now we have more nodes
    t0 = map(v->get(new_cache.t0, v, 0.0), vertices(G))
    new_cache = initialize_planning_cache(new_schedule;t0=t0)

    high_level_search!(solver, env_graph, new_schedule, problem_spec, Gurobi.Optimizer;
        milp_model=SparseAdjacencyMILP(),
        t0_ = Dict{AbstractID,Int}(get_vtx_id(new_schedule, v)=>t0 for (v,t0) in enumerate(new_cache.t0))
    )

    # Replan!
    model = formulate_milp(SparseAdjacencyMILP(),new_schedule,problem_spec;
        Presolve=1,
        TimeLimit=20,
        t0_ = Dict{AbstractID,Int}(get_vtx_id(new_schedule, v)=>t0 for (v,t0) in enumerate(new_cache.t0)) # TODO figure out a better way to do this
        )
    exclude_solutions!(model) # NOTE this is a high object-to-robot ratio. Consider changing that for the demo
    retval, elapsed_time, byte_ct, gc_time, mem_ct = @timed optimize!(model)
    @show elapsed_time
    @show primal_status(model)
    @show dual_status(model)
    @show objective_bound(model), value(objective_function(model))
    @show termination_status(model)
    @test termination_status(model) == MathOptInterface.OPTIMAL

    assignment_matrix = get_assignment_matrix(model);
    update_project_schedule!(new_schedule,problem_spec,assignment_matrix)
    @test validate(new_schedule)

    # print_project_schedule(new_schedule,"dummy_schedule";mode=:leaf_aligned)


end

# Collaborative transport
let
    vtx_grid = initialize_dense_vtx_grid(8,8)
    # 1   9  17  25  33  41  49  57
    # 2  10  18  26  34  42  50  58
    # 3  11  19  27  35  43  51  59
    # 4  12  20  28  36  44  52  60
    # 5  13  21  29  37  45  53  61
    # 6  14  22  30  38  46  54  62
    # 7  15  23  31  39  47  55  63
    # 8  16  24  32  40  48  56  64
    env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
    # dist_matrix = get_dist_matrix(env_graph)
    dist_matrix = DistMatrixMap(env_graph.vtx_map)
    r0 = [1,25,4,29]
    # r0 = [1,25,8,28] # check that planning works even when it takes longer for some robots to arrive than others
    s0 = [10]#,18,11,19]
    sF = [42] #,50,43,51]

    task_shapes = Dict(1=>(2,2))
    shape_dict = Dict(
        10=>Dict((2,2)=>[10,18,11,19]),
        42=>Dict((2,2)=>[42,50,43,51]),
        )

    project_spec, robot_ICs = TaskGraphs.initialize_toy_problem(r0,[s0],[sF],(v1,v2)->dist_matrix[v1,v2])
    add_operation!(project_spec,construct_operation(project_spec,-1,[1],[],0))

    cost_function = MakeSpan
    project_spec, problem_spec, object_ICs, object_FCs, robot_ICs = construct_task_graphs_problem(
        project_spec,r0,s0,sF,dist_matrix;cost_function=cost_function,
        task_shapes=task_shapes,shape_dict=shape_dict)

    solver = PC_TAPF_Solver(
        DEBUG=true,
        LIMIT_A_star_iterations=5*nv(env_graph),
        verbosity=1,
        l4_verbosity=1
        );

    solution, _, cost, env = high_level_search!(SparseAdjacencyMILP(),solver,env_graph,project_spec,problem_spec,robot_ICs,Gurobi.Optimizer)
    solution, _, cost, env = high_level_search!(GreedyAssignment(),solver,env_graph,project_spec,problem_spec,robot_ICs,Gurobi.Optimizer)

    project_schedule = construct_partial_project_schedule(project_spec, problem_spec, map(i->robot_ICs[i], 1:problem_spec.N))

    # GreedyAssignment
    print_project_schedule(project_schedule,"project_schedule")

    G = get_graph(project_schedule);
    (missing_successors, missing_predecessors, n_eligible_successors, n_eligible_predecessors,
        n_required_successors, n_required_predecessors, upstream_vertices, non_upstream_vertices) = preprocess_project_schedule(project_schedule)

    downstream_vertices = map(v->[v, map(e->e.dst,collect(edges(bfs_tree(G,v;dir=:out))))...], vertices(G))
    traversal = topological_sort(G)
    available_outgoing = Set{Int}(vertices(G))
    available_incoming = Set{Int}(vertices(G))

    for v in traversal
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        potential_match = false
        if indegree(G,v) < n_eligible_predecessors[v] # NOTE: Trying this out to save time on formulation
            for v2 in downstream_vertices[v]
                if v2 != v
                    setdiff!(available_incoming, v2)
                    setdiff!(available_outgoing, v2)
                end
            end
        else
            setdiff!(available_incoming, v)
        end
        if outdegree(G,v) < n_eligible_successors[v]
        else
            setdiff!(available_outgoing, v)
        end
    end
    for v in available_incoming
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        @show node
    end
    for v in available_outgoing
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        @show node
    end

    while length(available_outgoing) > 0
        v = pop!(available_outgoing)
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        options = []
        costs = []
        for v2 in available_incoming
            node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
            for (template, val) in missing_successors[v]
                if !matches_template(template, typeof(node2)) # possible to add an edge
                    continue
                end
                for (template2, val2) in missing_predecessors[v2]
                    if !matches_template(template2, typeof(node)) # possible to add an edge
                        continue
                    end
                    if (val > 0 && val2 > 0)
                        new_node = align_with_successor(node,node2)
                        dt_min = generate_path_spec(project_schedule,problem_spec,new_node).min_path_duration
                        push!(options, v2)
                        push!(costs, dt_min)
                    end
                end
            end
        end
        # add best edge
        v2 = get(options,1,-1)
        if v2 != -1
            add_edge!(G,v,v2)
            if outdegree(G,v) < n_required_successors[v]
                push!(available_outgoing,v)
            end
            if indegree(G,v2) < get(n_required_predecessors,v2,Inf)
                push!(available_incoming,v2)
            else
                pop!(available_incoming,v2)
            end
            # update available_incoming and available_outgoing
            union!(available_outgoing, vertices(G))
            union!(available_incoming, vertices(G))

            for v in traversal
                if indegree(G,v) < n_eligible_predecessors[v] # NOTE: Trying this out to save time on formulation
                    for v2 in downstream_vertices[v]
                        if v2 != v
                            setdiff!(available_incoming, v2)
                            setdiff!(available_outgoing, v2)
                        end
                    end
                else
                    setdiff!(available_incoming, v)
                end
                if outdegree(G,v) < n_eligible_successors[v]
                else
                    setdiff!(available_outgoing, v)
                end
            end
        end
    end


    set_leaf_operation_nodes!(project_schedule)
    model = formulate_milp(SparseAdjacencyMILP(),project_schedule,problem_spec)
    optimize!(model)
    @test termination_status(model) == MOI.OPTIMAL
    adj_mat = get_assignment_matrix(model)
    update_project_schedule!(project_schedule,problem_spec,adj_mat)


    # print_project_schedule(project_schedule,"team_schedule")

    # Path planning
    solver = PC_TAPF_Solver(DEBUG=true,verbosity=1,LIMIT_A_star_iterations=5*nv(env_graph));
    env, mapf = construct_search_env(project_schedule, problem_spec, env_graph);
    pc_mapf = PC_MAPF(env,mapf);
    ##### Call CBS Search Routine (LEVEL 2) #####
    root_node = initialize_root_node(mapf)
    low_level_search!(solver, pc_mapf.env, pc_mapf.mapf,root_node)

    @show convert_to_vertex_lists(root_node.solution)
end
let
    # build custom envs for different robot sizes
    env_id = 2
    env_file = joinpath(ENVIRONMENT_DIR,string("env_",env_id,".toml"))
    factory_env = read_env(env_file)
    dist_matrix = get_dist_matrix(factory_env)

    dist_mtx_map =DistMatrixMap(factory_env.vtx_map)
    s = (2,2)
    v1 = 79
    v2 = 82
    @test get_distance(dist_mtx_map,v1,v2) == dist_matrix[v1,v2]
    @test get_distance(dist_mtx_map,v1,v2,(1,1)) == dist_matrix[v1,v2]
    @test get_distance(dist_mtx_map,v1,v2,(1,2)) == dist_matrix[v1,v2]
    @test get_distance(dist_mtx_map,v1,v2,(2,1)) == dist_matrix[v1,v2] + 2
    @test get_distance(dist_mtx_map,v1,v2,(2,2)) == dist_matrix[v1,v2] + 2


    N = 12
    M = 12
    r0,s0,sF = get_random_problem_instantiation(N,M,get_pickup_zones(factory_env),get_dropoff_zones(factory_env),
            get_free_zones(factory_env))
    project_spec = construct_random_project_spec(M,s0,sF)
    task_sizes = (1=>1.0,2=>1.0,4=>1.0)
    shapes = choose_random_object_sizes(M,Dict(task_sizes...))
    # problem_def = SimpleProblemDef(project_spec,r0,s0,sF,shapes)

    project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
        project_spec, r0, s0, sF, dist_matrix;
        task_shapes=shapes,
        shape_dict=factory_env.expanded_zones
        );

    project_spec.initial_conditions
end
