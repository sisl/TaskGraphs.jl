"""
The number of edges in each "gadget" per original edge
"""
const GADGET_EDGE_MULTIPLIER = 5
const GADGET_VTX_MULTIPLIER = 2


"""
    convert_env_graph_to_undirected(G)

It is necessary to convert the env graph to an undirected graph because the
gadget is based on undirected edges. Self-edges also need to be removed, as 
these are already accounted for in the gadget.
"""
function convert_env_graph_to_undirected(G)
    env_graph = Graph(G)
end

@enum FlowNodeType begin
    _STAY # wait for a time step
    _BRIDGE # carry over between gadget layers
    _EDGE # edge vtx
end

struct FlowNode
    type::FlowNodeType # type of node
    v::Int # vertex of node in original graph
    t::Int # time step of node
end

"""
    GadgetGraph

Represents a time-extended graph useful for MILP formulations. Each vertex of
the `GadgetGraph` corresponds to a "flow edge"
"""
const GadgetGraph = NGraph{DiGraph,FlowNode,VtxID}
get_vtx_from_var(G::GadgetGraph,v)      = has_vertex(G,v) ? node_val(get_node(G,v)).v : -1
get_t_from_var(G::GadgetGraph,v)        = has_vertex(G,v) ? node_val(get_node(G,v)).t : -1
is_movement_vtx(graph::GadgetGraph,v)   = node_val(get_node(graph,v)).type == _EDGE
is_stay_vtx(graph::GadgetGraph,v)       = node_val(get_node(graph,v)).type == _STAY
is_bridge_vtx(graph::GadgetGraph,v)     = node_val(get_node(graph,v)).type == _BRIDGE

"""
    add_movement_vtx!

Flags a vertex of the gadget graph as corresponding to a non-"wait" edge
"""
# add_movement_vtx!(graph::GadgetGraph,t) = add_vertex!(graph,t,0)
# add_bridge_vtx!(graph::GadgetGraph,t,v) = add_vertex!(graph,t,v)
# add_stay_vtx!(graph::GadgetGraph,t)     = add_vertex!(graph,t)
GraphUtils.add_node!(graph::GadgetGraph,n::FlowNode)       = add_node!(graph,n,get_unique_id(VtxID))
add_movement_vtx!(graph::GadgetGraph,v,t)   = add_node!(graph,FlowNode(_EDGE,   v,t))
add_bridge_vtx!(graph::GadgetGraph,v,t)     = add_node!(graph,FlowNode(_BRIDGE, v,t))
add_stay_vtx!(graph::GadgetGraph,v,t)       = add_node!(graph,FlowNode(_STAY,   v,t))

"""
    get_source_map(G::GadgetGraph,env_graph,TMAX)

Return a source map such that source_map[v][t] points to the corresponding 
vertex in the gadget graph.
"""
function get_source_map(G::GadgetGraph,env_graph,TMAX)
    # source_map = zeros(Int,nv(env_graph),TMAX)
    source_map = [Dict{Int,Int}() for v in 1:nv(env_graph)]
    for idx in vertices(G)
        # if has_vertex(env_graph,v) && !is_bridge_vtx(G,idx)
        # if has_vertex(env_graph,v) && !is_stay_vtx(G,idx)
        if is_bridge_vtx(G,idx)
            v = get_vtx_from_var(G,idx)
            t = get_t_from_var(G,idx)
            @assert has_vertex(env_graph,v)
            source_map[v][t] = idx
        end
    end
    return source_map
end

function add_vertex_layer!(G::GadgetGraph,incoming::Dict{Int,Vector{Int}},t)
    outgoing = Dict{Int,Int}()
    for v in sort(collect(keys(incoming)))
        # add_stay_vtx!(G,t,v)
        add_bridge_vtx!(G,v,t)
        outgoing[v] = nv(G)
        for v2 in incoming[v]
            add_edge!(G,v2,nv(G))
        end
    end
    return outgoing
end
function add_gadget_layer!(G::GadgetGraph,edge_list,t,vtx_map=Dict{Int,Int}())
    outgoing = Dict{Int,Vector{Int}}()
    for e in edge_list
        if !haskey(vtx_map,e.src) && !haskey(vtx_map,e.dst)
            continue
        end
        if e.src == e.dst
            # shortcut
            # |
            # add_bridge_vtx!(G,t)
            # add_stay_vtx!(G,t)
            add_stay_vtx!(G,e.src,t)
            add_edge!(G,vtx_map[e.src],nv(G))
            push!(get!(outgoing,e.dst,Int[]),nv(G))
        else
            if !haskey(vtx_map,e.src)
                e = reverse(e)
            end
            # Gadget
            # \ /
            #  |
            # / \
            # add_movement_vtx!(G,t)
            add_movement_vtx!(G,e.src,t)
            v1 = nv(G)
            add_edge!(G,vtx_map[e.src],v1)
            if haskey(vtx_map,e.dst)
                # add_movement_vtx!(G,t)
                add_movement_vtx!(G,e.dst,t)
                v2 = nv(G)
                add_edge!(G,vtx_map[e.dst],v2)
            end
            # add_movement_vtx!(G,t)
            add_movement_vtx!(G,-1,t)
            v3 = nv(G)
            add_edge!(G,v1,v3)
            if haskey(vtx_map,e.dst)
                add_edge!(G,v2,v3)
                # add_movement_vtx!(G,t)
                add_movement_vtx!(G,e.src,t)
                v4 = nv(G)
                add_edge!(G,v3,v4)
                push!(get!(outgoing,e.src,Int[]),v4)
            end
            # add_movement_vtx!(G,t)
            add_movement_vtx!(G,e.dst,t)
            v5 = nv(G)
            add_edge!(G,v3,v5)
            push!(get!(outgoing,e.dst,Int[]),v5)
        end
    end
    return outgoing
end

function construct_gadget_graph(env_graph,TMAX,t0=0,cap=true)
    G = GadgetGraph()
    incoming = Dict(v=>Int[] for v in vertices(env_graph))
    edge_list = collect(edges(env_graph))
    for t in t0:TMAX-1
        outgoing = add_vertex_layer!(G,incoming,t)
        incoming = add_gadget_layer!(G,edge_list,t,outgoing)
    end
    if cap
        add_vertex_layer!(G,incoming,TMAX)
    end
    return G
end

"""
    add_point_constraint!(model,source_map,flow,v,t=0)

Constrain flow to be equal to 1 at vertex `v`, time `t`
"""
function add_point_constraint!(model,source_map,flow,v,t=0)
    idx = source_map[v][t]
    @constraint(model, flow[idx] == 1)
    return model
end
function add_continuity_constraints!(model,G,flow)
    for v in vertices(G)
        @constraint(model,1 >= sum(map(vp->flow[vp],outneighbors(G,v)))) # unit capacity edges
        @constraint(model,1 >= sum(map(vp->flow[vp],inneighbors(G,v)))) # unit capacity edges
        if outdegree(G,v) > 0
            @constraint(model,
                flow[v] <= sum(map(vp->flow[vp],outneighbors(G,v))) # No disappearing robots
                )
        end
        if indegree(G,v) > 0
            @constraint(model,
                flow[v] <= sum(map(vp->flow[vp],inneighbors(G,v))) # No appearing robots
                )
        end
    end
    return model
end
function add_single_transporter_constraint!(model,G,source_map,robot_flow,object_flow,start,goal)
    T = length(source_map[goal])-1
    for v in vertices(G)
        if is_bridge_vtx(G,v) || is_stay_vtx(G,v)
            vtx = get_vtx_from_var(G,v)
            if !(vtx == start || vtx == goal)
                # if object is not at its start or goal, it cannot be without a robot
                @constraint(model, object_flow[v] <= robot_flow[v])
            end
        end
    end
    return model
end
function add_single_object_per_robot_constraints!(model,G,object_flows)
    for v in vertices(G)
        if is_movement_vtx(G,v)
            # objects may not overlap on movement vtxs (meaning that robot may not
            # collect multiple objects at once)
            @constraint(model, sum(flow[v] for flow in values(object_flows)) <= 1)
        end
    end
    return model
end
function add_carrying_constraints!(model,G,robot_flow,object_flow)
    for v in vertices(G)
        if is_movement_vtx(G,v)
            # object may not traverse an edge unless it is being carried
            @constraint(model, object_flow[v] - robot_flow[v] <= 0)
        end
    end
    return model
end
function add_precedence_constraints!(model,source_map,inflow,outflow,goal,start,Δt=0)
    T = length(source_map[goal])-1
    for t in 0:T-(1+Δt) 
        vtx1 = source_map[goal][t]
        vtx2 = source_map[start][t+1+Δt]
        # if flow1 is not at vtx1, flow2 must still be at vtx2
        @constraint(model,outflow[vtx2] + inflow[vtx1] >= 1)
    end
end
function add_makespan_constraint!(model,T,flow,source_map,goal,op_duration=0)
    for (t,v) in source_map[goal]
        @constraint(model,T >= 1+t*(1-flow[v])+op_duration)
    end
    return model
end
function add_total_flow_constraint!(model,source_map,flow,n,t=0)
    @constraint(model,sum(map(d->flow[d[t]],source_map)) == n)
end

struct PCTAPF_MILP
    model::JuMP.Model
    G::GadgetGraph
    source_map::Vector{Dict{Int,Int}}
    robot_flow::Vector{VariableRef}
    object_flows::Dict{ObjectID,Vector{VariableRef}}
end
for op in [
    :(JuMP.optimize!),
    :(JuMP.primal_status),
    :(JuMP.termination_status),
    :(JuMP.objective_bound),
    :(JuMP.objective_value),
    :(JuMP.set_time_limit_sec),
]
    @eval $op(m::PCTAPF_MILP,args...) = $op(m.model,args...)
end
extract_robot_flow(m::PCTAPF_MILP) = Int.(round.(value.(m.robot_flow)))
extract_object_flows(m::PCTAPF_MILP) = Dict(id=>Int.(round.(value.(var))) for (id,var) in m.object_flows)
function extract_flow_path(m::PCTAPF_MILP,flow,v0,t0)
    vtx = m.source_map[v0][0]
    path = Int[vtx]
    while outdegree(m.G,vtx) > 0
        for vp in outneighbors(m.G,vtx)
            if flow[vp] == 1
                vtx = vp
                break
            end
        end
        push!(path,vtx)
    end
    return path
end
function extract_true_path(m::PCTAPF_MILP,flow_path)
    path = Int[]
    for vtx in flow_path
        # if get_vtx_from_var(m.G,vtx) > 0
        if is_bridge_vtx(m.G,vtx)
            push!(path,get_vtx_from_var(m.G,vtx))
        end
    end
    return path
end
function extract_robot_paths(prob::PC_TAPF,m::PCTAPF_MILP)
    sched = get_schedule(get_env(prob))
    robot_ICs = get_robot_ICs(sched)
    flow = Int.(round.(value.(m.robot_flow)))
    paths = Dict{RobotID,Vector{Int}}()
    for (id,pred) in robot_ICs
        v0 = get_id(get_initial_location_id(pred))
        t0 = get_t0(get_env(prob),id)
        flow_path = extract_flow_path(m,flow,v0,t0)
        paths[id] = extract_true_path(m,flow_path)
    end
    return paths
end
function extract_object_paths(prob::PC_TAPF,m::PCTAPF_MILP)
    sched = get_schedule(get_env(prob))
    object_ICs = get_object_ICs(sched)
    paths = Dict{ObjectID,Vector{Int}}()
    for (id,pred) in object_ICs
        flow = Int.(round.(value.(m.object_flows[id])))
        v0 = get_id(get_initial_location_id(pred))
        t0 = get_t0(get_env(prob),id)
        flow_path = extract_flow_path(m,flow,v0,t0)
        paths[id] = extract_true_path(m,flow_path)
    end
    return paths
end

export BigMILPSolver

@with_kw struct BigMILPSolver <: AbstractPCTAPFSolver
    EXTRA_T::Int                       = 2
    LIMIT_EXTRA_T::Int                 = 64
    logger::SolverLogger{Float64}   = SolverLogger{Float64}()
end

"""
    formulate_big_milp

Formulate a PCTAPF problem as a giant network flow MILP.
"""
function formulate_big_milp(prob::PC_TAPF,EXTRA_T=1;
        direct::Bool = true,
    )
    if direct
        model = direct_model(default_milp_optimizer()()) # reduced memory footprint
    else
        model = JuMP.Model(default_milp_optimizer())
    end
    set_optimizer_attributes(model,default_optimizer_attributes()...)

    sched = get_schedule(get_env(prob))
    robot_ICs = get_robot_ICs(sched)
    object_ICs = get_object_ICs(sched)
    env_graph = convert_env_graph_to_undirected(get_graph(get_env(prob)).graph)
    # Choose the time horizon to fix for the flow formulation
    TMAX = makespan_lower_bound(sched,get_problem_spec(get_env(prob)))+EXTRA_T
    G = construct_gadget_graph(env_graph,TMAX) # for robot flow
    source_map = get_source_map(G,env_graph,TMAX)
    # Robot flows
    @variable(model,robot_flow[1:nv(G)], binary=true)
    # Object flows
    object_flows = Dict(id=>@variable(model,[1:nv(G)], binary=true) for (id,pred) in object_ICs)
    # Flow constraints
    add_continuity_constraints!(model,G,robot_flow)
    # initial constraints
    for (id,pred) in robot_ICs
        v0 = get_id(get_initial_location_id(pred))
        t0 = Int(round(get_t0(get_env(prob),id)))
        add_point_constraint!(model,source_map,robot_flow,v0,t0)
    end
    add_total_flow_constraint!(model,source_map,robot_flow,length(robot_ICs))
    # Object constraints
    for (id,pred) in object_ICs
        object_flow = object_flows[id]
        add_total_flow_constraint!(model,source_map,object_flow,1)
        # initial location
        v0 = get_id(get_initial_location_id(pred))
        t0 = Int(round(get_t0(get_env(prob),id)))
        add_point_constraint!(model,source_map,object_flow,v0,t0)
        add_continuity_constraints!(model,G,object_flow)
        add_carrying_constraints!(model,G,robot_flow,object_flow)
    end
    add_single_object_per_robot_constraints!(model,G,object_flows)
    for node in get_nodes(sched)
        if matches_template(BOT_CARRY,node)
            pred = node.node
            start = get_id(get_initial_location_id(pred))
            goal = get_id(get_destination_location_id(pred))
            id = get_object_id(pred)
            add_single_transporter_constraint!(model,G,source_map,robot_flow,
                object_flows[id],start,goal)
        end
    end
    # precedence constraints
    for op in values(get_operations(sched))
        for (id2,o2) in postconditions(op)
            outflow = object_flows[id2]
            start = get_id(get_initial_location_id(o2))
            @assert get_id(get_destination_location_id(object_ICs[id2])) == start
            for (id1,o1) in preconditions(op)
                inflow = object_flows[id1]
                goal = get_id(get_destination_location_id(o1))
                # @show string(id1)=>string(id2)
                add_precedence_constraints!(model,source_map,inflow,outflow,goal,start,duration(op))
            end
        end
    end
    # Objective
    @variable(model,T >= 0) # makespan
    for op in values(get_operations(sched))
        for (id,o) in preconditions(op)
            # makespans
            object_flow = object_flows[id]
            goal = get_id(get_destination_location_id(o))
            # @show string(o)
            add_point_constraint!(model,source_map,object_flow,goal,TMAX)
            add_makespan_constraint!(model,T,object_flow,source_map,goal,duration(op))
        end
    end
    @objective(model,Min,T)
    return PCTAPF_MILP(model, G, source_map, robot_flow, object_flows)
end


function extract_solution(prob::PC_TAPF,m::PCTAPF_MILP)
    sched = deepcopy(get_schedule(get_env(prob)))
    cache = initialize_planning_cache(sched)
    search_env = SearchEnv(
        schedule=sched,
        cache=cache,
        env_layers=get_env(prob).env_layers,
        cost_model=MakeSpan(sched,cache),
        heuristic_model=NullHeuristic())
    route_plan = get_route_plan(search_env)

    robot_paths = extract_robot_paths(prob,m)
    object_paths = extract_object_paths(prob,m)
    # extract route plan
    for (robot_id,robot_path) in robot_paths
        path = get_paths(route_plan)[get_id(robot_id)]
        for (t,(v1,v2)) in enumerate(zip(robot_path[1:end-1],robot_path[2:end]))
            s = GraphState(v1,t-1)
            a = GraphAction(LightGraphs.Edge(v1,v2),1)
            sp = GraphState(v2,t)
            push!(path,PathNode(s,a,sp))
        end
    end
    # extract schedule
    assignments = Set{Tuple{Int,RobotID,ObjectID}}()
    unmatched_object_ids = Set{ObjectID}(keys(object_paths))
    for (object_id,object_path) in object_paths
        object_id in unmatched_object_ids ? nothing : continue
        # t_collect = -1
        # t_deposit = -1
        # for (t,o_vtx) in zip(Base.Iterators.countfrom(0),object_path)
        #     if !(o_vtx == object_path[1])
        #         t_collect = t-1
        #         break
        #     end
        # end
        # for (t,o_vtx) in zip(Base.Iterators.countfrom(length(object_path)-1,-1),reverse(object_path))
        #     if !(o_vtx == object_path[end])
        #         t_deposit = t+1
        #         break
        #     end
        # end
        # @show t_collect, t_deposit
        for (robot_id,robot_path) in robot_paths
            object_id in unmatched_object_ids ? nothing : continue
            for (t,(r_vtx,o_vtx)) in enumerate(zip(robot_path,object_path)) 
                if (o_vtx != object_path[1])
                    if r_vtx == o_vtx 
                        assignment = (t,robot_id,object_id)
                        # @show assignment
                        push!(assignments,assignment)
                        delete!(unmatched_object_ids, object_id)
                        break
                    end
                end
            end
        end
    end
    sched = get_schedule(search_env)
    # Add assignments to schedule
    robot_tips = robot_tip_map(sched)
    collect_tips = Dict{ObjectID,ScheduleNode}(
        get_object_id(n) => n for n in get_nodes(sched) if matches_template(BOT_COLLECT,n)
    )
    for assignment in sort(collect(assignments);by=a->a[1])
        t, robot_id, object_id = assignment
        collect_node = collect_tips[object_id]
        robot_node = get_node(sched,robot_tips[robot_id])
        # @show string(robot_node.node)
        # @show string(align_with_successor(robot_node.node,collect_node.node))
        robot_node = replace_in_schedule!(sched,
            get_problem_spec(search_env),
            align_with_successor(robot_node.node,collect_node.node),
            node_id(robot_node)
            )
        # @show string(robot_node.node), robot_node.spec
        add_edge!(sched,robot_node,collect_node)
        propagate_valid_ids!(sched,get_problem_spec(search_env))
        robot_tips = robot_tip_map(sched)
        # robot_tips[robot_id] = new_tip[robot_id]
    end
    # Sync schedule with route plan
    propagate_valid_ids!(sched,get_problem_spec(search_env))
    for node in get_nodes(sched)
        if matches_template(BOT_COLLECT,node)
            object_path = object_paths[get_object_id(node.node)]
            for (t,o_vtx) in zip(Base.Iterators.countfrom(0),object_path)
                if !(o_vtx == object_path[1])
                    set_t0!(node,t-1)
                    break
                end
            end
        elseif matches_template(BOT_DEPOSIT,node)
            object_path = object_paths[get_object_id(node.node)]
            for (t,o_vtx) in zip(Base.Iterators.countfrom(length(object_path)-1,-1),reverse(object_path))
                if !(o_vtx == object_path[end])
                    set_t0!(node,t+1)
                    break
                end
            end
        end
    end
    process_schedule!(sched)
    # for node in node_iterator(sched,topological_sort_by_dfs(sched))
    #     if matches_template(Union{BOT_COLLECT,BOT_CARRY,BOT_DEPOSIT},node)
    #         pred = node.node
    #         robot_path = robot_paths[get_robot_id(pred)]
    #         for (t,vtx) in enumerate(robot_path)
    #             t <= get_t0(node) ? continue : nothing
    #             if vtx == get_id(get_initial_location_id(pred))
    #                 set_t0!(node,t-1)
    #                 # for n in node_iterator(sched,inneighbors(sched,node))
    #                 #     if matches_template(Union{BOT_CARRY,BOT_GO},n)
    #                 #         set_tF!(n,get_t0(node))
    #                 #     end
    #                 # end
    #                 break
    #             end
    #         end
    #         update_schedule_times!(sched,Set{Int}(get_vtx(sched,node)))
    #     end
    # end
    for node in get_nodes(sched)
        if matches_template(Union{BOT_CARRY,BOT_GO},node)
            if outdegree(sched,node) > 0
                set_tF!(node,get_t0(sched,outneighbors(sched,node)[1]))
            end
        end
    end
    return search_env
end

function CRCBS.solve!(solver::BigMILPSolver,prob::PC_TAPF)
    set_runtime_limit!(solver,max(0,min(runtime_limit(solver),1000)))
    EXTRA_T = solver.EXTRA_T
    milp = formulate_big_milp(prob, EXTRA_T)
    set_time_limit_sec(milp, runtime_limit(solver))
    optimize!(milp)
    while !(primal_status(milp) == MOI.FEASIBLE_POINT)
        @show EXTRA_T = max(EXTRA_T,1) * 2
        if EXTRA_T > solver.LIMIT_EXTRA_T
            break
        end
        milp = formulate_big_milp(prob, EXTRA_T)
        set_time_limit_sec(milp, runtime_limit(solver))
        optimize!(milp)
    end
    if primal_status(milp) == MOI.FEASIBLE_POINT
        bound = Int(round(objective_bound(milp)))
        cost = Int(round(objective_value(milp)))
        set_lower_bound!(solver,bound)
        set_best_cost!(solver,cost)
        env = extract_solution(prob,milp)
    else
        cost = Inf
        set_best_cost!(solver,Inf)
        env = deepcopy(get_env(prob))
    end
    return env, cost
end

construct_cost_model(solver::BigMILPSolver,sched,cache,args...;kwargs...) = MakeSpan(sched,cache), NullHeuristic()