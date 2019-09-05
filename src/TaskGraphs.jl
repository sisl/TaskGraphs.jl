module TaskGraphs

using Parameters
using LightGraphs, MetaGraphs
using GraphUtils
using LinearAlgebra
using DataStructures
using JuMP
using TOML

include("planning_predicates.jl")

export
    Operation,
    construct_operation,
    get_o,
    get_s,
    preconditions,
    postconditions,
    duration

@with_kw struct Operation
    pre::Set{OBJECT_AT}     = Set{OBJECT_AT}()
    post::Set{OBJECT_AT}    = Set{OBJECT_AT}()
    Δt::Float64             = 0
    station_id::StationID   = StationID(-1)
end
function construct_operation(station_id, input_ids, output_ids, Δt)
    Operation(
        Set{OBJECT_AT}(map(i->OBJECT_AT(i,station_id), input_ids)),
        Set{OBJECT_AT}(map(i->OBJECT_AT(i,station_id), output_ids)),
        Δt,
        StationID(station_id)
    )
end
get_s(op::Operation) = op.station_id
duration(op::Operation) = op.Δt
preconditions(op::Operation) = op.pre
postconditions(op::Operation) = op.post

export
    TaskGraphProblemSpec

"""
    `TaskGraphProblemSpec{G}`

    Elements:
    - N::Int - num robots
    - M::Int - num tasks
    - graph::G - delivery graph
    - Drs::Matrix{Float64} - distance matrix robot initial locations -> pickup stations
    - Dss::Matrix{Float64} - distance matrix piickup stations -> dropoff stations
    - Δt::Vector{Float64}  - durations of operations
    - tr0_::Dict{Int,Float64} - robot start times
    - to0_::Dict{Int,Float64} - object start times
"""
@with_kw struct TaskGraphProblemSpec{G}
    N::Int                  = 0 # num robots
    M::Int                  = 0 # num tasks
    graph::G                = DiGraph() # delivery graph
    D::Matrix{Float64}      = zeros(0,0)
    Drs::Matrix{Float64}    = zeros(N+M,M) # distance matrix robot initial locations -> pickup stations
    Dss::Matrix{Float64}    = zeros(M,M) # distance matrix piickup stations -> dropoff stations
    Δt::Vector{Float64}     = Vector{Float64}() # durations of operations
    tr0_::Dict{Int,Float64} = Dict{Int,Float64}() # robot start times
    to0_::Dict{Int,Float64} = Dict{Int,Float64}() # object start times
end

export
    ProjectSpec,
    get_initial_nodes,
    get_input_ids,
    get_output_ids,
    add_operation!,
    get_duration_vector,
    read_project_spec

"""
    `ProjectSpec{G}`

    Defines a list of operations that must be performed in order to complete a
    specific project, in addition to the dependencies between those operations
"""
@with_kw struct ProjectSpec{G}
    initial_conditions::Dict{Int,OBJECT_AT} = Dict{Int,OBJECT_AT}()
    final_conditions::Dict{Int,OBJECT_AT} = Dict{Int,OBJECT_AT}()
    operations::Vector{Operation} = Vector{Operation}()
    pre_deps::Dict{Int,Set{Int}}  = Dict{Int,Set{Int}}() # id => (pre_conditions)
    post_deps::Dict{Int,Set{Int}} = Dict{Int,Set{Int}}()
    graph::G                      = MetaDiGraph()
    M::Int                        = -1
end
get_initial_nodes(tg::ProjectSpec) = setdiff(
    Set(collect(vertices(tg.graph))),collect(keys(tg.pre_deps)))
get_input_ids(op::Operation) = Set{Int}([get_id(get_o(p)) for p in op.pre])
get_output_ids(op::Operation) = Set{Int}([get_id(get_o(p)) for p in op.post])
get_pre_deps(tg::ProjectSpec, i::Int) = get(tg.pre_deps, i, Set{Int}())
get_post_deps(tg::ProjectSpec, i::Int) = get(tg.post_deps, i, Set{Int}())
get_num_delivery_tasks(spec::ProjectSpec) = length(collect(union(map(op->union(get_input_ids(op),get_output_ids(op)),spec.operations)...)))
function get_duration_vector(spec::P) where {P<:ProjectSpec}
    Δt = zeros(get_num_delivery_tasks(spec))
    for op in spec.operations
        for id in get_output_ids(op)
            Δt[id] = duration(op)
        end
    end
    Δt
end
function add_pre_dep!(tg::ProjectSpec, i::Int, op_idx::Int)
    push!(get!(tg.pre_deps, i, Set{Int}()),op_idx)
end
function add_post_dep!(tg::ProjectSpec, i::Int, op_idx::Int)
    push!(get!(tg.post_deps, i, Set{Int}()),op_idx)
end
function add_operation!(task_graph::ProjectSpec, op::Operation)
    G = task_graph.graph
    ops = task_graph.operations
    add_vertex!(G)
    push!(ops, op)
    op_id = length(ops)
    for id in get_output_ids(op)
        # object[id] is a product of operation[op_id]
        add_pre_dep!(task_graph, id, op_id)
        for op0_id in get_post_deps(task_graph, id)
            # for each operation that requires object[id]
            add_edge!(G, op_id, op0_id)
        end
    end
    for id in get_input_ids(op)
        # object[id] is a prereq for operation[op_id]
        add_post_dep!(task_graph, id, op_id)
        for op0_id in get_pre_deps(task_graph, id)
            # for each (1) operation that generates object[id]
            add_edge!(G, op0_id, op_id)
        end
    end
    task_graph
end
function construct_operation(spec::ProjectSpec, station_id, input_ids, output_ids, Δt)
    Operation(
        Set{OBJECT_AT}(map(o->get(spec.final_conditions, o, OBJECT_AT(o,station_id)), input_ids)),
        Set{OBJECT_AT}(map(o->get(spec.initial_conditions, o, OBJECT_AT(o,station_id)), output_ids)),
        Δt,
        StationID(station_id)
    )
end

# Some tools for writing and reading project specs
function TOML.parse(pred::OBJECT_AT)
    [get_id(get_o(pred)),get_id(get_s(pred))]
end
function TOML.parse(op::Operation)
    toml_dict = Dict()
    toml_dict["pre"] = map(pred->[get_id(get_o(pred)),get_id(get_s(pred))], collect(op.pre))
    toml_dict["post"] = map(pred->[get_id(get_o(pred)),get_id(get_s(pred))], collect(op.post))
    toml_dict["Δt"] = duration(op)
    return toml_dict
end
function TOML.parse(project_spec::ProjectSpec)
    toml_dict = Dict()
    toml_dict["title"]      = "ProjectSpec"
    toml_dict["operations"] = map(op->TOML.parse(op),project_spec.operations)
    toml_dict["initial_conditions"] = Dict(string(k)=>TOML.parse(pred) for (k,pred) in project_spec.initial_conditions)
    toml_dict["final_conditions"] = Dict(string(k)=>TOML.parse(pred) for (k,pred) in project_spec.final_conditions)
    toml_dict
end
function read_project_spec(io)
    toml_dict = TOML.parsefile(io)
    project_spec = ProjectSpec()
    for (k,arr) in toml_dict["initial_conditions"]
        object_id = arr[1]
        station_id = arr[2]
        project_spec.initial_conditions[object_id] = OBJECT_AT(object_id, station_id)
    end
    for (k,arr) in toml_dict["final_conditions"]
        object_id = arr[1]
        station_id = arr[2]
        project_spec.final_conditions[object_id] = OBJECT_AT(object_id, station_id)
    end
    for op_dict in toml_dict["operations"]
        op = Operation(
            pre     = Set{OBJECT_AT}(map(arr->OBJECT_AT(arr[1],arr[2]),op_dict["pre"])),
            post    = Set{OBJECT_AT}(map(arr->OBJECT_AT(arr[1],arr[2]),op_dict["post"])),
            Δt      = op_dict["Δt"]
            )
        add_operation!(project_spec, op)
    end
    return project_spec
end

export
    DeliveryTask,
    DeliveryGraph,
    construct_delivery_graph

"""
    `DeliveryTask`
"""
struct DeliveryTask
    o::Int
    s1::Int
    s2::Int
end

"""
    `DeliveryGraph{G}`
"""
struct DeliveryGraph{G}
    tasks::Vector{DeliveryTask}
    graph::G
end

"""
    `construct_delivery_graph(project_spec::ProjectSpec,M::Int)`

    Assumes that initial station ids correspond to object ids.
"""
function construct_delivery_graph(project_spec::ProjectSpec,M::Int)
    delivery_graph = DeliveryGraph(Vector{DeliveryTask}(),MetaDiGraph(M))
    for op in project_spec.operations
        # for i in get_input_ids(op)
        for pred in preconditions(op)
            i = get_id(get_o(pred))
            pre = project_spec.initial_conditions[i]
            post = project_spec.final_conditions[i]
            push!(delivery_graph.tasks,DeliveryTask(i,get_id(get_s(pre)),get_id(get_s(post))))
            for j in get_output_ids(op)
                add_edge!(delivery_graph.graph, i, j)
                # push!(delivery_graph.tasks,DeliveryTask(i,i,j))
            end
        end
    end
    delivery_graph
end

export
    PathSpec,
    ProjectSchedule,
    get_graph,
    get_object_ICs,
    get_object_FCs,
    get_robot_ICs,
    get_actions,
    get_operations,
    get_num_actions,
    get_num_operations,
    get_num_object_ICs,
    get_num_robot_ICs,
    get_completion_time,
    get_duration,
    get_vtx,
    add_to_schedule!,
    construct_project_schedule,
    process_schedule


@with_kw struct PathSpec
    # element             ::T             = nothing
    op_duration         ::Float64       = 0.0
    # agent_id            ::Int           = -1
    # path_id             ::Int           = -1
    # object_id           ::Int           = -1
    start_vtx           ::Int           = -1
    final_vtx           ::Int           = -1
    min_path_duration   ::Int           = 0.0
    # slack               ::Int           = 0.0
    # deadline            ::Int           = 0.0
end

"""
    `ProjectSchedule`
"""
@with_kw struct ProjectSchedule{G<:AbstractGraph}
    graph               ::G                 = MetaDiGraph()
    object_ICs          ::Dict{Int,OBJECT_AT} = Dict{Int,OBJECT_AT}()
    # object_FCs          ::Dict{Int,OBJECT_AT} = Dict{Int,OBJECT_AT}()
    robot_ICs           ::Dict{Int,ROBOT_AT}  = Dict{Int,ROBOT_AT}()
    # robot_FCs           ::Dict{Int,ROBOT_AT}  = Dict{Int,ROBOT_AT}()
    actions             ::Dict{Int,AbstractRobotAction} = Dict{Int,AbstractRobotAction}()
    operations          ::Dict{Int,Operation} = Dict{Int,Operation}()
    #
    completion_times    ::Vector{Float64} = Vector{Float64}()
    durations           ::Vector{Float64} = Vector{Float64}()
    completed           ::Vector{Bool}    = Vector{Bool}()
    path_specs          ::Vector{PathSpec}= Vector{PathSpec}()
    #
    object_vtx_map      ::Dict{Int,Int}   = Dict{Int,Int}()
    robot_vtx_map       ::Dict{Int,Int}   = Dict{Int,Int}()
    operation_vtx_map   ::Dict{Int,Int}   = Dict{Int,Int}()
    action_vtx_map      ::Dict{Int,Int}   = Dict{Int,Int}()
end
get_graph(schedule::P) where {P<:ProjectSchedule} = schedule.graph
get_object_ICs(schedule::P) where {P<:ProjectSchedule} = schedule.object_ICs
# get_object_FCs(schedule::P) where {P<:ProjectSchedule} = schedule.object_FCs
get_robot_ICs(schedule::P) where {P<:ProjectSchedule} = schedule.robot_ICs
# get_robot_FCs(schedule::P) where {P<:ProjectSchedule} = schedule.robot_FCs
get_actions(schedule::P) where {P<:ProjectSchedule} = schedule.actions
get_operations(schedule::P) where {P<:ProjectSchedule} = schedule.operations
get_num_actions(schedule::P) where {P<:ProjectSchedule} = length(get_actions(schedule))
get_num_operations(schedule::P) where {P<:ProjectSchedule} = length(get_operations(schedule))
get_num_object_ICs(schedule::P) where {P<:ProjectSchedule} = length(get_object_ICs(schedule))
get_num_robot_ICs(schedule::P) where {P<:ProjectSchedule} = length(get_robot_ICs(schedule))
get_completion_time(schedule::ProjectSchedule,i::Int) = schedule.completion_times[i]
function set_completion_time!(schedule::ProjectSchedule,i::Int,t::Int)
    schedule.completion_times[i] = t
end
get_duration(schedule::ProjectSchedule,i::Int) = schedule.durations[i]
const DEFAULT_ORDER = (:objects,:robots,:operations,:actions)
get_vtx(schedule::ProjectSchedule,i::ObjectID)      = get(schedule.object_vtx_map,      get_id(i), -1)
get_vtx(schedule::ProjectSchedule,i::RobotID)       = get(schedule.robot_vtx_map,       get_id(i), -1)
get_vtx(schedule::ProjectSchedule,i::ActionID)      = get(schedule.action_vtx_map,      get_id(i), -1)
get_vtx(schedule::ProjectSchedule,i::OperationID)   = get(schedule.operation_vtx_map,   get_id(i), -1)

function get_path_spec(schedule::P,spec::T,a::GO) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec,A<:AbstractRobotAction}
    r = get_id(get_r(a))
    s = get_id(get_s(a))
    robot_pred = get_robot_ICs(schedule)[r]
    s0 = get_id(get_s(robot_pred))
    PathSpec(
        op_duration=0.0,
        start_vtx=s0,
        final_vtx=s,
        min_path_duration=spec.D[s0,s] # TaskGraphProblemSpec distance matrix
        )
end
function get_path_spec(schedule::P,spec::T,a::CARRY) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec,A<:AbstractRobotAction}
    r = get_id(get_r(a))
    o = get_id(get_o(a))
    s = get_id(get_s(a))
    object_pred = get_object_ICs(schedule)[o]
    s0 = get_id(get_s(object_pred))
    PathSpec(
        op_duration=0.0,
        start_vtx=s0,
        final_vtx=s,
        min_path_duration=spec.D[s0,s] # TaskGraphProblemSpec distance matrix
        )
end
function get_path_spec(schedule::P,spec::T,a::COLLECT) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec,A<:AbstractRobotAction}
    r = get_id(get_r(a))
    o = get_id(get_o(a))
    s = get_id(get_s(a))
    PathSpec(
        op_duration=0.0,
        start_vtx=s,
        final_vtx=s,
        min_path_duration=0.0
        )
end
function get_path_spec(schedule::P,spec::T,a::DEPOSIT) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec,A<:AbstractRobotAction}
    r = get_id(get_r(a))
    o = get_id(get_o(a))
    s = get_id(get_s(a))
    object_pred = get_object_ICs(schedule)[o]
    s0 = get_id(get_s(object_pred))
    PathSpec(
        op_duration=0.0,
        start_vtx=s,
        final_vtx=s,
        min_path_duration=0.0
        )
end
function add_to_schedule!(schedule::P,spec::T,pred::OBJECT_AT,id::ObjectID) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    @assert get_vtx(schedule, id) == -1
    graph = get_graph(schedule)
    add_vertex!(graph)
    push!(schedule.path_specs, PathSpec(
        op_duration=0.0,
        start_vtx=get_id(get_s(pred)),
        final_vtx=get_id(get_s(pred)),
        min_path_duration=0.0
        ))
    push!(schedule.completion_times, 0.0)
    push!(schedule.durations, 0.0)
    set_props!(graph,nv(graph),Dict(:vtype=>:object_ic,:id=>id,:pred=>pred))
    get_object_ICs(schedule)[get_id(id)] = pred
    schedule.object_vtx_map[get_id(id)] = nv(graph)
    schedule
end
function add_to_schedule!(schedule::P,spec::T,pred::ROBOT_AT,id::RobotID) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    @assert get_vtx(schedule, id) == -1
    graph = get_graph(schedule)
    add_vertex!(graph)
    push!(schedule.path_specs, PathSpec(
        op_duration=0.0,
        start_vtx=get_id(get_s(pred)),
        final_vtx=get_id(get_s(pred)),
        min_path_duration=0.0
        ))
    push!(schedule.completion_times, 0.0)
    push!(schedule.durations, 0.0)
    set_props!(graph,nv(graph),Dict(:vtype=>:robot_ic,:id=>id,:pred=>pred))
    get_robot_ICs(schedule)[get_id(id)] = pred
    schedule.robot_vtx_map[get_id(id)] = nv(graph)
    schedule
end
function add_to_schedule!(schedule::P,spec::T,op::Operation,id::OperationID) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    @assert get_vtx(schedule, id) == -1
    graph = get_graph(schedule)
    add_vertex!(graph)
    push!(schedule.path_specs, PathSpec(
        op_duration=duration(op),
        start_vtx=-1,
        final_vtx=-1,
        min_path_duration=0.0
        ))
    push!(schedule.completion_times, 0.0)
    push!(schedule.durations, 0.0)
    set_props!(graph,nv(graph),Dict(:vtype=>:operation,:id=>id,:op=>op))
    schedule.operations[get_id(id)] = op
    schedule.operation_vtx_map[get_id(id)] = nv(graph)
    schedule
end
function add_to_schedule!(schedule::P,spec::T,a::A,id::ActionID) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec,A<:AbstractRobotAction}
    @assert get_vtx(schedule, id) == -1
    graph = get_graph(schedule)
    add_vertex!(graph)
    push!(schedule.completion_times, 0.0)
    push!(schedule.durations, 0.0)
    push!(schedule.path_specs, get_path_spec(schedule,spec,a))
    set_props!(graph,nv(graph),Dict(:vtype=>:action,:id=>id,:a=>a))
    schedule.actions[get_id(id)] = a
    schedule.action_vtx_map[get_id(id)] = nv(graph)
    schedule
end
function LightGraphs.add_edge!(schedule::P,id1::A,id2::B) where {P<:ProjectSchedule,A<:AbstractID,B<:AbstractID}
    success = add_edge!(get_graph(schedule), get_vtx(schedule,id1), get_vtx(schedule,id2))
    # @show success, get_vtx(schedule,id1), get_vtx(schedule,id2)
    schedule
end

"""
    `construct_project_schedule`

    Args:
    - `spec` - a ProjectSpec
    - `object_ICs` - a list of initial object locations
    - `robot_ICs` - a list of initial robot locations
    - `assignments` - a list of robot assignments. `assignments[i] == j` means
    that robot `i` is assigned to transport object `j`
"""
function construct_project_schedule(
    project_spec::P,
    problem_spec::T,
    object_ICs::Dict{Int,OBJECT_AT},
    object_FCs::Dict{Int,OBJECT_AT},
    robot_ICs::Dict{Int,ROBOT_AT},
    assignments::V=Dict{Int,Int}()
    ) where {P<:ProjectSpec,T<:TaskGraphProblemSpec,V<:Union{Dict{Int,Int},Vector{Int}}}
    schedule = ProjectSchedule()
    graph = get_graph(schedule)
    # add object ICs to graph
    for (id, pred) in object_ICs
        add_to_schedule!(schedule, problem_spec, pred, get_o(pred))
    end
    # add robot ICs to graph
    for (id, pred) in robot_ICs
        add_to_schedule!(schedule, problem_spec, pred, get_r(pred))
    end
    M = length(object_ICs) # number of objects
    N = length(robot_ICs) - M # number of robots
    # add operations to graph
    for op_vtx in topological_sort(project_spec.graph)
        op = project_spec.operations[op_vtx]
        operation_id = OperationID(get_num_operations(schedule) + 1)
        add_to_schedule!(schedule, problem_spec, op, operation_id)
        for object_id in get_input_ids(op)
            # add action sequence
            robot_id = assignments[object_id]
            robot_pred = get_robot_ICs(schedule)[robot_id]
            robot_start_station = get_id(get_s(robot_pred))

            object_ic = get_object_ICs(schedule)[object_id]
            pickup_station_id = get_id(get_s(object_ic))

            object_fc = object_FCs[object_id]
            dropoff_station_id = get_id(get_s(object_fc))

            action_id = ActionID(get_num_actions(schedule) + 1)
            add_to_schedule!(schedule, problem_spec, GO(robot_id, pickup_station_id), action_id)
            add_edge!(schedule, RobotID(robot_id), action_id)

            action_id += 1
            add_to_schedule!(schedule, problem_spec, COLLECT(robot_id, object_id, pickup_station_id), action_id)
            add_edge!(schedule, action_id-1, action_id)
            add_edge!(schedule, ObjectID(object_id), action_id)

            action_id += 1
            add_to_schedule!(schedule, problem_spec, CARRY(robot_id, object_id, dropoff_station_id), action_id)
            add_edge!(schedule, action_id-1, action_id)

            action_id += 1
            add_to_schedule!(schedule, problem_spec, DEPOSIT(robot_id, object_id, dropoff_station_id), action_id)
            add_edge!(schedule, action_id-1, action_id)

            new_robot_id = object_id + N
            add_edge!(schedule, action_id, RobotID(new_robot_id))
            add_edge!(schedule, action_id, operation_id)
        end
        for object_id in get_output_ids(op)
            # robot_id = assignments[object_id]
            # robot_pred = get_robot_ICs(schedule)[object_id]
            # object_pred = get_object_ICs(schedule)[object_id]

            # add_to_schedule!(schedule, object_pred, object_id)
            add_edge!(schedule, operation_id, ObjectID(object_id))
        end
    end
    return schedule
end

"""
    `process_schedule(schedule::P) where {P<:ProjectSchedule}`

    Compute the optimistic start and end times, along with the slack associated
    with each vertex in the `schedule`.
"""
function process_schedule(schedule::P) where {P<:ProjectSchedule}
    solution_graph = schedule.graph
    traversal = topological_sort_by_dfs(solution_graph)
    t0 = zeros(nv(solution_graph))
    tF = zeros(nv(solution_graph))
    slack = zeros(nv(solution_graph))
    local_slack = zeros(nv(solution_graph))
    # Compute Lower Bounds Via Forward Dynamic Programming pass
    for v in traversal
        path_spec = schedule.path_specs[v]
        Δt = path_spec.op_duration
        for v2 in inneighbors(solution_graph,v)
            t0[v] = max(t0[v], tF[v2] + Δt)
        end
        tF[v] = t0[v] + path_spec.min_path_duration
    end
    # Compute Slack Via Backward Dynamic Programming pass
    for v in reverse(traversal)
        for v2 in inneighbors(solution_graph,v)
            local_slack[v2] = t0[v] - tF[v2]
            slack[v2]       = slack[v] + local_slack[v2]
        end
    end
    t0,tF,slack,local_slack
end

include("utils.jl")

end
