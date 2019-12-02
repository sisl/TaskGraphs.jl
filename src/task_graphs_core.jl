module TaskGraphsCore

using Parameters
using LightGraphs, MetaGraphs
using GraphUtils
using DataStructures
using JuMP
using Gurobi
using TOML

using ..PlanningPredicates


export
    EnvironmentState

@with_kw struct EnvironmentState
    robot_states::Vector{ROBOT_AT}      = Vector{ROBOT_AT}()
    object_states::Vector{OBJECT_AT}    = Vector{OBJECT_AT}()
    object_id_map::Dict{ObjectID,Int}   = Dict{ObjectID,Int}(get_object_id(o)=>i for (i,o) in enumerate(object_states))
end


export
    TaskGraphProblemSpec

"""
    `TaskGraphProblemSpec{G}`

    This containts all information necessary to formulate the Task Assignment
    problem as a MILP.

    Elements:
    - N::Int - num robots
    - M::Int - num tasks
    - graph::G - delivery graph
    - Drs::Matrix{Float64} - distance matrix robot initial locations -> pickup stations
    - Dss::Matrix{Float64} - distance matrix piickup stations -> dropoff stations
    - Δt::Vector{Float64}  - durations of operations
    - tr0_::Dict{Int,Float64} - robot start times
    - to0_::Dict{Int,Float64} - object start times
    - root_nodes::Vector{Set{Int}} - identifies "project heads"
    - weights::Dict{Int,Float64} - stores weights associated with each project head
    - s0::Vector{Int} - pickup stations for each task
    - sF::Vector{Int} - delivery station for each task
    - nR::Vector{Int} - num robots required for each task (>1 => collaborative task)
"""
@with_kw struct TaskGraphProblemSpec{G}
    N::Int                  = 0 # num robots
    M::Int                  = 0 # num tasks
    graph::G                = DiGraph() # delivery graph
    D::Matrix{Float64}      = zeros(0,0) # full distance matrix
    Drs::Matrix{Float64}    = zeros(N+M,M) # distance matrix robot initial locations -> pickup stations
    Dss::Matrix{Float64}    = zeros(M,M) # distance matrix pickup stations -> dropoff stations
    Δt::Vector{Float64}     = Vector{Float64}() # durations of operations
    Δt_collect::Vector{Float64} = zeros(M) # duration of COLLECT operations
    Δt_deliver::Vector{Float64} = zeros(M) # duration of DELIVER operations
    tr0_::Dict{Int,Float64} = Dict{Int,Float64}() # robot start times
    to0_::Dict{Int,Float64} = Dict{Int,Float64}() # object start times
    root_nodes::Vector{Set{Int}} = [get_all_root_nodes(graph)]
    weights::Dict{Int,Float64} = Dict{Int,Float64}(v=>1.0 for v in 1:length(root_nodes))
    s0::Vector{Int} = zeros(M) # pickup stations for each task
    sF::Vector{Int} = zeros(M) # delivery station for each task
    nR::Vector{Int} = ones(M) # num robots required for each task (>1 => collaborative task)
end

export
    ProjectSpec,
    get_initial_nodes,
    get_input_ids,
    get_output_ids,
    add_operation!,
    get_duration_vector,
    read_project_spec,
    construct_operation

"""
    `ProjectSpec{G}`

    Defines a list of operations that must be performed in order to complete a
    specific project, in addition to the dependencies between those operations

    Elements:
    - initial_conditions::Vector{OBJECT_AT} - maps object id to initial condition predicate
    - final_conditions::Vector{OBJECT_AT} - maps object id to final condition predicate
    - operations::Vector{Operation} - list of manufacturing operations
    - pre_deps::Dict{Int,Set{Int}} - maps object id to ids of operations that are required to produce that object
    - post_deps::Dict{Int,Set{Int}} - maps object id to ids of operations that depend on that object
    - graph::G
    - root_nodes::Set{Int}
    - weights::Dict{Int,Float64}
    - M::Int
    - weight::Float64
    - object_id_to_idx::Dict{Int,Int}
"""
@with_kw struct ProjectSpec{G}
    # initial state
    initial_conditions::Vector{OBJECT_AT} = Vector{OBJECT_AT}()
    final_conditions::Vector{OBJECT_AT} = Vector{OBJECT_AT}()
    # project specifications
    operations::Vector{Operation} = Vector{Operation}()
    pre_deps::Dict{Int,Set{Int}}  = Dict{Int,Set{Int}}() # id => (pre_conditions)
    post_deps::Dict{Int,Set{Int}} = Dict{Int,Set{Int}}()
    graph::G                      = MetaDiGraph()
    root_nodes::Set{Int}          = Set{Int}()
    weights::Dict{Int,Float64}    = Dict{Int,Float64}(v=>1.0 for v in root_nodes)
    weight::Float64               = 1.0
    M::Int                        = length(initial_conditions)
    object_id_to_idx::Dict{Int,Int} = Dict{Int,Int}(get_id(get_object_id(id))=>k for (k,id) in enumerate(initial_conditions))
    op_id_to_vtx::Dict{Int,Int}    = Dict{Int,Int}()
end
get_initial_nodes(spec::ProjectSpec) = setdiff(
    Set(collect(vertices(spec.graph))),collect(keys(spec.pre_deps)))
get_input_ids(op::Operation) = Set{Int}([get_id(get_object_id(p)) for p in op.pre])
get_output_ids(op::Operation) = Set{Int}([get_id(get_object_id(p)) for p in op.post])
get_pre_deps(spec::ProjectSpec, op_id::Int) = get(spec.pre_deps, op_id, Set{Int}())
get_post_deps(spec::ProjectSpec, op_id::Int) = get(spec.post_deps, op_id, Set{Int}())
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
function add_pre_dep!(spec::ProjectSpec, object_id::Int, op_id::Int)
    # operation[op_id] is a prereq of object[object_id]
    push!(get!(spec.pre_deps, object_id, Set{Int}()),op_id)
end
function add_post_dep!(spec::ProjectSpec, object_id::Int, op_id::Int)
    # operation[op_id] is a postreq of object[object_id]
    push!(get!(spec.post_deps, object_id, Set{Int}()),op_id)
end
function add_operation!(spec::ProjectSpec, op::Operation)
    G = spec.graph
    ops = spec.operations
    add_vertex!(G)
    push!(ops, op)
    spec.op_id_to_vtx[get_id(op)] = nv(G)
    op_id = length(ops)
    for object_id in get_output_ids(op)
        # object[id] is a product of operation[op_id]
        add_pre_dep!(spec, object_id, op_id)
        for op0_id in get_post_deps(spec, object_id)
            # for each operation that requires object[id]
            add_edge!(G, op_id, op0_id)
        end
    end
    for object_id in get_input_ids(op)
        # object[id] is a prereq for operation[op_id]
        add_post_dep!(spec, object_id, op_id)
        for op0_id in get_pre_deps(spec, object_id)
            # for each (1) operation that generates object[id]
            add_edge!(G, op0_id, op_id)
        end
    end
    # set spec.root_nodes = get_all_root_nodes(spec.graph)
    union!(spec.root_nodes, get_all_root_nodes(spec.graph))
    intersect!(spec.root_nodes, get_all_root_nodes(spec.graph))
    spec
end
function construct_operation(spec::ProjectSpec, station_id, input_ids, output_ids, Δt, id=OperationID(get_unique_operation_id()))
    Operation(
        pre = Set{OBJECT_AT}(map(id->get(spec.final_conditions, spec.object_id_to_idx[id], OBJECT_AT(id,station_id)), input_ids)),
        post = Set{OBJECT_AT}(map(id->get(spec.initial_conditions, spec.object_id_to_idx[id], OBJECT_AT(id,station_id)), output_ids)),
        Δt = Δt,
        station_id = StationID(station_id),
        id = id
    )
end

# Some tools for writing and reading project specs
function TOML.parse(pred::OBJECT_AT)
    [get_id(get_object_id(pred)),get_id(get_location_id(pred))]
end
function TOML.parse(op::Operation)
    toml_dict = Dict()
    toml_dict["pre"] = map(pred->[get_id(get_object_id(pred)),get_id(get_location_id(pred))], collect(op.pre))
    toml_dict["post"] = map(pred->[get_id(get_object_id(pred)),get_id(get_location_id(pred))], collect(op.post))
    toml_dict["dt"] = duration(op)
    toml_dict["station_id"] = get_id(op.station_id)
    toml_dict["id"] = get_id(op.id)
    return toml_dict
end
function read_operation(toml_dict::Dict)
    op = Operation(
        pre     = Set{OBJECT_AT}(map(arr->OBJECT_AT(arr[1],arr[2]),toml_dict["pre"])),
        post    = Set{OBJECT_AT}(map(arr->OBJECT_AT(arr[1],arr[2]),toml_dict["post"])),
        Δt      = toml_dict["dt"],
        station_id = StationID(toml_dict["station_id"]),
        id = OperationID(toml_dict["id"])
        )
end
function TOML.parse(project_spec::ProjectSpec)
    toml_dict = Dict()
    toml_dict["operations"] = map(op->TOML.parse(op),project_spec.operations)
    # toml_dict["initial_conditions"] = Dict(string(k)=>TOML.parse(pred) for (k,pred) in project_spec.initial_conditions)
    # toml_dict["final_conditions"] = Dict(string(k)=>TOML.parse(pred) for (k,pred) in project_spec.final_conditions)
    toml_dict["initial_conditions"] = map(pred->TOML.parse(pred), project_spec.initial_conditions)
    toml_dict["final_conditions"] = map(pred->TOML.parse(pred), project_spec.final_conditions)
    toml_dict
end
function read_project_spec(toml_dict::Dict)
    project_spec = ProjectSpec()
    # for (k,arr) in toml_dict["initial_conditions"]
    #     object_id = arr[1]
    #     station_id = arr[2]
    #     project_spec.initial_conditions[object_id] = OBJECT_AT(object_id, station_id)
    # end
    for (i,arr) in enumerate(toml_dict["initial_conditions"])
        object_id = arr[1]
        station_id = arr[2]
        push!(project_spec.initial_conditions, OBJECT_AT(object_id, station_id))
        project_spec.object_id_to_idx[object_id] = i
    end
    # for (k,arr) in toml_dict["final_conditions"]
    #     object_id = arr[1]
    #     station_id = arr[2]
    #     project_spec.final_conditions[object_id] = OBJECT_AT(object_id, station_id)
    # end
    for arr in toml_dict["final_conditions"]
        object_id = arr[1]
        station_id = arr[2]
        push!(project_spec.final_conditions, OBJECT_AT(object_id, station_id))
    end
    for op_dict in toml_dict["operations"]
        op = read_operation(op_dict)
        add_operation!(project_spec, op)
    end
    return project_spec
end
function read_project_spec(io)
    read_project_spec(TOML.parsefile(io))
end

export
    SimpleProblemDef,
    read_problem_def

struct SimpleProblemDef
    project_spec::ProjectSpec
    r0::Vector{Int}
    s0::Vector{Int}
    sF::Vector{Int}
end

function TOML.parse(def::SimpleProblemDef)
    toml_dict = TOML.parse(def.project_spec)
    toml_dict["r0"] = def.r0
    toml_dict["s0"] = def.s0
    toml_dict["sF"] = def.sF
    toml_dict
end
function read_problem_def(toml_dict::Dict)
    SimpleProblemDef(
        read_project_spec(toml_dict),
        toml_dict["r0"],
        toml_dict["s0"],
        toml_dict["sF"]
    )
end
function read_problem_def(io)
    read_problem_def(TOML.parsefile(io))
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
            id = get_id(get_object_id(pred))
            idx = project_spec.object_id_to_idx[id]
            pre = project_spec.initial_conditions[idx]
            post = project_spec.final_conditions[idx]
            push!(delivery_graph.tasks,DeliveryTask(id,get_id(get_location_id(pre)),get_id(get_location_id(post))))
            for j in get_output_ids(op)
                idx2 = project_spec.object_id_to_idx[j]
                add_edge!(delivery_graph.graph, idx, idx2)
                # push!(delivery_graph.tasks,DeliveryTask(i,i,j))
            end
        end
    end
    delivery_graph
end

export
    PathSpec,
    generate_path_spec

@with_kw struct PathSpec
    # element             ::T             = nothing
    # op_duration         ::Int           = 0 # time between end of predecessor and start of current
    node_type           ::Symbol        = :EMPTY
    start_vtx           ::Int           = -1
    final_vtx           ::Int           = -1
    min_path_duration   ::Int           =  0 # duration

    agent_id            ::Int           = -1
    dummy_id            ::Int           = -1
    path_id             ::Int           = -1
    object_id           ::Int           = -1
    # tight==true => planning goal time must be tight to beginning of successors
    # (local slack == 0). E.g., GO nodes must not end before COLLECT can begin,
    # because then we have empty time between planning phases
    tight               ::Bool          = false
    # static==true => the robot must remain in place for this planning phase (
    # e.g., COLLECT, DEPOSIT)
    static              ::Bool          = false
    # (free==true) && is_terminal_node => the planning goal_time must go on until
    # all non-free nodes are completed
    free                ::Bool          = false
    # slack               ::Int           = 0.0
    # deadline            ::Int           = 0.0
end

export
    ProjectSchedule,
    get_graph,
    get_object_ICs,
    get_object_FCs,
    get_robot_ICs,
    get_actions,
    get_operations,
    get_vtx_ids,
    get_node_from_id,
    get_num_actions,
    get_num_operations,
    get_num_object_ICs,
    get_num_robot_ICs,
    get_num_vtxs,
    get_num_paths,
    get_next_path_id,
    get_completion_time,
    get_duration,
    get_vtx,
    get_vtx_id,
    replace_in_schedule!,
    add_to_schedule!,
    construct_project_schedule,
    process_schedule

# abstract type AbstractProjectSchedule end
# struct CompositeProjectSchedule <: AbstractProjectSchedule
#     schedules::Vector{ProjectSchedule}
#     weights::Vector{Float64}
# end
"""
    `ProjectSchedule`
"""
@with_kw struct ProjectSchedule{G<:AbstractGraph}# <: AbstractProjectSchedule
    graph               ::G                 = MetaDiGraph()
    object_ICs          ::Dict{Int,OBJECT_AT} = Dict{Int,OBJECT_AT}()
    robot_ICs           ::Dict{Int,ROBOT_AT}  = Dict{Int,ROBOT_AT}()
    actions             ::Dict{Int,AbstractRobotAction} = Dict{Int,AbstractRobotAction}()
    operations          ::Dict{Int,Operation} = Dict{Int,Operation}()
    #
    path_specs          ::Vector{PathSpec}= Vector{PathSpec}()
    #
    robot_id_map        ::Dict{Int,Int}   = Dict{Int,Int}() # maps dummy id to true id
    root_nodes          ::Vector{Int}     = Vector{Int}() # list of "project heads"
    weights             ::Dict{Int,Float64} = Dict{Int,Float64}() # weights corresponding to project heads
    path_id_to_vtx_map  ::Dict{Int,Int}   = Dict{Int,Int}() # maps path_id to vertex
    vtx_ids             ::Vector{AbstractID} = Vector{AbstractID}() # maps vertex to actual graph node
    object_vtx_map      ::Dict{Int,Int}   = Dict{Int,Int}()
    robot_vtx_map       ::Dict{Int,Int}   = Dict{Int,Int}()
    operation_vtx_map   ::Dict{Int,Int}   = Dict{Int,Int}()
    action_vtx_map      ::Dict{Int,Int}   = Dict{Int,Int}()
end
get_graph(schedule::P) where {P<:ProjectSchedule}       = schedule.graph
get_object_ICs(schedule::P) where {P<:ProjectSchedule}  = schedule.object_ICs
get_robot_ICs(schedule::P) where {P<:ProjectSchedule}   = schedule.robot_ICs
get_actions(schedule::P) where {P<:ProjectSchedule}     = schedule.actions
get_operations(schedule::P) where {P<:ProjectSchedule}  = schedule.operations
get_vtx_ids(schedule::P) where {P<:ProjectSchedule}     = schedule.vtx_ids
get_root_nodes(schedule::P) where {P<:ProjectSchedule}  = schedule.root_nodes
get_root_node_weights(schedule::P) where {P<:ProjectSchedule}  = schedule.weights

get_node_from_id(schedule::P,id::ActionID) where {P<:ProjectSchedule}       = get_actions(schedule)[get_id(id)]
get_node_from_id(schedule::P,id::OperationID) where {P<:ProjectSchedule}    = get_operations(schedule)[get_id(id)]
get_node_from_id(schedule::P,id::ObjectID) where {P<:ProjectSchedule}       = get_object_ICs(schedule)[get_id(id)]
get_node_from_id(schedule::P,id::RobotID) where {P<:ProjectSchedule}        = get_robot_ICs(schedule)[get_id(id)]

get_vtx(schedule::ProjectSchedule,i::ObjectID)      = get(schedule.object_vtx_map,      get_id(i), -1)
get_vtx(schedule::ProjectSchedule,i::RobotID)       = get(schedule.robot_vtx_map,       get_id(i), -1)
get_vtx(schedule::ProjectSchedule,i::ActionID)      = get(schedule.action_vtx_map,      get_id(i), -1)
get_vtx(schedule::ProjectSchedule,i::OperationID)   = get(schedule.operation_vtx_map,   get_id(i), -1)
get_vtx_id(schedule::P,v::Int) where {P<:ProjectSchedule} = schedule.vtx_ids[v]

get_num_actions(schedule::P) where {P<:ProjectSchedule}     = length(get_actions(schedule))
get_num_operations(schedule::P) where {P<:ProjectSchedule}  = length(get_operations(schedule))
get_num_object_ICs(schedule::P) where {P<:ProjectSchedule}  = length(get_object_ICs(schedule))
get_num_robot_ICs(schedule::P) where {P<:ProjectSchedule}   = length(get_robot_ICs(schedule))
get_num_vtxs(schedule::P) where {P<:ProjectSchedule}        = nv(get_graph(schedule))
get_num_paths(schedule::P) where {P<:ProjectSchedule}       = get_num_actions(schedule) + get_num_robot_ICs(schedule)

get_next_path_id(schedule::P) where {P<:ProjectSchedule}    = length(schedule.path_id_to_vtx_map) + 1

export
    get_path_spec,
    set_path_spec!,
    add_path_spec!

get_path_spec(schedule::P,v::Int) where {P<:ProjectSchedule} = schedule.path_specs[v]
function set_path_spec!(schedule::P,v::Int,spec::S) where {P<:ProjectSchedule,S<:PathSpec}
    schedule.path_specs[v] = spec
    schedule.path_id_to_vtx_map[spec.path_id] = v
end
function add_path_spec!(schedule::P,spec::S) where {P<:ProjectSchedule,S<:PathSpec}
    push!(schedule.path_specs, spec)
    if spec.path_id != -1
        schedule.path_id_to_vtx_map[spec.path_id] = nv(get_graph(schedule))
    end
    # push!(schedule.completion_times, 0.0)
    # push!(schedule.durations, 0.0)
end

export
    set_vtx_map!,
    insert_to_vtx_map!

function set_vtx_map!(schedule::P,pred::OBJECT_AT,id::ObjectID,idx::Int) where {P<:ProjectSchedule}
    get_object_ICs(schedule)[get_id(id)] = pred
    schedule.object_vtx_map[get_id(id)] = idx
end
function set_vtx_map!(schedule::P,pred::ROBOT_AT,id::RobotID,idx::Int) where {P<:ProjectSchedule}
    get_robot_ICs(schedule)[get_id(id)] = pred
    schedule.robot_vtx_map[get_id(id)] = idx
end
function set_vtx_map!(schedule::P,op::Operation,id::OperationID,idx::Int) where {P<:ProjectSchedule}
    get_operations(schedule)[get_id(id)] = op
    schedule.operation_vtx_map[get_id(id)] = idx
end
function set_vtx_map!(schedule::P,a::A,id::ActionID,idx::Int) where {P<:ProjectSchedule,A<:AbstractRobotAction}
    get_actions(schedule)[get_id(id)] = a
    schedule.action_vtx_map[get_id(id)] = idx
end
function insert_to_vtx_map!(schedule::P,pred,id::ID,idx::Int) where {P<:ProjectSchedule,ID<:AbstractID}
    push!(schedule.vtx_ids, id)
    set_vtx_map!(schedule,pred,id,idx)
end

"""
    PathSpec provides details about the path that corresponds to this node in
    the schedule
"""
function generate_path_spec(schedule::P,spec::T,a::GO) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    s0 = get_id(get_initial_location_id(a))
    s = get_id(get_destination_location_id(a))
    r = get_id(get_robot_id(a))
    path_spec = PathSpec(
        node_type=Symbol(typeof(a)),
        start_vtx=s0,
        final_vtx=s,
        # min_path_duration=get(spec.D,(s0,s),typemax(Int)), # TaskGraphProblemSpec distance matrix
        min_path_duration=get(spec.D,(s0,s),0), # TaskGraphProblemSpec distance matrix
        path_id=get_next_path_id(schedule),
        dummy_id=r,
        agent_id=get(schedule.robot_id_map,r,-1),
        tight=true
        )
end
function generate_path_spec(schedule::P,spec::T,a::CARRY) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    s0 = get_id(get_initial_location_id(a))
    s = get_id(get_destination_location_id(a))
    r = get_id(get_robot_id(a))
    o = get_id(get_object_id(a))
    path_spec = PathSpec(
        node_type=Symbol(typeof(a)),
        start_vtx=s0,
        final_vtx=s,
        # min_path_duration=get(spec.D,(s0,s),typemax(Int)), # TaskGraphProblemSpec distance matrix
        min_path_duration=get(spec.D,(s0,s),0), # TaskGraphProblemSpec distance matrix
        path_id=get_next_path_id(schedule),
        dummy_id=r,
        agent_id=get(schedule.robot_id_map,r,-1),
        object_id = o
        )
end
function generate_path_spec(schedule::P,spec::T,a::COLLECT) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    s0 = get_id(get_initial_location_id(a))
    s = get_id(get_destination_location_id(a))
    r = get_id(get_robot_id(a))
    o = get_id(get_object_id(a))
    path_spec = PathSpec(
        node_type=Symbol(typeof(a)),
        start_vtx=s0,
        final_vtx=s,
        # min_path_duration=get(spec.Δt_collect,o,typemax(Int)),
        min_path_duration=get(spec.Δt_collect,o,0),
        path_id=get_next_path_id(schedule),
        dummy_id=r,
        agent_id=get(schedule.robot_id_map,r,-1),
        object_id = o,
        static=true
        )
end
function generate_path_spec(schedule::P,spec::T,a::DEPOSIT) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    s0 = get_id(get_initial_location_id(a))
    s = get_id(get_destination_location_id(a))
    r = get_id(get_robot_id(a))
    o = get_id(get_object_id(a))
    path_spec = PathSpec(
        node_type=Symbol(typeof(a)),
        start_vtx=s0,
        final_vtx=s,
        # min_path_duration=get(spec.Δt_deliver,o,typemax(Int)),
        min_path_duration=get(spec.Δt_deliver,o,0),
        path_id=get_next_path_id(schedule),
        dummy_id=r,
        agent_id=get(schedule.robot_id_map,r,-1),
        object_id = o,
        static=true
        )
end
function generate_path_spec(schedule::P,spec::T,pred::OBJECT_AT) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    path_spec = PathSpec(
        node_type=Symbol(typeof(pred)),
        start_vtx=get_id(get_location_id(pred)),
        final_vtx=get_id(get_location_id(pred)),
        min_path_duration=0,
        object_id = get_id(get_object_id(pred))
        )
end
function generate_path_spec(schedule::P,spec::T,pred::ROBOT_AT) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    r = get_id(get_robot_id(pred))
    if !(r in collect(keys(schedule.robot_id_map)))
        println("generate_path_spec: r = ",r," not in schedule.robot_id_map = ",schedule.robot_id_map)
    end
    path_spec = PathSpec(
        node_type=Symbol(typeof(pred)),
        start_vtx=get_id(get_location_id(pred)),
        final_vtx=get_id(get_location_id(pred)),
        min_path_duration=0,
        path_id=get_next_path_id(schedule),
        dummy_id=r,
        agent_id=get(schedule.robot_id_map,r,r),
        free=true
        )
end
function generate_path_spec(schedule::P,spec::T,op::Operation) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    path_spec = PathSpec(
        node_type=Symbol(typeof(op)),
        start_vtx = -1,
        final_vtx = -1,
        min_path_duration=duration(op)
        )
end
function generate_path_spec(schedule::P,pred) where {P<:ProjectSchedule}
    generate_path_spec(schedule,TaskGraphProblemSpec(),pred)
end

function replace_in_schedule!(schedule::P,spec::T,pred,id::ID) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec,ID<:AbstractID}
    v = get_vtx(schedule, id)
    @assert v != -1
    set_vtx_map!(schedule,pred,id,v)
    path_spec = generate_path_spec(schedule,spec,pred)
    set_path_spec!(schedule,v,path_spec)
    schedule
end
function add_to_schedule!(schedule::P,spec::T,pred,id::ID) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec,ID<:AbstractID}
    @assert get_vtx(schedule, id) == -1
    add_vertex!(get_graph(schedule))
    insert_to_vtx_map!(schedule,pred,id,nv(get_graph(schedule)))
    path_spec = generate_path_spec(schedule,spec,pred)
    add_path_spec!(schedule,path_spec)
    schedule
end
function add_to_schedule!(schedule::P,pred,id::ID) where {P<:ProjectSchedule,ID<:AbstractID}
    @assert get_vtx(schedule, id) == -1
    add_vertex!(get_graph(schedule))
    insert_to_vtx_map!(schedule,pred,id,nv(get_graph(schedule)))
    path_spec = generate_path_spec(schedule,pred)
    add_path_spec!(schedule,path_spec)
    schedule
end

function LightGraphs.add_edge!(schedule::P,id1::A,id2::B) where {P<:ProjectSchedule,A<:AbstractID,B<:AbstractID}
    success = add_edge!(get_graph(schedule), get_vtx(schedule,id1), get_vtx(schedule,id2))
    # @show success, get_vtx(schedule,id1), get_vtx(schedule,id2)
    schedule
end

"""
    `get_task_sequences`

    Returns a dict mapping robot i => task sequence. The Vector `assignments`
    must map task j -> robot i (including dummy robots).
"""
function get_task_sequences(N::Int,M::Int,assignments)
    task_sequences = Dict{Int,Vector{Int}}()
    for i in 1:N
        robot_id = i
        seq = Vector{Int}()
        for j in 1:M
            if assignments[j] == robot_id
                push!(seq, j)
                robot_id = j + N
            end
        end
        task_sequences[i] = seq
    end
    task_sequences
end

export
    populate_agent_ids!

"""
    `populate_agent_ids!`

    Fills the schedule's dictionary mapping path_id (including dummy robots) to
    real agent id.
"""
function populate_agent_ids!(robot_id_map::Dict{Int,Int},spec::T,assignments) where {T<:TaskGraphProblemSpec}
    N, M = spec.N, spec.M
    for agent_id in 1:N
        path_id = agent_id
        robot_id_map[path_id] = agent_id
        j = 1
        while j <= M
            if get(assignments,j,-1) == path_id
                path_id = j + N
                robot_id_map[path_id] = agent_id
                j = 0
            end
            j += 1
        end
    end
    schedule
end
function populate_agent_ids!(schedule::P,spec::T,assignments) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    populate_agent_ids!(schedule.robot_id_map,spec,assignments)
end

export
    add_single_robot_delivery_task!

function add_single_robot_delivery_task!(
        schedule::S,
        problem_spec::T,
        robot_id::Int,
        object_id::Int,
        pickup_station_id::Int,
        dropoff_station_id::Int
        ) where {S<:ProjectSchedule,T<:TaskGraphProblemSpec}

    if robot_id != -1
        robot_pred = get_robot_ICs(schedule)[robot_id]
        robot_start_station_id = get_id(get_location_id(robot_pred))
    else
        robot_start_station_id = -1
    end

    # THIS NODE IS DETERMINED BY THE TASK ASSIGNMENT.
    # TODO Enable "leave-it-blank" so that the incomplete project schedule
    # can be used as an input to the MILP formulation and solver.
    action_id = ActionID(get_num_actions(schedule))
    add_to_schedule!(schedule, problem_spec, GO(robot_id, robot_start_station_id, pickup_station_id), action_id+=1)
    add_edge!(schedule, RobotID(robot_id), action_id)
    # END EMPTY GO NODE

    add_to_schedule!(schedule, problem_spec, COLLECT(robot_id, object_id, pickup_station_id), action_id+=1)
    add_edge!(schedule, action_id-1, action_id)
    add_edge!(schedule, ObjectID(object_id), action_id)

    add_to_schedule!(schedule, problem_spec, CARRY(robot_id, object_id, pickup_station_id, dropoff_station_id), action_id+=1)
    add_edge!(schedule, action_id-1, action_id)

    add_to_schedule!(schedule, problem_spec, DEPOSIT(robot_id, object_id, dropoff_station_id), action_id+=1)
    add_edge!(schedule, action_id-1, action_id)

    # new_robot_id = object_id + problem_spec.N
    # if get_vtx(schedule, RobotID(new_robot_id)) == -1
    #     add_to_schedule!(schedule, problem_spec, ROBOT_AT(RobotID(new_robot_id),StationID(dropoff_station_id)), RobotID(new_robot_id))
    # end
    # add_edge!(schedule, action_id, RobotID(new_robot_id))

    schedule
end
"""
    `construct_project_schedule`

    Args:
    - `project_spec` - a ProjectSpec
    - `problem_spec` - a TaskGraphProblemSpec
    - `object_ICs` - a list of initial object locations
    - `object_fCs` - a list of final object locations
    - `robot_ICs` - a list of initial robot locations
    - `assignments` - a list of robot assignments. `assignments[i] == j` means
    that robot `i` is assigned to transport object `j`
"""
function construct_project_schedule(
        project_spec::P,
        problem_spec::T,
        object_ICs::Vector{OBJECT_AT},
        object_FCs::Vector{OBJECT_AT},
        robot_ICs::Dict{Int,ROBOT_AT},
        assignments::V=Dict{Int,Int}()
        ) where {P<:ProjectSpec,T<:TaskGraphProblemSpec,V<:Union{Dict{Int,Int},Vector{Int}}}
    schedule = ProjectSchedule()
    populate_agent_ids!(schedule,problem_spec,assignments)
    graph = get_graph(schedule)
    # add object ICs to graph
    for pred in object_ICs
        add_to_schedule!(schedule, problem_spec, pred, get_object_id(pred))
    end
    # add robot ICs to graph
    for (id,pred) in robot_ICs
        # if (id in assignments)
        add_to_schedule!(schedule, problem_spec, pred, get_robot_id(pred))
        # end
    end
    M = length(object_ICs) # number of objects
    N = length(robot_ICs) - M # number of robots
    # add operations to graph
    for op_vtx in topological_sort(project_spec.graph)
        op = project_spec.operations[op_vtx]
        # operation_id = OperationID(get_num_operations(schedule) + 1)
        operation_id = OperationID(get_id(op))
        add_to_schedule!(schedule, problem_spec, op, operation_id)
        v = nv(get_graph(schedule))
        if op_vtx in project_spec.root_nodes
            push!(schedule.root_nodes, v)
            schedule.weights[v] = get(project_spec.weights, op_vtx, 1.0)
        end
        for object_id in get_input_ids(op)
            # add action sequence
            object_ic = get_object_ICs(schedule)[object_id]
            pickup_station_id = get_id(get_location_id(object_ic))
            object_fc = object_FCs[object_id]
            dropoff_station_id = get_id(get_location_id(object_fc))

            # TODO Handle collaborative tasks
            # if is_single_robot_task(project_spec, object_id)
            robot_id = get(assignments, object_id, -1)
            add_single_robot_delivery_task!(schedule,problem_spec,robot_id,
                object_id,pickup_station_id,dropoff_station_id)
            # elseif is_collaborative_robot_task(project_spec, object_id)
            # end

            action_id = ActionID(get_num_actions(schedule))
            add_edge!(schedule, action_id, operation_id)
        end
        for object_id in get_output_ids(op)
            add_edge!(schedule, operation_id, ObjectID(object_id))
        end
    end
    sort!(schedule.root_nodes)
    return schedule
end
function construct_project_schedule(
    project_spec::P,
    problem_spec::T,
    robot_ICs::Dict{Int,ROBOT_AT},
    assignments::V=Dict{Int,Int}()
    ) where {P<:ProjectSpec,T<:TaskGraphProblemSpec,V<:Union{Dict{Int,Int},Vector{Int}}}
    construct_project_schedule(
        project_spec,
        problem_spec,
        project_spec.initial_conditions,
        project_spec.final_conditions,
        robot_ICs,
        assignments
    )
end

export
    construct_partial_project_schedule

"""
    `construct_partial_project_schedule`

    Constructs a partial project graph
"""
function construct_partial_project_schedule(
    object_ICs::Vector{OBJECT_AT},
    object_FCs::Vector{OBJECT_AT},
    robot_ICs::Vector{ROBOT_AT},
    operations::Vector{Operation},
    root_ops::Vector{OperationID},
    problem_spec::TaskGraphProblemSpec
    )

    # Construct Partial Project Schedule
    project_schedule = ProjectSchedule();
    for pred in object_ICs
       add_to_schedule!(project_schedule, problem_spec, pred, get_object_id(pred))
    end
    for pred in robot_ICs
       add_to_schedule!(project_schedule, problem_spec, pred, get_robot_id(pred))
    end
    for op in operations
       add_to_schedule!(project_schedule, problem_spec, op, get_operation_id(op))
    end
    # add root nodes
    for operation_id in root_ops
        v = get_vtx(project_schedule, operation_id)
        push!(project_schedule.root_nodes, v)
        project_schedule.weights[v] = 1.0
    end
    # Fill in gaps in project schedule (except for GO assignments)
    for op in operations
        operation_id = get_operation_id(op)
        for object_id in get_input_ids(op)
            # add action sequence
            object_ic = get_object_ICs(project_schedule)[object_id]
            pickup_station_id = get_id(get_location_id(object_ic))
            object_fc = object_FCs[object_id]
            dropoff_station_id = get_id(get_location_id(object_fc))
            # TODO Handle collaborative tasks
            # if is_single_robot_task(project_spec, object_id)
            robot_id = -1
            add_single_robot_delivery_task!(project_schedule,problem_spec,robot_id,
                object_id,pickup_station_id,dropoff_station_id)
            # elseif is_collaborative_robot_task(project_spec, object_id)
            # end
            action_id = ActionID(get_num_actions(project_schedule)) # retrieve id of final DEPOSIT action
            add_edge!(project_schedule, action_id, operation_id)
        end
        for object_id in get_output_ids(op)
            add_edge!(project_schedule, operation_id, ObjectID(object_id))
        end
    end
    project_schedule
end
function construct_partial_project_schedule(spec::ProjectSpec,problem_spec::TaskGraphProblemSpec,robot_ICs=Vector{ROBOT_AT}())
    construct_partial_project_schedule(
        spec.initial_conditions,
        spec.final_conditions,
        robot_ICs,
        spec.operations,
        map(op->op.id, spec.operations[collect(spec.root_nodes)]),
        problem_spec
    )
end

"""
    `process_schedule(schedule::P) where {P<:ProjectSchedule}`

    Compute the optimistic start and end times, along with the slack associated
    with each vertex in the `schedule`. Slack for each vertex is represented as
    a vector in order to handle multi-headed projects.
"""
function process_schedule(schedule::P; t0=zeros(Int,get_num_vtxs(schedule)),
        tF=zeros(Int,get_num_vtxs(schedule))
    ) where {P<:ProjectSchedule}

    solution_graph = get_graph(schedule)
    traversal = topological_sort_by_dfs(solution_graph)
    n_roots = length(schedule.root_nodes)
    slack = map(i->Inf*ones(n_roots), vertices(solution_graph))
    local_slack = map(i->Inf*ones(n_roots), vertices(solution_graph))
    # True terminal nodes
    for (i,v) in enumerate(schedule.root_nodes)
        slack[v] = Inf*ones(n_roots)
        slack[v][i] = 0 # only slack for corresponing head is set to 0
    end
    ########## Compute Lower Bounds Via Forward Dynamic Programming pass
    for v in traversal
        path_spec = schedule.path_specs[v]
        for v2 in inneighbors(solution_graph,v)
            t0[v] = max(t0[v], tF[v2])
        end
        tF[v] = max(tF[v], t0[v] + path_spec.min_path_duration)
    end
    ########### Compute Slack Via Backward Dynamic Programming pass
    for v in reverse(traversal)
        for v2 in outneighbors(solution_graph,v)
            local_slack[v] = min.(local_slack[v], (t0[v2] - tF[v]) )
            slack[v] = min.(slack[v], slack[v2] .+ (t0[v2] - tF[v]) )
        end
    end
    t0,tF,slack,local_slack
end

export
    get_collect_node,
    get_deposit_node,
    add_job_shop_constraints!

function get_collect_node(schedule::P,id::ObjectID) where {P<:ProjectSchedule}
    current_id = id
    node = get_node_from_id(schedule,current_id)
    while typeof(node) != COLLECT
        current_id = get_vtx_id(schedule, outneighbors(get_graph(schedule),get_vtx(schedule,current_id))[1])
        node = get_node_from_id(schedule, current_id)
    end
    return current_id, node
end
function get_deposit_node(schedule::P,id::ObjectID) where {P<:ProjectSchedule}
    current_id = id
    node = get_node_from_id(schedule,current_id)
    while typeof(node) != DEPOSIT
        current_id = get_vtx_id(schedule, outneighbors(get_graph(schedule),get_vtx(schedule,current_id))[1])
        node = get_node_from_id(schedule, current_id)
    end
    return current_id, node
end

function add_job_shop_constraints!(schedule::P,spec::T,model::JuMP.Model) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    M = spec.M
    s0 = spec.s0
    sF = spec.sF
    tor = Int.(round.(value.(model[:tor]))) # collect begin time
    toc = Int.(round.(value.(model[:toc]))) # collect end time
    tod = Int.(round.(value.(model[:tod]))) # deposit begin time
    tof = Int.(round.(value.(model[:tof]))) # deposit end time
    for j in 1:M
        for j2 in j+1:M
            if (s0[j] == s0[j2]) || (s0[j] == sF[j2]) || (sF[j] == s0[j2]) || (sF[j] == sF[j2])
                if s0[j] == s0[j2]
                    id1, n1 = get_collect_node(schedule, ObjectID(j))
                    id2, n2 = get_collect_node(schedule, ObjectID(j2))
                    t1 = [tor[j], toc[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif s0[j] == sF[j2]
                    id1, n1 = get_collect_node(schedule, ObjectID(j))
                    id2, n2 = get_deposit_node(schedule, ObjectID(j2))
                    t1 = [tor[j], toc[j]]
                    t2 = [tod[j2], tof[j2]]
                elseif sF[j] == s0[j2]
                    id1, n1 = get_deposit_node(schedule, ObjectID(j))
                    id2, n2 = get_collect_node(schedule, ObjectID(j2))
                    t1 = [tod[j], tof[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif sF[j] == sF[j2]
                    id1, n1 = get_deposit_node(schedule, ObjectID(j))
                    id2, n2 = get_deposit_node(schedule, ObjectID(j2))
                    t1 = [tod, tof[j]]
                    t2 = [tod, tof[j2]]
                end
                if t1[2] < t2[1]
                    add_edge!(schedule, id1, id2)
                elseif t2[2] < t1[1]
                    add_edge!(schedule, id2, id1)
                else
                    throw(ErrorException("JOB SHOP CONSTRAINT VIOLATED"))
                end
            end
        end
    end
end

export
    formulate_schedule_milp

"""
    `formulate_schedule_milp`

    Formulate the adjacency matrix milp problem over the project schedule.
    #TODO: I am not positive that the current version guarantees t0[v2] <= tF[v]
    for all (v,v2) in edges(project_schedule.graph) following optimization.
"""
function formulate_schedule_milp(project_schedule::ProjectSchedule,problem_spec::TaskGraphProblemSpec;
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
    model, job_shop_variables
end



export
    update_project_schedule!

"""
    `update_project_schedule!`

    Args:
    - project_schedule
    - adj_matrix - adjacency_matrix encoding the edges that need to be added to
        the project schedule

    Adds all required edges to the project graph and modifies all nodes to
    reflect the appropriate valid IDs (e.g., `Action` nodes are populated with
    the correct `RobotID`s)
"""
function update_project_schedule!(project_schedule::P,problem_spec::T,adj_matrix,DEBUG::Bool=false) where {P<:ProjectSchedule,T<:TaskGraphProblemSpec}
    # Add all new edges to project schedule
    G = get_graph(project_schedule)
    DEBUG ? @assert(is_cyclic(G) == false) : nothing
    for v in vertices(G)
        for v2 in vertices(G)
            if adj_matrix[v,v2] == 1
                add_edge!(G,v,v2)
                DEBUG ? @assert(is_cyclic(G) == false) : nothing
                # if is_cyclic(G)
                #     node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
                #     node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
                #     rem_edge!(G,v,v2)
                # end
            end
        end
    end
    # Propagate valid IDs through the schedule
    for v in topological_sort(G)
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        for v2 in outneighbors(G,v)
            id2 = get_vtx_id(project_schedule, v2)
            node2 = get_node_from_id(project_schedule, id2)
            new_node = align_with_predecessor(node2,node)
            replace_in_schedule!(project_schedule, problem_spec, new_node, id2)
        end
    end
    project_schedule
end


end # module TaskGraphCore
