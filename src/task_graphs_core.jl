module TaskGraphsCore

using Parameters
using LightGraphs, MetaGraphs
using GraphUtils
using DataStructures
using JuMP
using Gurobi
using TOML
using CRCBS
using SparseArrays

# using ..PlanningPredicates
# using ..FactoryWorlds
using ..TaskGraphs

export
    get_debug_file_id,
    reset_debug_file_id!

DEBUG_ID_COUNTER = 0
get_debug_file_id() = Int(global DEBUG_ID_COUNTER += 1)
function reset_debug_file_id!()
    global DEBUG_ID_COUNTER = 0
end

export
    EnvironmentState

@with_kw struct EnvironmentState
    robot_states::Vector{ROBOT_AT}      = Vector{ROBOT_AT}()
    object_states::Vector{OBJECT_AT}    = Vector{OBJECT_AT}()
    object_id_map::Dict{ObjectID,Int}   = Dict{ObjectID,Int}(get_object_id(o)=>i for (i,o) in enumerate(object_states))
end

export
    ProblemSpec

"""
    `ProblemSpec{G}`

    This containts all information necessary to formulate the Task Assignment
    problem as a MILP.

    Elements:
    - N::Int - num robots
    - M::Int - num tasks
    - graph::G - delivery graph
    - Δt::Vector{Float64}  - durations of operations
    - tr0_::Dict{Int,Float64} - robot start times
    - to0_::Dict{Int,Float64} - object start times
    - root_nodes::Vector{Set{Int}} - identifies "project heads"
    - weights::Dict{Int,Float64} - stores weights associated with each project head
    - cost_function::F - the optimization objective (default is SumOfMakeSpans)
    - s0::Vector{Int} - pickup stations for each task
    - sF::Vector{Int} - delivery station for each task
    - nR::Vector{Int} - num robots required for each task (>1 => collaborative task)
"""
@with_kw struct ProblemSpec{G,F,T}
    N::Int                  = 0 # num robots
    M::Int                  = 0 # num tasks
    graph::G                = DiGraph() # delivery graph
    D::T                    = zeros(0,0) # full distance matrix
    Δt::Vector{Float64}     = Vector{Float64}() # durations of operations
    Δt_collect::Vector{Float64} = zeros(M) # duration of COLLECT operations
    Δt_deliver::Vector{Float64} = zeros(M) # duration of DELIVER operations
    tr0_::Dict{Int,Float64} = Dict{Int,Float64}() # robot start times
    to0_::Dict{Int,Float64} = Dict{Int,Float64}() # object start times
    root_nodes::Vector{Set{Int}} = [get_all_root_nodes(graph)]
    weights::Dict{Int,Float64} = Dict{Int,Float64}(v=>1.0 for v in 1:length(root_nodes))
    cost_function::F        = SumOfMakeSpans
    r0::Vector{Int} = zeros(N)
    s0::Vector{Int} = zeros(M) # pickup stations for each task
    sF::Vector{Int} = zeros(M) # delivery station for each task
    nR::Vector{Int} = ones(M) # num robots required for each task (>1 => collaborative task)
end
TaskGraphs.get_distance(spec::ProblemSpec,args...) = get_distance(spec.D,args...)

export
    ProjectSpec,
    get_initial_nodes,
    get_input_ids,
    get_output_ids,
    add_operation!,
    get_duration_vector,
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
    team_sizes::Dict{ObjectID,Int} = Dict{ObjectID,Int}() # TODO: store team configuration somewhere
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
get_input_ids(op::Operation) = sort([get_id(get_object_id(p)) for p in op.pre])
get_output_ids(op::Operation) = sort([get_id(get_object_id(p)) for p in op.post])
get_pre_deps(spec::ProjectSpec, op_id::Int) = get(spec.pre_deps, op_id, Set{Int}())
get_post_deps(spec::ProjectSpec, op_id::Int) = get(spec.post_deps, op_id, Set{Int}())
get_num_delivery_tasks(spec::ProjectSpec) = length(collect(union(map(op->union(get_input_ids(op),get_output_ids(op)),spec.operations)...)))
get_team_size(spec::ProjectSpec, object_id::ObjectID) = get(spec.team_sizes, object_id, 1)
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

export
    read_operation,
    read_project_spec

# Some tools for writing and reading project specs
function TOML.parse(pred::OBJECT_AT)
    [get_id(get_object_id(pred)),get_id(get_location_id(pred))]
end
function TOML.parse(op::Operation)
    toml_dict = Dict()
    toml_dict["pre"] = map(pred->TOML.parse(pred), collect(op.pre))
    toml_dict["post"] = map(pred->TOML.parse(pred), collect(op.post))
    toml_dict["dt"] = duration(op)
    toml_dict["station_id"] = get_id(op.station_id)
    toml_dict["id"] = get_id(op.id)
    return toml_dict
end
function read_operation(toml_dict::Dict)
    op_id = OperationID(get(toml_dict,"id",-1))
    if get_id(op_id) == -1
        op_id = OperationID(get_unique_operation_id())
    end
    op = Operation(
        pre     = Set{OBJECT_AT}(map(arr->OBJECT_AT(arr[1],arr[2]),toml_dict["pre"])),
        post    = Set{OBJECT_AT}(map(arr->OBJECT_AT(arr[1],arr[2]),toml_dict["post"])),
        Δt      = get(toml_dict,"dt",get(toml_dict,"Δt",0.0)),
        station_id = StationID(get(toml_dict,"station_id",-1)),
        id = op_id
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
    if typeof(toml_dict["initial_conditions"]) <: Dict
        object_ids = Int[]
        for (i,arr) in toml_dict["initial_conditions"]
            object_id = arr[1]
            push!(object_ids, object_id)
            station_id = arr[2]
            push!(project_spec.initial_conditions, OBJECT_AT(object_id, station_id))
        end
        sort!(object_ids)
        for (i, object_id) in enumerate(object_ids)
            project_spec.object_id_to_idx[object_id] = i
        end
    elseif typeof(toml_dict["initial_conditions"]) <: AbstractArray
        object_ids = Int[]
        for (i,arr) in enumerate(toml_dict["initial_conditions"])
            object_id = arr[1]
            push!(object_ids, object_id)
            station_id = arr[2]
            push!(project_spec.initial_conditions, OBJECT_AT(object_id, station_id))
            # project_spec.object_id_to_idx[object_id] = i
        end
        sort!(object_ids)
        for (i, object_id) in enumerate(object_ids)
            project_spec.object_id_to_idx[object_id] = i
        end
    end
    if typeof(toml_dict["final_conditions"]) <: Dict
        for (k,arr) in toml_dict["final_conditions"]
            object_id = arr[1]
            station_id = arr[2]
            push!(project_spec.final_conditions, OBJECT_AT(object_id, station_id))
        end
    elseif typeof(toml_dict["final_conditions"]) <: AbstractArray
        for (i,arr) in enumerate(toml_dict["final_conditions"])
            object_id = arr[1]
            station_id = arr[2]
            push!(project_spec.final_conditions, OBJECT_AT(object_id, station_id))
        end
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

@with_kw struct SimpleProblemDef
    project_spec::ProjectSpec       = ProjectSpec()
    r0::Vector{Int}                 = Int[]
    s0::Vector{Int}                 = Int[]
    sF::Vector{Int}                 = Int[]
    shapes::Vector{Tuple{Int,Int}}  = Vector{Tuple{Int,Int}}([(1,1) for o in s0])
end
SimpleProblemDef(project_spec,r0,s0,sF) = SimpleProblemDef(project_spec=project_spec,r0=r0,s0=s0,sF=sF)

function TOML.parse(def::SimpleProblemDef)
    toml_dict = TOML.parse(def.project_spec)
    toml_dict["r0"] = def.r0
    toml_dict["s0"] = def.s0
    toml_dict["sF"] = def.sF
    toml_dict["shapes"] = map(s->[s...], def.shapes)
    toml_dict
end
function read_problem_def(toml_dict::Dict)
    SimpleProblemDef(
        read_project_spec(toml_dict),
        toml_dict["r0"],
        toml_dict["s0"],
        toml_dict["sF"],
        # map(s->tuple(s...), toml_dict["shapes"])
        map(s->tuple(s...), get(toml_dict,"shapes",[[1,1] for o in toml_dict["s0"]]) )
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
    node_type           ::Symbol        = :EMPTY
    start_vtx           ::Int           = -1
    final_vtx           ::Int           = -1
    min_path_duration   ::Int           =  0 # duration

    agent_id            ::Int           = -1
    object_id           ::Int           = -1
    # flag indicating that a path must be planned
    plan_path           ::Bool          = true
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
    # fixed==true => do not plan path because it is already fixed. Instead,
    # retrieve the portion of the path directly from the pre-existing solution.
    fixed               ::Bool          = false
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
    get_node_from_vtx,
    get_num_actions,
    get_num_operations,
    get_num_object_ICs,
    get_num_robot_ICs,
    get_num_vtxs,
    get_num_paths,
    get_completion_time,
    get_duration,
    get_vtx,
    get_vtx_id,
    replace_in_schedule!,
    add_to_schedule!,
    construct_project_schedule,
    process_schedule

"""
    `ProjectSchedule`
"""
@with_kw struct ProjectSchedule{G<:AbstractGraph} <: AbstractGraph{Int}
    graph               ::G                     = DiGraph()
    planning_nodes      ::Dict{AbstractID,AbstractPlanningPredicate}    = Dict{AbstractID,AbstractPlanningPredicate}()
    vtx_map             ::Dict{AbstractID,Int}  = Dict{AbstractID,Int}()
    # TODO add UID vector so that vertex deletion can be constant time
    vtx_ids             ::Vector{AbstractID}    = Vector{AbstractID}() # maps vertex uid to actual graph node
    path_specs          ::Vector{PathSpec}      = Vector{PathSpec}()
    root_nodes          ::Vector{Int}           = Vector{Int}() # list of "project heads"
    weights             ::Dict{Int,Float64}     = Dict{Int,Float64}() # weights corresponding to project heads
end

Base.zero(schedule::ProjectSchedule{G}) where {G}                           = ProjectSchedule(graph=G())
LightGraphs.edges(schedule::ProjectSchedule)                                = edges(schedule.graph)
LightGraphs.edgetype(schedule::ProjectSchedule{G}, args...) where {G}       = edgetype(schedule.graph, args...)
LightGraphs.has_edge(schedule::ProjectSchedule{G}, args...) where {G}       = has_edge(schedule.graph, args...)
LightGraphs.has_vertex(schedule::ProjectSchedule{G}, args...) where {G}     = has_vertex(schedule.graph, args...)
LightGraphs.inneighbors(schedule::ProjectSchedule{G}, args...) where {G}    = inneighbors(schedule.graph, args...)
# LightGraphs.is_directed(schedule::ProjectSchedule{G}, args...) where {G}    = is_directed(schedule.graph, args...)
# LightGraphs.is_directed(schedule::ProjectSchedule, args...)                 = true
LightGraphs.is_directed(schedule::ProjectSchedule)                          = true
LightGraphs.ne(schedule::ProjectSchedule{G}, args...) where {G}             = ne(schedule.graph, args...)
LightGraphs.nv(schedule::ProjectSchedule{G}, args...) where {G}             = nv(schedule.graph, args...)
LightGraphs.outneighbors(schedule::ProjectSchedule{G}, args...) where {G}   = outneighbors(schedule.graph, args...)
LightGraphs.vertices(schedule::ProjectSchedule{G}, args...) where {G}       = vertices(schedule.graph, args...)

get_graph(schedule::P) where {P<:ProjectSchedule}       = schedule.graph
get_vtx_ids(schedule::P) where {P<:ProjectSchedule}     = schedule.vtx_ids
get_root_nodes(schedule::P) where {P<:ProjectSchedule}  = schedule.root_nodes
get_root_node_weights(schedule::P) where {P<:ProjectSchedule}  = schedule.weights

get_node_from_id(schedule::P,id::A) where {P<:ProjectSchedule,A<:AbstractID}= schedule.planning_nodes[id]
get_vtx(schedule::P,id::A) where {P<:ProjectSchedule,A<:AbstractID}         = get(schedule.vtx_map, id, -1)
get_vtx_id(schedule::P,v::Int) where {P<:ProjectSchedule}                   = schedule.vtx_ids[v]
get_node_from_vtx(schedule::P,v::Int) where {P<:ProjectSchedule}= schedule.planning_nodes[schedule.vtx_ids[v]]

get_nodes_of_type(schedule::P,T) where {P<:ProjectSchedule} = Dict(get_id(id)=>get_node_from_id(schedule, id) for id in schedule.vtx_ids if typeof(id)<:T)
get_object_ICs(schedule::P) where {P<:ProjectSchedule}  = get_nodes_of_type(schedule,ObjectID)
get_robot_ICs(schedule::P) where {P<:ProjectSchedule}   = get_nodes_of_type(schedule,RobotID)
get_robot_FCs(schedule::P) where {P<:ProjectSchedule}   = get_nodes_of_type(schedule,TerminalRobotID)
get_actions(schedule::P) where {P<:ProjectSchedule}     = get_nodes_of_type(schedule,ActionID)
get_operations(schedule::P) where {P<:ProjectSchedule}  = get_nodes_of_type(schedule,OperationID)

get_num_actions(schedule::P) where {P<:ProjectSchedule}     = length(get_actions(schedule))
get_num_operations(schedule::P) where {P<:ProjectSchedule}  = length(get_operations(schedule))
get_num_object_ICs(schedule::P) where {P<:ProjectSchedule}  = length(get_object_ICs(schedule))
get_num_robot_ICs(schedule::P) where {P<:ProjectSchedule}   = length(get_robot_ICs(schedule))
get_num_vtxs(schedule::P) where {P<:ProjectSchedule}        = nv(get_graph(schedule))
get_num_paths(schedule::P) where {P<:ProjectSchedule}       = get_num_actions(schedule) + get_num_robot_ICs(schedule)

export
    set_vtx_map!,
    insert_to_vtx_map!

function set_vtx_map!(schedule::S,pred::P,id::A,v::Int) where {S<:ProjectSchedule,P<:AbstractPlanningPredicate,A<:AbstractID}
    schedule.planning_nodes[id] = pred
    schedule.vtx_map[id] = v
end
function insert_to_vtx_map!(schedule::P,pred,id::ID,idx::Int) where {P<:ProjectSchedule,ID<:AbstractID}
    push!(schedule.vtx_ids, id)
    set_vtx_map!(schedule,pred,id,idx)
end

export
    get_path_spec,
    set_path_spec!,
    add_path_spec!

get_path_spec(schedule::P,v::Int) where {P<:ProjectSchedule} = schedule.path_specs[v]
function set_path_spec!(schedule::P,v::Int,spec::S) where {P<:ProjectSchedule,S<:PathSpec}
    schedule.path_specs[v] = spec
end
function add_path_spec!(schedule::P,spec::S) where {P<:ProjectSchedule,S<:PathSpec}
    push!(schedule.path_specs, spec)
    set_path_spec!(schedule, nv(schedule.graph), spec)
end

"""
    PathSpec provides details about the path that corresponds to this node in
    the schedule
"""
function generate_path_spec(schedule::P,spec::T,a::GO) where {P<:ProjectSchedule,T<:ProblemSpec}
    s0 = get_id(get_initial_location_id(a))
    s = get_id(get_destination_location_id(a))
    r = get_id(get_robot_id(a))
    path_spec = PathSpec(
        node_type=Symbol(typeof(a)),
        start_vtx=s0,
        final_vtx=s,
        min_path_duration=get_distance(spec.D,s0,s), # ProblemSpec distance matrix
        agent_id=r,
        tight=true,
        free = (s==-1) # if destination is -1, there is no goal location
        )
end
function generate_path_spec(schedule::P,spec::T,a::CARRY) where {P<:ProjectSchedule,T<:ProblemSpec}
    s0 = get_id(get_initial_location_id(a))
    s = get_id(get_destination_location_id(a))
    r = get_id(get_robot_id(a))
    o = get_id(get_object_id(a))
    path_spec = PathSpec(
        node_type=Symbol(typeof(a)),
        start_vtx=s0,
        final_vtx=s,
        min_path_duration=get_distance(spec.D,s0,s), # ProblemSpec distance matrix
        agent_id=r,
        object_id = o
        )
end
function generate_path_spec(schedule::P,spec::T,a::COLLECT) where {P<:ProjectSchedule,T<:ProblemSpec}
    s0 = get_id(get_initial_location_id(a))
    s = get_id(get_destination_location_id(a))
    r = get_id(get_robot_id(a))
    o = get_id(get_object_id(a))
    path_spec = PathSpec(
        node_type=Symbol(typeof(a)),
        start_vtx=s0,
        final_vtx=s,
        min_path_duration=get(spec.Δt_collect,o,0),
        agent_id=r,
        object_id = o,
        static=true
        )
end
function generate_path_spec(schedule::P,spec::T,a::DEPOSIT) where {P<:ProjectSchedule,T<:ProblemSpec}
    s0 = get_id(get_initial_location_id(a))
    s = get_id(get_destination_location_id(a))
    r = get_id(get_robot_id(a))
    o = get_id(get_object_id(a))
    path_spec = PathSpec(
        node_type=Symbol(typeof(a)),
        start_vtx=s0,
        final_vtx=s,
        min_path_duration=get(spec.Δt_deliver,o,0),
        agent_id=r,
        object_id = o,
        static=true
        )
end
function generate_path_spec(schedule::P,spec::T,pred::OBJECT_AT) where {P<:ProjectSchedule,T<:ProblemSpec}
    path_spec = PathSpec(
        node_type=Symbol(typeof(pred)),
        start_vtx=get_id(get_location_id(pred)),
        final_vtx=get_id(get_location_id(pred)),
        min_path_duration=0,
        plan_path = false,
        object_id = get_id(get_object_id(pred))
        )
end
function generate_path_spec(schedule::P,spec::T,pred::ROBOT_AT) where {P<:ProjectSchedule,T<:ProblemSpec}
    r = get_id(get_robot_id(pred))
    path_spec = PathSpec(
        node_type=Symbol(typeof(pred)),
        start_vtx=get_id(get_location_id(pred)),
        final_vtx=get_id(get_location_id(pred)),
        min_path_duration=0,
        agent_id=r,
        free=true
        )
end
function generate_path_spec(schedule::P,spec::T,op::Operation) where {P<:ProjectSchedule,T<:ProblemSpec}
    path_spec = PathSpec(
        node_type=Symbol(typeof(op)),
        start_vtx = -1,
        final_vtx = -1,
        plan_path = false,
        min_path_duration=duration(op)
        )
end
function generate_path_spec(schedule::P,spec::T,pred::TEAM_ACTION{A}) where {P<:ProjectSchedule,T<:ProblemSpec,A}
    s0 = get_id(get_initial_location_id(pred.instructions[1]))
    s = get_id(get_destination_location_id(pred.instructions[1]))
    path_spec = PathSpec(
        node_type=Symbol(typeof(pred)),
        # min_path_duration = maximum(map(a->generate_path_spec(schedule,spec,a).min_path_duration, pred.instructions)),
        min_path_duration = get_distance(spec.D,s0,s,pred.shape),
        plan_path = true,
        static = (A <: Union{COLLECT,DEPOSIT})
        )
end
function generate_path_spec(schedule::P,pred) where {P<:ProjectSchedule}
    generate_path_spec(schedule,ProblemSpec(),pred)
end

function replace_in_schedule!(schedule::P,path_spec::T,pred,id::ID) where {P<:ProjectSchedule,T<:PathSpec,ID<:AbstractID}
    v = get_vtx(schedule, id)
    @assert v != -1
    set_vtx_map!(schedule,pred,id,v)
    set_path_spec!(schedule,v,path_spec)
    schedule
end
function replace_in_schedule!(schedule::P,spec::T,pred,id::ID) where {P<:ProjectSchedule,T<:ProblemSpec,ID<:AbstractID}
    replace_in_schedule!(schedule,generate_path_spec(schedule,spec,pred),pred,id)
end
function replace_in_schedule!(schedule::P,pred,id::ID) where {P<:ProjectSchedule,ID<:AbstractID}
    replace_in_schedule!(schedule,ProblemSpec(),pred,id)
end
function add_to_schedule!(schedule::P,path_spec::T,pred,id::ID) where {P<:ProjectSchedule,T<:PathSpec,ID<:AbstractID}
    @assert get_vtx(schedule, id) == -1
    add_vertex!(get_graph(schedule))
    insert_to_vtx_map!(schedule,pred,id,nv(get_graph(schedule)))
    add_path_spec!(schedule,path_spec)
    schedule
end
function add_to_schedule!(schedule::P,spec::T,pred,id::ID) where {P<:ProjectSchedule,T<:ProblemSpec,ID<:AbstractID}
    add_to_schedule!(schedule,generate_path_spec(schedule,spec,pred),pred,id)
end
function add_to_schedule!(schedule::P,pred,id::ID) where {P<:ProjectSchedule,ID<:AbstractID}
    add_to_schedule!(schedule,ProblemSpec(),pred,id)
end

function LightGraphs.add_edge!(schedule::P,id1::A,id2::B) where {P<:ProjectSchedule,A<:AbstractID,B<:AbstractID}
    success = add_edge!(get_graph(schedule), get_vtx(schedule,id1), get_vtx(schedule,id2))
    schedule
end
function LightGraphs.rem_edge!(schedule::P,id1::A,id2::B) where {P<:ProjectSchedule,A<:AbstractID,B<:AbstractID}
    success = rem_edge!(get_graph(schedule), get_vtx(schedule,id1), get_vtx(schedule,id2))
    schedule
end

export
    get_leaf_operation_nodes,
    set_leaf_operation_nodes!,
    delete_node!,
    delete_nodes!

function get_leaf_operation_nodes(schedule::ProjectSchedule)
    G = get_graph(schedule)
    root_vtxs = Int[]
    for v in vertices(G)
        if is_terminal_node(G,v)
            if typeof(get_node_from_id(schedule, get_vtx_id(schedule,v))) == Operation
                push!(root_vtxs,v)
            end
        end
    end
    return root_vtxs
end
function set_leaf_operation_nodes!(schedule::ProjectSchedule)
    empty!(get_root_nodes(schedule))
    empty!(get_root_node_weights(schedule))
    for vtx in  get_leaf_operation_nodes(schedule)
        push!(get_root_nodes(schedule),vtx)
        get_root_node_weights(schedule)[vtx] = 1.0
    end
    schedule
end

"""
    delete_node!

    removes a node (by id) from schedule.
"""
function delete_node!(schedule::ProjectSchedule, id::AbstractID)
    v = get_vtx(schedule, id)
    delete!(schedule.vtx_map, id)
    delete!(schedule.planning_nodes, id)
    rem_vertex!(get_graph(schedule), v)
    deleteat!(schedule.vtx_ids, v)
    deleteat!(schedule.path_specs, v)
    for vtx in v:nv(get_graph(schedule))
        node_id = schedule.vtx_ids[vtx]
        schedule.vtx_map[node_id] = vtx
    end
    schedule
end
function delete_nodes!(schedule::ProjectSchedule, vtxs::Union{Int,Vector{Int}})
    node_ids = map(v->get_vtx_id(schedule,v), vtxs)
    for id in node_ids
        delete_node!(schedule,id)
    end
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
    validate,
    sanity_check

function validate(path::Path, msg::String)
    @assert( !any(convert_to_vertex_lists(path) .== -1), msg )
    return true
end
function validate(path::Path, v::Int)
    validate(path, string("invalid path with -1 vtxs: v = ",v,", path = ",convert_to_vertex_lists(path)))
end
function validate(path::Path, v::Int, cbs_env)
    validate(path, string("v = ",v,", path = ",convert_to_vertex_lists(path),", goal: ",cbs_env.goal))
end
function sanity_check(project_schedule::ProjectSchedule,append_string="")
    G = get_graph(project_schedule)
    try
        @assert !is_cyclic(G) "is_cyclic(G)"
        for (id,node) in project_schedule.planning_nodes
            if typeof(node) <: COLLECT
                @assert(get_location_id(node) != -1, string("get_location_id(node) != -1 for node id ", id))
            end
        end
        for v in vertices(G)
            node = get_node_from_vtx(project_schedule, v)
            if isa(node,Operation)
                input_ids = Set(map(o->get_id(get_object_id(o)),collect(node.pre)))
                for v2 in inneighbors(G,v)
                    node2 = get_node_from_vtx(project_schedule, v2)
                    @assert(get_id(get_object_id(node2)) in input_ids, string(string(node2), " should not be an inneighbor of ",string(node), " whose inputs should be ",input_ids))
                end
                @assert(
                    indegree(G,v) == length(node.pre),
                    string("Operation ",string(node),
                        " needs edges from objects ",collect(input_ids),
                        " but only has edges from objects ",map(v2->get_id(get_object_id(get_node_from_vtx(project_schedule, v2))),inneighbors(G,v))
                    )
                    )
            end
        end
    catch e
        if typeof(e) <: AssertionError
            print(string(e.msg, append_string))
        else
            throw(e)
        end
        return false
    end
    return true
end
function validate(project_schedule::ProjectSchedule)
    G = get_graph(project_schedule)
    try
        @assert !is_cyclic(G) "is_cyclic(G)"
        for (id,node) in project_schedule.planning_nodes
            if typeof(node) <: COLLECT
                @assert(get_location_id(node) != -1, string("get_location_id(node) != -1 for node id ", id))
            end
        end
        for e in edges(G)
            node1 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, e.src))
            node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, e.dst))
            @assert(validate_edge(node1,node2), string(" INVALID EDGE: ", string(node1), " --> ",string(node2)))
        end
        for v in vertices(G)
            node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
            @assert( outdegree(G,v) >= sum([0, values(required_successors(node))...]) , string("node = ", string(node), " outdegree = ",outdegree(G,v), " "))
            @assert( indegree(G,v) >= sum([0, values(required_predecessors(node))...]), string("node = ", string(node), " indegree = ",indegree(G,v), " ") )
            if isa(node, Union{GO,COLLECT,CARRY,DEPOSIT})
                for v2 in outneighbors(G,v)
                    node2 = get_node_from_vtx(project_schedule, v2)
                    if isa(node2, Union{GO,COLLECT,CARRY,DEPOSIT})
                        if length(intersect(resources_reserved(node),resources_reserved(node2))) == 0 # job shop constraint
                            @assert( get_robot_id(node) == get_robot_id(node2), string("robot IDs do not match: ",string(node), " --> ", string(node2)))
                        end
                    end
                end
            end
        end
    catch e
        if typeof(e) <: AssertionError
            print(e.msg)
        else
            throw(e)
        end
        return false
    end
    return true
end
function validate(project_schedule::ProjectSchedule,paths::Vector{Vector{Int}},t0::Vector{Int},tF::Vector{Int})
    G = get_graph(project_schedule)
    for v in vertices(G)
        node = get_node_from_vtx(project_schedule, v)
        path_spec = get_path_spec(project_schedule, v)
        agent_id = path_spec.agent_id
        if agent_id != -1
            path = paths[agent_id]
            start_vtx = path_spec.start_vtx
            final_vtx = path_spec.final_vtx
            try
                if start_vtx != -1
                    @assert(path[t0[v] + 1] == start_vtx, string("node: ",string(node), ", start_vtx: ",start_vtx, ", t0+1: ",t0[v]+1,", path: ", path))
                end
                if final_vtx != -1
                    @assert(path[tF[v] + 1] == final_vtx, string("node: ",string(node), ", final vtx: ",final_vtx, ", tF+1: ",tF[v]+1,", path: ", path))
                end
            catch e
                if typeof(e) <: AssertionError
                    print(e.msg)
                else
                    throw(e)
                end
                return false
            end
        end
    end
    return true
end

export
    add_single_robot_delivery_task!

function add_single_robot_delivery_task!(
        schedule::S,
        problem_spec::T,
        pred_id::AbstractID,
        robot_id::RobotID,
        object_id::ObjectID,
        pickup_station_id::StationID,
        dropoff_station_id::StationID
        ) where {S<:ProjectSchedule,T<:ProblemSpec}

    if robot_id != -1
        robot_pred = get_node_from_id(schedule,RobotID(robot_id))
        robot_start_station_id = get_initial_location_id(get_node_from_id(schedule, pred_id))
    else
        robot_start_station_id = StationID(-1)
    end

    # THIS NODE IS DETERMINED BY THE TASK ASSIGNMENT.
    action_id = ActionID(get_unique_action_id())
    add_to_schedule!(schedule, problem_spec, GO(robot_id, robot_start_station_id, pickup_station_id), action_id)
    add_edge!(schedule, pred_id, action_id)

    prev_action_id = action_id
    action_id = ActionID(get_unique_action_id())
    add_to_schedule!(schedule, problem_spec, COLLECT(robot_id, object_id, pickup_station_id), action_id)
    add_edge!(schedule, prev_action_id, action_id)
    add_edge!(schedule, ObjectID(object_id), action_id)

    prev_action_id = action_id
    action_id = ActionID(get_unique_action_id())
    add_to_schedule!(schedule, problem_spec, CARRY(robot_id, object_id, pickup_station_id, dropoff_station_id), action_id)
    add_edge!(schedule, prev_action_id, action_id)

    prev_action_id = action_id
    action_id = ActionID(get_unique_action_id())
    add_to_schedule!(schedule, problem_spec, DEPOSIT(robot_id, object_id, dropoff_station_id), action_id)
    add_edge!(schedule, prev_action_id, action_id)

    action_id
end
function add_headless_delivery_task!(
        schedule::ProjectSchedule,
        problem_spec::ProblemSpec,
        object_id::ObjectID,
        operation_id::OperationID,
        pickup_station_id::StationID,
        dropoff_station_id::StationID
        )

    robot_id = RobotID(-1)
    action_id = ActionID(get_unique_action_id())
    add_to_schedule!(schedule, problem_spec, COLLECT(robot_id, object_id, pickup_station_id), action_id)
    add_edge!(schedule, object_id, action_id)

    prev_action_id = action_id
    action_id = ActionID(get_unique_action_id())
    add_to_schedule!(schedule, problem_spec, CARRY(robot_id, object_id, pickup_station_id, dropoff_station_id), action_id)
    add_edge!(schedule, prev_action_id, action_id)

    prev_action_id = action_id
    action_id = ActionID(get_unique_action_id())
    add_to_schedule!(schedule, problem_spec, DEPOSIT(robot_id, object_id, dropoff_station_id), action_id)
    add_edge!(schedule, action_id, operation_id)
    add_edge!(schedule, prev_action_id, action_id)

    prev_action_id = action_id
    action_id = ActionID(get_unique_action_id())
    add_to_schedule!(schedule, problem_spec, GO(robot_id, dropoff_station_id,StationID(-1)), action_id)
    add_edge!(schedule, prev_action_id, action_id)

    return
end
function add_headless_delivery_task!(
        schedule::S,
        problem_spec::T,
        object_id::ObjectID,
        operation_id::OperationID,
        pickup_station_ids::Vector{StationID},
        dropoff_station_ids::Vector{StationID}
        ) where {S<:ProjectSchedule,T<:ProblemSpec}

    robot_id = RobotID(-1)
    @assert length(pickup_station_ids) == length(dropoff_station_ids)
    n = length(pickup_station_ids)

    object_node = get_node_from_id(schedule,object_id)
    shape = object_node.shape
    # COLLABORATIVE COLLECT
    prev_team_action_id = ActionID(-1)
    team_action_id = ActionID(get_unique_action_id())
    team_action = TEAM_ACTION(
        n = n,
        instructions = map(x->COLLECT(robot_id, object_id, x), pickup_station_ids),
        shape=shape
        )
    add_to_schedule!(schedule, problem_spec, team_action, team_action_id)
    # Add Edge from object
    add_edge!(schedule,object_id,team_action_id)
    # Add GO inputs to TEAM_COLLECT
    for x in pickup_station_ids
        action_id = ActionID(get_unique_action_id())
        add_to_schedule!(schedule,problem_spec,GO(robot_id,x,x),action_id)
        add_edge!(schedule,action_id,team_action_id)
    end
    # Add TEAM_CARRY
    prev_team_action_id = team_action_id
    team_action_id = ActionID(get_unique_action_id())
    team_action = TEAM_ACTION(
        n = n,
        instructions = [CARRY(robot_id, object_id, x1, x2) for (x1,x2) in zip(pickup_station_ids,dropoff_station_ids)],
        shape = shape
        )
    add_to_schedule!(schedule, problem_spec, team_action, team_action_id)
    add_edge!(schedule,prev_team_action_id,team_action_id)
    # TEAM_DEPOSIT
    prev_team_action_id = team_action_id
    team_action_id = ActionID(get_unique_action_id())
    team_action = TEAM_ACTION(
        n = n,
        instructions = map(x->DEPOSIT(robot_id, object_id, x), dropoff_station_ids),
        shape=shape
        )
    add_to_schedule!(schedule, problem_spec, team_action, team_action_id)
    add_edge!(schedule,prev_team_action_id,team_action_id)
    # Edge to Operation
    add_edge!(schedule,team_action_id,operation_id)
    # Individual GO nodes
    for x in dropoff_station_ids
        action_id = ActionID(get_unique_action_id())
        add_to_schedule!(schedule,problem_spec,GO(robot_id,x,-1),action_id)
        add_edge!(schedule, team_action_id, action_id)
    end
    return
end
function add_headless_delivery_task!(
        schedule::ProjectSchedule,
        problem_spec::ProblemSpec,
        object_id::Int,
        operation_id::Int,
        pickup_station_ids::Vector{Int},
        dropoff_station_ids::Vector{Int}
        )
    add_headless_delivery_task!(
        schedule,
        problem_spec,
        ObjectID(object_id),
        OperationID(operation_id),
        map(id->StationID(id), pickup_station_ids),
        map(id->StationID(id), dropoff_station_ids)
        )
end
function add_headless_delivery_task!(
        schedule::ProjectSchedule,
        problem_spec::ProblemSpec,
        object_id::Int,
        operation_id::Int,
        pickup_station_id::Int,
        dropoff_station_id::Int
        )
    add_headless_delivery_task!(
        schedule,
        problem_spec,
        ObjectID(object_id),
        OperationID(operation_id),
        StationID(pickup_station_id),
        StationID(dropoff_station_id)
        )
end

"""
    `construct_project_schedule`

    Args:
    - `project_spec` - a ProjectSpec
    - `problem_spec` - a ProblemSpec
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
        ) where {P<:ProjectSpec,T<:ProblemSpec,V<:Union{Dict{Int,Int},Vector{Int}}}
    schedule = ProjectSchedule()
    robot_pred_tips = Dict{AbstractID,AbstractID}()
    object_ops = Dict{ObjectID,OperationID}()
    graph = get_graph(schedule)
    for pred in object_ICs
        add_to_schedule!(schedule, problem_spec, pred, get_object_id(pred))
    end
    # for (id,pred) in robot_ICs
    for pred in map(i->robot_ICs[i], 1:problem_spec.N)
        add_to_schedule!(schedule, problem_spec, pred, get_robot_id(pred))
        robot_pred_tips[get_robot_id(pred)] = get_robot_id(pred)
        # push!(robot_pred_tips, get_robot_id(pred))
    end
    # M = length(object_ICs) # number of objects
    # N = length(robot_ICs) - M # number of robots
    # add operations to graph
    for op_vtx in topological_sort(project_spec.graph)
        op = project_spec.operations[op_vtx]
        operation_id = OperationID(get_id(op))
        add_to_schedule!(schedule, problem_spec, op, operation_id)
        v = nv(get_graph(schedule))
        if op_vtx in project_spec.root_nodes
            push!(schedule.root_nodes, v)
            schedule.weights[v] = get(project_spec.weights, op_vtx, 1.0)
        end
        for object_id in get_input_ids(op)
            object_ops[ObjectID(object_id)] = operation_id
        end
        for object_id in get_output_ids(op)
            add_edge!(schedule, operation_id, ObjectID(object_id))
        end
    end
    # for (robot_id, pred_id) in robot_pred_tips
    for (task_idx, robot_id) in enumerate(assignments)
        object_id = get_object_id(object_ICs[task_idx])
        # add action sequence
        object_ic           = get_node_from_id(schedule, object_id)
        pickup_station_id   = get_location_id(object_ic)
        object_fc           = object_FCs[get_id(object_id)]
        dropoff_station_id  = get_location_id(object_fc)

        # TODO Handle collaborative tasks
        # robot_id = get(assignments, task_idx, -1)
        pred_id = robot_pred_tips[RobotID(robot_id)]
        action_id = add_single_robot_delivery_task!(schedule,problem_spec,pred_id,
            RobotID(robot_id),object_id,pickup_station_id,dropoff_station_id) # DEPOSIT task id
        robot_pred_tips[RobotID(robot_id)] = action_id
        operation_id = object_ops[ObjectID(object_id)]
        add_edge!(schedule, action_id, operation_id)

    end
    # end
    # add final GO to all robot/deposit nodes
    for (robot_id, pred_id) in robot_pred_tips
        action_id = ActionID(get_unique_action_id())
        add_to_schedule!(schedule, problem_spec, GO(get_id(robot_id), -1, -1,), action_id)
        add_edge!(schedule, pred_id, action_id)
    end
    sort!(schedule.root_nodes)
    propagate_valid_ids!(schedule,problem_spec)
    return schedule
end
function construct_project_schedule(
    project_spec::P,
    problem_spec::T,
    robot_ICs::Dict{Int,ROBOT_AT},
    assignments::V=Dict{Int,Int}()
    ) where {P<:ProjectSpec,T<:ProblemSpec,V<:Union{Dict{Int,Int},Vector{Int}}}
    project_schedule = construct_project_schedule(
        project_spec,
        problem_spec,
        project_spec.initial_conditions,
        project_spec.final_conditions,
        robot_ICs,
        assignments
    )
    return project_schedule
    # # TODO just repair a partial schedule by incorporating the assignments: much more straightforward!
    # project_schedule = construct_partial_project_schedule(
    #     project_spec,
    #     problem_spec,
    #     map(i->robot_ICs[i], 1:problem_spec.N)
    # )
    # tip_type = GO
    # G = get_graph(project_schedule)
    # for (robot_id,task_list) in assignment_dict
    #     v = get_vtx(project_schedule, RobotID(robot_id))
    #     node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
    #     done = false
    #     # find leaf node
    #     for task_idx in assignment_dict[v]
    #         for e in edges(bfs_parents(G, v))
    #             v2 = e.dst
    #             node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
    #             if typeof(node2) <: tip_type && outdegree(G,v2) < sum(values(required_predecessors(node)))
    #                 v = v2
    #                 break
    #             end
    #         end
    #         task_node =
    #         add_edge!(G,v,)
    #     end
    # end
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
    problem_spec::ProblemSpec,
    team_sizes=Dict{ObjectID,Int}()
    )

    # Construct Partial Project Schedule
    project_schedule = ProjectSchedule();
    for op in operations
        add_to_schedule!(project_schedule, problem_spec, op, get_operation_id(op))
    end
    for pred in object_ICs
        add_to_schedule!(project_schedule, problem_spec, pred, get_object_id(pred))
    end
    for pred in robot_ICs
        robot_id = get_robot_id(pred)
        add_to_schedule!(project_schedule, problem_spec, pred, robot_id)
        action_id = ActionID(get_unique_action_id())
        action = GO(r=robot_id,x1=get_location_id(pred))
        add_to_schedule!(project_schedule, problem_spec, action, action_id)
        add_edge!(project_schedule, robot_id, action_id)
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
            object_ic           = get_node_from_id(project_schedule, ObjectID(object_id))
            pickup_station_ids  = get_location_ids(object_ic)
            object_fc           = object_FCs[object_id]
            dropoff_station_ids = get_location_ids(object_fc)
            # TODO Handle collaborative tasks
            if length(pickup_station_ids) > 1 # COLLABORATIVE TRANSPORT
                # println("FORMULATING COLLABORATIVE TRANSPORT TASK")
                add_headless_delivery_task!(project_schedule,problem_spec,
                    ObjectID(object_id),operation_id,pickup_station_ids,dropoff_station_ids)
            else # SINGLE AGENT TRANSPORT
                # println("FORMULATING NON-COLLABORATIVE TRANSPORT TASK")
                add_headless_delivery_task!(project_schedule,problem_spec,
                    ObjectID(object_id),operation_id,pickup_station_ids[1],dropoff_station_ids[1])
            end
        end
        for object_id in get_output_ids(op)
            add_edge!(project_schedule, operation_id, ObjectID(object_id))
        end
    end
    # NOTE: A hack to get speed up on SparseAdjacencyMILP. Not sure if it will work
    for (id, pred) in get_object_ICs(project_schedule)
        v = get_vtx(project_schedule, ObjectID(id))
        if indegree(get_graph(project_schedule),v) == 0
            op_id = OperationID(get_unique_operation_id())
            op = Operation(post=Set{OBJECT_AT}([pred]),id=op_id)
            add_to_schedule!(project_schedule,op,op_id)
            add_edge!(project_schedule,op_id,ObjectID(id))
        end
    end
    set_leaf_operation_nodes!(project_schedule)
    project_schedule
end
function construct_partial_project_schedule(spec::ProjectSpec,problem_spec::ProblemSpec,robot_ICs=Vector{ROBOT_AT}())
    construct_partial_project_schedule(
        spec.initial_conditions,
        spec.final_conditions,
        map(i->robot_ICs[i], 1:min(length(robot_ICs),problem_spec.N)),
        spec.operations,
        map(op->op.id, spec.operations[collect(spec.root_nodes)]),
        problem_spec,
        spec.team_sizes
    )
end

"""
    `process_schedule(schedule::P) where {P<:ProjectSchedule}`

    Compute the optimistic start and end times, along with the slack associated
    with each vertex in the `schedule`. Slack for each vertex is represented as
    a vector in order to handle multi-headed projects.
"""
function process_schedule(schedule::P; t0=zeros(Int,nv(schedule)),
        tF=zeros(Int,nv(schedule))
    ) where {P<:ProjectSchedule}

    G = get_graph(schedule)
    traversal = topological_sort_by_dfs(G)
    n_roots = max(length(schedule.root_nodes),1)
    slack = map(i->Inf*ones(n_roots), vertices(G))
    local_slack = map(i->Inf*ones(n_roots), vertices(G))
    # max_deadlines = map(i->typemax(Int), vertices(G))
    # True terminal nodes
    for (i,v) in enumerate(schedule.root_nodes)
        slack[v] = Inf*ones(n_roots)
        slack[v][i] = 0 # only slack for corresponding head is set to 0
    end
    ########## Compute Lower Bounds Via Forward Dynamic Programming pass
    for v in traversal
        path_spec = schedule.path_specs[v]
        Δt_min = path_spec.min_path_duration
        for v2 in inneighbors(G,v)
            t0[v] = max(t0[v], tF[v2])
        end
        tF[v] = max(tF[v], t0[v] + Δt_min)
    end
    ########### Compute Slack Via Backward Dynamic Programming pass
    for v in reverse(traversal)
        for v2 in outneighbors(G,v)
            local_slack[v] = min.(local_slack[v], (t0[v2] - tF[v]) )
            slack[v] = min.(slack[v], slack[v2] .+ (t0[v2] - tF[v]) )
        end
    end
    t0,tF,slack,local_slack
end

export
    get_collect_node,
    get_deposit_node

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

################################################################################
############################## New Functionality ###############################
################################################################################

export
    TaskGraphsMILP,
    AssignmentMILP,
    AdjacencyMILP,
    SparseAdjacencyMILP

abstract type TaskGraphsMILP end
@with_kw struct AssignmentMILP <: TaskGraphsMILP
    model::JuMP.Model = Model()
end
@with_kw struct TeamAssignmentMILP <: TaskGraphsMILP
    model::JuMP.Model = Model()
    task_group::Vector{Vector{Int}}
end
@with_kw struct AdjacencyMILP <: TaskGraphsMILP
    model::JuMP.Model = Model()
    job_shop::Bool=false
end
@with_kw struct SparseAdjacencyMILP <: TaskGraphsMILP
    model::JuMP.Model = Model()
    Xa::SparseMatrixCSC{VariableRef,Int} = SparseMatrixCSC{VariableRef,Int}(0,0,ones(Int,1),Int[],VariableRef[]) # assignment adjacency matrix
    Xj::SparseMatrixCSC{VariableRef,Int} = SparseMatrixCSC{VariableRef,Int}(0,0,ones(Int,1),Int[],VariableRef[]) # job shop adjacency matrix
    job_shop::Bool=false
end
JuMP.optimize!(model::M) where {M<:TaskGraphsMILP}          = optimize!(model.model)
JuMP.termination_status(model::M) where {M<:TaskGraphsMILP} = termination_status(model.model)
JuMP.objective_function(model::M) where {M<:TaskGraphsMILP} = objective_function(model.model)
JuMP.objective_bound(model::M) where {M<:TaskGraphsMILP}    = objective_bound(model.model)
JuMP.primal_status(model::M) where {M<:TaskGraphsMILP}      = primal_status(model.model)
JuMP.dual_status(model::M) where {M<:TaskGraphsMILP}        = dual_status(model.model)
# for op = (:optimize!, :termination_status, :objective_function)
#     eval(quote
#         JuMP.$op(model::M,args...) where {M<:TaskGraphsMILP} = $op(model.model,args...)
#     end)
# end

export
    exclude_solutions!,
    exclude_current_solution!

"""
    `exclude_solutions!(model::JuMP.Model,forbidden_solutions::Vector{Matrix{Int}})`

    This is the key utility for finding the next best solution to the MILP
    problem. It simply excludes every specific solution passed to it. It requires
    that X be a binary matrix.
"""
function exclude_solutions!(model::JuMP.Model,X::Matrix{Int})
    @assert !any((X .< 0) .| (X .> 1))
    @constraint(model, sum(model[:X] .* X) <= sum(model[:X])-1)
end
exclude_solutions!(model::TaskGraphsMILP,args...) = exclude_solutions!(model.model, args...)
function exclude_solutions!(model::JuMP.Model,M::Int,forbidden_solutions::Vector{Matrix{Int}})
    for X in forbidden_solutions
        exclude_solutions!(model,X)
    end
end
function exclude_solutions!(model::JuMP.Model)
    if termination_status(model) != MOI.OPTIMIZE_NOT_CALLED
        X = get_assignment_matrix(model)
        exclude_solutions!(model,X)
    end
end
exclude_current_solution!(args...) = exclude_solutions!(args...)


export
    get_assignment_matrix,
    get_assignment_vector,
    get_assignment_dict

function get_assignment_matrix(model::M) where {M<:JuMP.Model}
    Matrix{Int}(min.(1, round.(value.(model[:X])))) # guarantees binary matrix
end
get_assignment_matrix(model::TaskGraphsMILP) = get_assignment_matrix(model.model)
function get_assignment_vector(assignment_matrix,M)
    assignments = -ones(Int,M)
    for j in 1:M
        r = findfirst(assignment_matrix[:,j] .== 1)
        if r != nothing
            assignments[j] = r
        end
    end
    assignments
end

"""
    Returns dictionary of the "joint mission", mapping each robot id to a
    sequence of tasks.
"""
function get_assignment_dict(assignment_matrix,N,M)
    assignment_dict = Dict{Int,Vector{Int}}()
    for i in 1:N
        vec = get!(assignment_dict,i,Int[])
        k = i
        j = 1
        while j <= M
            if assignment_matrix[k,j] == 1
                push!(vec,j)
                k = j + N
                j = 0
            end
            j += 1
        end
    end
    assignment_dict
    assignments = zeros(Int,M)
    for (robot_id, task_list) in assignment_dict
        for task_id in task_list
            assignments[task_id] = robot_id
        end
    end
    assignment_dict, assignments
end

export
    add_job_shop_constraints!,
    formulate_optimization_problem,
    formulate_schedule_milp,
    formulate_milp,
    update_project_schedule!

function add_job_shop_constraints!(schedule::P,spec::T,model::JuMP.Model) where {P<:ProjectSchedule,T<:ProblemSpec}
    # M = spec.M
    M = length(get_object_ICs(schedule))
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
function add_job_shop_constraints!(milp_model::AssignmentMILP,schedule::ProjectSchedule,spec::ProblemSpec,model::JuMP.Model)
    add_job_shop_constraints!(schedule,spec,model)
end

"""
    `formulate_optimization_problem()`

    Express the TaskGraphs assignment problem as a MILP using the JuMP optimization
    framework.

    Inputs:
        `G` - graph with inverted tree structure that encodes dependencies
            between tasks
        `D` - D[s₁,s₂] is distance from position s₁ to position s₂
        `Δt` - Δt[j] is the duraction of time that must elapse after all prereqs
            of task j have been satisfied before task j becomes available
        `Δt_collect` - Δt_collect[j] is the time required for a robot to pick up
            object j
        `Δt_deliver` - Δt_deliver[j] is the time required for a robot to set
            down object j
        `to0_` - a `Dict`, where `to0_[j]` gives the start time for task j
            (applies to leaf tasks only)
        `tr0_` - a `Dict`, where `tr0_[i]` gives the start time for robot i
            (applies to non-dummy robots only)
        `root_nodes` - a vector of integers specfying the graph vertices that
            are roots of the project
        `weights` - a vector of weights that determines the contribution of each
            root_node to the objective
        `s0` - pickup stations for the tasks
        `sF` - dropoff stations for the tasks
        `optimizer` - a JuMP optimizer (e.g., Gurobi.optimizer)
    Keyword Args:
        `TimeLimit=100`
        `OutputFlag=0`
        `assignments=Dict{Int64,Int64}()` - maps robot id to assignment that must be
            enforced
        `cost_model=:MakeSpan` - optimization objective, either `:MakeSpan` or
            `:SumOfMakeSpans`

    Outputs:
        `model` - the optimization model
"""
function formulate_optimization_problem(N,M,G,D,Δt,Δt_collect,Δt_deliver,to0_,tr0_,root_nodes,weights,r0,s0,sF,nR,optimizer;
    TimeLimit=100,
    OutputFlag=0,
    Presolve = -1, # automatic setting (-1), off (0), conservative (1), or aggressive (2)
    task_groups = Vector{Vector{Int}}(),
    shapes = [(1,1) for j in 1:M],
    assignments=Dict{Int64,Int64}(),
    cost_model=MakeSpan,
    t0_ = Dict(),
    tF_ = Dict(),
    Mm = 10000,
    # Mm = sum([D[s1,s2] for s1 in r0 for s2 in s0]) + sum([D[s1,s2] for s1 in s0 for s2 in sF])
    )

    model = Model(with_optimizer(optimizer,
        TimeLimit=TimeLimit,
        OutputFlag=OutputFlag,
        Presolve=Presolve
        ))

    r0 = vcat(r0,sF) # combine to get dummy robot ``spawn'' locations too
    @variable(model, to0[1:M] >= 0.0) # object availability time
    @variable(model, tor[1:M] >= 0.0) # object robot arrival time
    @variable(model, toc[1:M] >= 0.0) # object collection complete time
    @variable(model, tod[1:M] >= 0.0) # object deliver begin time
    @variable(model, tof[1:M] >= 0.0) # object termination time
    @variable(model, tr0[1:N+M] >= 0.0) # robot availability time

    # Assignment matrix x
    @variable(model, X[1:N+M,1:M], binary = true) # X[i,j] ∈ {0,1}
    @constraint(model, X * ones(M) .<= 1)         # each robot may have no more than 1 task
    @constraint(model, X' * ones(N+M) .== nR)     # each task must have exactly 1 assignment
    for (i,t) in tr0_
        # start time for robot i
        @constraint(model, tr0[i] == t)
    end
    for (j,t) in to0_
        # start time for task j (applies only to tasks with no prereqs)
        @constraint(model, to0[j] == t)
    end
    for (i,j) in assignments
        # start time for task j (applies only to tasks with no prereqs)
        @constraint(model, X[i,j] == 1)
    end
    # constraints
    for j in 1:M
        # constraint on task start time
        if !is_root_node(G,j)
            for v in inneighbors(G,j)
                @constraint(model, to0[j] >= tof[v] + Δt[j])
            end
        end
        # constraint on dummy robot start time (corresponds to moment of object delivery)
        @constraint(model, tr0[j+N] == tof[j])
        # dummy robots can't do upstream jobs
        upstream_jobs = [j, map(e->e.dst,collect(edges(bfs_tree(G,j;dir=:in))))...]
        for v in upstream_jobs
            @constraint(model, X[j+N,v] == 0)
        end
        # lower bound on task completion time (task can't start until it's available).
        @constraint(model, tor[j] >= to0[j])
        for i in 1:N+M
            @constraint(model, tor[j] - (tr0[i] + D[r0[i],s0[j]]) >= -Mm*(1 - X[i,j]))
        end
        @constraint(model, toc[j] == tor[j] + Δt_collect[j])
        @constraint(model, tod[j] == toc[j] + D[s0[j],sF[j]])
        @constraint(model, tof[j] == tod[j] + Δt_deliver[j])
        # "Job-shop" constraints specifying that no station may be double-booked. A station
        # can only support a single COLLECT or DEPOSIT operation at a time, meaning that all
        # the windows for these operations cannot overlap. In the constraints below, t1 and t2
        # represent the intervals for the COLLECT or DEPOSIT operations of tasks j and j2,
        # respectively. If eny of the operations for these two tasks require use of the same
        # station, we introduce a 2D binary variable y. if y = [1,0], the operation for task
        # j must occur before the operation for task j2. The opposite is true for y == [0,1].
        # We use the big M method here as well to tightly enforce the binary constraints.
        for j2 in j+1:M
            if (s0[j] == s0[j2]) || (s0[j] == sF[j2]) || (sF[j] == s0[j2]) || (sF[j] == sF[j2])
                # @show j, j2
                if s0[j] == s0[j2]
                    t1 = [tor[j], toc[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif s0[j] == sF[j2]
                    t1 = [tor[j], toc[j]]
                    t2 = [tod[j2], tof[j2]]
                elseif sF[j] == s0[j2]
                    t1 = [tod[j], tof[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif sF[j] == sF[j2]
                    t1 = [tod, tof[j]]
                    t2 = [tod, tof[j2]]
                end
                tmax = @variable(model)
                tmin = @variable(model)
                y = @variable(model, binary=true)
                @constraint(model, tmax >= t1[1])
                @constraint(model, tmax >= t2[1])
                @constraint(model, tmin <= t1[2])
                @constraint(model, tmin <= t2[2])

                @constraint(model, tmax - t2[1] <= (1 - y)*Mm)
                @constraint(model, tmax - t1[1] <= y*Mm)
                @constraint(model, tmin - t1[2] >= (1 - y)*-Mm)
                @constraint(model, tmin - t2[2] >= y*-Mm)
                @constraint(model, tmin + 1 <= tmax)
            end
        end
    end
    # cost depends only on root node(s)
    if cost_model == SumOfMakeSpans
        @variable(model, T[1:length(root_nodes)])
        for (i,project_head) in enumerate(root_nodes)
            for v in project_head
                @constraint(model, T[i] >= tof[v] + Δt[v])
            end
        end
        @objective(model, Min, sum(map(i->T[i]*get(weights,i,0.0), 1:length(root_nodes))))
        # @objective(model, Min, sum(map(v->tof[v]*get(weights,v,0.0), root_nodes)))
    elseif cost_model == MakeSpan
        @variable(model, T)
        @constraint(model, T .>= tof .+ Δt)
        @objective(model, Min, T)
    end
    AssignmentMILP(model)
end
function formulate_optimization_problem(spec::T,optimizer;
    kwargs...
    ) where {T<:ProblemSpec}
    formulate_optimization_problem(
        spec.N,
        spec.M,
        spec.graph,
        spec.D,
        spec.Δt,
        spec.Δt_collect,
        spec.Δt_deliver,
        spec.to0_,
        spec.tr0_,
        spec.root_nodes,
        spec.weights,
        spec.r0,
        spec.s0,
        spec.sF,
        spec.nR,
        optimizer;
        cost_model=spec.cost_function,
        kwargs...
        )
end
function formulate_milp(milp_model::AssignmentMILP,project_schedule::ProjectSchedule,problem_spec::ProblemSpec;
    optimizer=Gurobi.Optimizer,
    kwargs...)
    formulate_optimization_problem(problem_spec,optimizer;kwargs...)
end

export
    preprocess_project_schedule

"""
    preprocess_project_schedule
"""
function preprocess_project_schedule(project_schedule)
    G = get_graph(project_schedule);
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

    upstream_vertices = map(v->[v, map(e->e.dst,collect(edges(bfs_tree(G,v;dir=:in))))...], vertices(G))
    non_upstream_vertices = map(v->collect(setdiff(collect(vertices(G)),upstream_vertices[v])), vertices(G))

    return missing_successors, missing_predecessors, n_eligible_successors, n_eligible_predecessors, n_required_successors, n_required_predecessors, upstream_vertices, non_upstream_vertices
end

"""
    `formulate_schedule_milp`

    Formulate the adjacency matrix milp problem over the project schedule.
    #TODO: I am not positive that the current version guarantees t0[v2] <= tF[v]
    for all (v,v2) in edges(project_schedule.graph) following optimization.
"""
function formulate_schedule_milp(project_schedule::ProjectSchedule,problem_spec::ProblemSpec;
        optimizer = Gurobi.Optimizer,
        TimeLimit=100,
        OutputFlag=0,
        Presolve=-1, # automatic setting (-1), off (0), conservative (1), or aggressive (2)
        t0_ = Dict{AbstractID,Float64}(), # dictionary of initial times. Default is empty
        tF_ = Dict{AbstractID,Float64}(), # dictionary of initial times. Default is empty
        Mm = 10000, # for big M constraints
        cost_model = SumOfMakeSpans,
    )
    G = get_graph(project_schedule);
    assignments = [];
    Δt = map(v->get_path_spec(project_schedule, v).min_path_duration, vertices(G))

    model = Model(with_optimizer(optimizer,
        TimeLimit=TimeLimit,
        OutputFlag=OutputFlag,
        Presolve=Presolve
        ));
    @variable(model, t0[1:nv(G)] >= 0.0); # initial times for all nodes
    @variable(model, tF[1:nv(G)] >= 0.0); # final times for all nodes

    # Precedence relationships
    @variable(model, Xa[1:nv(G),1:nv(G)], binary = true); # Precedence Adjacency Matrix TODO make sparse
    @constraint(model, Xa .+ Xa' .<= 1) # no bidirectional or self edges
    # set all initial times that are provided
    for (id,t) in t0_
        v = get_vtx(project_schedule, id)
        @constraint(model, t0[v] >= t)
    end
    for (id,t) in tF_
        v = get_vtx(project_schedule, id)
        @constraint(model, tF[v] >= t)
    end
    # Precedence constraints and duration constraints for existing nodes and edges
    for v in vertices(G)
        @constraint(model, tF[v] >= t0[v] + Δt[v])
        for v2 in outneighbors(G,v)
            @constraint(model, Xa[v,v2] == 1)
            @constraint(model, t0[v2] >= tF[v]) # NOTE DO NOT CHANGE TO EQUALITY CONSTRAINT
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
    @constraint(model, Xa * ones(nv(G)) .<= n_eligible_successors);
    @constraint(model, Xa * ones(nv(G)) .>= n_required_successors);
    @constraint(model, Xa' * ones(nv(G)) .<= n_eligible_predecessors);
    @constraint(model, Xa' * ones(nv(G)) .>= n_required_predecessors);

    upstream_vertices = map(v->[v, map(e->e.dst,collect(edges(bfs_tree(G,v;dir=:in))))...], vertices(G))
    for v in vertices(G)
        for v2 in upstream_vertices[v]
            @constraint(model, Xa[v,v2] == 0) #TODO this variable is not needed
        end
    end
    non_upstream_vertices = map(v->collect(setdiff(collect(vertices(G)),upstream_vertices[v])), vertices(G))
    # Big M constraints
    for v in vertices(G)
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        for v2 in non_upstream_vertices[v] # for v2 in vertices(G)
            node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
            potential_match = false
            for (template, val) in missing_successors[v]
                if !matches_template(template, typeof(node2)) # possible to add an edge
                    continue
                end
                for (template2, val2) in missing_predecessors[v2]
                    if !matches_template(template2, typeof(node)) # possible to add an edge
                        continue
                    end
                    potential_match = true # TODO: investigate why the solver fails when this is moved inside the following if statement
                    if (val > 0 && val2 > 0)
                        new_node = align_with_successor(node,node2)
                        dt_min = generate_path_spec(project_schedule,problem_spec,new_node).min_path_duration
                        @constraint(model, tF[v] - (t0[v] + dt_min) >= -Mm*(1 - Xa[v,v2]))
                        @constraint(model, t0[v2] - tF[v] >= -Mm*(1 - Xa[v,v2]))
                        break
                    end
                end
            end
            if potential_match == false
                @constraint(model, Xa[v,v2] == 0) #TODO this variable is not needed
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
    # job_shop_variables = Dict{Tuple{Int,Int},JuMP.VariableRef}();
    @variable(model, Xj[1:nv(G),1:nv(G)], binary = true); # job shop adjacency matrix
    @constraint(model, Xj .+ Xj' .<= 1) # no bidirectional or self edges
    for v in 1:nv(G)
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        for v2 in v+1:nv(G)
            node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
            common_resources = intersect(resources_reserved(node),resources_reserved(node2))
            if length(common_resources) > 0
                # @show common_resources
                # tmax = @variable(model)
                # tmin = @variable(model)
                # y = @variable(model, binary=true)
                # job_shop_variables[(v,v2)] = y
                # @constraint(model, tmax >= t0[v])
                # @constraint(model, tmax >= t0[v2])
                # @constraint(model, tmin <= tF[v])
                # @constraint(model, tmin <= tF[v2])
                #
                # @constraint(model, tmax - t0[v2] <= (1 - y)*Mm)
                # @constraint(model, tmax - t0[v] <= y*Mm)
                # @constraint(model, tmin - tF[v] >= (1 - y)*-Mm)
                # @constraint(model, tmin - tF[v2] >= y*-Mm)
                # @constraint(model, tmin + 1 <= tmax)

                # Big M constraints
                @constraint(model, Xj[v,v2] + Xj[v2,v] == 1)
                if !(v2 in upstream_vertices[v])
                    @constraint(model, t0[v2] - tF[v] >= -Mm*(1 - Xj[v,v2]))
                end
                if !(v in upstream_vertices[v2])
                    @constraint(model, t0[v] - tF[v2] >= -Mm*(1 - Xj[v2,v]))
                end
            else
                @constraint(model, Xj[v,v2] == 0)
                @constraint(model, Xj[v2,v] == 0)
            end
        end
    end

    # Full adjacency matrix
    @variable(model, X[1:nv(G),1:nv(G)]); # Adjacency Matrix
    @constraint(model, X .== Xa .+ Xj)

    # Formulate Objective
    if cost_model == SumOfMakeSpans
        root_nodes = project_schedule.root_nodes
        @variable(model, T[1:length(root_nodes)])
        for (i,project_head) in enumerate(root_nodes)
            for v in project_head
                @constraint(model, T[i] >= tF[v])
            end
        end
        cost1 = @expression(model, sum(map(v->tF[v]*get(project_schedule.weights,v,0.0), root_nodes)))
    elseif cost_model == MakeSpan
        @variable(model, T)
        @constraint(model, T .>= tF)
        cost1 = @expression(model, T)
    end
    # sparsity_cost = @expression(model, (0.5/(nv(G)^2))*sum(Xa)) # cost term to encourage sparse X. Otherwise the solver may add pointless edges
    sparsity_cost = @expression(model, (0.5/(nv(G)^2))*sum(X)) # cost term to encourage sparse X. Otherwise the solver may add pointless edges
    @objective(model, Min, cost1 + sparsity_cost)
    # @objective(model, Min, cost1 )
    AdjacencyMILP(model=model) #, job_shop_variables
end
function formulate_milp(milp_model::AdjacencyMILP,project_schedule::ProjectSchedule,problem_spec::ProblemSpec;
    optimizer=Gurobi.Optimizer,
    kwargs...)
    formulate_schedule_milp(project_schedule,problem_spec;optimizer=optimizer,kwargs...)
end

"""
    Formulation for sparse adjacency MILP
"""
function formulate_milp(milp_model::SparseAdjacencyMILP,project_schedule::ProjectSchedule,problem_spec::ProblemSpec;
        optimizer = Gurobi.Optimizer,
        TimeLimit=100,
        OutputFlag=0,
        Presolve=-1, # automatic setting (-1), off (0), conservative (1), or aggressive (2)
        t0_ = Dict{AbstractID,Float64}(), # dictionary of initial times. Default is empty
        tF_ = Dict{AbstractID,Float64}(), # dictionary of initial times. Default is empty
        Mm = 10000, # for big M constraints
        cost_model = SumOfMakeSpans,
        job_shop=milp_model.job_shop
    )

    println("NBS TIME LIMIT: TimeLimit = $TimeLimit")
    model = Model(with_optimizer(optimizer,
        TimeLimit=TimeLimit,
        OutputFlag=OutputFlag,
        Presolve=Presolve
        ));

    G = get_graph(project_schedule);
    (missing_successors, missing_predecessors, n_eligible_successors, n_eligible_predecessors,
        n_required_successors, n_required_predecessors, upstream_vertices, non_upstream_vertices) = preprocess_project_schedule(project_schedule)
    Δt = map(v->get_path_spec(project_schedule, v).min_path_duration, vertices(G))

    @variable(model, t0[1:nv(G)] >= 0.0); # initial times for all nodes
    @variable(model, tF[1:nv(G)] >= 0.0); # final times for all nodes

    # Precedence relationships
    Xa = SparseMatrixCSC{VariableRef,Int}(nv(G),nv(G),ones(Int,nv(G)+1),Int[],VariableRef[])
    # @variable(model, Xa[1:nv(G),1:nv(G)], binary = true); # Precedence Adjacency Matrix TODO make sparse
    # @constraint(model, Xa .+ Xa' .<= 1) # no bidirectional or self edges
    # set all initial times that are provided
    for (id,t) in t0_
        v = get_vtx(project_schedule, id)
        @constraint(model, t0[v] >= t)
    end
    for (id,t) in tF_
        v = get_vtx(project_schedule, id)
        @constraint(model, tF[v] >= t)
    end
    # Precedence constraints and duration constraints for existing nodes and edges
    for v in vertices(G)
        @constraint(model, tF[v] >= t0[v] + Δt[v]) # NOTE Δt may change for some nodes
        for v2 in outneighbors(G,v)
            Xa[v,v2] = @variable(model, binary=true) # TODO remove this (MUST UPDATE n_eligible_successors, etc. accordingly)
            @constraint(model, Xa[v,v2] == 1) #TODO this edge already exists--no reason to encode it as a decision variable
            @constraint(model, t0[v2] >= tF[v]) # NOTE DO NOT CHANGE TO EQUALITY CONSTRAINT. Making this an equality constraint causes the solver to return a higher final value in some cases (e.g., toy problems 2,3,7). Why? Maybe the Big-M constraint forces it to bump up. I though the equality constraint might speed up the solver.
        end
    end

    # Big M constraints
    for v in vertices(G)
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        potential_match = false
        if outdegree(G,v) < n_eligible_successors[v] # NOTE: Trying this out to save time on formulation
            for v2 in non_upstream_vertices[v] # for v2 in vertices(G)
                if indegree(G,v2) < n_eligible_predecessors[v2]
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
                                potential_match = true
                                new_node = align_with_successor(node,node2)
                                dt_min = generate_path_spec(project_schedule,problem_spec,new_node).min_path_duration
                                Xa[v,v2] = @variable(model, binary=true) # initialize a new binary variable in the sparse adjacency matrix
                                @constraint(model, tF[v] - (t0[v] + dt_min) >= -Mm*(1 - Xa[v,v2]))
                                @constraint(model, t0[v2] - tF[v] >= -Mm*(1 - Xa[v,v2]))
                                break
                            end
                        end
                    end
                end
            end
        end
        if potential_match == false && job_shop == false
            @constraint(model, tF[v] == t0[v] + Δt[v]) # adding this constraint may provide some speedup
        end
    end

    # In the sparse implementation, these constraints must come after all possible edges are defined by a VariableRef
    @constraint(model, Xa * ones(nv(G)) .<= n_eligible_successors);
    @constraint(model, Xa * ones(nv(G)) .>= n_required_successors);
    @constraint(model, Xa' * ones(nv(G)) .<= n_eligible_predecessors);
    @constraint(model, Xa' * ones(nv(G)) .>= n_required_predecessors);
    for i in 1:nv(G)
        for j in i:nv(G)
            # prevent self-edges and cycles
            @constraint(model, Xa[i,j] + Xa[j,i] <= 1)
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
    # job_shop_variables = Dict{Tuple{Int,Int},JuMP.VariableRef}();
    Xj = SparseMatrixCSC{VariableRef,Int}(nv(G),nv(G),ones(Int,nv(G)+1),Int[],VariableRef[])
    if job_shop
        for v in 1:nv(G)
            node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
            for v2 in non_upstream_vertices[v] #v+1:nv(G)
                if v2 > v && ~(v in upstream_vertices[v2]) && ~(has_edge(G,v,v2) || has_edge(G,v2,v))
                    node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
                    common_resources = intersect(resources_reserved(node),resources_reserved(node2))
                    if length(common_resources) > 0
                        println("MILP FORMULATION: adding a job shop constraint between ",v, " (",string(node),") and ", v2, " (",string(node2),")")
                        # @show common_resources
                        # Big M constraints
                        Xj[v,v2] = @variable(model, binary=true) #
                        Xj[v2,v] = @variable(model, binary=true) # need both directions to have a valid adjacency matrix
                        @constraint(model, Xj[v,v2] + Xj[v2,v] == 1)
                        @constraint(model, t0[v2] - tF[v] >= -Mm*(1 - Xj[v,v2]))
                        @constraint(model, t0[v] - tF[v2] >= -Mm*(1 - Xj[v2,v]))
                    end
                end
            end
        end
    end

    # Full adjacency matrix
    @expression(model, X, Xa .+ Xj) # Make X an expression rather than a variable

    # Formulate Objective
    if cost_model == SumOfMakeSpans
        root_nodes = project_schedule.root_nodes
        @variable(model, T[1:length(root_nodes)])
        for (i,project_head) in enumerate(root_nodes)
            for v in project_head
                @constraint(model, T[i] >= tF[v])
            end
        end
        cost1 = @expression(model, sum(map(v->tF[v]*get(project_schedule.weights,v,0.0), root_nodes)))
    elseif cost_model == MakeSpan
        @variable(model, T)
        @constraint(model, T .>= tF)
        cost1 = @expression(model, T)
    end
    # sparsity_cost = @expression(model, (0.5/(nv(G)^2))*sum(Xa)) # cost term to encourage sparse X. Otherwise the solver may add pointless edges
    # sparsity_cost = @expression(model, (0.5/(nv(G)^2))*sum(X)) # cost term to encourage sparse X. Otherwise the solver may add pointless edges
    # @objective(model, Min, cost1 + sparsity_cost)
    @objective(model, Min, cost1)
    SparseAdjacencyMILP(model,Xa,Xj, milp_model.job_shop) #, job_shop_variables
end

export
    GreedyAssignment

"""
    GreedyAssignment - baseline for task assignment. It works by maintaining two
    "open" sets of vertices: one containing vertices that are eligible for a
"""
@with_kw struct GreedyAssignment{C} <: TaskGraphsMILP
    schedule::ProjectSchedule   = ProjectSchedule()
    problem_spec::ProblemSpec   = ProblemSpec()
    cost_model::C               = SumOfMakeSpans
    # X::M                        = sparse(zeros(Int,nv(project_schedule),nv(project_schedule)))
    t0::Vector{Int}             = zeros(Int,nv(schedule))
    # cost::Float64               = Inf
    # lower_bound::Float64        = Inf
end
exclude_solutions!(model::GreedyAssignment) = nothing # exclude most recent solution in order to get next best solution
JuMP.termination_status(model::GreedyAssignment)    = MOI.OPTIMAL
JuMP.primal_status(model::GreedyAssignment)         = MOI.FEASIBLE_POINT
get_assignment_matrix(model::GreedyAssignment)      = adjacency_matrix(get_graph(model.schedule))
function JuMP.objective_function(model::GreedyAssignment)
    t0,tF,slack,local_slack = process_schedule(model.schedule;t0=model.t0)
    if model.cost_model == MakeSpan
        return maximum(tF[model.schedule.root_nodes])
    elseif model.cost_model == SumOfMakeSpans
        return sum(tF[model.schedule.root_nodes] .* map(v->model.schedule.weights[v], model.schedule.root_nodes))
    end
    println("UNKNOWN COST FUNCTION!")
    return Inf
end
JuMP.objective_bound(model::GreedyAssignment)       = objective_function(model)
JuMP.value(c::Real) = c
function formulate_milp(milp_model::GreedyAssignment,project_schedule::ProjectSchedule,problem_spec::ProblemSpec;
        t0_ = Dict{AbstractID,Float64}(),
        cost_model = SumOfMakeSpans,
        kwargs...
    )

    model = GreedyAssignment(
        schedule = project_schedule,
        problem_spec = problem_spec,
        cost_model = cost_model
    )
    for (id,t) in t0_
        v = get_vtx(project_schedule, id)
        model.t0[v] = t
    end
    model
end
# function JuMP.optimize!(model::GreedyAssignment)
#
#     project_schedule    = model.schedule
#     problem_spec        = model.problem_spec
#     G                   = get_graph(project_schedule);
#     (missing_successors, missing_predecessors, n_eligible_successors, n_eligible_predecessors,
#         n_required_successors, n_required_predecessors, upstream_vertices, non_upstream_vertices) = preprocess_project_schedule(project_schedule)
#     downstream_vertices = map(v->[v, map(e->e.dst,collect(edges(bfs_tree(G,v;dir=:out))))...], vertices(G))
#     upstream_vertices = map(v->[v, map(e->e.dst,collect(edges(bfs_tree(G,v;dir=:in))))...], vertices(G))
#
#     traversal = topological_sort(G)
#     available_outgoing = Set{Int}(vertices(G))
#     required_incoming = Set{Int}()
#
#     for v in traversal
#         if outdegree(G,v) >= n_eligible_successors[v]
#             setdiff!(available_outgoing, v)
#         end
#         if indegree(G,v) < n_required_predecessors[v] # NOTE: Trying this out to save time on formulation
#             for v2 in downstream_vertices[v]
#                 if v2 != v
#                     setdiff!(available_outgoing, v2)
#                 end
#             end
#             push!(required_incoming, v)
#         end
#     end
#
#     @show length(available_outgoing)
#     println(string([string(get_node_from_vtx(project_schedule,v)) for v in available_outgoing]...))
#     @show length(required_incoming)
#     println(string([string(get_node_from_vtx(project_schedule,v)) for v in required_incoming]...))
#
#     # construct distance matrix
#     D = Inf * ones(nv(G),nv(G))
#     for v in vertices(G)
#         if outdegree(G,v) < n_eligible_successors[v] # NOTE: Trying this out to save time on formulation
#             node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
#             for v2 in non_upstream_vertices[v] # for v2 in vertices(G)
#                 if indegree(G,v2) < n_eligible_predecessors[v2]
#                     node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
#                     for (template, val) in missing_successors[v]
#                         if !matches_template(template, typeof(node2)) # possible to add an edge
#                             continue
#                         end
#                         for (template2, val2) in missing_predecessors[v2]
#                             if !matches_template(template2, typeof(node)) # possible to add an edge
#                                 continue
#                             end
#                             if (val > 0 && val2 > 0)
#                                 new_node = align_with_successor(node,node2)
#                                 D[v,v2] = generate_path_spec(project_schedule,problem_spec,new_node).min_path_duration
#                             end
#                         end
#                     end
#                 end
#             end
#         end
#     end
#
#
#     # downstream_vertices = Dict(v=>[v, map(e->e.dst,collect(edges(bfs_tree(G,v;dir=:out))))...] for v in available_outgoing)
#     # upstream_vertices = Dict(v=>[v, map(e->e.dst,collect(edges(bfs_tree(G,v;dir=:out))))...] for v in required_incoming)
#
#     assigned_outgoing = Set{Int}()
#     iters = 0
#     satisfied_set = Set{Int}()
#     while length(required_incoming) > 0 && iters < 100
#         iters += 1
#         @assert length(available_outgoing) > 0
#         println(string([string(get_node_from_vtx(project_schedule,v)) for v in available_outgoing]...))
#         @show length(available_outgoing)
#         # for v in traversal
#         for v in topological_sort(G)
#             node = get_node_from_vtx(project_schedule, v)
#             if !(v in satisfied_set) && isa(node,Union{COLLECT,TEAM_ACTION{COLLECT}})
#                 for v2 in [v, inneighbors(G,v)...]
#                     if indegree(G,v2) < n_required_predecessors[v2]
#                         # add possible input
#                         vtx = -1
#                         for v_ in available_outgoing
#                             if D[v_,v2] < get(D, (vtx,v2), Inf)
#                                 vtx = v_
#                             end
#                         end
#                         add_edge!(G,vtx,v2)
#                         setdiff!(available_outgoing, vtx)
#                         push!(assigned_outgoing, vtx)
#                     end
#                 end
#                 push!(satisfied_set, v)
#                 v2 = v
#                 while !isa(get_node_from_vtx(project_schedule, v2),Union{DEPOSIT,TEAM_ACTION{DEPOSIT}})
#                     @assert outdegree(G,v2) == 1
#                     v2 = outneighbors(G,v2)[1]
#                     @assert isa(get_node_from_vtx(project_schedule, v2),Union{DEPOSIT,CARRY})
#                 end
#                 for v3 in outneighbors(G,v2)
#                     if isa(get_node_from_vtx(project_schedule, v3), GO) && !(v3 in assigned_outgoing)
#                         push!(available_outgoing, v3)
#                     end
#                 end
#                 break
#             end
#         end
#     end
#     set_leaf_operation_nodes!(project_schedule)
#     update_project_schedule!(project_schedule,problem_spec,adjacency_matrix(G))
#     for e in edges(get_graph(project_schedule))
#         node1 = get_node_from_vtx(project_schedule,e.src)
#         node2 = get_node_from_vtx(project_schedule,e.dst)
#         @assert validate_edge(node1,node2)
#     end
#     model
# end

function construct_schedule_distance_matrix(project_schedule,problem_spec)
    G                   = get_graph(project_schedule);
    (missing_successors, missing_predecessors, n_eligible_successors, n_eligible_predecessors,
        n_required_successors, n_required_predecessors, upstream_vertices, non_upstream_vertices) = preprocess_project_schedule(project_schedule)
    D = Inf * ones(nv(G),nv(G))
    for v in vertices(G)
        if outdegree(G,v) < n_eligible_successors[v]
            node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
            # for v2 in vertices(G)
            for v2 in non_upstream_vertices[v]
                if indegree(G,v2) < n_eligible_predecessors[v2]
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
                                D[v,v2] = generate_path_spec(project_schedule,problem_spec,new_node).min_path_duration
                            end
                        end
                    end
                end
            end
        end
    end
    D
end

function update_greedy_sets!(model,G,C,Ai,Ao,v,n_required_predecessors,n_eligible_successors)
    if indegree(G,v) >= n_required_predecessors[v]
        push!(C,v)
        for v2 in inneighbors(G,v)
            if !(v2 in C)
                setdiff!(C,v)
            end
        end
    else
        push!(Ai,v)
        for v2 in inneighbors(G,v)
            if !(v2 in C)
                setdiff!(Ai,v)
            end
        end
    end
    if (outdegree(G,v) < n_eligible_successors[v]) && (v in C)
        push!(Ao,v)
    end
end

function select_next_edge(model,D,Ao,Ai)
    c = Inf
    a = -1
    b = -2
    for v in sort(collect(Ao))
        for v2 in sort(collect(Ai))
            if D[v,v2] < c
                c = D[v,v2]
                a = v
                b = v2
            end
        end
    end
    a,b
    if a < 0 || b < 0
        println("DEBUGGING edge selection for model ",typeof(model))
        for v in Ai
            node = get_node_from_vtx(model.schedule,v)
            println(string("node ",string(node), " needs assignment"))
            @show required_predecessors(node)
            @show indegree(model.schedule,v)
        end
    end
    a,b
end

function JuMP.optimize!(model::GreedyAssignment)

    project_schedule    = model.schedule
    problem_spec        = model.problem_spec
    G                   = get_graph(project_schedule);
    (_, _, n_eligible_successors, _, _, n_required_predecessors, _, _) = preprocess_project_schedule(project_schedule)
    D = construct_schedule_distance_matrix(project_schedule,problem_spec)

    C = Set{Int}()
    Ai = Set{Int}()
    Ao = Set{Int}()
    for v in topological_sort(G)
        update_greedy_sets!(model,G,C,Ai,Ao,v,n_required_predecessors,n_eligible_successors)
    end
    # while length(C) < nv(G)
    while length(Ai) > 0
        v,v2 = select_next_edge(model,D,Ao,Ai)
        setdiff!(Ao,v)
        setdiff!(Ai,v2)
        add_edge!(G,v,v2)
        for v in topological_sort(G)
            update_greedy_sets!(model,G,C,Ai,Ao,v,n_required_predecessors,n_eligible_successors)
        end
    end
    set_leaf_operation_nodes!(project_schedule)
    update_project_schedule!(project_schedule,problem_spec,adjacency_matrix(G))
    model
end

export
    propagate_valid_ids!

function propagate_valid_ids!(project_schedule::ProjectSchedule,problem_spec::ProblemSpec)
    G = get_graph(project_schedule)
    @assert(is_cyclic(G) == false, "is_cyclic(G)") # string(sparse(adj_matrix))
    # Propagate valid IDs through the schedule
    for v in topological_sort(G)
        # if !get_path_spec(project_schedule, v).fixed
            node_id = get_vtx_id(project_schedule, v)
            node = get_node_from_id(project_schedule, node_id)
            for v2 in inneighbors(G,v)
                node = align_with_predecessor(node,get_node_from_vtx(project_schedule, v2))
            end
            for v2 in outneighbors(G,v)
                node = align_with_successor(node,get_node_from_vtx(project_schedule, v2))
            end
            path_spec = get_path_spec(project_schedule, v)
            if path_spec.fixed
                replace_in_schedule!(project_schedule, path_spec, node, node_id)
            else
                replace_in_schedule!(project_schedule, problem_spec, node, node_id)
            end
        # end
    end
    # project_schedule
    return true
end

"""
    `update_project_schedule!`

    Args:
    - project_schedule
    - adj_matrix - adjacency_matrix encoding the edges that need to be added to
        the project schedule

    Adds all required edges to the project graph and modifies all nodes to
    reflect the appropriate valid IDs (e.g., `Action` nodes are populated with
    the correct `RobotID`s)
    Returns `false` if the new edges cause cycles in the project graph.
"""
function update_project_schedule!(project_schedule::P,problem_spec::T,adj_matrix,DEBUG::Bool=false) where {P<:ProjectSchedule,T<:ProblemSpec}
    # Add all new edges to project schedule
    G = get_graph(project_schedule)
    # remove existing edges first, so that there is no carryover between consecutive MILP iterations
    for e in collect(edges(G))
        rem_edge!(G, e)
    end
    # add all edges encoded by adjacency matrix
    for v in vertices(G)
        for v2 in vertices(G)
            if adj_matrix[v,v2] >= 1
                add_edge!(G,v,v2)
            end
        end
    end
    # DEBUG ? @assert(is_cyclic(G) == false) : nothing
    # @assert !is_cyclic(G) "update_project_schedule!() -------> is_cyclic(G)"
    try
        propagate_valid_ids!(project_schedule,problem_spec)
        @assert validate(project_schedule)
    catch e
        if isa(e, AssertionError)
            println(e.msg)
        else
            throw(e)
        end
        return false
    end
    return true
end
function update_project_schedule!(milp_model::M,
        project_schedule::P,
        problem_spec::T,
        adj_matrix,
        DEBUG::Bool=false
    ) where {M<:TaskGraphsMILP,P<:ProjectSchedule,T<:ProblemSpec}
    update_project_schedule!(project_schedule,problem_spec,adj_matrix,DEBUG)
end
function update_project_schedule!(milp_model::AssignmentMILP,
        project_schedule::P,
        problem_spec::T,
        assignment_matrix,
        DEBUG::Bool=false
    ) where {P<:ProjectSchedule,T<:ProblemSpec}
    N = length(get_robot_ICs(project_schedule))
    M = length(get_object_ICs(project_schedule))
    # assignment_dict, assignments = get_assignment_dict(assignment_matrix,problem_spec.N,problem_spec.M)
    assignment_dict, assignments = get_assignment_dict(assignment_matrix,N,M)
    G = get_graph(project_schedule)
    adj_matrix = adjacency_matrix(G)
    for (robot_id, task_list) in assignment_dict
        robot_node = get_node_from_id(project_schedule, RobotID(robot_id))
        v_go = outneighbors(G, get_vtx(project_schedule, RobotID(robot_id)))[1] # GO_NODE
        for object_id in task_list
            v_collect = outneighbors(G,get_vtx(project_schedule, ObjectID(object_id)))[1]
            adj_matrix[v_go,v_collect] = 1
            v_carry = outneighbors(G,v_collect)[1]
            v_deposit = outneighbors(G,v_carry)[1]
            for v in outneighbors(G,v_deposit)
                if isa(get_vtx_id(project_schedule, v), ActionID)
                    v_go = v
                    break
                end
            end
        end
    end
    update_project_schedule!(project_schedule,problem_spec,adj_matrix,DEBUG)
end

# function update_project_schedule!(milp_model::AssignmentMILP,project_schedule::P,problem_spec::T,adj_matrix,DEBUG::Bool=false) where {P<:ProjectSchedule,T<:ProblemSpec}
# end


end # module TaskGraphCore
