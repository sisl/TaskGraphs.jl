module TaskGraphs

using Parameters
using LightGraphs, MetaGraphs
using LinearAlgebra
using DataStructures
using JuMP


export
    Status,
    Location,
    BatteryLevel,
    EnvTime,
    ObjectID,
    RobotID,
    StationID,

    PlanningPredicate,
    OBJECT_AT,
    ROBOT_AT,

    PlanningAction,
    MOVE,
    TAKE,
    pre_conditions,
    add_conditions,
    delete_conditions,

    Operation,
    construct_operation,

    DeliveryTask,
    DeliveryGraph,
    construct_delivery_graph,

    ProjectSpec,
    get_initial_nodes,
    get_input_ids,
    get_output_ids,
    add_operation!,
    populate_lower_time_bound!,
    get_o,get_s,get_r,preconditions,postconditions,
    get_position,

    AgentState,
    ObjectState,
    EnvState,

    HighLevelEnvModel,
    get_distance,
    duration,
    get_charge_distance,

    AbstractRobotAction,
    GO,COLLECT,DEPOSIT,CHARGE,WAIT,COLLECT_AND_DELIVER,GO_AND_CHARGE,

    transition


const Status            = Symbol
const Status()          = :waiting
const Location          = Int
const Location()        = -1
const BatteryLevel      = Int
const BatteryLevel()    = -1
const EnvTime           = Int
const EnvTime()         = -1
const ObjectID          = Int
const ObjectID()        = -1
const RobotID           = Int
const RobotID()         = -1
const StationID         = Int
const StationID()       = -1


abstract type PlanningPredicate end
struct OBJECT_AT <: PlanningPredicate
    o::ObjectID
    s::StationID
end
get_o(pred::OBJECT_AT) = pred.o
get_s(pred::OBJECT_AT) = pred.s

struct CAN_CARRY <: PlanningPredicate
    r::RobotID
    o::ObjectID
end
get_r(pred::CAN_CARRY) = pred.r
get_o(pred::CAN_CARRY) = pred.o

struct ROBOT_AT <: PlanningPredicate
    r::RobotID
    s::StationID
end
get_r(pred::ROBOT_AT) = pred.r
get_s(pred::ROBOT_AT) = pred.s

abstract type PlanningAction end
struct MOVE <: PlanningAction
    r::RobotID
    s1::StationID # from
    s2::StationID # to
end
preconditions(a::MOVE) = Set{PlanningPredicate}([
        ROBOT_AT(a.r,a.s1)
        ])
add_conditions(a::MOVE) = Set{PlanningPredicate}([
        ROBOT_AT(a.r,a.s2)
        ])
delete_conditions(a::MOVE) = Set{PlanningPredicate}([
        ROBOT_AT(a.r,a.s1)
        ])

struct TAKE <: PlanningAction
    r::RobotID
    o::ObjectID
    s1::StationID # from
    s2::StationID # to
end
preconditions(a::TAKE) = Set{PlanningPredicate}([
        CAN_CARRY(a.r,a.o),
        OBJECT_AT(a.o,a.s1),
        ROBOT_AT(a.r,a.s1)
        ])
add_conditions(a::TAKE) = Set{PlanningPredicate}([
        ROBOT_AT(a.r,a.s2),
        OBJECT_AT(a.o,a.s2)
        ])
delete_conditions(a::TAKE) = Set{PlanningPredicate}([
        ROBOT_AT(a.r,a.s1),
        OBJECT_AT(a.o,a.s1)
        ])

@with_kw struct Operation
    pre::Set{OBJECT_AT}     = Set{OBJECT_AT}()
    post::Set{OBJECT_AT}    = Set{OBJECT_AT}()
    Δt::Float64             = 0
end
function construct_operation(station_id, input_ids, output_ids, Δt)
    Operation(
        Set{OBJECT_AT}(map(i->OBJECT_AT(i,station_id), input_ids)),
        Set{OBJECT_AT}(map(i->OBJECT_AT(i,station_id), output_ids)),
        Δt
    )
end
get_s(op::Operation) = get_s(rand(op.pre))
duration(op::Operation) = op.Δt
preconditions(op::Operation) = op.pre
postconditions(op::Operation) = op.post

"""
    ProjectSpec{G}

    Defines a list of operations that must be performed in order to complete a
    specific project, in addition to the dependencies between those operations
"""
@with_kw struct ProjectSpec{G}
    operations::Vector{Operation} = Vector{Operation}()
    pre_deps::Dict{Int,Set{Int}}  = Dict{Int,Set{Int}}() # id => (pre_conditions)
    post_deps::Dict{Int,Set{Int}} = Dict{Int,Set{Int}}()
    graph::G                      = DiGraph()
    M::Int                          = -1
end
get_initial_nodes(tg::ProjectSpec) = setdiff(
    Set(collect(vertices(tg.graph))),collect(keys(tg.pre_deps)))
get_input_ids(op::Operation) = Set{Int}([p.o for p in op.pre])
get_output_ids(op::Operation) = Set{Int}([p.o for p in op.post])
get_pre_deps(tg::ProjectSpec, i::ObjectID) = get(tg.pre_deps, i, Set{Int}())
get_post_deps(tg::ProjectSpec, i::ObjectID) = get(tg.post_deps, i, Set{Int}())
function add_pre_dep!(tg::ProjectSpec, i::ObjectID, op_idx::Int)
    push!(get!(tg.pre_deps, i, Set{Int}()),op_idx)
end
function add_post_dep!(tg::ProjectSpec, i::ObjectID, op_idx::Int)
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

"""
    DeliveryTask
"""
struct DeliveryTask
    o::ObjectID
    s1::StationID
    s2::StationID
end
"""
    DeliveryGraph{G}
"""
struct DeliveryGraph{G}
    tasks::Vector{DeliveryTask}
    graph::G
end
"""
    construct_delivery_graph(project_spec::ProjectSpec,M::Int)

    Assumes that station ids correspond to object ids.
"""
function construct_delivery_graph(project_spec::ProjectSpec,M::Int)
    delivery_graph = DeliveryGraph(Vector{DeliveryTask}(),MetaDiGraph(M))
    for op in project_spec.operations
        for i in get_input_ids(op)
            for j in get_output_ids(op)
                add_edge!(delivery_graph.graph, i, j)
                push!(delivery_graph.tasks,DeliveryTask(i,i,j))
            end
        end
    end
    delivery_graph
end

"""
    `AgentState`

    high-level state description for coarse planning
    - s : status (high-level operating mode)
    - x : position
    - b : battery level
    - t : time at which this state is reached
    - o : object being carried
"""
@with_kw struct AgentState
    s   ::Status        = Status()
    x   ::Location      = Location()
    b   ::BatteryLevel  = BatteryLevel()
    t   ::EnvTime       = EnvTime()
    o   ::ObjectID      = ObjectID() # object id that the robot is carrying
end
Base.copy(s::AgentState) = AgentState(s=s.s,x=s.x,b=s.b,t=s.t,o=s.o)
get_location(s::AgentState) = s.x

"""
    `ObjectState`

    high-level state description for coarse planning
"""
@with_kw struct ObjectState
    x   ::Location  = Location()
    r   ::RobotID   = RobotID() # robot id
    t   ::EnvTime   = EnvTime()
end
Base.copy(s::ObjectState) = ObjectState(x=s.x,r=s.r)
get_location(s::ObjectState) = s.x

"""
    `EnvState`

    high-level state description for coarse plannings.
    - action_queue stores robot indices action order (based on state.t, the time
        at which state is realized)
"""
@with_kw struct EnvState
    robot_states    ::Vector{AgentState} = Vector{AgentState}()
    object_states   ::Vector{ObjectState}= Vector{ObjectState}()
    action_queue    ::Vector{Int}        = Vector{Int}(collect(1:length(robot_states)))
end
Base.copy(env_state::EnvState) = EnvState(
    robot_states = [s for s in env_state.robot_states],
    object_states = [s for s in env_state.object_states],
    action_queue = copy(env_state.action_queue)
    )
function Base.sort!(env_state::EnvState)
    sortperm!(env_state.action_queue, [s.t for s in env_state.robot_states])
    env_state
end

"""
    `HighLevelEnvModel`

    high-level env description.
"""
struct HighLevelEnvModel{G}
    station_ids             ::Vector{Int}
    charger_ids             ::Vector{Int}
    graph                   ::G
    # cached helpers
    dist_matrix             ::Matrix{Int}
    dist_to_nearest_charger ::Vector{Int}
end
function compute_distance_matrix(graph::G where G,weight_mtx::M where M)
    D = zeros(Int,nv(graph),nv(graph))
    for v1 in vertices(graph)
        ds = dijkstra_shortest_paths(graph,v1,weight_mtx)
        D[v1,:] = ds.dists
    end
    D
end
function HighLevelEnvModel(station_ids::Vector{Int},charger_ids::Vector{Int},graph::G) where G
    D = compute_distance_matrix(graph,adjacency_matrix(graph))
    DC = minimum(D[charger_ids,:],dims=1)[:]
    HighLevelEnvModel(station_ids,charger_ids,graph,D,DC)
end
get_distance(model::HighLevelEnvModel, x1::Location, x2::Location) = model.dist_matrix[x1,x2]
duration(model::HighLevelEnvModel, x1::Location, x2::Location) = 1 * get_distance(model,x1,x2)
get_charge_distance(model::HighLevelEnvModel, x::Location) = minimum(model.dist_to_nearest_charger[x])

function can_carry(env_model::HighLevelEnvModel, r::RobotID, o::ObjectID)
    # TODO
    return true
end
function populate_lower_time_bound!(
        task_graph::ProjectSpec,
        env_state::EnvState,
        env_model::HighLevelEnvModel,
        t_lower::Vector{Int},
        op_idx::Int
    )
    op = task_graph.operations[op_idx]
    G = task_graph.graph
    t = 0
    if length(inneighbors(G,op_idx)) == 0
        for pred in preconditions(op)
            o = pred.o
            object_state = env_state.object_states[o]
            ox = get_location(object_state)
            sx = get_s(pred)
            t_pred = Inf
            for (r, robot_state) in enumerate(env_state.robot_states)
                # TODO: if robot r can carry object o
                if can_carry(env_model, r, o)
                    rx = get_location(robot_state)
                    t_pred = min(t_pred, duration(env_model,rx,ox))
                end
            end
            t_pred += duration(env_model,ox,sx)
            t = max(t_pred,t)
        end
    else
        for parent_op_idx in inneighbors(G,op_idx)
            parent_op = task_graph.operations[parent_op_idx]
            dt = 0
            for pred in postconditions(parent_op)
                dt = max(dt, duration(env_model,get_s(pred),get_s(op)))
            end
            t = max(t, duration(parent_op) + dt + populate_lower_time_bound!(
                    task_graph,env_state,env_model,t_lower,parent_op_idx))
        end
    end
    t_lower[op_idx] = t
    return t
end

# robot actions
abstract type AbstractRobotAction end
duration(model, state::AgentState, action::AbstractRobotAction) = 0
struct GO <: AbstractRobotAction # go to position x
    r::RobotID
    x::Location
end
struct COLLECT <: AbstractRobotAction # collect object o
    r::RobotID
    o::ObjectID
end
struct DEPOSIT <: AbstractRobotAction # deposit object o
    r::RobotID
    o::ObjectID
    x::Location
end
struct CHARGE <: AbstractRobotAction
    r::RobotID
    b::BatteryLevel
end
struct WAIT <: AbstractRobotAction
    r::RobotID
    t::EnvTime
end

struct COLLECT_AND_DELIVER <: AbstractRobotAction
    o   ::Int
    x1  ::Location # start location
    x2  ::Location # end location
end
struct GO_AND_CHARGE <: AbstractRobotAction
    x   ::Location
    b   ::BatteryLevel
end

duration(model, state::AgentState, action::GO) = duration(model, state.x, action.x)
duration(model, state::AgentState, action::CHARGE) = 0
function transition(model, state::AgentState, action::GO)
    next_state = AgentState(
        s = state.s,
        x = action.x,
        b = state.b - get_distance(model, state.x, action.x),
        t = state.t + duration(model, state, action),
        o = state.o
    )
end
function transition(model, state::AgentState, action::COLLECT)
    next_state = AgentState(
        s = state.s,
        x = state.x,
        b = state.b,
        t = state.t,
        o = action.o
    )
end
function transition(model, state::AgentState, action::DEPOSIT)
    next_state = AgentState(
        s = state.s,
        x = state.x,
        b = state.b,
        t = state.t,
        o = ObjectID()
    )
end
function transition(model, state::AgentState, action::CHARGE)
    next_state = AgentState(
        s = state.s,
        x = state.x,
        b = action.b,
        t = state.t + duration(model, state, action),
        o = state.o
    )
end
function transition(model, state::AgentState, action::WAIT)
    next_state = AgentState(
        s = state.s,
        x = state.x,
        b = state.b,
        t = state.t + action.t,
        o = state.o
    )
end
function transition(model, state::AgentState, action::COLLECT_AND_DELIVER)
    s = state
    s = transition(model, s, GO(action.x1))
    s = transition(model, s, COLLECT(action.o))
    s = transition(model, s, GO(action.x2))
    s = transition(model, s, DEPOSIT(action.o))
end
function transition(model, state::AgentState, action::GO_AND_CHARGE)
    s = state
    s = transition(model, s, GO(action.x))
    s = transition(model, s, CHARGE(action.b))
end

function transition(model, env_state::EnvState, agent_id::RobotID, action)
    next_env_state = copy(env_state)
    robot_state = env_state.robot_states[agent_id]
    next_robot_state = transition(model,robot_state,action)
    next_env_state.robot_states[agent_id] = next_robot_state
    next_env_state
end

function transition(model, env_state::EnvState, agent_id::RobotID, action::GO)
    next_env_state = copy(env_state)
    robot_state = env_state.robot_states[agent_id]
    next_robot_state = transition(model,robot_state,action)
    next_env_state.robot_states[agent_id] = next_robot_state
    if robot_state.o != -1
        # Object moves with robot
        next_env_state.object_states[robot_state.o] = ObjectState(
            x = next_robot_state.x,
            r = agent_id,
            t = next_robot_state.t
        )
    end
    next_env_state
end
function transition(model, env_state::EnvState, agent_id::RobotID, action::COLLECT)
    next_env_state = copy(env_state)
    robot_state = env_state.robot_states[agent_id]
    next_robot_state = transition(model,robot_state,action)
    next_env_state.robot_states[agent_id] = next_robot_state
    object_state = env_state.object_states[action.o]
    @assert(object_state.x == robot_state.x)
    next_env_state.object_states[action.o] = ObjectState(
        x = next_robot_state.x,
        r = agent_id,
        t = next_robot_state.t
    )
    next_env_state
end
function transition(model, env_state::EnvState, agent_id::RobotID, action::DEPOSIT)
    next_env_state = copy(env_state)
    robot_state = env_state.robot_states[agent_id]
    next_robot_state = transition(model,robot_state,action)
    next_env_state.robot_states[agent_id] = next_robot_state
    object_state = env_state.object_states[action.o]
    next_env_state.object_states[action.o] = ObjectState(
        x = next_robot_state.x,
        r = RobotID(),
        t = next_robot_state.t
    )
    next_env_state
end
function transition(model, env_state::EnvState, agent_id::RobotID, action::CHARGE)
    next_env_state = copy(env_state)
    robot_state = env_state.robot_states[agent_id]
    @assert(robot_state.x in env.charger_ids)
    next_env_state.robot_states[agent_id] = transition(model,robot_state,action)
    next_env_state
end
function transition(model, env_state::EnvState, agent_id::RobotID, action::COLLECT_AND_DELIVER)
    next_env_state = copy(env_state)
    robot_state = env_state.robot_states[agent_id]
    next_robot_state = transition(model,robot_state,action)
    next_env_state.robot_states[agent_id] = next_robot_state
    next_env_state.object_states[action.o] = ObjectState(
        x = next_robot_state.x,
        r = RobotID(),
        t = next_robot_state.t
    )
    next_env_state
end
function transition(model, env_state::EnvState, agent_id::RobotID, action::GO_AND_CHARGE)
    next_env_state = copy(env_state)
    robot_state = env_state.robot_states[agent_id]
    next_robot_state = transition(model,robot_state,action)
    next_env_state.robot_states[agent_id] = next_robot_state
    if robot_state.o != -1
        # Object moves with robot
        next_env_state.object_states[robot_state.o] = ObjectState(
            x = next_robot_state.x,
            r = agent_id
        )
    end
    next_env_state
end

include("utils.jl")

end
