module PlanningPredicates

using Parameters

export
    AbstractID,
    get_id,
    LocationID,
    ObjectID,
    RobotID,
    StationID,
    ActionID,
    OperationID,

    AbstractPlanningPredicate,
    OBJECT_AT,
    ROBOT_AT,
    get_object_id, get_location_id, get_robot_id,

    AbstractPlanningAction,
    MOVE,
    TAKE,
    pre_conditions,
    add_conditions,
    delete_conditions,

    transition


export
    reset_task_id_counter!,
    get_unique_task_id,
    reset_operation_id_counter!,
    get_unique_operation_id

TASK_ID_COUNTER = 0
get_unique_task_id() = Int(global TASK_ID_COUNTER += 1)
function reset_task_id_counter!()
    global TASK_ID_COUNTER = 0
end
OPERATION_ID_COUNTER = 0
get_unique_operation_id() = Int(global OPERATION_ID_COUNTER += 1)
function reset_operation_id_counter!()
    global OPERATION_ID_COUNTER = 0
end

""" Planning Resources (at the assignment level) """
abstract type AbstractResource end
abstract type ActiveResource <: AbstractResource end
abstract type PassiveResource <: AbstractResource end
""" Robot Types """
abstract type AbstractRobotType end
struct GenericRobot <: AbstractRobotType end
""" Object Types """
abstract type AbstractObjectType end
struct GenericObject <: AbstractObjectType end
""" Manufacturing Station Types """
abstract type AbstractStationType end
struct GenericStation <: AbstractStationType end
""" Loading Zone Types """
abstract type AbstractLoadingZoneType end
struct GenericLoadingZone <: AbstractLoadingZoneType end

@with_kw struct PlanningResource{T} <: AbstractResource
    id::Int 	= -1
    rtype::T 	= GenericRobot()
end
@with_kw struct RobotResource{T<:AbstractRobotType} <: AbstractResource
    id::Int 	= -1
    rtype::T 	= GenericRobot()
end
@with_kw struct ObjectResource{T<:AbstractObjectType} <: AbstractResource
    id::Int		= -1
    rtype::T	= GenericObject()
end
@with_kw struct LoadingZoneResource{T<:AbstractLoadingZoneType} <: AbstractResource
    id::Int		= -1
    rtype::T	= GenericLoadingZone()
end
struct StationResource{T<:AbstractStationType} <: AbstractResource
    id::Int
    rtype::T
end

get_id(r::R) where {R<:AbstractResource} = r.id
get_type(r::R) where {R<:AbstractResource} = r.rtype

function matches_resource_spec(required_type::T,required_id::Int,available::R) where {T<:DataType,R<:AbstractResource}
    if get_type(available) <: required_type
        if (get_id(available) == required_id) || (required_id == -1)
            return true
        end
    end
    return false
end
function matches_resource_spec(required::T,available::R) where {T<:AbstractResource,R<:AbstractResource}
    matches_resource_spec(get_type(required),get_id(required),available)
end

"""
	`ResourceTable`

	Defines all available resources
"""
struct ResourceTable
    robots::Dict{Int,RobotResource}
    objects::Dict{Int,ObjectResource}
    loading_zones::Dict{Int,LoadingZoneResource}
    stations::Dict{Int,StationResource}
end

export
    get_id,
    get_object_id,
    get_location_id,
    get_robot_id

abstract type AbstractID end
get_id(id::AbstractID) = id.id
get_id(id::Int) = id
Base.:+(id::A,i::Int) where {A<:AbstractID} = A(get_id(id)+i)
Base.:-(id::A,i::Int) where {A<:AbstractID} = A(get_id(id)-i)
@with_kw struct LocationID <: AbstractID
	id::Int = 0
end
@with_kw struct ObjectID <: AbstractID
	id::Int = 0
end
@with_kw struct RobotID <: AbstractID
	id::Int = 0
end
@with_kw struct StationID <: AbstractID
	id::Int = 0
end
@with_kw struct ActionID <: AbstractID
	id::Int = 0
end
@with_kw struct OperationID <: AbstractID
	id::Int = 0
end

abstract type AbstractPlanningPredicate end
struct OBJECT_AT <: AbstractPlanningPredicate
    o::ObjectID
    s::StationID
end
OBJECT_AT(o::Int,s::Int) = OBJECT_AT(ObjectID(o),StationID(s))
get_object_id(pred::OBJECT_AT) = pred.o
get_location_id(pred::OBJECT_AT) = pred.s

struct ROBOT_AT <: AbstractPlanningPredicate
    r::RobotID
    s::StationID
end
ROBOT_AT(r::Int,s::Int) = ROBOT_AT(RobotID(r),StationID(s))
get_robot_id(pred::ROBOT_AT) = pred.r
get_location_id(pred::ROBOT_AT) = pred.s

struct CAN_CARRY <: AbstractPlanningPredicate
    r::RobotID
    o::ObjectID
end
get_robot_id(pred::CAN_CARRY) = pred.r
get_object_id(pred::CAN_CARRY) = pred.o

export
    Operation,
    preconditions,
    postconditions,
    duration

@with_kw struct Operation
    pre::Set{OBJECT_AT}     = Set{OBJECT_AT}()
    post::Set{OBJECT_AT}    = Set{OBJECT_AT}()
    Δt::Int 				= 0
    station_id::StationID   = StationID(-1)
    id::OperationID         = OperationID(-1)
end
get_location_id(op::Operation) = op.station_id
duration(op::Operation) = op.Δt
preconditions(op::Operation) = op.pre
postconditions(op::Operation) = op.post
add_conditions(op::Operation) = op.post
delete_conditions(op::Operation) = op.pre
get_id(op::Operation) = get_id(op.id)

export
    AbstractRobotAction,
    GO,COLLECT,CARRY,DEPOSIT,
	get_initial_location_id, get_destination_location_id

abstract type AbstractRobotAction end
get_robot_id(a::A) where {A<:AbstractRobotAction} = a.r

struct GO <: AbstractRobotAction # go to position x
    r::RobotID
    x1::StationID
    x2::StationID
end
GO(r::Int,x1::Int,x2::Int) = GO(RobotID(r),StationID(x1),StationID(x2))

struct CARRY <: AbstractRobotAction # carry object o to position x
    r::RobotID
    o::ObjectID
    x1::StationID
    x2::StationID
end
CARRY(r::Int,o::Int,x1::Int,x2::Int) = CARRY(RobotID(r),ObjectID(o),StationID(x1),StationID(x2))

struct COLLECT <: AbstractRobotAction
    r::RobotID
    o::ObjectID
    x::StationID
end
COLLECT(r::Int,o::Int,x::Int) = COLLECT(RobotID(r),ObjectID(o),StationID(x))

struct DEPOSIT <: AbstractRobotAction
    r::RobotID
    o::ObjectID
    x::StationID
end
DEPOSIT(r::Int,o::Int,x::Int) = DEPOSIT(RobotID(r),ObjectID(o),StationID(x))

get_initial_location_id(a::A) where {A<:Union{GO,CARRY}}        	= a.x1
get_destination_location_id(a::A) where {A<:Union{GO,CARRY}}    	= a.x2
get_location_id(a::A) where {A<:Union{COLLECT,DEPOSIT}}             = a.x
get_initial_location_id(a::A) where {A<:Union{COLLECT,DEPOSIT}}     = a.x
get_destination_location_id(a::A) where {A<:Union{COLLECT,DEPOSIT}} = a.x
get_object_id(a::A) where {A<:Union{CARRY,COLLECT,DEPOSIT}}         = a.o

"""
    COLLABORATE

    Collaborative action (CARRY, COLLECT, or DEPOSIT).

    `robots` specifies the ordered list of robots in the team
    `configuration` specifies their configuration relative to the object center
    `a` provides the underlying action (CARRY, COLLECT, DEPOSIT)
"""
struct COLLABORATE{A} <: AbstractRobotAction
    robots::Vector{RobotID}
    configuration::Vector{Tuple{Int}}
    a::A
end

get_robot_id(a::A) where {A<:COLLABORATE}                           = a.robots
get_initial_location_id(a::A) where {A<:COLLABORATE}                = get_initial_location_id(a.a)
get_destination_location_id(a::A) where {A<:COLLABORATE}            = get_destination_location_id(a.a)
get_object_id(a::A) where {A<:COLLABORATE}                          = get_object_id(a.a)

# abstract type AbstractPlanningAction end
# struct MOVE <: AbstractPlanningAction
#     r::RobotID
#     s1::StationID # from
#     s2::StationID # to
# end
# preconditions(a::MOVE) = Set{AbstractPlanningPredicate}([
#         ROBOT_AT(a.r,a.s1)
#         ])
# add_conditions(a::MOVE) = Set{AbstractPlanningPredicate}([
#         ROBOT_AT(a.r,a.s2)
#         ])
# delete_conditions(a::MOVE) = Set{AbstractPlanningPredicate}([
#         ROBOT_AT(a.r,a.s1)
#         ])
#
# struct TAKE <: AbstractPlanningAction
#     r::RobotID
#     o::ObjectID
#     s1::StationID
#     s2::StationID
# end
# preconditions(a::TAKE) = Set{AbstractPlanningPredicate}([
#         CAN_CARRY(a.r,a.o),
#         OBJECT_AT(a.o,a.s1),
#         ROBOT_AT(a.r,a.s1)
#         ])
# add_conditions(a::TAKE) = Set{AbstractPlanningPredicate}([
#         ROBOT_AT(a.r,a.s2),
#         OBJECT_AT(a.o,a.s2)
#         ])
# delete_conditions(a::TAKE) = Set{AbstractPlanningPredicate}([
#         ROBOT_AT(a.r,a.s1),
#         OBJECT_AT(a.o,a.s1)
#         ])

# const Status            = Symbol
# const Status()          = :waiting
# struct BatteryLevel
# 	b::Int
# end
# struct EnvTime
# 	t::Int
# end

# struct CHARGE <: AbstractRobotAction
#     r::RobotID
#     b::BatteryLevel
# end
# struct WAIT <: AbstractRobotAction
#     r::RobotID
#     t::EnvTime
# end
#
# struct COLLECT_AND_DELIVER <: AbstractRobotAction
#     o   ::Int
#     x1  ::LocationID # start location
#     x2  ::LocationID # end location
# end
# struct GO_AND_CHARGE <: AbstractRobotAction
#     x   ::LocationID
#     b   ::BatteryLevel
# end

# """
#     `AgentState`
#
#     high-level state description for coarse planning
#     - s : status (high-level operating mode)
#     - x : position
#     - b : battery level
#     - t : time at which this state is reached
#     - o : object being carried
# """
# @with_kw struct AgentState
#     s   ::Status        = Status()
#     x   ::LocationID      = LocationID()
#     b   ::BatteryLevel  = BatteryLevel()
#     t   ::EnvTime       = EnvTime()
#     o   ::ObjectID      = ObjectID() # object id that the robot is carrying
# end
# Base.copy(s::AgentState) = AgentState(s=s.s,x=s.x,b=s.b,t=s.t,o=s.o)
# get_location(s::AgentState) = s.x
#
# """
#     `ObjectState`
#
#     high-level state description for coarse planning
# """
# @with_kw struct ObjectState
#     x   ::LocationID  = LocationID()
#     r   ::RobotID   = RobotID() # robot id
#     t   ::EnvTime   = EnvTime()
# end
# Base.copy(s::ObjectState) = ObjectState(x=s.x,r=s.r)
# get_location(s::ObjectState) = s.x
#
# """
#     `EnvState`
#
#     high-level state description for coarse plannings.
#     - action_queue stores robot indices action order (based on state.t, the time
#         at which state is realized)
# """
# @with_kw struct EnvState
#     robot_states    ::Vector{AgentState} = Vector{AgentState}()
#     object_states   ::Vector{ObjectState}= Vector{ObjectState}()
#     action_queue    ::Vector{Int}        = Vector{Int}(collect(1:length(robot_states)))
# end
# Base.copy(env_state::EnvState) = EnvState(
#     robot_states = [s for s in env_state.robot_states],
#     object_states = [s for s in env_state.object_states],
#     action_queue = copy(env_state.action_queue)
#     )
# function Base.sort!(env_state::EnvState)
#     sortperm!(env_state.action_queue, [s.t for s in env_state.robot_states])
#     env_state
# end
#
# """
#     `HighLevelEnvModel`
#
#     high-level env description.
# """
# struct HighLevelEnvModel{G}
#     station_ids             ::Vector{Int}
#     charger_ids             ::Vector{Int}
#     graph                   ::G
#     # cached helpers
#     dist_matrix             ::Matrix{Int}
#     dist_to_nearest_charger ::Vector{Int}
# end
#
# function HighLevelEnvModel(station_ids::Vector{Int},charger_ids::Vector{Int},graph::G) where G
#     D = get_dist_matrix(graph,adjacency_matrix(graph))
#     DC = minimum(D[charger_ids,:],dims=1)[:]
#     HighLevelEnvModel(station_ids,charger_ids,graph,D,DC)
# end
# get_distance(model::HighLevelEnvModel, x1::LocationID, x2::LocationID) = model.dist_matrix[x1,x2]
# duration(model::HighLevelEnvModel, x1::LocationID, x2::LocationID) = 1 * get_distance(model,x1,x2)
# get_charge_distance(model::HighLevelEnvModel, x::LocationID) = minimum(model.dist_to_nearest_charger[x])

# duration(model, state::AgentState, action::GO) = duration(model, state.x, action.x)
# duration(model, state::AgentState, action::CHARGE) = 0
# function transition(model, state::AgentState, action::GO)
#     next_state = AgentState(
#         s = state.s,
#         x = action.x,
#         b = state.b - get_distance(model, state.x, action.x),
#         t = state.t + duration(model, state, action),
#         o = state.o
#     )
# end
# function transition(model, state::AgentState, action::COLLECT)
#     next_state = AgentState(
#         s = state.s,
#         x = state.x,
#         b = state.b,
#         t = state.t,
#         o = action.o
#     )
# end
# function transition(model, state::AgentState, action::DEPOSIT)
#     next_state = AgentState(
#         s = state.s,
#         x = state.x,
#         b = state.b,
#         t = state.t,
#         o = ObjectID()
#     )
# end
# function transition(model, state::AgentState, action::CHARGE)
#     next_state = AgentState(
#         s = state.s,
#         x = state.x,
#         b = action.b,
#         t = state.t + duration(model, state, action),
#         o = state.o
#     )
# end
# function transition(model, state::AgentState, action::WAIT)
#     next_state = AgentState(
#         s = state.s,
#         x = state.x,
#         b = state.b,
#         t = state.t + action.t,
#         o = state.o
#     )
# end
# function transition(model, state::AgentState, action::COLLECT_AND_DELIVER)
#     s = state
#     s = transition(model, s, GO(action.x1))
#     s = transition(model, s, COLLECT(action.o))
#     s = transition(model, s, GO(action.x2))
#     s = transition(model, s, DEPOSIT(action.o))
# end
# function transition(model, state::AgentState, action::GO_AND_CHARGE)
#     s = state
#     s = transition(model, s, GO(action.x))
#     s = transition(model, s, CHARGE(action.b))
# end
#
# function transition(model, env_state::EnvState, agent_id::RobotID, action)
#     next_env_state = copy(env_state)
#     robot_state = env_state.robot_states[agent_id]
#     next_robot_state = transition(model,robot_state,action)
#     next_env_state.robot_states[agent_id] = next_robot_state
#     next_env_state
# end
#
# function transition(model, env_state::EnvState, agent_id::RobotID, action::GO)
#     next_env_state = copy(env_state)
#     robot_state = env_state.robot_states[agent_id]
#     next_robot_state = transition(model,robot_state,action)
#     next_env_state.robot_states[agent_id] = next_robot_state
#     if robot_state.o != -1
#         # Object moves with robot
#         next_env_state.object_states[robot_state.o] = ObjectState(
#             x = next_robot_state.x,
#             r = agent_id,
#             t = next_robot_state.t
#         )
#     end
#     next_env_state
# end
# function transition(model, env_state::EnvState, agent_id::RobotID, action::COLLECT)
#     next_env_state = copy(env_state)
#     robot_state = env_state.robot_states[agent_id]
#     next_robot_state = transition(model,robot_state,action)
#     next_env_state.robot_states[agent_id] = next_robot_state
#     object_state = env_state.object_states[action.o]
#     @assert(object_state.x == robot_state.x)
#     next_env_state.object_states[action.o] = ObjectState(
#         x = next_robot_state.x,
#         r = agent_id,
#         t = next_robot_state.t
#     )
#     next_env_state
# end
# function transition(model, env_state::EnvState, agent_id::RobotID, action::DEPOSIT)
#     next_env_state = copy(env_state)
#     robot_state = env_state.robot_states[agent_id]
#     next_robot_state = transition(model,robot_state,action)
#     next_env_state.robot_states[agent_id] = next_robot_state
#     object_state = env_state.object_states[action.o]
#     next_env_state.object_states[action.o] = ObjectState(
#         x = next_robot_state.x,
#         r = RobotID(),
#         t = next_robot_state.t
#     )
#     next_env_state
# end
# function transition(model, env_state::EnvState, agent_id::RobotID, action::CHARGE)
#     next_env_state = copy(env_state)
#     robot_state = env_state.robot_states[agent_id]
#     @assert(robot_state.x in env.charger_ids)
#     next_env_state.robot_states[agent_id] = transition(model,robot_state,action)
#     next_env_state
# end
# function transition(model, env_state::EnvState, agent_id::RobotID, action::COLLECT_AND_DELIVER)
#     next_env_state = copy(env_state)
#     robot_state = env_state.robot_states[agent_id]
#     next_robot_state = transition(model,robot_state,action)
#     next_env_state.robot_states[agent_id] = next_robot_state
#     next_env_state.object_states[action.o] = ObjectState(
#         x = next_robot_state.x,
#         r = RobotID(),
#         t = next_robot_state.t
#     )
#     next_env_state
# end
# function transition(model, env_state::EnvState, agent_id::RobotID, action::GO_AND_CHARGE)
#     next_env_state = copy(env_state)
#     robot_state = env_state.robot_states[agent_id]
#     next_robot_state = transition(model,robot_state,action)
#     next_env_state.robot_states[agent_id] = next_robot_state
#     if robot_state.o != -1
#         # Object moves with robot
#         next_env_state.object_states[robot_state.o] = ObjectState(
#             x = next_robot_state.x,
#             r = agent_id
#         )
#     end
#     next_env_state
# end

end # module PlanningPredicates
