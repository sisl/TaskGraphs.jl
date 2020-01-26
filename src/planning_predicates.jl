module PlanningPredicates

using Parameters

export
    AbstractID,
    get_id,
    LocationID,
    ObjectID,
    RobotID,
    TerminalRobotID,
    StationID,
    ActionID,
    OperationID,

    AbstractPlanningPredicate,
    OBJECT_AT,
    ROBOT_AT,
	TERMINAL_ROBOT_AT,
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
    get_unique_operation_id,
    reset_action_id_counter!,
    get_unique_action_id

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
ACTION_ID_COUNTER = 0
get_unique_action_id() = Int(global ACTION_ID_COUNTER += 1)
function reset_action_id_counter!()
    global ACTION_ID_COUNTER = 0
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
	id::Int = -1
end
@with_kw struct ObjectID <: AbstractID
	id::Int = -1
end
@with_kw struct RobotID <: AbstractID
	id::Int = -1
end
@with_kw struct TerminalRobotID <: AbstractID
	id::Int = -1
end
@with_kw struct StationID <: AbstractID
	id::Int = -1
end
@with_kw struct ActionID <: AbstractID
	id::Int = -1
end
@with_kw struct OperationID <: AbstractID
	id::Int = -1
end

abstract type AbstractPlanningPredicate end
struct OBJECT_AT <: AbstractPlanningPredicate
    o::ObjectID
    x::StationID
end
OBJECT_AT(o::Int,x::Int) = OBJECT_AT(ObjectID(o),StationID(x))
get_object_id(pred::OBJECT_AT) = pred.o
get_location_id(pred::OBJECT_AT) = pred.x

struct ROBOT_AT <: AbstractPlanningPredicate
    r::RobotID
    x::StationID
end
ROBOT_AT(r::Int,x::Int) = ROBOT_AT(RobotID(r),StationID(x))
get_robot_id(pred::ROBOT_AT) = pred.r
get_location_id(pred::ROBOT_AT) = pred.x

struct TERMINAL_ROBOT_AT <: AbstractPlanningPredicate
    r::TerminalRobotID
    x::StationID
end
TERMINAL_ROBOT_AT(r::Int,x::Int) = TERMINAL_ROBOT_AT(TerminalRobotID(r),StationID(x))
get_robot_id(pred::TERMINAL_ROBOT_AT) = pred.r
get_location_id(pred::TERMINAL_ROBOT_AT) = pred.x

struct CAN_CARRY <: AbstractPlanningPredicate
    r::RobotID
    o::ObjectID
end
get_robot_id(pred::CAN_CARRY) = pred.r
get_object_id(pred::CAN_CARRY) = pred.o

export
    Operation,
	get_operation_id,
    preconditions,
    postconditions,
    duration

@with_kw struct Operation <: AbstractPlanningPredicate
    pre::Set{OBJECT_AT}     = Set{OBJECT_AT}()
    post::Set{OBJECT_AT}    = Set{OBJECT_AT}()
    Δt::Int 				= 0
    station_id::StationID   = StationID(-1)
    id::OperationID         = OperationID(-1)
end
get_location_id(op::Operation) = op.station_id
get_operation_id(op::Operation) = op.id
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

abstract type AbstractRobotAction <: AbstractPlanningPredicate end
get_robot_id(a::A) where {A<:AbstractRobotAction} = a.r

@with_kw struct GO <: AbstractRobotAction # go to position x
    r::RobotID 		= RobotID()
    x1::StationID	= StationID()
    x2::StationID	= StationID()
end
GO(r::Int,x1::Int,x2::Int) = GO(RobotID(r),StationID(x1),StationID(x2))

@with_kw struct CARRY <: AbstractRobotAction # carry object o to position x
    r::RobotID		= RobotID()
    o::ObjectID		= ObjectID()
    x1::StationID	= StationID()
    x2::StationID	= StationID()
end
CARRY(r::Int,o::Int,x1::Int,x2::Int) = CARRY(RobotID(r),ObjectID(o),StationID(x1),StationID(x2))

@with_kw struct COLLECT <: AbstractRobotAction
    r::RobotID 		= RobotID()
    o::ObjectID 	= ObjectID()
    x::StationID 	= StationID()
end
COLLECT(r::Int,o::Int,x::Int) = COLLECT(RobotID(r),ObjectID(o),StationID(x))

@with_kw struct DEPOSIT <: AbstractRobotAction
    r::RobotID 		= RobotID()
    o::ObjectID 	= ObjectID()
    x::StationID 	= StationID()
end
DEPOSIT(r::Int,o::Int,x::Int) = DEPOSIT(RobotID(r),ObjectID(o),StationID(x))

get_initial_location_id(a::A) where {A<:Union{GO,CARRY}}        						= a.x1
get_destination_location_id(a::A) where {A<:Union{GO,CARRY}}    						= a.x2
get_location_id(a::A) where {A<:Union{COLLECT,DEPOSIT}}             					= a.x
get_initial_location_id(a::A) where {A<:Union{COLLECT,DEPOSIT,ROBOT_AT,OBJECT_AT}}     	= a.x
get_destination_location_id(a::A) where {A<:Union{COLLECT,DEPOSIT,ROBOT_AT,OBJECT_AT}} 	= a.x
get_object_id(a::A) where {A<:Union{CARRY,COLLECT,DEPOSIT}}         					= a.o

"""
    For collaborative tasks

    TODO: 
    - in the project schedule, multiple GO nodes should point to a TEAM_GO node. 
    - Add a preprocessing step (on the project schedule) where the incoming GO nodes 
        either become "remain in place" (might cause some problems with the default 
        path_spec.free/static/etc--be sure to check) or get deleted so the TEAM_GO node 
        can take care of getting the robots there all at once.
    - [ROBOT_AT, ...] -> TEAM_GO -> TEAM_CARRY -> TEAM_DEPOSIT -> [GO, ...]
    - implement a custom flow, meta-agent, or other model to move the full team of robots to 
        the goal configuration (for TEAM_GO node) simultaneously. Meta-agent path planning 
        might require positive CBS constraints so that paths of different start lengths can 
        go into the search.
    - TEAM_CARRY must be done on a different graph, but with the ability to check constraints 
        between them. Maybe simplest if actions can actually be indexed by :NORTH, :SOUTH, etc. 
    - there should be a way to prove that the milp assignment (if each robot is actually 
        assigned to a particular spot in the configuration) can be realized if all conflicts
        (except between collaborating team members) are ignored for a TEAM_GO task. This is 
        because of the "push-and-rotate" thing once they reach the goal vertices.
"""
@with_kw TEAM_ACTION{A<:AbstractRobotAction}
    n::Int = 2 # number of robots
    instructions::Vector{A} = Vector{GO}()
    # config::Matrix{Int} = ones(n) # defines configuration of agents relative to each other during collaborative task
end


export
    eligible_successors,
    eligible_predecessors,
    required_predecessors,
    required_successors,
	matches_template,
    resources_reserved,
	align_with_predecessor,
	align_with_successor

eligible_predecessors(node::GO)         = Dict((ROBOT_AT,DEPOSIT)=>1)
eligible_successors(node::GO)           = Dict(COLLECT=>1)
eligible_predecessors(node::COLLECT)    = Dict(OBJECT_AT=>1,GO=>1)
eligible_successors(node::COLLECT)      = Dict(CARRY=>1)
eligible_predecessors(node::CARRY)      = Dict(COLLECT=>1)
eligible_successors(node::CARRY)        = Dict(DEPOSIT=>1)
eligible_predecessors(node::DEPOSIT)    = Dict(CARRY=>1)
eligible_successors(node::DEPOSIT)      = Dict(Operation=>1,GO=>1)
eligible_predecessors(node::Operation)  = Dict(DEPOSIT=>length(node.pre))
eligible_successors(node::Operation)    = Dict(OBJECT_AT=>length(node.post))
eligible_predecessors(node::OBJECT_AT)  = Dict(Operation=>1)
eligible_successors(node::OBJECT_AT)    = Dict(COLLECT=>1)
eligible_predecessors(node::ROBOT_AT)   = Dict()
eligible_successors(node::ROBOT_AT)     = Dict(GO=>1)

required_predecessors(node::GO)         = Dict((ROBOT_AT,DEPOSIT)=>1)
# required_successors(node::GO)           = Dict(COLLECT=>1)
required_successors(node::GO)           = Dict()
required_predecessors(node::COLLECT)    = Dict(OBJECT_AT=>1,GO=>1)
required_successors(node::COLLECT)      = Dict(CARRY=>1)
required_predecessors(node::CARRY)      = Dict(COLLECT=>1)
required_successors(node::CARRY)        = Dict(DEPOSIT=>1)
required_predecessors(node::DEPOSIT)    = Dict(CARRY=>1)
# required_successors(node::DEPOSIT)      = Dict(Operation=>1)
required_successors(node::DEPOSIT)      = Dict(Operation=>1,GO=>1)
required_predecessors(node::Operation)  = Dict(DEPOSIT=>length(node.pre))
required_successors(node::Operation)    = Dict(OBJECT_AT=>length(node.post))
required_predecessors(node::OBJECT_AT)  = Dict()
required_successors(node::OBJECT_AT)    = Dict(COLLECT=>1)
required_predecessors(node::ROBOT_AT)   = Dict()
# required_successors(node::ROBOT_AT)     = Dict()
required_successors(node::ROBOT_AT)     = Dict(GO=>1)

matches_template(template,node) = false
matches_template(template::T,node::T) where {T} = true
matches_template(template::DataType,node::DataType) = template == node
matches_template(template::T,node::DataType) where {T<:Tuple} = (node in template)
matches_template(template::T,node) where {T<:Tuple} = matches_template(template, typeof(node))

resources_reserved(node)                = AbstractID[]
resources_reserved(node::COLLECT)       = AbstractID[get_location_id(node)]
resources_reserved(node::DEPOSIT)       = AbstractID[get_location_id(node)]

is_valid(id::A) where {A<:AbstractID} = get_id(id) != -1
first_valid(a,b) = is_valid(a) ? a : b

align_with_predecessor(node,pred) 						= node
align_with_predecessor(node::GO,pred::ROBOT_AT) 		= GO(first_valid(node.r,pred.r), first_valid(node.x1,pred.x), node.x2)
align_with_predecessor(node::GO,pred::DEPOSIT) 			= GO(first_valid(node.r,pred.r), first_valid(node.x1,pred.x), node.x2)
align_with_predecessor(node::COLLECT,pred::OBJECT_AT) 	= COLLECT(node.r, first_valid(node.o,pred.o), first_valid(node.x,pred.x))
align_with_predecessor(node::COLLECT,pred::GO) 			= COLLECT(first_valid(node.r,pred.r), node.o, first_valid(node.x,pred.x2))
align_with_predecessor(node::CARRY,pred::COLLECT) 		= CARRY(first_valid(node.r,pred.r), first_valid(node.o,pred.o), first_valid(node.x1,pred.x), node.x2)
align_with_predecessor(node::DEPOSIT,pred::CARRY)		= DEPOSIT(first_valid(node.r,pred.r), first_valid(node.o,pred.o), first_valid(node.x,pred.x2))

align_with_successor(node,pred) 						= node
align_with_successor(node::GO,succ::COLLECT) 			= GO(first_valid(node.r,succ.r), node.x1, first_valid(node.x2,succ.x))


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

end # module PlanningPredicates
