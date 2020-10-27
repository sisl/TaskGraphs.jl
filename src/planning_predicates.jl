module PlanningPredicates

using Parameters

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

export
	AbstractRobotType,
	DeliveryBot,
	CleanUpBot

abstract type AbstractRobotType end
struct DeliveryBot <: AbstractRobotType end
struct CleanUpBot <: AbstractRobotType end

export
    AbstractID,
    ObjectID,
    RobotID,
    LocationID,
    ActionID,
    OperationID,
	AgentID,
	VtxID

abstract type AbstractID end
@with_kw struct ObjectID <: AbstractID
	id::Int = -1
end
@with_kw struct RobotID <: AbstractID
	id::Int = -1
end
""" Clean-Up Bot ID """
@with_kw struct CLBotID <: AbstractID
	id::Int = -1
end
@with_kw struct LocationID <: AbstractID
	id::Int = -1
end
@with_kw struct ActionID <: AbstractID
	id::Int = -1
end
@with_kw struct OperationID <: AbstractID
	id::Int = -1
end
"""
	AgentID
Special helper for identifying agents.
"""
@with_kw struct AgentID <: AbstractID
	id::Int = -1
end
"""
	VtxID
Special helper for identifying schedule vertices.
"""
@with_kw struct VtxID <: AbstractID
	id::Int = -1
end

export
	get_id

get_id(id::AbstractID) = id.id
get_id(id::Int) = id
Base.:+(id::A,i::Int) where {A<:AbstractID} = A(get_id(id)+i)
Base.:-(id::A,i::Int) where {A<:AbstractID} = A(get_id(id)-i)
Base.:(<)(id1::AbstractID,id2::AbstractID) = get_id(id1) < get_id(id2)
Base.:(>)(id1::AbstractID,id2::AbstractID) = get_id(id1) > get_id(id2)

export
    AbstractPlanningPredicate,
    OBJECT_AT,
    ROBOT_AT

abstract type AbstractPlanningPredicate end

"""
	OBJECT_AT <: AbstractPlanningPredicate

Encodes the location and shape of the object with id `o`
"""
struct OBJECT_AT <: AbstractPlanningPredicate
    o::ObjectID
    x::Vector{LocationID} # can occupy multiple locations
	# n::Int # number of robots required for transport
	shape::Tuple{Int,Int}
end
OBJECT_AT(o::ObjectID,x::Vector{LocationID}) = OBJECT_AT(o,x,(1,1))
OBJECT_AT(o::ObjectID,x::LocationID,args...) = OBJECT_AT(o,[x],args...)
OBJECT_AT(o::ObjectID,x::Vector{Int},args...) = OBJECT_AT(o,map(idx->LocationID(idx),x),args...)
OBJECT_AT(o::ObjectID,x::Int,args...) 		= OBJECT_AT(o,[LocationID(x)],args...)
OBJECT_AT(o::Int,args...) 					= OBJECT_AT(ObjectID(o),args...)

"""
	ROBOT_AT <: AbstractPlanningPredicate

Encodes the location of robot with id `r`
"""
struct ROBOT_AT <: AbstractPlanningPredicate
    r::RobotID
    x::LocationID
end
ROBOT_AT(r::Int,args...) = ROBOT_AT(RobotID(r),args...)
ROBOT_AT(r::RobotID,x::Int) = ROBOT_AT(r,LocationID(x))

export
    get_object_id,
    get_location_id,
    get_location_ids,
    get_robot_id

get_object_id(pred::OBJECT_AT) = pred.o
get_location_id(pred::ROBOT_AT) = pred.x
get_location_id(pred::OBJECT_AT) = pred.x[1]
get_location_ids(pred::OBJECT_AT) = pred.x
get_robot_id(pred::ROBOT_AT) = pred.r

export
    Operation,
	get_input_ids,
	get_output_ids,
	get_operation_id,
    preconditions,
    postconditions,
    add_conditions,
    delete_conditions,
    duration

@with_kw struct Operation <: AbstractPlanningPredicate
    pre::Set{OBJECT_AT}     = Set{OBJECT_AT}()
    post::Set{OBJECT_AT}    = Set{OBJECT_AT}()
    Δt::Int 				= 0
    station_id::LocationID   = LocationID(-1)
    id::OperationID         = OperationID(-1)
end
get_location_id(op::Operation) = op.station_id
get_operation_id(op::Operation) = op.id
get_input_ids(op::Operation) = sort([get_id(get_object_id(p)) for p in op.pre])
get_output_ids(op::Operation) = sort([get_id(get_object_id(p)) for p in op.post])
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

"""
	GO <: AbstractRobotAction

Encodes the event "robot `r` goes from `x1` to `x2`"
"""
@with_kw struct GO <: AbstractRobotAction # go to position x
    r::RobotID 		= RobotID()
    x1::LocationID	= LocationID()
    x2::LocationID	= LocationID()
end
GO(r::Int,args...) = GO(RobotID(r),args...)
GO(r::RobotID,x1::Int,args...) = GO(r,LocationID(x1),args...)
GO(r::RobotID,x1::LocationID,x2::Int) = GO(r,x1,LocationID(x2))

"""
	CARRY <: AbstractRobotAction

Encodes the event "robot `r` carries object `o` from `x1` to `x2`"
"""
@with_kw struct CARRY <: AbstractRobotAction # carry object o to position x
    r::RobotID		= RobotID()
    o::ObjectID		= ObjectID()
    x1::LocationID	= LocationID()
    x2::LocationID	= LocationID()
end
CARRY(r::Int,args...) = CARRY(RobotID(r),args...)
CARRY(r::RobotID,o::Int,args...) 					= CARRY(r,ObjectID(o),args...)
CARRY(r::RobotID,o::ObjectID,x1::Int,args) 			= CARRY(r,o,LocationID(x1),args...)
CARRY(r::RobotID,o::ObjectID,x1::LocationID,x2::Int) = CARRY(r,o,x1,LocationID(x2))

"""
	COLLECT <: AbstractRobotAction

Encodes the event "robot `r` collects object `o` from `x`
"""
@with_kw struct COLLECT <: AbstractRobotAction
    r::RobotID 		= RobotID()
    o::ObjectID 	= ObjectID()
    x::LocationID 	= LocationID()
end
COLLECT(r::Int,args...) 				= COLLECT(RobotID(r),args...)
COLLECT(r::RobotID,o::Int,args...) 		= COLLECT(r,ObjectID(o),args...)
COLLECT(r::RobotID,o::ObjectID,x::Int) 	= COLLECT(r,o,LocationID(x))

"""
	DEPOSIT <: AbstractRobotAction

Encodes the event "robot `r` deposits object `o` at `x`
"""
@with_kw struct DEPOSIT <: AbstractRobotAction
    r::RobotID 		= RobotID()
    o::ObjectID 	= ObjectID()
    x::LocationID 	= LocationID()
end
DEPOSIT(r::Int,args...) 				= DEPOSIT(RobotID(r),args...)
DEPOSIT(r::RobotID,o::Int,args...) 		= DEPOSIT(r,ObjectID(o),args...)
DEPOSIT(r::RobotID,o::ObjectID,x::Int) 	= DEPOSIT(r,o,LocationID(x))

get_initial_location_id(a::A) where {A<:Union{GO,CARRY}}        						= a.x1
get_destination_location_id(a::A) where {A<:Union{GO,CARRY}}    						= a.x2
get_location_id(a::A) where {A<:Union{COLLECT,DEPOSIT}}             					= a.x
get_initial_location_id(a::A) where {A<:Union{COLLECT,DEPOSIT,ROBOT_AT,OBJECT_AT}}     	= get_location_id(a)
get_destination_location_id(a::A) where {A<:Union{COLLECT,DEPOSIT,ROBOT_AT,OBJECT_AT}} 	= get_location_id(a)
get_object_id(a::A) where {A<:Union{CARRY,COLLECT,DEPOSIT}}         					= a.o
get_robot_id(a::A) where {A<:AbstractRobotAction} = a.r

export
	split_node

"""
	split_node(node::N,x::LocationID)

Creates two new nodes of type `N`, where the destination of the first node
and the starting location of the second node are both set to `x`.
"""
function split_node(node::N,x::LocationID) where {N<:Union{GO,CARRY}}
	N(node, x2=x), N(node, x1=x)
end
function split_node(node::N,x::LocationID) where {N<:Union{DEPOSIT,COLLECT}}
	N(node, x=x), N(node, x=x)
end

export
	TEAM_ACTION,
	sub_nodes,
	team_configuration

"""
	TEAM_ACTION{A}

For collaborative tasks.

[GO, ...] -> TEAM_COLLECT -> TEAM_CARRY -> TEAM_DEPOSIT -> [GO, ...]
- there should be a way to prove that the milp assignment (if each robot is actually
    assigned to a particular spot in the configuration) can be realized if all conflicts
    (except between collaborating team members) are ignored for a TEAM_GO task. This is
    because of the "push-and-rotate" thing once they reach the goal vertices.
"""
@with_kw struct TEAM_ACTION{A<:AbstractRobotAction} <: AbstractRobotAction
    instructions::Vector{A} = Vector{CARRY}()
	shape::Tuple{Int,Int} 	= (1,1)
    n::Int 					= length(instructions) # number of robots
    # config::Matrix{Int} = ones(n) # defines configuration of agents relative to each other during collaborative task
end
sub_nodes(n) = [n]
sub_nodes(n::TEAM_ACTION) = n.instructions
team_configuration(n) = (1,1)
team_configuration(n::TEAM_ACTION) = n.shape

export
	required_predecessors,
	required_successors,
	num_required_predecessors,
	num_required_successors,
    eligible_successors,
    eligible_predecessors,
	num_eligible_predecessors,
	num_eligible_successors,
	matches_template,
    resources_reserved,
	align_with_predecessor,
	align_with_successor

"""
	required_predecessors(node)

Identifies the types (and how many) of required predecessors to `node`

Example:
	`required_predecessors(node::COLLECT) = Dict(OBJECT_AT=>1,GO=>1)`
"""
function required_predecessors end

"""
	required_successors(node)

Identifies the types (and how many) of required successors to `node`

Example:
	`required_successors(node::COLLECT) = Dict(CARRY=>1)`
"""
function required_successors end

"""
	eligible_predecessors(node)

Identifies the types (and how many) of eligible predecessors to `node`

Example:
	`eligible_predecessors(node::OBJECT_AT) = Dict(Operation=>1)`
"""
function eligible_predecessors end

"""
	eligible_successors(node)

Identifies the types (and how many) of eligible successors to `node`

Example:
	`eligible_successors(node::GO) = Dict((GO,TEAM_ACTION{COLLECT},TEAM_ACTION{GO},COLLECT)=>1)`
"""
function eligible_successors end

required_predecessors(node::GO)         = Dict((GO,ROBOT_AT,DEPOSIT,TEAM_ACTION{DEPOSIT})=>1)
required_successors(node::GO)           = Dict()
required_predecessors(node::COLLECT)    = Dict(OBJECT_AT=>1,GO=>1)
required_successors(node::COLLECT)      = Dict(CARRY=>1)
required_predecessors(node::CARRY)      = Dict(COLLECT=>1)
required_successors(node::CARRY)        = Dict(DEPOSIT=>1)
required_predecessors(node::DEPOSIT)    = Dict(CARRY=>1)
required_successors(node::DEPOSIT)      = Dict(Operation=>1,GO=>1)
required_predecessors(node::Operation)  = Dict((DEPOSIT,OBJECT_AT)=>length(node.pre))
required_successors(node::Operation)    = Dict(OBJECT_AT=>length(node.post))
required_predecessors(node::OBJECT_AT)  = Dict()
required_successors(node::OBJECT_AT)    = Dict(COLLECT=>1)
required_predecessors(node::ROBOT_AT)   = Dict()
required_successors(node::ROBOT_AT)     = Dict(GO=>1)

"""
	num_required_predecessors(node)

Returns the total number of required predecessors to `node`.
"""
function num_required_predecessors(node)
	n = 1
	for (key,val) in required_predecessors(node)
		n += val
	end
	n
end

"""
	num_required_successors(node)

Returns the total number of required successors to `node`.
"""
function num_required_successors(node)
	n = 1
	for (key,val) in required_successors(node)
		n += val
	end
	n
end

eligible_predecessors(node) 			= required_predecessors(node)
eligible_successors(node) 				= required_successors(node)

eligible_successors(node::GO)           = Dict((GO,TEAM_ACTION{COLLECT},TEAM_ACTION{GO},COLLECT)=>1)
eligible_predecessors(node::OBJECT_AT)  = Dict(Operation=>1)


"""
	num_eligible_predecessors(node)

Returns the total number of eligible predecessors to `node`.
"""
function num_eligible_predecessors(node)
	n = 1
	for (key,val) in eligible_predecessors(node)
		n += val
	end
	n
end

"""
	num_eligible_successors(node)

Returns the total number of eligible successors to `node`.
"""
function num_eligible_successors(node)
	n = 1
	for (key,val) in eligible_successors(node)
		n += val
	end
	n
end

required_predecessors(node::TEAM_ACTION{GO})        = Dict(GO=>length(node.instructions))
required_predecessors(node::TEAM_ACTION{COLLECT})   = Dict(GO=>length(node.instructions),OBJECT_AT=>1)
required_predecessors(node::TEAM_ACTION{CARRY})     = Dict(TEAM_ACTION{COLLECT}=>1)
required_predecessors(node::TEAM_ACTION{DEPOSIT})   = Dict(TEAM_ACTION{CARRY}=>1)

required_successors(node::TEAM_ACTION{GO})         	= Dict(TEAM_ACTION{COLLECT}=>1)
required_successors(node::TEAM_ACTION{COLLECT})    	= Dict(TEAM_ACTION{CARRY}=>1)
required_successors(node::TEAM_ACTION{CARRY})      	= Dict(TEAM_ACTION{DEPOSIT}=>1)
required_successors(node::TEAM_ACTION{DEPOSIT})    	= Dict(GO=>length(node.instructions),Operation=>1)

"""
	matches_template(template,node)

Checks if a candidate `node` satisfies the criteria encoded by `template`.
"""
matches_template(template,node) = false
matches_template(template::T,node::T) where {T} = true
matches_template(template::DataType,node::DataType) = template == node
matches_template(template::T,node::DataType) where {T<:Tuple} = (node in template)
matches_template(template::T,node) where {T<:Tuple} = matches_template(template, typeof(node))

"""
	resouces_reserved(node)

Identifies the resources reserved by a particular `node` for its duration.
For example, `resources_reserved(node::COLLECT) = AbstractID[get_location_id(node)]`
"""
resources_reserved(node)                = AbstractID[]
resources_reserved(node::COLLECT)       = AbstractID[get_location_id(node)]
resources_reserved(node::DEPOSIT)       = AbstractID[get_location_id(node)]
resources_reserved(node::TEAM_ACTION)	= union(map(pred->resources_reserved(pred), node.instructions)...)

is_valid(id::A) where {A<:AbstractID} = get_id(id) != -1
first_valid(a,b) = is_valid(a) ? a : b

"""
	align_with_predecessor(node)

Modifies a node to match the information encoded by its predecessor. This is
how e.g., robot ids are propagated through an existing operating schedule
after assignments (or re-assignments) have been made.
"""
align_with_predecessor(node,pred) 						= node
align_with_predecessor(node::GO,pred::ROBOT_AT) 		= GO(first_valid(node.r,pred.r), first_valid(node.x1,pred.x), node.x2)
align_with_predecessor(node::GO,pred::GO) 				= GO(first_valid(node.r,pred.r), first_valid(node.x1,pred.x2), node.x2)
align_with_predecessor(node::GO,pred::DEPOSIT) 			= GO(first_valid(node.r,pred.r), first_valid(node.x1,pred.x), node.x2)
align_with_predecessor(node::COLLECT,pred::OBJECT_AT) 	= COLLECT(node.r, first_valid(node.o,pred.o), node.x) # NOTE: Because the object could occupy multiple vertices, we do not want to dispatch first_valid between COLLECT.x and OBJECT_AT.x
align_with_predecessor(node::COLLECT,pred::GO) 			= COLLECT(first_valid(node.r,pred.r), node.o, first_valid(node.x,pred.x2))
align_with_predecessor(node::CARRY,pred::COLLECT) 		= CARRY(first_valid(node.r,pred.r), first_valid(node.o,pred.o), first_valid(node.x1,pred.x), node.x2)
align_with_predecessor(node::CARRY,pred::CARRY) 		= CARRY(first_valid(node.r,pred.r), first_valid(node.o,pred.o), first_valid(node.x1,pred.x2), node.x2)
align_with_predecessor(node::DEPOSIT,pred::CARRY)		= DEPOSIT(first_valid(node.r,pred.r), first_valid(node.o,pred.o), first_valid(node.x,pred.x2))

# NOTE job shop constraints were wreaking havoc with id propagation between COLLECT nodes and DEPOSIT nodes! Hence the alignment functions have been removed
# align_with_predecessor(node::COLLECT,pred::COLLECT) 	= COLLECT(node.r, node.o, first_valid(node.x,pred.x))
# align_with_predecessor(node::DEPOSIT,pred::DEPOSIT) 	= DEPOSIT(node.r, node.o, first_valid(node.x,pred.x))

function align_with_predecessor(node::TEAM_ACTION,pred)
	for i in 1:length(node.instructions)
		p = node.instructions[i]
		if get_destination_location_id(pred) == get_initial_location_id(p)
			node.instructions[i] = align_with_predecessor(p,pred)
		end
	end
	node
end
function align_with_predecessor(node::AbstractRobotAction,pred::TEAM_ACTION)
	for i in 1:length(pred.instructions)
		p = pred.instructions[i]
		if get_destination_location_id(p) == get_initial_location_id(node)
			return align_with_predecessor(node,p)
		end
	end
	return node
end
function align_with_predecessor(node::TEAM_ACTION,pred::TEAM_ACTION)
	for i in 1:length(node.instructions)
		p = node.instructions[i]
		for (j,pj) in enumerate(pred.instructions)
			if get_destination_location_id(pj) == get_initial_location_id(p)
				@assert(i == j, "TEAM_ACTION indices $i and $j do not match")
				node.instructions[i] = align_with_predecessor(p,pj)
			end
		end
	end
	node
end

"""
	align_with_successor(node)

Modifies a node to match the information encoded by its successor. This is
how e.g., robot ids are propagated through an existing operating schedule
after assignments (or re-assignments) have been made.
"""
align_with_successor(node,pred) 						= node
align_with_successor(node::GO,succ::COLLECT) 			= GO(first_valid(node.r,succ.r), node.x1, first_valid(node.x2,succ.x))
align_with_successor(node::GO,succ::GO) 				= GO(first_valid(node.r,succ.r), node.x1, first_valid(node.x2,succ.x1))


export
	validate_edge

"""
	validate_edge(n1,n2)

For an edge (n1) --> (n2), checks whether the edge is legal and the nodes
"agree". For example,

`validate_edge(n1::ROBOT_AT, n2::GO) = (n1.x == n2.x1) && (n1.r == n2.r)`

meaning that the robot ids and the initial initial locations must match.
"""
validate_edge(n1,n2) = true
validate_edge(n1::N1,n2::N2) where {N1<:Union{ROBOT_AT,OBJECT_AT},N2<:Union{ROBOT_AT,OBJECT_AT}} = false
validate_edge(n1::ROBOT_AT,		n2::GO			) = (n1.x 	== n2.x1) && (n1.r == n2.r)
validate_edge(n1::GO,			n2::GO			) = (n1.x2 	== n2.x1) && (n1.r == n2.r)
validate_edge(n1::GO,			n2::COLLECT		) = (n1.x2 	== n2.x ) && (n1.r == n2.r)
validate_edge(n1::COLLECT,		n2::CARRY		) = (n1.x 	== n2.x1) && (n1.o == n2.o)
validate_edge(n1::CARRY,		n2::COLLECT		) = false
validate_edge(n1::CARRY,		n2::DEPOSIT		) = (n1.x2 	== n2.x) && (n1.o == n2.o)
validate_edge(n1::DEPOSIT,		n2::CARRY		) = false
validate_edge(n1::DEPOSIT,		n2::GO			) = (n1.x 	== n2.x1)
validate_edge(n1::N,n2::N) where {N<:Union{COLLECT,DEPOSIT}} = (n1.x == n2.x) # job shop edges are valid

# """ Planning Resources (at the assignment level) """
# abstract type AbstractResource end
# abstract type ActiveResource <: AbstractResource end
# abstract type PassiveResource <: AbstractResource end
# """ Robot Types """
# abstract type AbstractRobotType end
# struct GenericRobot <: AbstractRobotType end
# """ Object Types """
# abstract type AbstractObjectType end
# struct GenericObject <: AbstractObjectType end
# """ Manufacturing Station Types """
# abstract type AbstractStationType end
# struct GenericStation <: AbstractStationType end
# """ Loading Zone Types """
# abstract type AbstractLoadingZoneType end
# struct GenericLoadingZone <: AbstractLoadingZoneType end
#
# @with_kw struct PlanningResource{T} <: AbstractResource
#     id::Int 	= -1
#     rtype::T 	= GenericRobot()
# end
# @with_kw struct RobotResource{T<:AbstractRobotType} <: AbstractResource
#     id::Int 	= -1
#     rtype::T 	= GenericRobot()
# end
# @with_kw struct ObjectResource{T<:AbstractObjectType} <: AbstractResource
#     id::Int		= -1
#     rtype::T	= GenericObject()
# end
# @with_kw struct LoadingZoneResource{T<:AbstractLoadingZoneType} <: AbstractResource
#     id::Int		= -1
#     rtype::T	= GenericLoadingZone()
# end
# struct StationResource{T<:AbstractStationType} <: AbstractResource
#     id::Int
#     rtype::T
# end
#
# get_id(r::R) where {R<:AbstractResource} = r.id
# get_type(r::R) where {R<:AbstractResource} = r.rtype
#
# function matches_resource_spec(required_type::T,required_id::Int,available::R) where {T<:DataType,R<:AbstractResource}
#     if get_type(available) <: required_type
#         if (get_id(available) == required_id) || (required_id == -1)
#             return true
#         end
#     end
#     return false
# end
# function matches_resource_spec(required::T,available::R) where {T<:AbstractResource,R<:AbstractResource}
#     matches_resource_spec(get_type(required),get_id(required),available)
# end
#
# """
# 	`ResourceTable`
#
# 	Defines all available resources
# """
# struct ResourceTable
#     robots::Dict{Int,RobotResource}
#     objects::Dict{Int,ObjectResource}
#     loading_zones::Dict{Int,LoadingZoneResource}
#     stations::Dict{Int,StationResource}
# end

end # module PlanningPredicates
