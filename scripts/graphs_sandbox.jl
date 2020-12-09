using Graphs, Parameters

abstract type AbstractID end
@with_kw struct ObjectID <: AbstractID
	id::Int = -1
end
# @with_kw struct BotID{R<:AbstractRobotType} <: AbstractID
# 	id::Int = -1
# end
# const RobotID = BotID{DeliveryBot}
@with_kw struct LocationID <: AbstractID
	id::Int = -1
end
@with_kw struct ActionID <: AbstractID
	id::Int = -1
end
@with_kw struct OperationID <: AbstractID
	id::Int = -1
end

mutable struct PlanningNode{I,N}
    id::I
    node::N
    t0::Int
    min_duration::Int
    tF::Int
	PlanningNode(id::I,node::N) where {I,N} = new{I,N}(id,node,0,0,0)
	PlanningNode{I,N}(id::I,node::N) where {I,N} = new{I,N}(id,node,0,0,0)
end
mutable struct ScheduleVertex{N<:PlanningNode}
    idx::Int
	node::N
end
Graphs.vertex_index(v::ScheduleVertex,g::AbstractGraph{ScheduleVertex}) = v.idx
Graphs.make_vertex(g::AbstractGraph{ScheduleVertex},node) = ScheduleVertex(
	num_vertices(g)+1, node)

g = AdjacencyList{ScheduleVertex}(
	true, ScheduleVertex[], 0, Vector{ScheduleVertex}[]
)
g = IncidenceList{ScheduleVertex,}(
	true, ScheduleVertex[], 0, Vector{ScheduleVertex}[]
)
add_vertex!(g,PlanningNode(ObjectID(1),"object_string"))
add_vertex!(g,PlanningNode(ActionID(2),"Collect"))
add_edge!(g,1,2)


g = simple_edgelist(10,[(1,2)])
