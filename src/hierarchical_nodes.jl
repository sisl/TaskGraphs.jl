module HierarchicalGraphs

# using Parameters
# using LightGraphs, MetaGraphs
# using GraphUtils
# using DataStructures
# using JuMP
# using TOML
#
# using ..PlanningPredicates

abstract type AbstractNode end
struct NestedNode{G,M} <: AbstractNode
    graph::G
    v::Int # vertex associated with this node
    metadata::M # e.g. start time, end time, etc.
    parent::Int # parent node
    children::Set{Int} # nodes that are bundled into this one
end

struct NestedGraph{G}
    graph::G
    nodes::Vector{NestedNode}
end

function add_child!(node::N, v::Int) where {N<:NestedNode}
    push!(node.children, v)

end

end
