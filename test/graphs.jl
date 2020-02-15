# Redefine project schedule using Graphs.jl
using Graphs

abstract type AbstractScheduleNode end
struct GO <: AbstractScheduleNode
    r::Int
    x::Int
end
V = KeyVertex{AbstractScheduleNode}
E = Edge{V}

G = graph(V[],E[];is_directed=true)
add_vertex!(G,GO(1,2))
add_vertex!(G,GO(1,2))
add_vertex!(G,GO(1,2))
