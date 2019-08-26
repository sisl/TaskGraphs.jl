module TaskGraphs

using Parameters
using LightGraphs, MetaGraphs
using GraphUtils
using LinearAlgebra
using DataStructures
using JuMP
using TOML


export
    PlanningPredicate,
    OBJECT_AT,

    Operation,
    construct_operation,
    get_o,
    get_s,
    preconditions,
    postconditions,
    duration,

    DeliveryTask,
    DeliveryGraph,
    construct_delivery_graph,

    ProjectSpec,
    get_initial_nodes,
    get_input_ids,
    get_output_ids,
    add_operation!,
    read_project_spec

abstract type PlanningPredicate end
struct OBJECT_AT <: PlanningPredicate
    o::Int
    s::Int
end
get_o(pred::OBJECT_AT) = pred.o
get_s(pred::OBJECT_AT) = pred.s

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
get_pre_deps(tg::ProjectSpec, i::Int) = get(tg.pre_deps, i, Set{Int}())
get_post_deps(tg::ProjectSpec, i::Int) = get(tg.post_deps, i, Set{Int}())
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

# Some tools for writing and reading project specs
function TOML.parse(op::Operation)
    toml_dict = Dict()
    toml_dict["pre"] = map(pred->[get_o(pred),get_s(pred)], collect(op.pre))
    toml_dict["post"] = map(pred->[get_o(pred),get_s(pred)], collect(op.post))
    toml_dict["Δt"] = duration(op)
    return toml_dict
end
function TOML.parse(project_spec::ProjectSpec)
    toml_dict = Dict()
    toml_dict["title"]      = "ProjectSpec"
    toml_dict["operations"] = map(op->TOML.parse(op),project_spec.operations)
    toml_dict
end
function read_project_spec(io)
    toml_dict = TOML.parsefile(io)
    project_spec = ProjectSpec()
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

"""
    DeliveryTask
"""
struct DeliveryTask
    o::Int
    s1::Int
    s2::Int
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

include("utils.jl")

end
