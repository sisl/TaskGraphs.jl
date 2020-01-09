
using TaskGraphs
using LightGraphs, MetaGraphs, GraphUtils

# @with_kw struct ProjectSchedule{G<:AbstractGraph}# <: AbstractProjectSchedule
#     graph               ::G                     = MetaDiGraph()
#     planning_nodes      ::Dict{AbstractID,AbstractPlanningPredicate}    = Dict{AbstractID,AbstractPlanningPredicate}()
#     vtx_map             ::Dict{AbstractID,Int}  = Dict{AbstractID,Int}()
#     vtx_ids             ::Vector{AbstractID}    = Vector{AbstractID}() # maps vertex to actual graph node
#     path_specs          ::Vector{PathSpec}      = Vector{PathSpec}()
#     root_nodes          ::Vector{Int}           = Vector{Int}() # list of "project heads"
#     weights             ::Dict{Int,Float64}     = Dict{Int,Float64}() # weights corresponding to project heads
#     path_id_to_vtx_map  ::Dict{Int,Int}         = Dict{Int,Int}() # maps path_id to vertex
#
#     robot_id_map        ::Dict{Int,Int}   = Dict{Int,Int}() # maps dummy id to true id
# end
#
# @with_kw struct PlanningCache
#     closed_set::Set{Int}    = Set{Int}()    # nodes that are completed
#     active_set::Set{Int}    = Set{Int}()    # active nodes
#     node_queue::PriorityQueue{Int,Float64} = PriorityQueue{Int,Float64}() # active nodes prioritized by slack
#     t0::Vector{Int}         = Vector{Int}()
#     tF::Vector{Int}         = Vector{Int}()
#     slack::Vector{Vector{Float64}}       = Vector{Vector{Float64}}()
#     local_slack::Vector{Vector{Float64}} = Vector{Vector{Float64}}()
# end
#
# @with_kw struct SearchEnv{C,E<:AbstractLowLevelEnv{PCCBS.State,PCCBS.Action,C}} <: AbstractLowLevelEnv{PCCBS.State,PCCBS.Action,C}
#     schedule::ProjectSchedule   = ProjectSchedule()
#     cache::PlanningCache        = PlanningCache()
#     env::E                      = PCCBS.LowLevelEnv()
#     cost_model::C               = get_cost_model(env)
#     num_agents::Int             = -1
# end

# Construct and modify project schedules
let
    P = ProjectSchedule()
    add_to_schedule!(P, ROBOT_AT(1,1),RobotID(1))
end
