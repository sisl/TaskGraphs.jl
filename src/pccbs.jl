export PCCBS

module PCCBS

using CRCBS
using Parameters, LightGraphs, DataStructures
using GraphUtils
using ..PlanningPredicates

################################################################################
############################### ENVIRONMENT DEF ################################
################################################################################
const State = CRCBS.GraphState
const Action = CRCBS.GraphAction
# TODO simplify LowLevelEnv and include SearchEnv within it
# LowLevelEnv
@with_kw struct LowLevelEnv{E,N,I,T,C<:AbstractCostModel,H<:AbstractCostModel} <: GraphEnv{State,Action,C}
    search_env::E                   = nothing
    schedule_node::N                = nothing
    node_id::I                      = nothing
    agent_idx::Int                  = -1
    # graph::G                        = get_graph(search_env)
    constraints::T                  = discrete_constraint_table(
                                        nv(get_graph(search_env)),
                                        nv(get_graph(search_env))^2,
                                        agent_idx)
    goal::State                     = State()
    cost_model::C                   = get_cost_model(search_env)
    heuristic::H                    = get_heuristic_model(search_env)
end
CRCBS.get_graph(env::LowLevelEnv)            = get_graph(env.search_env) #graph
CRCBS.get_cost_model(env::LowLevelEnv)       = get_cost_model(env.search_env)
CRCBS.get_agent_id(env::LowLevelEnv)         = env.agent_idx
CRCBS.get_constraints(env::LowLevelEnv)      = env.constraints
CRCBS.get_goal(env::LowLevelEnv)             = env.goal
CRCBS.get_heuristic_model(env::LowLevelEnv)  = get_heuristic_model(env.search_env)

get_schedule_node(env::LowLevelEnv)          = env.schedule_node
CRCBS.get_next_state(s::State,a::Action)    = State(get_e(a).dst,get_t(s)+get_dt(a))
CRCBS.get_next_state(env::LowLevelEnv,s,a)  = get_next_state(s,a)
CRCBS.wait(env::LowLevelEnv,s)              = Action(e=Edge(get_vtx(s),get_vtx(s)))

# CRCBS.is_valid(env::LowLevelEnv,s::State) = (1 <= s.vtx <= nv(get_graph(env)))
# CRCBS.is_valid(env::LowLevelEnv,a::Action) = (1 <= a.e.src <= nv(get_graph(env))) && (1 <= a.e.dst <= nv(get_graph(env)))

CRCBS.get_possible_actions(env::LowLevelEnv,s::State) = get_possible_actions(get_schedule_node(env),env,s)
function CRCBS.get_possible_actions(node,env::LowLevelEnv,s::State)
    if is_valid(env,s)
        return map(v2->Action(e=Edge(s.vtx,v2)), outneighbors(get_graph(env),s.vtx))
    end
    return Action[]
end
function CRCBS.get_possible_actions(node::Union{COLLECT,DEPOSIT},env::LowLevelEnv,s::State)
    if is_valid(env,s)
        return [CRCBS.wait(env,s)]
    end
    return Action[]
end
function CRCBS.get_possible_actions(env::MetaAgentCBS.TeamMetaEnv,s::MetaAgentCBS.State{State})
    d_set = Set{Tuple}([(0,0),(-1,0),(0,1),(1,0),(0,-1)])
    for (e,s) in zip(env.envs, s.states)
        intersect!(d_set, CRCBS.get_graph(e).edge_cache[s.vtx])
    end
    meta_actions = Vector{MetaAgentCBS.Action{Action}}()
    for d in d_set
        meta_action = MetaAgentCBS.Action{Action}()
        for (e,s) in zip(env.envs, s.states)
            vtx = get_graph(e).vtxs[s.vtx]
            next_vtx = (vtx[1] + d[1], vtx[2] + d[2])
            push!(meta_action.actions, Action(e = Edge(
                get_graph(e).vtx_map[vtx...],
                get_graph(e).vtx_map[next_vtx...]))
                )
        end
        push!(meta_actions, meta_action)
    end
    meta_actions
end

end # end module PCCBS
