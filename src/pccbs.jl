# export PCCBS
#
# module PCCBS
#
# using CRCBS
# using Parameters, LightGraphs, DataStructures
# using GraphUtils
# using ..PlanningPredicates

################################################################################
############################### ENVIRONMENT DEF ################################
################################################################################
const State = CRCBS.GraphState
const Action = CRCBS.GraphAction
# TODO simplify PCCBSEnv and include SearchEnv within it
# PCCBSEnv
export PCCBSEnv

@with_kw_noshow struct PCCBSEnv{E,N,I,T,C<:AbstractCostModel,H<:AbstractCostModel} <: GraphEnv{State,Action,C}
    search_env::E                   = nothing
    schedule_node::N                = nothing
    node_id::I                      = nothing
    agent_idx::Int                  = -1
    # graph::G                        = get_graph(search_env)
    constraints::T                  = discrete_constraint_table(search_env,agent_idx)
    goal::State                     = State()
    cost_model::C                   = get_cost_model(search_env)
    heuristic::H                    = get_heuristic_model(search_env)
end
CRCBS.get_graph(env::PCCBSEnv)            = get_graph(env.search_env) #graph
CRCBS.get_cost_model(env::PCCBSEnv)       = get_cost_model(env.search_env)
CRCBS.get_agent_id(env::PCCBSEnv)         = env.agent_idx
CRCBS.get_constraints(env::PCCBSEnv)      = env.constraints
CRCBS.get_goal(env::PCCBSEnv)             = env.goal
CRCBS.get_heuristic_model(env::PCCBSEnv)  = get_heuristic_model(env.search_env)

get_schedule_node(env::PCCBSEnv)          = env.schedule_node

function Base.show(io::IO, env::PCCBSEnv)
    print(io,"PCCBSEnv: \n",
        "\t","schedule_node: ",string(get_schedule_node(env)),"\n",
        "\t","agent_idx:     ",get_agent_id(env),"\n",
        "\t","goal:          ",string(get_goal(env)),"\n")
end

CRCBS.get_next_state(s::State,a::Action)    = State(get_e(a).dst,get_t(s)+get_dt(a))
CRCBS.get_next_state(env::PCCBSEnv,s,a)  = get_next_state(s,a)
CRCBS.wait(env::PCCBSEnv,s)              = Action(e=Edge(get_vtx(s),get_vtx(s)))

CRCBS.get_possible_actions(env::PCCBSEnv,s::State) = get_possible_actions(get_schedule_node(env),env,s)
function CRCBS.get_possible_actions(node,env::PCCBSEnv,s::State)
    if CRCBS.is_valid(env,s)
        return map(v2->Action(e=Edge(s.vtx,v2)), outneighbors(get_graph(env),s.vtx))
    end
    return Action[]
end
function CRCBS.get_possible_actions(node::Union{COLLECT,DEPOSIT},env::PCCBSEnv,s::State)
    # @assert get_vtx(s) == get_id(get_initial_location_id(node)) "Robot $(get_agent_id(env)) should not be at s=$(string(s)) when task=$(string(node))"
    if !(get_vtx(s) == get_id(get_initial_location_id(node)))
        @log_info(-1,0,"Robot $(get_agent_id(env)) should not be at s=$(string(s)) when task=$(string(node))")
    end
    if CRCBS.is_valid(env,s)
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
        valid_action = true
        for (e,s) in zip(env.envs, s.states)
            vtx = get_graph(e).vtxs[s.vtx]
            next_vtx = (vtx[1] + d[1], vtx[2] + d[2])
            edge = Edge(get_graph(e).vtx_map[vtx...],get_graph(e).vtx_map[next_vtx...])
            if has_edge(get_graph(e),edge)
                push!(meta_action.actions, Action(e = edge))
            else
                valid_action = false
                break
            end
        end
        if valid_action
            push!(meta_actions, meta_action)
        end
    end
    meta_actions
end

function CRCBS.is_goal(env::PCCBSEnv,s)
    if get_t(s) >= get_t(get_goal(env))
        if states_match(s, get_goal(env))
            #########################################
            # updated goal time
            v = get_vtx(get_schedule(env.search_env),env.node_id)
            path_spec = get_path_spec(get_schedule(env.search_env), v)
            # goal_time = env.search_env.cache.tF[v] # time after which goal can be satisfied
            goal_time = get_t(get_goal(env))
            if path_spec.tight == true
                goal_time = env.search_env.cache.tF[v] + minimum(env.search_env.cache.local_slack[v])
                # goal_time += minimum(env.search_env.cache.local_slack[v])
            end
            if (path_spec.free == true) && is_terminal_node(get_graph(get_schedule(env.search_env)),v)
                goal_time = maximum(env.search_env.cache.tF)
            end
            # check that all predecessor nodes have been completed.
            if get_t(s) < goal_time
                return false
            end
            #########################################
            return true
        elseif !CRCBS.is_valid(get_goal(env))
            return true
        end
    end
    return false
end

# end # end module PCCBS
