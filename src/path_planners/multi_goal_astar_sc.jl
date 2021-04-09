@with_kw struct MState{N} <: CRCBS.AbstractGraphState
    vtx     ::Int           = -1
    t       ::Int           = -1
    stage   ::Int           = -1
    node    ::N             = nothing
    active  ::Bool          = true
end
MState(s::GraphState) = MState(s.vtx,s.t)
get_stage(s::MState) = s.stage
Base.convert(::Type{MState},s::GraphState) = MState(s)
MState(vtx,t) = MState(vtx=vtx,t=t)
_state_active(s) = true
_state_active(s::MState) = s.active
const MAction = CRCBS.GraphAction

function CRCBS.detect_state_conflict(n1::N,n2::N) where {S<:MState,A<:AbstractGraphAction,N<:PathNode{S,A}}
    if _state_active(n1.sp) && _state_active(n2.sp)
        if get_vtx(n1.sp) == get_vtx(n2.sp) && get_t(n1.sp) == get_t(n2.sp)
            return true
        end
    end
    return false
end
function CRCBS.detect_action_conflict(n1::N,n2::N) where {S<:MState,A<:AbstractGraphAction,N<:PathNode{S,A}}
    if _state_active(n1.sp) && _state_active(n2.sp)
        if (get_e(n1.a).src == get_e(n2.a).dst) && (get_e(n1.a).dst == get_e(n2.a).src) && (get_t(n1.sp) == get_t(n2.sp))
            return true
        end
    end
    return false
end

@with_kw struct MPCCBSEnv{E,I<:BotID,T,C<:AbstractCostModel,H<:AbstractCostModel} <: GraphEnv{MState,MAction,C}
    search_env::E                   = nothing
    agent_id::I                     = I(-1)
    goal_sequence::Vector{ScheduleNode} = Vector{ScheduleNode}() # robot mission
    constraints::T                  = discrete_constraint_table(search_env,get_id(agent_id))
    cost_model::C                   = get_cost_model(search_env)
    heuristic::H                    = get_heuristic_model(search_env)
end
CRCBS.get_cost_model(env::MPCCBSEnv)       = env.cost_model
CRCBS.get_heuristic_model(env::MPCCBSEnv)  = env.heuristic
GraphUtils.get_graph(env::MPCCBSEnv)       = get_graph(env.search_env,graph_key(env.agent_id))

# function Base.show(io::IO, env::MPCCBSEnv)
#     print(io,"MPCCBSEnv: \n",
#         "\t","schedule_node: ",string(get_node(env)),"\n",
#         "\t","agent_idx:     ",get_agent_id(env),"\n",
#         "\t","goal:          ",string(get_goal(env)),"\n")
# end

construct_goal(env::MPCCBSEnv,s::MState) = construct_goal(env,s.node)
function construct_goal(env::MPCCBSEnv,node::ScheduleNode)
    goal_idx = get_id(get_destination_location_id(node))
    goal_time = get_tF(node)
    GraphState(goal_idx,goal_time)
end
function check_stage_goal(env::MPCCBSEnv,s,node=s.node)
    if get_t(s) >= get_tF(node)
        goal = construct_goal(env,node)
        if states_match(s, goal)
            return true
        elseif !CRCBS.is_valid(goal)
            return true
        end
    end
    return false
end
function CRCBS.get_next_state(env::MPCCBSEnv,s::MState,a::MAction)
    stage = get_stage(s)
    node = s.node
    # update multiple nodes at once, if possible
    while check_stage_goal(env,s,node) && get_stage(s) < length(env.goal_sequence)
        stage = get_stage(s) + 1
        node = env.goal_sequence[stage]
    end
    MState(
        get_e(a).dst,
        get_t(s)+get_dt(a),
        stage,
        node,
        _state_active(s))
end
CRCBS.wait(::MPCCBSEnv,s)              = MAction(e=Edge(get_vtx(s),get_vtx(s)))

CRCBS.get_possible_actions(env::MPCCBSEnv,s::MState) = get_possible_actions(s.node,env,s)
function CRCBS.get_possible_actions(node,env::MPCCBSEnv,s::MState)
    if CRCBS.is_valid(env,s)
        return map(v2->MAction(e=Edge(s.vtx,v2)), outneighbors(get_graph(env),s.vtx))
    end
    return MAction[]
end
function CRCBS.get_possible_actions(node::Union{COLLECT,DEPOSIT},env::MPCCBSEnv,s::MState)
    # @assert get_vtx(s) == get_id(get_initial_location_id(node)) "Robot $(get_agent_id(env)) should not be at s=$(string(s)) when task=$(string(node))"
    if !(get_vtx(s) == get_id(get_initial_location_id(node)))
        @log_info(-1,0,"Robot $(get_agent_id(env)) should not be at s=$(string(s)) when task=$(string(node))")
    end
    if CRCBS.is_valid(env,s)
        return [CRCBS.wait(env,s)]
    end
    return MAction[]
end

function CRCBS.is_goal(env::MPCCBSEnv,s)
    if get_stage(s) >= length(env.goal_sequence)
        return check_stage_goal(env,s)
    end
    return false
end


