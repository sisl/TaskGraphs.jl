export PCCBS

module PCCBS

using CRCBS
using Parameters, LightGraphs, DataStructures
using ..FactoryWorlds
using ..PlanningPredicates

################################################################################
############################### ENVIRONMENT DEF ################################
################################################################################
# State
@with_kw struct State
    vtx::Int        = -1 # vertex of graph
    t::Int          = -1
end
# Action
@with_kw struct Action
    e::Edge{Int}    = Edge(-1,-1)
    Δt::Int         = 1
end
Base.:(==)(s1::S,s2::S) where {S<:State} = hash(s1) == hash(s2)
# LowLevelEnv
@with_kw struct LowLevelEnv{C<:AbstractCostModel,H<:AbstractCostModel,N,I} <: AbstractLowLevelEnv{State,Action,C}
    graph::GridFactoryEnvironment   = GridFactoryEnvironment()
    schedule_node::N                = nothing
    node_id::I                      = nothing
    constraints::ConstraintTable    = ConstraintTable()
    goal::State                     = State()
    agent_idx::Int                  = -1
    heuristic::H                    = NullHeuristic()
    cost_model::C                   = SumOfTravelTime()
end
CRCBS.get_cost_model(env::E) where {E<:LowLevelEnv} = env.cost_model
CRCBS.get_heuristic_model(env::E) where {E<:LowLevelEnv} = env.heuristic
CRCBS.is_valid(env::LowLevelEnv,s::State) = (1 <= s.vtx <= nv(env.graph))
CRCBS.is_valid(env::LowLevelEnv,a::Action) = (1 <= a.e.src <= nv(env.graph)) && (1 <= a.e.dst <= nv(env.graph))
################################################################################
######################## Low-Level (Independent) Search ########################
################################################################################
# heuristic
CRCBS.get_heuristic_cost(env::E,s::State) where {E<:LowLevelEnv} = CRCBS.get_heuristic_cost(env,get_heuristic_model(env),s)
function CRCBS.get_heuristic_cost(env::E,h::H,s::State) where {E<:LowLevelEnv,H<:Union{PerfectHeuristic,DefaultPerfectHeuristic}}
    get_heuristic_cost(h, env.goal.vtx, s.vtx)
end
function CRCBS.get_heuristic_cost(env::E,h::H,s::State) where {E<:LowLevelEnv, H<:ConflictTableHeuristic}
    get_heuristic_cost(h, env.agent_idx, s.vtx, s.t)
end

# states_match
CRCBS.states_match(s1::State,s2::State) = (s1.vtx == s2.vtx)
CRCBS.states_match(env::LowLevelEnv,s1::State,s2::State) = states_match(s1,s2)
# is_goal
function CRCBS.is_goal(env::LowLevelEnv,s::State)
    if s.t < env.goal.t
        return false
    end
    if states_match(s, env.goal)
        ###########################
        # for constraint in env.constraints.sorted_state_constraints
        #     if s.t < get_time_of(constraint)
        #         if states_match(s, get_sp(constraint.v))
        #             println("Constraint on goal vtx ", env.goal.vtx, " at time ",get_time_of(constraint)," - current time = ",s.t)
        #             # return false
        #         end
        #     end
        # end
        ###########################
        return true
    elseif env.goal.vtx == -1
        return true
    end
    return false
end
# check_termination_criteria
# CRCBS.check_termination_criteria(env::LowLevelEnv,cost,s) = false
# wait
CRCBS.wait(s::State) = Action(e=Edge(s.vtx,s.vtx))
CRCBS.wait(env::LowLevelEnv,s::State) = Action(e=Edge(s.vtx,s.vtx))
# get_possible_actions
function CRCBS.get_possible_actions(env,s::State)
    if 1 <= s.vtx <= nv(env.graph)
        return map(v2->Action(e=Edge(s.vtx,v2)), outneighbors(env.graph,s.vtx))
    else
        return Action[]
    end
end
function CRCBS.get_possible_actions(env::E,s::State) where {C,H,N<:Union{COLLECT,DEPOSIT},I,E<:LowLevelEnv{C,H,N,I}}
    if 1 <= s.vtx <= nv(env.graph)
        return [CRCBS.wait(env,s)]
    else
        return Action[]
    end
end
function CRCBS.get_possible_actions(env::MetaAgentCBS.LowLevelEnv,s::MetaAgentCBS.State{PCCBS.State})
    d_set = Set{Tuple}([(0,0),(-1,0),(0,1),(1,0),(0,-1)])
    for (e,s) in zip(env.envs, s.states)
        intersect!(d_set, e.graph.edge_cache[s.vtx])
    end
    meta_actions = Vector{MetaAgentCBS.Action{PCCBS.Action}}()
    for d in d_set
        meta_action = MetaAgentCBS.Action{PCCBS.Action}()
        for (e,s) in zip(env.envs, s.states)
            vtx = e.graph.vtxs[s.vtx]
            next_vtx = (vtx[1] + d[1], vtx[2] + d[2])
            push!(meta_action.actions, PCCBS.Action(e = Edge(e.graph.vtx_map[vtx...], e.graph.vtx_map[next_vtx...])))
        end
        push!(meta_actions, meta_action)
    end
    meta_actions
end
# get_next_state
CRCBS.get_next_state(s::State,a::Action) = State(a.e.dst,s.t+a.Δt)
CRCBS.get_next_state(env::LowLevelEnv,s::State,a::Action) = get_next_state(s,a)
# get_transition_cost
function CRCBS.get_transition_cost(env::E,c::TravelTime,s::State,a::Action,sp::State) where {E<:LowLevelEnv}
    return get_cost_type(c)(a.Δt)
end
function CRCBS.get_transition_cost(env::E,c::C,s::State,a::Action,sp::State) where {E<:LowLevelEnv,C<:ConflictCostModel}
    state_conflict_value = get_conflict_value(c, env.agent_idx, sp.vtx, sp.t)
    edge_conflict_value = min(
        get_conflict_value(c,env.agent_idx, sp.vtx, s.t),
        get_conflict_value(c,env.agent_idx, s.vtx, sp.t),
        )
    if edge_conflict_value > 0
        # println("Possible Edge Conflict")
        edge_conflict_value = 0
        for (i,p) in enumerate(c.table.paths)
            if (get_planned_vtx(c.table, i, s.t) == sp.vtx) && (get_planned_vtx(c.table, i, sp.t) == s.vtx)
                edge_conflict_value += 1
            end
        end
    end
    return state_conflict_value + edge_conflict_value
end
function CRCBS.get_transition_cost(env::E,c::TravelDistance,s::State,a::Action,sp::State) where {E<:LowLevelEnv}
    return (s.vtx == sp.vtx) ? 0.0 : 1.0
end
# violates_constraints
function CRCBS.violates_constraints(env::LowLevelEnv, s::State, a::Action, sp::State)
    t = sp.t
    if StateConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t) in env.constraints.state_constraints
        return true
    elseif ActionConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t) in env.constraints.action_constraints
        return true
    end
    return false
end
function CRCBS.violates_constraints(env::LowLevelEnv, path, s::State, a::Action, sp::State)
    violates_constraints(env,s,a,sp)
end

################################################################################
###################### Conflict-Based Search (High-Level) ######################
################################################################################
# detect_state_conflict
function CRCBS.detect_state_conflict(n1::PathNode{State,Action},n2::PathNode{State,Action})
    if n1.sp.vtx == n2.sp.vtx
        return true
    end
    return false
end
CRCBS.detect_state_conflict(env::LowLevelEnv,n1::PathNode{State,Action},n2::PathNode{State,Action}) = detect_state_conflict(n1,n2)
# detect_action_conflict
function CRCBS.detect_action_conflict(n1::PathNode{State,Action},n2::PathNode{State,Action})
    if (n1.a.e.src == n2.a.e.dst) && (n1.a.e.dst == n2.a.e.src)
        return true
    end
    return false
end
CRCBS.detect_action_conflict(env::LowLevelEnv,n1::PathNode{State,Action},n2::PathNode{State,Action}) = detect_action_conflict(n1,n2)

################################################################################
############################### HELPER FUNCTIONS ###############################
################################################################################
""" Helper for displaying Paths """
function CRCBS.convert_to_vertex_lists(path::Path{State,Action})
    vtx_list = [n.sp.vtx for n in path.path_nodes]
    if length(path) > 0
        vtx_list = [get_s(get_path_node(path,1)).vtx, vtx_list...]
    else
        vtx_list = [path.s0.vtx, vtx_list...]
    end
    vtx_list
end
function CRCBS.convert_to_vertex_lists(solution::L) where {T,C,L<:LowLevelSolution{State,Action,T,C}}
    return [convert_to_vertex_lists(path) for path in get_paths(solution)]
end

Base.string(s::State) = "($(s.vtx),$(s.t))"
Base.string(a::Action) = "($(a.e.src)->$(a.e.dst))"
Base.string(s::MetaAgentCBS.State) = string("(",prod(map(s->"$(string(s)),",s.states)),")")
Base.string(a::MetaAgentCBS.Action) = string("(",prod(map(a->"$(string(a)),",s.actions)),")")

end
