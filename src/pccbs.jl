export PCCBS

module PCCBS

using CRCBS
using Parameters, LightGraphs, DataStructures
using ..FactoryWorlds

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
# @enum GridWorldDirection begin
#    NORTH
#    EAST
#    WEST
#    SOUTH
# end
# LowLevelEnv
@with_kw struct LowLevelEnv{C<:AbstractCostModel,H<:AbstractCostModel} <: AbstractLowLevelEnv{State,Action,C}
    graph::GridFactoryEnvironment   = GridFactoryEnvironment()
    constraints::ConstraintTable    = ConstraintTable()
    goal::State                     = State()
    agent_idx::Int                  = -1
    heuristic::H                    = NullHeuristic()
    cost_model::C                   = SumOfTravelTime()
end
CRCBS.get_cost_model(env::E) where {E<:LowLevelEnv} = env.cost_model
CRCBS.get_heuristic_model(env::E) where {E<:LowLevelEnv} = env.heuristic
# TODO implement a check to be sure that no two agents have the same goal
################################################################################
######################## Low-Level (Independent) Search ########################
################################################################################
# build_env
function CRCBS.build_env(mapf::MAPF{E,S,G}, node::ConstraintTreeNode, idx::Int) where {S,G,E<:LowLevelEnv}
    typeof(mapf.env)(
        graph = mapf.env.graph,
        constraints = get_constraints(node,idx),
        goal = mapf.goals[idx],
        agent_idx = idx,
        heuristic = get_heuristic_model(mapf.env),  # TODO update the heuristic model
        cost_model = get_cost_model(mapf.env)       # TODO update the cost model
        )
end
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
CRCBS.states_match(env::LowLevelEnv,s1::State,s2::State) = (s1.vtx == s2.vtx)
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
CRCBS.check_termination_criteria(env::LowLevelEnv,cost,path,s) = false
# wait
CRCBS.wait(s::State) = Action(e=Edge(s.vtx,s.vtx))
CRCBS.wait(env::LowLevelEnv,s::State) = Action(e=Edge(s.vtx,s.vtx))
# get_possible_actions
struct ActionIter
    s::Int # source state
    neighbor_list::Vector{Int} # length of target edge list
end
struct ActionIterState
    idx::Int # idx of target node
end
ActionIterState() = ActionIterState(0)
function Base.iterate(it::ActionIter)
    iter_state = ActionIterState(0)
    return iterate(it,iter_state)
end
function Base.iterate(it::ActionIter, iter_state::ActionIterState)
    iter_state = ActionIterState(iter_state.idx+1)
    if iter_state.idx > length(it.neighbor_list)
        return nothing
    end
    Action(e=Edge(it.s,it.neighbor_list[iter_state.idx])), iter_state
end
Base.length(iter::ActionIter) = length(iter.neighbor_list)
CRCBS.get_possible_actions(env::LowLevelEnv,s::State) = ActionIter(s.vtx,outneighbors(env.graph,s.vtx))
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
function CRCBS.violates_constraints(env::LowLevelEnv, path, s::State, a::Action, sp::State)
    t = length(path) + 1
    if StateConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t) in env.constraints.state_constraints
        return true
    elseif ActionConstraint(get_agent_id(env.constraints),PathNode(s,a,sp),t) in env.constraints.action_constraints
        return true
    end
    return false
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


end
