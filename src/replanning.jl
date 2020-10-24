# module Replanning
#
# using Parameters
# using LightGraphs
# using GraphUtils
# using CRCBS
# using ..TaskGraphs

using Printf: @sprintf

export
    ProjectRequest

"""
    ProjectRequest

Encodes a "request" that arrives in the factory command center.
    `t_request` encodes the time at which the request reaches the command center
    `t_arrival` is the time at which the project materials will be available

TODO more flexibility with t_arrival by allowing different materials to arrive
at different times
"""
@with_kw struct ProjectRequest
    schedule::OperatingSchedule = OperatingSchedule()
    t_request::Int              = -1
    t_arrival::Int              = -1
end

export
    RepeatedAbstractPC_TAPF,
    RepeatedPC_TAPF,
    RepeatedC_PC_TAPF

abstract type RepeatedAbstractPC_TAPF end

"""
    RepeatedPC_TAPF{S<:SearchEnv} <: RepeatedAbstractPC_TAPF

Encodes a Repeated PC_TAPF problem.
Usage:
    `prob = RepeatedPC_TAPF(env::SearchEnv,requests::Vector{ProjectRequest})`
"""
struct RepeatedPC_TAPF{S<:SearchEnv} <: RepeatedAbstractPC_TAPF
    env::S
    requests::Vector{ProjectRequest}
end
RepeatedPC_TAPF(env::SearchEnv) = RepeatedPC_TAPF(env,Vector{ProjectRequest}())
struct RepeatedC_PC_TAPF{S<:SearchEnv} <: RepeatedAbstractPC_TAPF
    env::S
    requests::Vector{ProjectRequest}
end
RepeatedC_PC_TAPF(env::SearchEnv) = RepeatedC_PC_TAPF(env,Vector{ProjectRequest}())

construct_routing_problem(prob::RepeatedPC_TAPF,env) = PC_TAPF(env)
construct_routing_problem(prob::RepeatedC_PC_TAPF,env) = C_PC_TAPF(env)



export
    get_env_snapshot,
    trim_route_plan

"""
    get_env_snapshot(route_plan::S,t)
"""
function get_env_snapshot(route_plan::S,t) where {S<:LowLevelSolution}
    Dict(RobotID(i)=>ROBOT_AT(i, get_sp(get_path_node(path,t)).vtx) for (i,path) in enumerate(get_paths(route_plan)))
end
get_env_snapshot(env::SearchEnv,args...) = get_env_snapshot(env.route_plan,args...)

"""
    trim_route_plan(search_env, route_plan, T)

Construct a trimmed route_plan that stops at a certain time step
"""
function trim_route_plan(search_env, route_plan, T)
    trim_solution!(build_env(search_env),route_plan,T)
end



export
    get_active_and_fixed_vtxs,
    split_active_vtxs!,
    fix_precutoff_nodes!,
    break_assignments!,
    prune_schedule,
    prune_project_schedule,
    splice_schedules!

"""
    get_active_and_fixed_vtxs(project_schedule::OperatingSchedule,
        cache::PlanningCache,t)

"active" vertices "straddle" the query time ti
"fixed" vertices finish before the query time t
"""
function get_active_and_fixed_vtxs(project_schedule::OperatingSchedule,cache::PlanningCache,t)
    active_vtxs = Set{Int}()
    fixed_vtxs = Set{Int}()
    for v in vertices(get_graph(project_schedule))
        if cache.tF[v] + minimum(cache.local_slack[v]) <= t
            push!(fixed_vtxs, v)
        elseif cache.t0[v] <= t < cache.tF[v] + minimum(cache.local_slack[v])
            push!(active_vtxs, v)
        end
    end
    @assert all(map(v->cache.tF[v] + minimum(cache.local_slack[v]), collect(active_vtxs)) .>= t)
    active_vtxs, fixed_vtxs
end

"""
    split_active_vtxs!(project_schedule::OperatingSchedule,
        problem_spec::ProblemSpec,cache::PlanningCache,t;

Split all GO nodes that "straddle" the cutoff time.
"""
function split_active_vtxs!(project_schedule::OperatingSchedule,problem_spec::ProblemSpec,cache::PlanningCache,t;
    robot_positions::Dict{RobotID,ROBOT_AT}=Dict{RobotID,ROBOT_AT}()
    )
    G = get_graph(project_schedule)
    # identify nodes "cut" by timestep t
    active_vtxs, fixed_vtxs = get_active_and_fixed_vtxs(project_schedule,cache,t)
    # split active nodes
    t0 = deepcopy(cache.t0)
    tF = deepcopy(cache.tF)
    for v in active_vtxs
        node_id = get_vtx_id(project_schedule,v)
        node = get_node_from_id(project_schedule, node_id)
        # if isa(node,Union{GO,CARRY,COLLECT,DEPOSIT}) # split
        if isa(node,GO) # split
            x = robot_positions[node.r].x
            node1,node2 = split_node(node,x)
            replace_in_schedule!(project_schedule,problem_spec,node1,node_id)
            node_id2 = ActionID(get_unique_action_id())
            add_to_schedule!(project_schedule,problem_spec,node2,node_id2)
            # remap edges
            v2 = get_vtx(project_schedule, node_id2)
            for vp in outneighbors(G,v)
                rem_edge!(G,v,vp)
                add_edge!(G,v2,vp)
            end
            add_edge!(G,v,v2)
            # fix start and end times
            push!(t0,t)
            push!(tF,tF[v])
            tF[v] = t
            # reset path specs
            set_path_spec!(project_schedule,v,PathSpec(get_path_spec(project_schedule,v),fixed=true,plan_path=false,min_path_duration=tF[v]-t0[v]))
            set_path_spec!(project_schedule,v2,PathSpec(get_path_spec(project_schedule,v2),tight=true,min_path_duration=tF[v2]-t0[v2]))
        end
    end
    set_leaf_operation_vtxs!(project_schedule)
    new_cache = initialize_planning_cache(project_schedule,t0,tF)
    project_schedule, new_cache
end

"""
    fix_precutoff_nodes!(project_schedule::OperatingSchedule,
        problem_spec::ProblemSpec,cache::PlanningCache,t)

Identify all nodes that end before the cutoff time, and change their path spec
    so that the route planner will not actually plan a path for them.
"""
function fix_precutoff_nodes!(project_schedule::OperatingSchedule,problem_spec::ProblemSpec,cache::PlanningCache,t)
    # active_vtxs = Set{Int}()
    active_vtxs, fixed_vtxs = get_active_and_fixed_vtxs(project_schedule,cache,t)
    # set all fixed_vtxs to plan_path=false
    for v in fixed_vtxs
        set_path_spec!(project_schedule,v,PathSpec(get_path_spec(project_schedule,v), plan_path=false, fixed=true))
    end
    # verify that all vertices following active_vtxs have a start time > 0
    @assert all(map(v->cache.t0[v], collect(fixed_vtxs)) .<= t)
    @assert all(map(v->cache.tF[v] + minimum(cache.local_slack[v]), collect(active_vtxs)) .>= t)
    project_schedule, cache
end

"""
    break_assignments!(project_schedule::OperatingSchedule,problem_spec::ProblemSpec)

Break all assignments that are eligible for replanning
"""
function break_assignments!(project_schedule::OperatingSchedule,problem_spec::ProblemSpec)
    G = get_graph(project_schedule)
    for v in vertices(G)
        path_spec = get_path_spec(project_schedule,v)
        if path_spec.fixed == true
            continue
        end
        node_id = get_vtx_id(project_schedule,v)
        node = get_node_from_id(project_schedule,node_id)
        if isa(node, AbstractRobotAction)
            new_node = typeof(node)(node,r=RobotID(-1))
            if isa(node,GO)
                new_node = GO(new_node,x2=LocationID(-1))
                for v2 in outneighbors(G,v)
                    rem_edge!(G,v,v2)
                end
            end
            replace_in_schedule!(project_schedule,problem_spec,new_node,node_id)
        elseif isa(node,TEAM_ACTION)
            for i in 1:length(node.instructions)
                n = node.instructions[i]
                node.instructions[i] = typeof(n)(n,r=RobotID(-1))
            end
        end
    end
    propagate_valid_ids!(project_schedule,problem_spec)

    project_schedule
end

"""
    prune_schedule(project_schedule::OperatingSchedule,
        problem_spec::ProblemSpec,cache::PlanningCache,t)

    remove nodes that don't need to be kept around any longer
"""
function prune_schedule(project_schedule::OperatingSchedule,problem_spec::ProblemSpec,cache::PlanningCache,t)
    G = get_graph(project_schedule)

    # identify nodes "cut" by timestep
    active_vtxs, fixed_vtxs = get_active_and_fixed_vtxs(project_schedule,cache,t)
    # construct set of all nodes to prune out.
    remove_set = Set{Int}()
    for v in active_vtxs
        if isa(get_node_from_vtx(project_schedule,v),Operation)
            push!(remove_set, v)
        end
        for e in edges(bfs_tree(G,v;dir=:in))
            if isa(get_node_from_vtx(project_schedule, e.dst),Operation)
                push!(remove_set, e.dst)
            end
        end
    end
    for v in collect(remove_set)
        for e in edges(bfs_tree(G,v;dir=:in))
            if !(isa(get_node_from_vtx(project_schedule, e.dst),ROBOT_AT))
                push!(remove_set, e.dst)
            end
        end
    end
    # Construct new graph
    new_schedule = OperatingSchedule()
    keep_vtxs = setdiff(Set{Int}(collect(vertices(G))), remove_set)
    # add all non-deleted nodes to new project schedule
    for v in keep_vtxs
        node_id = get_vtx_id(project_schedule,v)
        node = get_node_from_id(project_schedule, node_id)
        path_spec = get_path_spec(project_schedule,v)
        add_to_schedule!(new_schedule,path_spec,node,node_id)
    end
    # add all edges between nodes that still exist
    for e in edges(get_graph(project_schedule))
        add_edge!(new_schedule, get_vtx_id(project_schedule, e.src), get_vtx_id(project_schedule, e.dst))
    end
    # Initialize new cache
    G = get_graph(new_schedule)
    t0 = map(v->get(cache.t0, get_vtx(project_schedule, get_vtx_id(new_schedule, v)), 0.0), vertices(G))
    tF = map(v->get(cache.tF, get_vtx(project_schedule, get_vtx_id(new_schedule, v)), 0.0), vertices(G))
    # draw new ROBOT_AT -> GO edges where necessary
    for v in vertices(get_graph(new_schedule))
        node_id = get_vtx_id(new_schedule,v)
        node = get_node_from_id(new_schedule, node_id)
        if isa(node,GO) && indegree(get_graph(new_schedule),v) == 0
            robot_id = get_robot_id(node)
            replace_in_schedule!(new_schedule, ROBOT_AT(robot_id, node.x1), robot_id)
            t0[get_vtx(new_schedule,robot_id)] = t0[v]
            add_edge!(new_schedule, robot_id, node_id)
        end
        if isa(node,Operation) && indegree(get_graph(new_schedule),v) < num_required_predecessors(node)
            input_ids = Set(map(v2->get_object_id(get_node_from_vtx(new_schedule,v2)), inneighbors(G,v)))
            for o in node.pre
                if !(get_object_id(o) in input_ids)
                    add_to_schedule!(new_schedule,o,get_object_id(o))
                    push!(t0,t0[v])
                    push!(tF,t0[v])
                    add_edge!(new_schedule, get_object_id(o), node_id)
                end
            end
        end
    end
    set_leaf_operation_vtxs!(new_schedule)
    # init planning cache with the existing solution
    new_cache = initialize_planning_cache(new_schedule,t0,tF)

    @assert sanity_check(new_schedule,string(" in prune_schedule() at t = ",t,":\n",[string(string(get_node_from_vtx(project_schedule,v))," - t0 = ",cache.t0[v]," - tF = ",cache.tF[v]," - local_slack = ",cache.local_slack[v],"\n") for v in active_vtxs]...))

    new_schedule, new_cache
end
prune_schedule(env::SearchEnv,args...) = prune_schedule(env.schedule,env.problem_spec,env.cache,args...)

"""
    `prune_project_schedule`

Remove all vertices that have already been completed. The idea is to identify
all `Operation`s that are completed before `t`, remove all nodes upstream of
them (except for ROBOT_AT nodes), and create new edges between the ROBOT_AT
nodes and their first GO assignments.
"""
function prune_project_schedule(project_schedule::OperatingSchedule,problem_spec::ProblemSpec,cache::PlanningCache,t;
        robot_positions::Dict{RobotID,ROBOT_AT}=Dict{RobotID,ROBOT_AT}()
    )
    new_schedule, new_cache = prune_schedule(project_schedule,problem_spec,cache,t)
    # split active nodes
    new_schedule, new_cache = split_active_vtxs!(new_schedule,problem_spec,new_cache,t;robot_positions=robot_positions)
    # freeze nodes that terminate before cutoff time
    fix_precutoff_nodes!(new_schedule,problem_spec,new_cache,t)
    # Remove all "assignments" from schedule
    break_assignments!(new_schedule,problem_spec)

    new_cache = initialize_planning_cache(new_schedule,new_cache.t0,min.(new_cache.tF,t))
    new_schedule, new_cache
end

"""
    splice_schedules!(project_schedule::P,next_schedule::P) where {P<:OperatingSchedule}

Merge next_schedule into project_schedule
"""
function splice_schedules!(project_schedule::P,next_schedule::P,enforce_unique=true) where {P<:OperatingSchedule}
    for v in vertices(get_graph(next_schedule))
        node_id = get_vtx_id(next_schedule, v)
        if !has_vertex(project_schedule,get_vtx(project_schedule,node_id))
            path_spec = get_path_spec(next_schedule,v)
            node = get_node_from_id(next_schedule, node_id)
            add_to_schedule!(project_schedule, path_spec, node, node_id)
        elseif enforce_unique
            throw(ErrorException(string("Vertex ",v," = ",
                string(get_node_from_id(next_schedule,node_id)),
                " already in project_schedule."
                )))
        end
    end
    for e in edges(get_graph(next_schedule))
        node_id1 = get_vtx_id(next_schedule, e.src)
        node_id2 = get_vtx_id(next_schedule, e.dst)
        add_edge!(project_schedule, node_id1, node_id2)
    end
    set_leaf_operation_vtxs!(project_schedule)
    project_schedule
end
splice_schedules!(project_schedule,next_schedule) = project_schedule

export
    ReplannerModel,
    ReplannerConfig,
    DeferUntilCompletion,
    ReassignFreeRobots,
    MergeAndBalance,
    Oracle,
    NullReplanner,
    get_commit_time,
    replan!

function get_timeout_buffer end
function get_route_planning_buffer end
function get_commit_time end
function break_assignments! end
function set_time_limits! end
function split_active_vtxs! end
function fix_precutoff_nodes! end
function replan! end

"""
    ReplannerModel

Abstract type. Concrete subtypes currently include `DeferUntilCompletion`,
`ReassignFreeRobots`, `MergeAndBalance`, `Oracle`, `NullReplanner`
"""
abstract type ReplannerModel end

"""
    ReplannerConfig

Stores parameters for configuring an instance of `ReplannerModel`.
Fields:
* real_time::Bool - if `true`, adjust solver runtime limits to meet real time
    operation constraints.
* timeout_buffer::Float64 - defines the minimum amount of time that must be for
    replanning the route
* route_planning_buffer::Float64 - defines the minimum amount of time that must
    be allocated for replanning the route
* commit_threshold::Float64 - defines where the ``freeze time'', or the time
    step from which the solver may change the existing plan.
"""
@with_kw mutable struct ReplannerConfig
    real_time::Bool                 = true
    timeout_buffer::Float64         = 1
    route_planning_buffer::Float64  = 2
    commit_threshold::Int           = 10
end
get_replanner_config(config::ReplannerConfig) = config
get_replanner_config(model) = model.config

export
    get_real_time_flag,
    get_timeout_buffer,
    get_route_planning_buffer,
    get_commit_threshold,
    set_real_time_flag!,
    set_timeout_buffer!,
    set_route_planning_buffer!,
    set_commit_threshold!

get_real_time_flag(model)           = get_replanner_config(model).real_time
get_timeout_buffer(model)           = get_replanner_config(model).timeout_buffer
get_route_planning_buffer(model)    = get_replanner_config(model).route_planning_buffer
get_commit_threshold(model)         = get_replanner_config(model).commit_threshold
function set_real_time_flag!(model,val) get_replanner_config(model).real_time = val end
function set_timeout_buffer!(model,val) get_replanner_config(model).timeout_buffer = val end
function set_route_planning_buffer!(model,val) get_replanner_config(model).route_planning_buffer = val end
function set_commit_threshold!(model,val) get_replanner_config(model).commit_threshold = val end

"""
    DeferUntilCompletion <: ReplannerModel

Allow work to begin on the new project only after all other work is completed.
"""
@with_kw struct DeferUntilCompletion <: ReplannerModel
    max_time_limit::Float64 = 100
    config::ReplannerConfig = ReplannerConfig()
end
"""
    ReassignFreeRobots   <: ReplannerModel

Allow robots to begin working on the new project as soon as they have finished
their current assignments.
"""
@with_kw struct ReassignFreeRobots   <: ReplannerModel
    config::ReplannerConfig = ReplannerConfig()
end
"""
    MergeAndBalance      <: ReplannerModel

Allow replanning from scratch for all assignments and routes except for those
that will take place
"""
@with_kw struct MergeAndBalance      <: ReplannerModel
    config::ReplannerConfig = ReplannerConfig()
end
@with_kw struct Oracle               <: ReplannerModel
    config::ReplannerConfig = ReplannerConfig(
        timeout_buffer         = -110,
        route_planning_buffer   = 10,
        commit_threshold        = 0,
    )
end
@with_kw struct NullReplanner        <: ReplannerModel
    config::ReplannerConfig = ReplannerConfig()
end

export
    ReplanningProfilerCache,
    FullReplanner

"""
    ReplanningProfilerCache

Stores information during replanning
* `schedule` - maintains (non-pruned) overall project schedule
* `stage_ids` - maps id of each node in schedule to stage index
* `stage_results` - vector of result dicts
* `features` - vector of `::FeatureExtractors` to extract after each stage
* `final_features` - vector of `::FeatureExtractors` to extract after final stage
"""
@with_kw struct ReplanningProfilerCache
    schedules::Vector{OperatingSchedule}    = Vector{OperatingSchedule}()
    project_ids::Dict{AbstractID,Int}       = Dict{AbstractID,Int}()
    stage_results::Vector{Dict{String,Any}} = Vector{Dict{String,Any}}()
    final_results::Dict{String,Any}         = Dict{String,Any}()
    features::Vector{FeatureExtractor}      = Vector{FeatureExtractor}()
    final_features::Vector{FeatureExtractor}= Vector{FeatureExtractor}()
end
function reset_cache!(cache::ReplanningProfilerCache)
    empty!(cache.schedules)
    empty!(cache.project_ids)
    empty!(cache.stage_results)
    empty!(cache.final_results)
    cache
end

@with_kw struct FullReplanner{R,S}
    solver::S                       = NBSSolver()
    replanner::R                    = MergeAndBalance()
    cache::ReplanningProfilerCache  = ReplanningProfilerCache()
end
reset_cache!(planner::FullReplanner) = reset_cache!(planner.cache)
get_replanner_config(planner::FullReplanner) = get_replanner_config(planner.replanner)
for op in [
        :get_commit_time,
        :break_assignments!,
        :set_time_limits!,
        :split_active_vtxs!,
        :fix_precutoff_nodes!,
        ]
    @eval $op(planner::FullReplanner,args...) = $op(planner.replanner,args...)
end
CRCBS.get_logger(planner::FullReplanner) = CRCBS.get_logger(planner.solver)

export ReplannerWithBackup

"""
    ReplannerWithBackup{A,B}

Consists of a primary and a backup solver. The backup solver is supposed to be
fast--it computes a "fallback" plan in case the primary solver fails to find a
good enough solution before time out.
"""
struct ReplannerWithBackup{A,B}
    primary_planner::A
    backup_planner::B
end
function set_real_time_flag!(model::ReplannerWithBackup,val)
    set_real_time_flag!(model.backup_planner,val)
    set_real_time_flag!(model.primary_planner,val)
end
function reset_cache!(planner::ReplannerWithBackup)
    reset_cache!(planner.primary_planner)
    reset_cache!(planner.backup_planner)
end

function get_commit_time(replan_model, search_env, t_request, commit_threshold=get_commit_threshold(replan_model))
    t_request + commit_threshold
end
get_commit_time(replan_model::Oracle, search_env, t_request, args...) = t_request
get_commit_time(replan_model::DeferUntilCompletion, search_env, t_request, commit_threshold) = max(t_request + commit_threshold,maximum(search_env.cache.tF))
get_commit_time(replan_model::NullReplanner,args...) = get_commit_time(DeferUntilCompletion(),args...)
function get_commit_time(replan_model::ReassignFreeRobots, search_env, t_request, commit_threshold)
    free_time = maximum(search_env.cache.tF)
    for v in vertices(search_env.schedule)
        node = get_node_from_vtx(search_env.schedule,v)
        if isa(node,GO)
            if get_id(get_destination_location_id(node)) == -1
                free_time = min(free_time, search_env.cache.t0[v])
            end
        end
    end
    max(t_request + commit_threshold,free_time)
end

break_assignments!(replan_model::ReplannerModel,args...) = break_assignments!(args...)
break_assignments!(replan_model::ReassignFreeRobots,args...) = nothing
break_assignments!(replan_model::DeferUntilCompletion,args...) = nothing
break_assignments!(replan_model::NullReplanner,args...) = nothing

function set_time_limits!(replan_model,solver,t_request,t_commit)
    real_time = get_real_time_flag(replan_model)
    if real_time
        set_time_limits!(real_time,replan_model,solver,t_request,t_commit)
    end
end

function set_time_limits!(flag::Bool,replan_model,solver,t_request,t_commit)
    set_runtime_limit!(solver, (t_commit - t_request) - get_timeout_buffer(replan_model))
    # set_runtime_limit!(assignment_solver(solver), solver.time_limit - get_route_planning_buffer(replan_model))
    set_runtime_limit!(solver.assignment_model, runtime_limit(solver) - get_route_planning_buffer(replan_model))
    @assert runtime_limit(solver) > 0.0
    solver
end
function set_time_limits!(flag::Bool,replan_model::DeferUntilCompletion,solver,t_request,t_commit)
    set_runtime_limit!(solver, (t_commit - t_request) - get_timeout_buffer(replan_model))
    set_runtime_limit!(solver, min(runtime_limit(solver),replan_model.max_time_limit))
    set_runtime_limit!(assignment_solver(solver), runtime_limit(solver) - get_route_planning_buffer(replan_model))
    @assert runtime_limit(solver) > 0.0
    solver
end
function set_time_limits!(flag::Bool,replan_model::NullReplanner,solver,t_request,t_commit)
    set_runtime_limit!(solver,-1.0)
    return solver
end

split_active_vtxs!(replan_model::ReplannerModel,new_schedule,problem_spec,new_cache,t_commit;kwargs...) = split_active_vtxs!(new_schedule,problem_spec,new_cache,t_commit;kwargs...)

fix_precutoff_nodes!(replan_model,new_schedule,problem_spec,new_cache,t_commit) = fix_precutoff_nodes!(new_schedule,problem_spec,new_cache,t_commit)


"""
    replan!(solver, replan_model, search_env, env_graph, problem_spec, route_plan,
        next_schedule, t_request, t_arrival; commit_threshold=5,kwargs...)

Combine an existing solution with a new project request `next_schedule` that
is requested at time `t_request`, and whose raw materials become available at
`t_arrival`.
The exact behavior of this function depends on `replan_model <: ReplannerModel`
"""
# function replan!(solver, replan_model, search_env, env_graph, problem_spec, route_plan, next_schedule, t_request, t_arrival; commit_threshold=5,kwargs...)
function replan!(solver, replan_model, search_env, request;
        commit_threshold=get_commit_threshold(replan_model),
        kwargs...
        )
    project_schedule    = search_env.schedule
    cache               = search_env.cache
    route_plan          = search_env.route_plan
    problem_spec        = search_env.problem_spec
    env_graph           = search_env.env_graph
    t_request           = request.t_request
    t_arrival           = request.t_arrival
    next_schedule       = request.schedule

    @assert sanity_check(project_schedule," in replan!()")
    # Freeze route_plan and schedule at t_commit
    t_commit = get_commit_time(replan_model, search_env, t_request, commit_threshold)
    reset_solver!(solver)
    set_time_limits!(replan_model,solver,t_request,t_commit)
    # Update operating schedule
    new_schedule, new_cache = prune_schedule(search_env,t_commit)
    @assert sanity_check(new_schedule," after prune_schedule()")
    # split active nodes
    robot_positions=get_env_snapshot(search_env,t_commit)
    new_schedule, new_cache = split_active_vtxs!(replan_model,new_schedule,problem_spec,new_cache,t_commit;robot_positions=robot_positions)
    @assert sanity_check(new_schedule," after split_active_vtxs!()")
    # freeze nodes that terminate before cutoff time
    new_schedule, new_cache = fix_precutoff_nodes!(replan_model,new_schedule,problem_spec,new_cache,t_commit)
    # Remove all "assignments" from schedule
    break_assignments!(replan_model,new_schedule,problem_spec)
    @assert sanity_check(new_schedule," after break_assignments!()")
    new_cache = initialize_planning_cache(new_schedule,new_cache.t0,min.(new_cache.tF,t_commit))
    # splice projects together!
    splice_schedules!(new_schedule,next_schedule)
    @assert sanity_check(new_schedule," after splice_schedules!()")
    # NOTE: better performance is obtained when t_commit is the default t0 (tighter constraint on milp)
    t0 = map(v->get(new_cache.t0, v, t_commit), vertices(get_graph(new_schedule)))
    tF = map(v->get(new_cache.tF, v, t_commit), vertices(get_graph(new_schedule)))
    base_search_env = construct_search_env(
        solver,
        new_schedule,
        search_env,
        initialize_planning_cache(new_schedule,t0,tF)
        )
    base_route_plan = initialize_route_plan(search_env,get_cost_model(base_search_env))
    # @log_info(3,solver,"Previous route plan: \n",sprint_route_plan(route_plan))
    trimmed_route_plan = trim_route_plan(base_search_env, base_route_plan, t_commit)
    # @log_info(3,solver,"Trimmed route plan: \n",sprint_route_plan(trimmed_route_plan))
    SearchEnv(base_search_env, route_plan=trimmed_route_plan)
end
# replan!(solver, replan_model::NullReplanner, search_env, args...;kwargs...) = search_env
function replan!(solver::FullReplanner,args...;kwargs...)
    replan!(solver.solver,solver.replanner,args...;kwargs...)
end

# end
