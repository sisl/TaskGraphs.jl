# module Solvers
#
# using Parameters
# using MathOptInterface, JuMP
# using GraphUtils
# using DataStructures
#
# using CRCBS
# using ..TaskGraphs

"""
    AbstractPCTAPFSolver

Abstract type of which all PC-TAPF solvers must be concrete subtypes. All
concrete solvers must implement the following interface for solving PC-TAPF
problems:
- `solution, cost = solve!(solver,problem_def)`
- `check_runtime(solver)` should trigger an interrupt + early return if the
    allowable runtime has been exceeded

Also, we need a good Logger type for keeping track of thing like runtime,
iterations, optimality gap (including upper and lower bound), etc.
"""
abstract type AbstractPCTAPFSolver end

abstract type SearchTrait end
struct Prioritized <: SearchTrait end
struct NonPrioritized <: SearchTrait end
function search_trait end

# get_primary_cost(::NonPrioritized,model,cost) = cost[1]
# get_primary_cost(::Prioritized,model,cost) = cost[2]
# get_primary_cost(solver,cost) = get_primary_cost(search_trait(solver),solver,cost)

primary_cost(solver,cost) = cost[1]
primary_cost_type(solver) = Float64

export AStarSC

"""
    AStarSC

Low-level path planner that employs Slack-and-Collision-Aware A*.
Fields:
- logger
- replan : if true, planner will replan with an empty conflict table following
    timeout.
"""
@with_kw struct AStarSC{C} <: AbstractAStarPlanner
    logger::SolverLogger{C} = SolverLogger{C}()
    replan::Bool            = false
end
AStarSC() = AStarSC{NTuple{5,Float64}}()
search_trait(solver::AStarSC) = NonPrioritized()
function CRCBS.logger_step_a_star!(solver::AStarSC, env, base_path, s, q_cost)
    increment_iteration_count!(solver)
    # if solver.DEBUG
    #     push!(solver.search_history, s)
    # end
    if check_iterations(solver)
        # if solver.DEBUG
        #     # Dump env to JLD2 environment
        #     filename = joinpath(DEBUG_PATH,string("A_star_dump_",get_debug_file_id(),".jld2"))
        #     mkpath(DEBUG_PATH)
        #     log_info(-1,solver,"Dumping A* env to $filename")
        #     agent_id = env.agent_idx
        #     history = map(s->(s.vtx,s.t),solver.search_history)
        #     start   = (get_final_state(base_path).vtx,get_final_state(base_path).t)
        #     goal    = (env.goal.vtx,env.goal.t)
        #     idx = isa(solver.AStarSC) ? 2 : 1
        #     paths   = get_cost_model(env).cost_models[idx].model.table.paths
        #     state_constraints = map(c->(c.a,(c.v.s.vtx,c.v.sp.vtx),c.t),collect(env.constraints.state_constraints))
        #     action_constraints = map(c->(c.a,(c.v.s.vtx,c.v.sp.vtx),c.t),collect(env.constraints.action_constraints))
        #     @save filename agent_id history start goal paths state_constraints action_constraints
        #     # @assert false "making a bogus assertion to hack my way out of this block"
        # end
        # throw(SolverAstarMaxOutException(string("# MAX OUT: A* limit of ",solver.LIMIT_A_star_iterations," exceeded.")))
    end
    log_info(2,solver,"A* iter $(iterations(solver)): s = $(string(s)), q_cost = $q_cost")
end

export
    PrioritizedAStarSC

"""
    PrioritizedAStarSC

Low-level proritized path planner that employs Slack-and-Collision-Aware A*.
"""
@with_kw struct PrioritizedAStarSC{C}
    logger::SolverLogger{C} = SolverLogger{C}()
    replan::Bool            = false
end
search_trait(solver::PrioritizedAStarSC) = Prioritized()
primary_cost(::PrioritizedAStarSC,cost::NTuple{5,Float64}) = cost[2]

# export
#     construct_heuristic_model

"""
    construct_heuristic_model(solver,env_graph;kwargs...)

Construct the heuristic model to be used by solver.
"""
function construct_heuristic_model(trait::NonPrioritized,solver,env_graph,
        ph = PerfectHeuristic(get_dist_matrix(env_graph))
        )
    construct_composite_heuristic(ph,NullHeuristic(),ph,ph,NullHeuristic())
end
function construct_heuristic_model(trait::Prioritized,args...)
    h = construct_heuristic_model(NonPrioritized(),args...)
    construct_composite_heuristic(
        h.cost_models[2],
        h.cost_models[1],
        h.cost_models[3:end]...
    )
end
function construct_heuristic_model(solver, args...)
    construct_heuristic_model(search_trait(solver),solver,args...)
end

# export
#     construct_cost_model

"""
    construct_cost_model(solver::AStarSC, args...;kwargs...)

Defines the cost model used by Slack- and Collision-aware A*.
This particular setting of cost model is crucial for good performance of A_star,
because it encourages depth first search. If we were to replace terms 3-5 with
SumOfTravelTime(), we would get worst-case exponentially slow breadth-first
search!
"""
function construct_cost_model(trait::NonPrioritized,
        solver, schedule, cache, problem_spec, env_graph, primary_objective=SumOfMakeSpans();
        extra_T::Int=400)
    N = problem_spec.N
    cost_model = construct_composite_cost_model(
        typeof(primary_objective)(schedule,cache),
        HardConflictCost(env_graph,maximum(cache.tF)+extra_T, N),
        SumOfTravelDistance(),
        FullCostModel(sum,NullCost()), # SumOfTravelTime(),
        FullCostModel(sum,TransformCostModel(c->-1*c,TravelTime()))
    )
    heuristic_model = construct_heuristic_model(trait,solver,env_graph)
    cost_model, heuristic_model
end
function construct_cost_model(trait::SearchTrait,
        solver, env::SearchEnv,
        primary_objective=env.problem_spec.cost_function
        ;
        kwargs...)
    construct_cost_model(trait,solver,
        env.schedule, env.cache, env.problem_spec, env.env_graph,
        primary_objective;kwargs...)
end
function construct_cost_model(trait::Prioritized,args...;kwargs...)
    c, h = construct_cost_model(NonPrioritized(),args...;kwargs...)
    # switch the first two elements of the cost and heuristic models
    cost_model = construct_composite_cost_model(
        c.cost_models[2],
        c.cost_models[1],
        c.cost_models[3:end]...
    )
    heuristic = construct_composite_heuristic(
        h.cost_models[2],
        h.cost_models[1],
        h.cost_models[3:end]...
    )
    cost_model, heuristic
end
function construct_cost_model(solver, args...;kwargs...)
    construct_cost_model(search_trait(solver),solver,args...;kwargs...)
end

export
    update_route_plan!

"""
    update_route_plan!()
"""
function update_route_plan!(solver,env,v,path,cost,schedule_node,agent_id = get_path_spec(env.schedule, v).agent_id)
    # add to solution
    # log_info(3,solver,string("agent_path = ", convert_to_vertex_lists(path)))
    # log_info(3,solver,string("cost = ", get_cost(path)))
    set_solution_path!(env, path, agent_id)
    set_path_cost!(env, cost, agent_id)
    update_env!(solver,env,v,path)
    set_cost!(env,aggregate_costs(get_cost_model(env),get_path_costs(env)))
    return env
end
function update_route_plan!(solver,env,v,meta_path,meta_cost,schedule_node::TEAM_ACTION)
    # add to solution
    paths = MetaAgentCBS.split_path(meta_path)
    # @show length(paths)
    # @show length(meta_env.envs)
    # @show length(meta_cost.independent_costs)
    # @show length(schedule_node.instructions)
    for (new_path, sub_node) in zip(paths, schedule_node.instructions)
        agent_id = get_id(get_robot_id(sub_node))
        path = get_paths(env)[agent_id]
        for p in new_path.path_nodes
            push!(path, p)
            # path.cost = accumulate_cost(cbs_env, get_cost(path), get_transition_cost(cbs_env, p.s, p.a, p.sp))
        end
        set_cost!(path,get_cost(new_path))
        update_route_plan!(solver,env,v,path,path.cost,sub_node,agent_id)
        # update_env!(solver,env,route_plan,v,path,agent_id)
        # Print for debugging
        # @show agent_id
        log_info(3,solver,"agent_id = ", agent_id)
        log_info(3,solver,string("agent_path = ", convert_to_vertex_lists(path)))
        log_info(3,solver,string("cost = ", get_cost(new_path)))
    end
    return env
end

export
    plan_path!

path_finder(solver::AStarSC,args...) = CRCBS.a_star(solver,args...)
"""
    plan_path!

    Computes nxxt path specified by the project schedule and updates the
    solution in the ConstraintTreeNode::node accordingly.
"""
function plan_path!(solver::AStarSC, env::SearchEnv, node::N,
        schedule_node::T, v::Int;
        kwargs...) where {N<:ConstraintTreeNode,T}

    cache = env.cache
    node_id = get_vtx_id(env.schedule,v)

    reset_solver!(solver)
    cbs_env = build_env(solver, env, node, VtxID(v))#schedule_node, v)
    base_path = get_base_path(env,cbs_env)
    ### PATH PLANNING ###
    # solver.DEBUG ? validate(base_path,v) : nothing
    path, cost = path_finder(solver, cbs_env, base_path)
    @assert get_cost(path) == cost
    if cost == get_infeasible_cost(cbs_env)
        if solver.replan == true
            log_info(-1,solver,"A*: replanning without conflict cost", string(schedule_node))
            reset_solver!(solver)
            cost_model, _ = construct_cost_model(solver, env;
                primary_objective=env.problem_spec.cost_function)
            cbs_env = build_env(solver, env, node, VtxID(v);cost_model=cost_model)
            base_path = get_base_path(env,cbs_env)
            path, cost = path_finder(solver, cbs_env, base_path)
        end
        if cost == get_infeasible_cost(cbs_env)
            log_info(-1,solver,"A*: returned infeasible path for node ", string(schedule_node))
            return false
        end
    end
    # solver.DEBUG ? validate(path,v,cbs_env) : nothing
    #####################
    log_info(2,solver,string("A* iterations = ",iterations(solver)))
    # Make sure every robot sticks around for the entire time horizon
    if is_terminal_node(get_graph(env.schedule),v)
        log_info(2,solver,"ISPS: Extending terminal node")
        extend_path!(cbs_env,path,maximum(cache.tF))
        # solver.DEBUG ? validate(path,v) : nothing
    end
    # add to solution
    update_route_plan!(solver,env,v,path,cost,schedule_node)
    if get_cost(node) >= best_cost(solver)
        log_info(0,solver,"ISPS: get_cost(node) >= best_cost(solver) ... Exiting early")
        return false
    end

    return true
end

export
    ISPS

"""
    ISPS

Path planner that employs Incremental Slack-Prioritized Search.
"""
@with_kw struct ISPS{L,C} <: BiLevelPlanner
    low_level_planner::L = AStarSC()
    logger::SolverLogger{C} = SolverLogger{cost_type(low_level_planner)}(
        iteration_limit = 2
    )
end
ISPS(planner) = ISPS(low_level_planner=planner)
construct_cost_model(solver::ISPS,args...;kwargs...) = construct_cost_model(low_level(solver),args...;kwargs...)
search_trait(solver::ISPS) = search_trait(low_level(solver))
primary_cost(solver::ISPS,cost) = primary_cost(low_level(solver),cost)
primary_cost_type(solver::ISPS) = primary_cost_type(low_level(solver))
function CRCBS.set_best_cost!(solver::ISPS,cost)
    set_best_cost!(get_logger(solver),cost)
    set_best_cost!(low_level(solver),cost)
end
function CRCBS.hard_reset_solver!(solver::ISPS)
    hard_reset_solver!(get_logger(solver))
    hard_reset_solver!(low_level(solver))
end

export
    plan_next_path!

"""
    `plan_next_path`
"""
function plan_next_path!(solver::ISPS, env::SearchEnv, node::N
        ) where {N<:ConstraintTreeNode}

    valid_flag = true
    if ~isempty(env.cache.node_queue)
        v,priority = dequeue_pair!(env.cache.node_queue)
        node_id = get_vtx_id(env.schedule,v)
        schedule_node = get_node_from_id(env.schedule,node_id)
        if get_path_spec(env.schedule, v).plan_path == true
            try
                valid_flag = plan_path!(low_level(solver),env,node,schedule_node,v)
            catch e
                if isa(e, SolverException)
                    showerror(stdout, e, catch_backtrace())
                    valid_flag = false
                    return valid_flag
                else
                    rethrow(e)
                end
            end
        else
            # dummy path - update planning cache only
            path = path_type(env)(
                s0=State(-1, -1),
                cost=get_initial_cost(env)
                )
            update_planning_cache!(solver,env,v,path) # NOTE I think this is all we need, since there is no actual path to update
        end
    end
    return valid_flag
end

"""
    Computes all paths specified by the project schedule and updates the
    solution in the ConstraintTreeNode::node accordingly.
"""
function CRCBS.low_level_search!(solver::ISPS, node::N, env::SearchEnv=node.solution;
        kwargs...
        ) where {N<:ConstraintTreeNode}

    # reset_route_plan!(node,env.route_plan)
    while length(env.cache.node_queue) > 0
        if !(plan_next_path!(solver,env,node))
            return false
        end
        # @show convert_to_vertex_lists(env.route_plan)
        enforce_time_limit(solver)
    end
    return true
end

"""
    low_level_search!
"""
function CRCBS.low_level_search!(solver::ISPS, pc_mapf::M,
        node::N=initialize_root_node(solver,pc_mapf),
        idxs::Vector{Int}=Vector{Int}()
    ) where {M<:PC_MAPF,N<:ConstraintTreeNode}

    # enter_low_level!(solver)
    reset_solver!(solver)
    valid_flag = true
    search_env = node.solution
    for i in 1:iteration_limit(solver)
        increment_iteration_count!(solver)
        # reset solution
        reset_cache!(
            search_env.cache,
            search_env.schedule,
            pc_mapf.env.cache.t0,
            pc_mapf.env.cache.tF)
        reset_route_plan!(node,
            pc_mapf.env.route_plan)
        valid_flag = low_level_search!(solver, node)
        if valid_flag == false
            log_info(0,solver,"ISPS: failed on ",i,"th repair iteration.")
            break
        end
    end
    # exit_low_level!(solver)
    # return search_env, valid_flag
    return valid_flag
end
function CRCBS.low_level_search!(solver::CBSSolver, pc_mapf::PC_MAPF,node::ConstraintTreeNode,args...;kwargs...)
    low_level_search!(low_level(solver),pc_mapf,node,args...)
end

"""
    solve!(solver::ISPS,mapf::M) where {M<:PC_MAPF}

Solve a PC-MAPF problem with the ISPS algorithm.
"""

function CRCBS.solve!(solver::ISPS,pc_mapf::PC_MAPF)
    node = initialize_root_node(solver,pc_mapf)
    valid_flag = low_level_search!(solver,pc_mapf,node)#,node)
    return node.solution, get_cost(node.solution)
end

construct_cost_model(solver::CBSSolver,args...;kwargs...) = construct_cost_model(low_level(solver),args...;kwargs...)
search_trait(solver::CBSSolver) = search_trait(low_level(solver))
primary_cost(solver::CBSSolver,cost) = primary_cost(low_level(solver),cost)
primary_cost_type(solver::CBSSolver) = primary_cost_type(low_level(solver))
CRCBS.solve!(solver::CBSSolver,pc_mapf::PC_MAPF) = CRCBS.cbs!(solver,pc_mapf)

export
    TaskGraphsMILPSolver

"""
    TaskGraphsMILPSolver

Wrapper for MILP solver for assignment problem.
"""
@with_kw struct TaskGraphsMILPSolver{M,C}
    milp::M = SparseAdjacencyMILP()
    logger::SolverLogger{C} = SolverLogger{Int}()
end
TaskGraphsMILPSolver(milp) = TaskGraphsMILPSolver(milp,SolverLogger{Int}())
get_assignment_matrix(solver::TaskGraphsMILPSolver) = get_assignment_matrix(solver.milp)

function formulate_milp(solver::TaskGraphsMILPSolver,args...;kwargs...)
    formulate_milp(solver.milp,args...;kwargs...)
end

export
    NBSSolver,
    assignment_solver,
    route_planner

"""
    NBSSolver{A,P}

A hierarchical PC-TAPF solver with an assignment level and a path-planning
level.
The solver works by alternating between assignment and path-planning until the
optimality gap between the lower bound (from task assignment) and the lower
bound (from path planning) disappears.
The input to the assignment problem is the full PC-TAPF problem specification.
The output of the assignment problem is a valid `OperatingSchedule`--that is,
an operating schedule wherein all assignments have been made in a legal way.
The input to the route planner is the PC-TAPF problem spec along with the
`OperatingSchedule` that comes from the assignment solution.
"""
@with_kw struct NBSSolver{A,P,C} <: AbstractPCTAPFSolver
    assignment_model::A     = TaskGraphsMILPSolver()
    path_planner    ::P     = CBSSolver(ISPS())
    logger          ::SolverLogger{C} = SolverLogger{primary_cost_type(path_planner)}()
end
NBSSolver(a,b) = NBSSolver(assignment_model=a,path_planner=b)
assignment_solver(solver::NBSSolver) = solver.assignment_model
route_planner(solver::NBSSolver) = solver.path_planner
construct_cost_model(solver::NBSSolver,args...;kwargs...) = construct_cost_model(route_planner(solver),args...;kwargs...)
search_trait(solver::NBSSolver) = search_trait(route_planner(solver))
primary_cost(solver::NBSSolver,cost) = primary_cost(route_planner(solver),cost)
primary_cost_type(solver::NBSSolver) = primary_cost_type(route_planner(solver))
function CRCBS.set_best_cost!(solver::NBSSolver,cost)
    set_best_cost!(get_logger(solver),cost)
    set_best_cost!(route_planner(solver),cost)
end
function CRCBS.hard_reset_solver!(solver::NBSSolver)
    hard_reset_solver!(get_logger(solver.logger))
    hard_reset_solver!(route_planner(solver))
end

"""
    solve!(solver, base_env::SearchEnv;kwargs...) where {A,P}

Use the planner defined by `solver` to solve the PC-TAPF problem encoded by
`base_env`. For solvers of type `NBSSolver`, the algorithm involves
repeatedly solving an assignment problem followed by a route-planning problem.
Within the generic `solve!` method it is possible to initialize an assignment
problem (the type is not constrained) and then modify it via
`update_assignment_problem!` prior to each new call to
`solve_assignment_problem!`. This is the approach taken for various MILP-based
assignment solvers. It is also possible to reconstruct the assignment problem
from scratch within each call to `solve_assignment_problem!`.

Arguments:
- solver <: AbstractPCTAPFSolver
- base_env::SearchEnv : a PC-TAPF problem

Outputs:
- best_env : a `SearchEnv` data structure that encodes a solution to the problem
- cost : the cost of the solution encoded by `best_env`
"""
function CRCBS.solve!(solver::NBSSolver{A,P,C}, base_env::E;kwargs...) where {A,P,C,E<:SearchEnv}
    best_env = SearchEnv(base_env,route_plan=initialize_route_plan(base_env))
    set_cost!(best_env,get_infeasible_cost(best_env))
    set_best_cost!(solver, primary_cost(solver, get_cost(best_env)))
    assignment_problem = formulate_assignment_problem(assignment_solver(solver), base_env;
        kwargs...)
    while optimality_gap(solver) > 0
        try
            update_assignment_problem!(assignment_solver(solver), assignment_problem, base_env)
            schedule, l_bound = solve_assignment_problem!(
                assignment_solver(solver),
                assignment_problem,
                base_env;kwargs...)
            set_lower_bound!(solver,l_bound)
            if optimality_gap(solver) > 0
                env, cost = plan_route!(
                    route_planner(solver),
                    schedule,
                    base_env;kwargs...)
                if cost < best_cost(solver)
                    best_env = env
                    set_best_cost!(solver,cost)
                end
            end
            increment_iteration_count!(solver)
            enforce_time_limit(solver)
            if check_iterations(solver)
                log_info(1,solver,
                    "NBS: Reached $(iteration_limit(solver))-iteration limit.")
                break
            end
        catch e
            if isa(e, SolverException)
                bt = catch_backtrace()
                showerror(stdout, e, bt)
                # log_info(-1, solver, sprint(showerror, e, bt))
                break
            else
                rethrow(e)
            end
        end
    end
    return best_env, best_cost(solver)
end
function CRCBS.solve!(solver, env_graph, schedule::OperatingSchedule, problem_spec::ProblemSpec;
    kwargs...)
    base_env = construct_search_env(solver,schedule,problem_spec,env_graph;kwargs...)
    # @show base_env.cache.t0
    solve!(solver, base_env;kwargs...)
end
function CRCBS.solve!(solver, env_graph, project_spec::ProjectSpec, problem_spec::ProblemSpec,
        robot_ICs;kwargs...)
    schedule = construct_partial_project_schedule(
        project_spec,problem_spec,map(i->robot_ICs[i], 1:problem_spec.N))
    solve!(solver, env_graph, schedule, problem_spec;kwargs...)
end

export
    formulate_assignment_problem

"""
    formulate_assignment_problem(solver,base_env::SearchEnv;

Returns an assignment problem instance that can be updated (as opposed to being
reconstructed from scratch) on each call to `update_assignment_problem!` prior
to being resolved.
"""
function formulate_assignment_problem(solver,base_env::SearchEnv;
        # cost_model=base_env.problem_spec.cost_function,
        # optimizer=get_optimizer(solver), #TODO where to pass in the optimizer?
        kwargs...)

    project_schedule    = base_env.schedule
    problem_spec        = base_env.problem_spec
    formulate_milp(solver,project_schedule,problem_spec;
        cost_model=base_env.problem_spec.cost_function,
        # optimizer=optimizer,
        kwargs...)
end

export
    update_assignment_problem!

"""
    update_assignment_problem!(solver, assignment_problem)

A helper method for updating an instance of an assignment problem. In the case
    of MILP-based models, this method simply excludes all previous solutions by
    adding new constraints on the assignment/adjacency matrix.
"""
function update_assignment_problem!(solver, model::T, base_env) where {T<:TaskGraphsMILP}
    exclude_solutions!(model) # exclude most recent solution in order to get next best solution
end

export
    solve_assignment_problem!

"""
    solve_assignment_problem!(solver,base_env;kwargs...)

Solve the "assignment problem"--i.e., the relaxation of the full PC-TAPF problem
wherein we ignore collisions--using the algorithm encoded by solver.
"""
function solve_assignment_problem!(solver::S, model, base_env;
        t0_ = Dict{AbstractID,Int}(get_vtx_id(base_env.schedule, v)=>t0 for (v,t0) in enumerate(base_env.cache.t0)),
        tF_ = Dict{AbstractID,Int}(get_vtx_id(base_env.schedule, v)=>tF for (v,tF) in enumerate(base_env.cache.tF)),
        TimeLimit=min(deadline(solver)-time(),runtime_limit(solver)),
        buffer=5.0, # to give some extra time to the path planner if the milp terminates late.
        kwargs...) where {S<:TaskGraphsMILPSolver}

    # enter_assignment!(solver)
    l_bound = lower_bound(solver)
    optimize!(model)
    if primal_status(model) != MOI.FEASIBLE_POINT
        throw(SolverException("Assignment problem is infeasible -- in `solve_assignment_problem!()`"))
    end
    set_lower_bound!(solver, Int(round(value(objective_bound(model)))) )
    set_best_cost!(solver, Int(round(value(objective_function(model)))) )
    if termination_status(model) == MOI.OPTIMAL
        @assert lower_bound(solver) == best_cost(solver)
    end
    schedule = deepcopy(base_env.schedule)
    update_project_schedule!(model, schedule, base_env.problem_spec)
    schedule, lower_bound(solver)
end


export
    plan_route!

"""
    plan_route!

Compute a route plan that corresponds to the OperatingSchedule.
Arguments:
- solver
- schedule
- search_env

Outputs:
- A `SearchEnv` the contains a valid solution
"""
function plan_route!(
        solver,
        schedule,
        base_env;
        kwargs...)

    env = construct_search_env(solver, schedule, base_env;kwargs...)
    solution, cost = solve!(solver, PC_MAPF(env))
    solution, primary_cost(solver,get_cost(solution))
end

# end
