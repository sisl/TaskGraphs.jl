export
    TaskGraphsMILP,
    AssignmentMILP,
    AdjacencyMILP,
    SparseAdjacencyMILP

"""
    TaskGraphsMILP

Concrete subtypes of `TaskGraphsMILP` define different ways to formulat the
sequential assignment portion of a PC-TAPF problem.
"""
abstract type TaskGraphsMILP end
"""
    AssignmentMILP <: TaskGraphsMILP

Used to formulate a MILP where the decision variable is a matrix `X`, where
`X[i,j] = 1` means that robot `i` is assigned to delivery task `j`. The
dimensionality of `X` is (N+M) × M, where N is the number of robots and M is the
number of delivery tasks. the last M rows of `X` correspond to "dummy robots",
i.e. the N+jth row corresponds to "the robot that already completed task j". The
use of these dummy robot variables allows the sequential assignment problem to
be posed as a one-off assignment problem with inter-task constraints.
"""
@with_kw struct AssignmentMILP <: TaskGraphsMILP
    model::JuMP.Model = Model()
end

"""
    adj_mat_from_assignment_mat(sched,assignment_matrix)

Compute an adjacency matrix from an assignment matrix
"""
function adj_mat_from_assignment_mat(sched::OperatingSchedule,assignment_matrix)
    N = length(get_robot_ICs(sched))
    M = length(get_object_ICs(sched))
    assignment_dict = get_assignment_dict(assignment_matrix,N,M)
    G = get_graph(sched)
    adj_matrix = adjacency_matrix(G)
    for (robot_id, task_list) in assignment_dict
        robot_node = get_node_from_id(sched, RobotID(robot_id))
        v_go = outneighbors(G, get_vtx(sched, RobotID(robot_id)))[1] # GO_NODE
        for object_id in task_list
            v_collect = outneighbors(G,get_vtx(sched, ObjectID(object_id)))[1]
            adj_matrix[v_go,v_collect] = 1
            v_carry = outneighbors(G,v_collect)[1]
            v_deposit = outneighbors(G,v_carry)[1]
            for v in outneighbors(G,v_deposit)
                if isa(get_vtx_id(sched, v), ActionID)
                    v_go = v
                    break
                end
            end
        end
    end
    adj_matrix
end

"""
    TeamAssignmentMILP

***Not yet implemented.***

Eextend the assignment matrix
formulation of `AssignmentMILP` to the "team-forming" case where robots must
collaboratively transport some objects.
"""
@with_kw struct TeamAssignmentMILP <: TaskGraphsMILP
    model::JuMP.Model = Model()
    task_group::Vector{Vector{Int}}
end
@with_kw struct AdjacencyMILP <: TaskGraphsMILP
    model::JuMP.Model = Model()
    job_shop::Bool=false
end
"""
    SparseAdjacencyMILP

Formulates a MILP where the decision variable is a sparse adjacency matrix `X`
    for the operating schedule graph. If `X[i,j] = 1`, there is an edge from
    node `i` to node `j`.
Experiments have shown that the sparse matrix approach leads to much faster
solve times than the dense matrix approach.
"""
@with_kw struct SparseAdjacencyMILP <: TaskGraphsMILP
    model::JuMP.Model = Model()
    Xa::SparseMatrixCSC{VariableRef,Int} = SparseMatrixCSC{VariableRef,Int}(0,0,ones(Int,1),Int[],VariableRef[]) # assignment adjacency matrix
    Xj::SparseMatrixCSC{VariableRef,Int} = SparseMatrixCSC{VariableRef,Int}(0,0,ones(Int,1),Int[],VariableRef[]) # job shop adjacency matrix
    job_shop::Bool=false
end
JuMP.optimize!(model::M) where {M<:TaskGraphsMILP}          = optimize!(model.model)
JuMP.termination_status(model::M) where {M<:TaskGraphsMILP} = termination_status(model.model)
JuMP.objective_function(model::M) where {M<:TaskGraphsMILP} = objective_function(model.model)
JuMP.objective_bound(model::M) where {M<:TaskGraphsMILP}    = objective_bound(model.model)
JuMP.primal_status(model::M) where {M<:TaskGraphsMILP}      = primal_status(model.model)
JuMP.dual_status(model::M) where {M<:TaskGraphsMILP}        = dual_status(model.model)
# for op = (:optimize!, :termination_status, :objective_function)
#     eval(quote
#         JuMP.$op(model::M,args...) where {M<:TaskGraphsMILP} = $op(model.model,args...)
#     end)
# end

export
    exclude_solutions!,
    exclude_current_solution!

"""
    exclude_solutions!(model::JuMP.Model,forbidden_solutions::Vector{Matrix{Int}})

Adds constraints to model such that the solution may not match any solution
contained in forbidden_solutions. Assumes that the model contains a variable
container called X whose entries are binary and whose dimensions are identical
to the dimensions of each solution in forbidden_solutions.
"""
function exclude_solutions!(model::JuMP.Model,X::Matrix{Int})
    @assert !any((X .< 0) .| (X .> 1))
    @constraint(model, sum(model[:X] .* X) <= sum(model[:X])-1)
end
exclude_solutions!(model::TaskGraphsMILP,args...) = exclude_solutions!(model.model, args...)
function exclude_solutions!(model::JuMP.Model,M::Int,forbidden_solutions::Vector{Matrix{Int}})
    for X in forbidden_solutions
        exclude_solutions!(model,X)
    end
end
function exclude_solutions!(model::JuMP.Model)
    if termination_status(model) != MOI.OPTIMIZE_NOT_CALLED
        X = get_assignment_matrix(model)
        exclude_solutions!(model,X)
    end
end
exclude_current_solution!(args...) = exclude_solutions!(args...)


export
    get_assignment_matrix

function get_assignment_matrix(model::M) where {M<:JuMP.Model}
    Matrix{Int}(min.(1, round.(value.(model[:X])))) # guarantees binary matrix
end
get_assignment_matrix(model::TaskGraphsMILP) = get_assignment_matrix(model.model)

export
    add_job_shop_constraints!,
    formulate_optimization_problem,
    formulate_schedule_milp,
    formulate_milp,
    update_project_schedule!

function add_job_shop_constraints!(schedule::P,spec::T,model::JuMP.Model) where {P<:OperatingSchedule,T<:ProblemSpec}
    # M = spec.M
    M = length(get_object_ICs(schedule))
    s0 = spec.s0
    sF = spec.sF
    tor = Int.(round.(value.(model[:tor]))) # collect begin time
    toc = Int.(round.(value.(model[:toc]))) # collect end time
    tod = Int.(round.(value.(model[:tod]))) # deposit begin time
    tof = Int.(round.(value.(model[:tof]))) # deposit end time
    for j in 1:M
        for j2 in j+1:M
            if (s0[j] == s0[j2]) || (s0[j] == sF[j2]) || (sF[j] == s0[j2]) || (sF[j] == sF[j2])
                if s0[j] == s0[j2]
                    id1, n1 = get_collect_node(schedule, ObjectID(j))
                    id2, n2 = get_collect_node(schedule, ObjectID(j2))
                    t1 = [tor[j], toc[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif s0[j] == sF[j2]
                    id1, n1 = get_collect_node(schedule, ObjectID(j))
                    id2, n2 = get_deposit_node(schedule, ObjectID(j2))
                    t1 = [tor[j], toc[j]]
                    t2 = [tod[j2], tof[j2]]
                elseif sF[j] == s0[j2]
                    id1, n1 = get_deposit_node(schedule, ObjectID(j))
                    id2, n2 = get_collect_node(schedule, ObjectID(j2))
                    t1 = [tod[j], tof[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif sF[j] == sF[j2]
                    id1, n1 = get_deposit_node(schedule, ObjectID(j))
                    id2, n2 = get_deposit_node(schedule, ObjectID(j2))
                    t1 = [tod, tof[j]]
                    t2 = [tod, tof[j2]]
                end
                if t1[2] < t2[1]
                    add_edge!(schedule, id1, id2)
                elseif t2[2] < t1[1]
                    add_edge!(schedule, id2, id1)
                else
                    throw(ErrorException("JOB SHOP CONSTRAINT VIOLATED"))
                end
            end
        end
    end
end
function add_job_shop_constraints!(milp_model::AssignmentMILP,schedule::OperatingSchedule,spec::ProblemSpec,model::JuMP.Model)
    add_job_shop_constraints!(schedule,spec,model)
end


"""
    get_objective_expr

Helper for setting the objective function for a milp model
"""
function get_objective_expr(milp::AssignmentMILP, f::SumOfMakeSpans,model,terminal_vtxs,weights,tof,Δt)
    @variable(model, T[1:length(terminal_vtxs)])
    for (i,project_head) in enumerate(terminal_vtxs)
        for v in project_head
            @constraint(model, T[i] >= tof[v] + Δt[v])
        end
    end
    cost1 = @expression(model, sum(map(i->T[i]*get(weights,i,0.0), 1:length(terminal_vtxs))))
    # @objective(model, Min, sum(map(v->tof[v]*get(weights,v,0.0), terminal_vtxs)))
    # model
end
function get_objective_expr(milp::AssignmentMILP, f::MakeSpan,model,terminal_vtxs,weights,tof,Δt)
    @variable(model, T)
    @constraint(model, T .>= tof .+ Δt)
    T
    # @objective(model, Min, T)
end

"""
    formulate_optimization_problem()

Express the TaskGraphs assignment problem as a MILP using the JuMP optimization
framework.

Inputs:
    `G` - graph with inverted tree structure that encodes dependencies
        between tasks
    `D` - D[s₁,s₂] is distance from position s₁ to position s₂
    `Δt` - Δt[j] is the duraction of time that must elapse after all prereqs
        of task j have been satisfied before task j becomes available
    `Δt_collect` - Δt_collect[j] is the time required for a robot to pick up
        object j
    `Δt_deliver` - Δt_deliver[j] is the time required for a robot to set
        down object j
    `to0_` - a `Dict`, where `to0_[j]` gives the start time for task j
        (applies to leaf tasks only)
    `tr0_` - a `Dict`, where `tr0_[i]` gives the start time for robot i
        (applies to non-dummy robots only)
    `terminal_vtxs` - a vector of integers specfying the graph vertices that
        are roots of the project
    `weights` - a vector of weights that determines the contribution of each
        root_node to the objective
    `s0` - pickup stations for the tasks
    `sF` - dropoff stations for the tasks
    `optimizer` - a JuMP optimizer (e.g., Gurobi.optimizer)

Keyword Args:
    `TimeLimit=100`
    `OutputFlag=0`
    `assignments=Dict{Int64,Int64}()` - maps robot id to assignment that must be
        enforced
    `cost_model=:MakeSpan` - optimization objective, either `:MakeSpan` or
        `:SumOfMakeSpans`

Outputs:
    `model` - the optimization model
"""
function formulate_optimization_problem(N,M,G,D,Δt,Δt_collect,Δt_deliver,to0_,tr0_,terminal_vtxs,weights,r0,s0,sF,nR,optimizer;
    TimeLimit=100,
    OutputFlag=0,
    Presolve = -1, # automatic setting (-1), off (0), conservative (1), or aggressive (2)
    task_groups = Vector{Vector{Int}}(),
    shapes = [(1,1) for j in 1:M],
    assignments=Dict{Int64,Int64}(),
    cost_model=MakeSpan(),
    t0_ = Dict(), # TODO start times for objects
    tF_ = Dict(), # TODO end times for objects
    Mm = 10000,
    # Mm = sum([D[s1,s2] for s1 in r0 for s2 in s0]) + sum([D[s1,s2] for s1 in s0 for s2 in sF])
    )

    model = Model(with_optimizer(optimizer,
        TimeLimit=TimeLimit,
        OutputFlag=OutputFlag,
        Presolve=Presolve
        ))

    r0 = vcat(r0,sF) # combine to get dummy robot ``spawn'' locations too
    @variable(model, to0[1:M] >= 0.0) # object availability time
    @variable(model, tor[1:M] >= 0.0) # object robot arrival time
    @variable(model, toc[1:M] >= 0.0) # object collection complete time
    @variable(model, tod[1:M] >= 0.0) # object deliver begin time
    @variable(model, tof[1:M] >= 0.0) # object termination time
    @variable(model, tr0[1:N+M] >= 0.0) # robot availability time

    # Assignment matrix x
    @variable(model, X[1:N+M,1:M], binary = true) # X[i,j] ∈ {0,1}
    @constraint(model, X * ones(M) .<= 1)         # each robot may have no more than 1 task
    @constraint(model, X' * ones(N+M) .== nR)     # each task must have exactly 1 assignment
    for (i,t) in tr0_
        # start time for robot i
        @constraint(model, tr0[i] == t)
    end
    for (j,t) in to0_
        # start time for task j (applies only to tasks with no prereqs)
        @constraint(model, to0[j] == t)
    end
    for (i,j) in assignments
        # start time for task j (applies only to tasks with no prereqs)
        @constraint(model, X[i,j] == 1)
    end
    # constraints
    for j in 1:M
        # constraint on task start time
        if !is_root_node(G,j)
            for v in inneighbors(G,j)
                @constraint(model, to0[j] >= tof[v] + Δt[j])
            end
        end
        # constraint on dummy robot start time (corresponds to moment of object delivery)
        @constraint(model, tr0[j+N] == tof[j])
        # dummy robots can't do upstream jobs
        upstream_jobs = [j, map(e->e.dst,collect(edges(bfs_tree(G,j;dir=:in))))...]
        for v in upstream_jobs
            @constraint(model, X[j+N,v] == 0)
        end
        # lower bound on task completion time (task can't start until it's available).
        @constraint(model, tor[j] >= to0[j])
        for i in 1:N+M
            @constraint(model, tor[j] - (tr0[i] + D[r0[i],s0[j]]) >= -Mm*(1 - X[i,j]))
        end
        @constraint(model, toc[j] == tor[j] + Δt_collect[j])
        @constraint(model, tod[j] == toc[j] + D[s0[j],sF[j]])
        @constraint(model, tof[j] == tod[j] + Δt_deliver[j])
        # "Job-shop" constraints specifying that no station may be double-booked. A station
        # can only support a single COLLECT or DEPOSIT operation at a time, meaning that all
        # the windows for these operations cannot overlap. In the constraints below, t1 and t2
        # represent the intervals for the COLLECT or DEPOSIT operations of tasks j and j2,
        # respectively. If eny of the operations for these two tasks require use of the same
        # station, we introduce a 2D binary variable y. if y = [1,0], the operation for task
        # j must occur before the operation for task j2. The opposite is true for y == [0,1].
        # We use the big M method here as well to tightly enforce the binary constraints.
        for j2 in j+1:M
            if (s0[j] == s0[j2]) || (s0[j] == sF[j2]) || (sF[j] == s0[j2]) || (sF[j] == sF[j2])
                # @show j, j2
                if s0[j] == s0[j2]
                    t1 = [tor[j], toc[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif s0[j] == sF[j2]
                    t1 = [tor[j], toc[j]]
                    t2 = [tod[j2], tof[j2]]
                elseif sF[j] == s0[j2]
                    t1 = [tod[j], tof[j]]
                    t2 = [tor[j2], toc[j2]]
                elseif sF[j] == sF[j2]
                    t1 = [tod, tof[j]]
                    t2 = [tod, tof[j2]]
                end
                tmax = @variable(model)
                tmin = @variable(model)
                y = @variable(model, binary=true)
                @constraint(model, tmax >= t1[1])
                @constraint(model, tmax >= t2[1])
                @constraint(model, tmin <= t1[2])
                @constraint(model, tmin <= t2[2])

                @constraint(model, tmax - t2[1] <= (1 - y)*Mm)
                @constraint(model, tmax - t1[1] <= y*Mm)
                @constraint(model, tmin - t1[2] >= (1 - y)*-Mm)
                @constraint(model, tmin - t2[2] >= y*-Mm)
                # @constraint(model, tmin + 1 <= tmax)
                @constraint(model, tmin + 1 - X[j+N,j2] - X[j2+N,j] <= tmax) # NOTE +1 not necessary if the same robot is doing both
            end
        end
    end
    # cost depends only on root node(s)
    # if cost_model <: SumOfMakeSpans
    #     @variable(model, T[1:length(terminal_vtxs)])
    #     for (i,project_head) in enumerate(terminal_vtxs)
    #         for v in project_head
    #             @constraint(model, T[i] >= tof[v] + Δt[v])
    #         end
    #     end
    #     @objective(model, Min, sum(map(i->T[i]*get(weights,i,0.0), 1:length(terminal_vtxs))))
    #     # @objective(model, Min, sum(map(v->tof[v]*get(weights,v,0.0), terminal_vtxs)))
    # elseif cost_model <: MakeSpan
    #     @variable(model, T)
    #     @constraint(model, T .>= tof .+ Δt)
    #     @objective(model, Min, T)
    # end
    milp = AssignmentMILP(model)
    cost1 = get_objective_expr(milp, cost_model, milp.model,terminal_vtxs,weights,tof,Δt)
    @objective(milp.model, Min, cost1)
    milp
end
function formulate_optimization_problem(spec::T,optimizer;
    kwargs...
    ) where {T<:ProblemSpec}
    formulate_optimization_problem(
        spec.N,
        spec.M,
        spec.graph,
        spec.D,
        spec.Δt,
        spec.Δt_collect,
        spec.Δt_deliver,
        spec.to0_,
        spec.tr0_,
        spec.terminal_vtxs,
        spec.weights,# # # #
        spec.r0,
        spec.s0,
        spec.sF,
        spec.nR,
        optimizer;
        # cost_model=spec.cost_function,
        kwargs...
        )
end

"""
    formulate_milp(milp_model::AssignmentMILP,project_schedule,problem_spec;kwargs...)

Express the TaskGraphs assignment problem as an `AssignmentMILP` using the JuMP
optimization framework.

Inputs:
    milp_model::T <: TaskGraphsMILP : a milp model that determines how the
        sequential task assignment problem is modeled. Current options are
        `AssignmentMILP`, `SparseAdjacencyMILP` and `GreedyAssignment`.
    project_schedule::OperatingSchedule : a partial operating schedule, where
        some or all assignment edges may be missing.
    problem_spec::ProblemSpec : encodes the distance matrix and other
        information about the problem.

Keyword Args:
    `optimizer` - a JuMP optimizer (e.g., Gurobi.optimizer)
    `cost_model=MakeSpan` - optimization objective, currently either `MakeSpan`
        or `SumOfMakeSpans`. Defaults to the cost_model associated with
        `problem_spec`
    `TimeLimit=100` : a keyword argument for the optimizer (see JuMP and Gurobi
        documentation)
    `OutputFlag=0` : a keyword argument for the optimizer (see JuMP and Gurobi
        documentation)

Outputs:
    `model` - an optimization model of type `T`
"""
function formulate_milp(milp_model::AssignmentMILP,project_schedule::OperatingSchedule,problem_spec::ProblemSpec;
    optimizer=Gurobi.Optimizer,
    cost_model=problem_spec.cost_function,
    kwargs...)
    formulate_optimization_problem(problem_spec,optimizer;cost_model=cost_model,kwargs...)
end

export
    preprocess_project_schedule

"""
    preprocess_project_schedule(project_schedule)

Returns information about the eligible and required successors and predecessors
of nodes in `project_schedule`

Arguments:
- `project_schedule::OperatingSchedule`

Outputs:
- missing_successors
- missing_predecessors
- n_eligible_successors
- n_eligible_predecessors
- n_required_successors
- n_required_predecessors
- upstream_vertices
- non_upstream_vertices
"""
function preprocess_project_schedule(project_schedule)
    G = get_graph(project_schedule);
    # Identify required and eligible edges
    missing_successors      = Dict{Int,Dict}()
    missing_predecessors    = Dict{Int,Dict}()
    n_eligible_successors   = zeros(Int,nv(G))
    n_eligible_predecessors = zeros(Int,nv(G))
    n_required_successors   = zeros(Int,nv(G))
    n_required_predecessors = zeros(Int,nv(G))
    for v in vertices(G)
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        for (key,val) in required_successors(node)
            n_required_successors[v] += val
        end
        for (key,val) in required_predecessors(node)
            n_required_predecessors[v] += val
        end
        for (key,val) in eligible_successors(node)
            n_eligible_successors[v] += val
        end
        for (key,val) in eligible_predecessors(node)
            n_eligible_predecessors[v] += val
        end
        missing_successors[v] = eligible_successors(node)
        for v2 in outneighbors(G,v)
            id2 = get_vtx_id(project_schedule, v2)
            node2 = get_node_from_id(project_schedule, id2)
            for key in collect(keys(missing_successors[v]))
                if matches_template(key,typeof(node2))
                    missing_successors[v][key] -= 1
                    break
                end
            end
        end
        missing_predecessors[v] = eligible_predecessors(node)
        for v2 in inneighbors(G,v)
            id2 = get_vtx_id(project_schedule, v2)
            node2 = get_node_from_id(project_schedule, id2)
            for key in collect(keys(missing_predecessors[v]))
                if matches_template(key,typeof(node2))
                    missing_predecessors[v][key] -= 1
                    break
                end
            end
        end
    end
    @assert(!any(n_eligible_predecessors .< n_required_predecessors))
    @assert(!any(n_eligible_successors .< n_required_successors))

    upstream_vertices = map(v->[v, map(e->e.dst,collect(edges(bfs_tree(G,v;dir=:in))))...], vertices(G))
    non_upstream_vertices = map(v->collect(setdiff(collect(vertices(G)),upstream_vertices[v])), vertices(G))

    return missing_successors, missing_predecessors, n_eligible_successors, n_eligible_predecessors, n_required_successors, n_required_predecessors, upstream_vertices, non_upstream_vertices
end

function formulate_schedule_milp(project_schedule::OperatingSchedule,problem_spec::ProblemSpec;
        optimizer = Gurobi.Optimizer,
        TimeLimit=100,
        OutputFlag=0,
        Presolve=-1, # automatic setting (-1), off (0), conservative (1), or aggressive (2)
        t0_ = Dict{AbstractID,Float64}(), # dictionary of initial times. Default is empty
        tF_ = Dict{AbstractID,Float64}(), # dictionary of initial times. Default is empty
        Mm = 10000, # for big M constraints
        cost_model = SumOfMakeSpans(),
    )
    G = get_graph(project_schedule);
    assignments = [];
    Δt = map(v->get_path_spec(project_schedule, v).min_path_duration, vertices(G))

    model = Model(with_optimizer(optimizer,
        TimeLimit=TimeLimit,
        OutputFlag=OutputFlag,
        Presolve=Presolve
        ));
    @variable(model, t0[1:nv(G)] >= 0.0); # initial times for all nodes
    @variable(model, tF[1:nv(G)] >= 0.0); # final times for all nodes

    # Precedence relationships
    @variable(model, Xa[1:nv(G),1:nv(G)], binary = true); # Precedence Adjacency Matrix TODO make sparse
    @constraint(model, Xa .+ Xa' .<= 1) # no bidirectional or self edges
    # set all initial times that are provided
    for (id,t) in t0_
        v = get_vtx(project_schedule, id)
        @constraint(model, t0[v] >= t)
    end
    for (id,t) in tF_
        v = get_vtx(project_schedule, id)
        @constraint(model, tF[v] >= t)
    end
    # Precedence constraints and duration constraints for existing nodes and edges
    for v in vertices(G)
        @constraint(model, tF[v] >= t0[v] + Δt[v])
        for v2 in outneighbors(G,v)
            @constraint(model, Xa[v,v2] == 1)
            @constraint(model, t0[v2] >= tF[v]) # NOTE DO NOT CHANGE TO EQUALITY CONSTRAINT
        end
    end
    # Identify required and eligible edges
    missing_successors      = Dict{Int,Dict}()
    missing_predecessors    = Dict{Int,Dict}()
    n_eligible_successors   = zeros(Int,nv(G))
    n_eligible_predecessors = zeros(Int,nv(G))
    n_required_successors   = zeros(Int,nv(G))
    n_required_predecessors = zeros(Int,nv(G))
    for v in vertices(G)
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        for (key,val) in required_successors(node)
            n_required_successors[v] += val
        end
        for (key,val) in required_predecessors(node)
            n_required_predecessors[v] += val
        end
        for (key,val) in eligible_successors(node)
            n_eligible_successors[v] += val
        end
        for (key,val) in eligible_predecessors(node)
            n_eligible_predecessors[v] += val
        end
        missing_successors[v] = eligible_successors(node)
        for v2 in outneighbors(G,v)
            id2 = get_vtx_id(project_schedule, v2)
            node2 = get_node_from_id(project_schedule, id2)
            for key in collect(keys(missing_successors[v]))
                if matches_template(key,typeof(node2))
                    missing_successors[v][key] -= 1
                    break
                end
            end
        end
        missing_predecessors[v] = eligible_predecessors(node)
        for v2 in inneighbors(G,v)
            id2 = get_vtx_id(project_schedule, v2)
            node2 = get_node_from_id(project_schedule, id2)
            for key in collect(keys(missing_predecessors[v]))
                if matches_template(key,typeof(node2))
                    missing_predecessors[v][key] -= 1
                    break
                end
            end
        end
    end
    @assert(!any(n_eligible_predecessors .< n_required_predecessors))
    @assert(!any(n_eligible_successors .< n_required_successors))
    @constraint(model, Xa * ones(nv(G)) .<= n_eligible_successors);
    @constraint(model, Xa * ones(nv(G)) .>= n_required_successors);
    @constraint(model, Xa' * ones(nv(G)) .<= n_eligible_predecessors);
    @constraint(model, Xa' * ones(nv(G)) .>= n_required_predecessors);

    upstream_vertices = map(v->[v, map(e->e.dst,collect(edges(bfs_tree(G,v;dir=:in))))...], vertices(G))
    for v in vertices(G)
        for v2 in upstream_vertices[v]
            @constraint(model, Xa[v,v2] == 0) #TODO this variable is not needed
        end
    end
    non_upstream_vertices = map(v->collect(setdiff(collect(vertices(G)),upstream_vertices[v])), vertices(G))
    # Big M constraints
    for v in vertices(G)
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        for v2 in non_upstream_vertices[v] # for v2 in vertices(G)
            node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
            potential_match = false
            for (template, val) in missing_successors[v]
                if !matches_template(template, typeof(node2)) # possible to add an edge
                    continue
                end
                for (template2, val2) in missing_predecessors[v2]
                    if !matches_template(template2, typeof(node)) # possible to add an edge
                        continue
                    end
                    potential_match = true # TODO: investigate why the solver fails when this is moved inside the following if statement
                    if (val > 0 && val2 > 0)
                        new_node = align_with_successor(node,node2)
                        dt_min = generate_path_spec(project_schedule,problem_spec,new_node).min_path_duration
                        @constraint(model, tF[v] - (t0[v] + dt_min) >= -Mm*(1 - Xa[v,v2]))
                        @constraint(model, t0[v2] - tF[v] >= -Mm*(1 - Xa[v,v2]))
                        break
                    end
                end
            end
            if potential_match == false
                @constraint(model, Xa[v,v2] == 0) #TODO this variable is not needed
            end
        end
    end

    # "Job-shop" constraints specifying that no station may be double-booked. A station
    # can only support a single COLLECT or DEPOSIT operation at a time, meaning that all
    # the windows for these operations cannot overlap. In the constraints below, t1 and t2
    # represent the intervals for the COLLECT or DEPOSIT operations of tasks j and j2,
    # respectively. If eny of the operations for these two tasks require use of the same
    # station, we introduce a 2D binary variable y. if y = [1,0], the operation for task
    # j must occur before the operation for task j2. The opposite is true for y == [0,1].
    # We use the big M method here as well to tightly enforce the binary constraints.
    # job_shop_variables = Dict{Tuple{Int,Int},JuMP.VariableRef}();
    @variable(model, Xj[1:nv(G),1:nv(G)], binary = true); # job shop adjacency matrix
    @constraint(model, Xj .+ Xj' .<= 1) # no bidirectional or self edges
    for v in 1:nv(G)
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        for v2 in v+1:nv(G)
            node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
            common_resources = intersect(resources_reserved(node),resources_reserved(node2))
            if length(common_resources) > 0
                # @show common_resources
                # tmax = @variable(model)
                # tmin = @variable(model)
                # y = @variable(model, binary=true)
                # job_shop_variables[(v,v2)] = y
                # @constraint(model, tmax >= t0[v])
                # @constraint(model, tmax >= t0[v2])
                # @constraint(model, tmin <= tF[v])
                # @constraint(model, tmin <= tF[v2])
                #
                # @constraint(model, tmax - t0[v2] <= (1 - y)*Mm)
                # @constraint(model, tmax - t0[v] <= y*Mm)
                # @constraint(model, tmin - tF[v] >= (1 - y)*-Mm)
                # @constraint(model, tmin - tF[v2] >= y*-Mm)
                # @constraint(model, tmin + 1 <= tmax)

                # Big M constraints
                @constraint(model, Xj[v,v2] + Xj[v2,v] == 1)
                if !(v2 in upstream_vertices[v])
                    @constraint(model, t0[v2] - tF[v] >= -Mm*(1 - Xj[v,v2]))
                end
                if !(v in upstream_vertices[v2])
                    @constraint(model, t0[v] - tF[v2] >= -Mm*(1 - Xj[v2,v]))
                end
            else
                @constraint(model, Xj[v,v2] == 0)
                @constraint(model, Xj[v2,v] == 0)
            end
        end
    end

    # Full adjacency matrix
    @variable(model, X[1:nv(G),1:nv(G)]); # Adjacency Matrix
    @constraint(model, X .== Xa .+ Xj)

    # Formulate Objective
    # if cost_model <: SumOfMakeSpans
    #     terminal_vtxs = project_schedule.terminal_vtxs
    #     @variable(model, T[1:length(terminal_vtxs)])
    #     for (i,project_head) in enumerate(terminal_vtxs)
    #         for v in project_head
    #             @constraint(model, T[i] >= tF[v])
    #         end
    #     end
    #     cost1 = @expression(model, sum(map(v->tF[v]*get(project_schedule.weights,v,0.0), terminal_vtxs)))
    # elseif cost_model <: MakeSpan
    #     @variable(model, T)
    #     @constraint(model, T .>= tF)
    #     cost1 = @expression(model, T)
    # end
    # sparsity_cost = @expression(model, (0.5/(nv(G)^2))*sum(Xa)) # cost term to encourage sparse X. Otherwise the solver may add pointless edges
    sparsity_cost = @expression(model, (0.5/(nv(G)^2))*sum(X)) # cost term to encourage sparse X. Otherwise the solver may add pointless edges
    # @objective(model, Min, cost1 )
    milp = AdjacencyMILP(model=model) #, job_shop_variables
    cost1 = get_objective_expr(milp,cost_model,milp.model,project_schedule,tF)
    @objective(milp.model, Min, cost1 + sparsity_cost)
    milp
end
function formulate_milp(milp_model::AdjacencyMILP,project_schedule::OperatingSchedule,problem_spec::ProblemSpec;
    optimizer=Gurobi.Optimizer,
    kwargs...)
    formulate_schedule_milp(project_schedule,problem_spec;optimizer=optimizer,kwargs...)
end
function formulate_milp(milp_model::SparseAdjacencyMILP,project_schedule::OperatingSchedule,problem_spec::ProblemSpec;
        optimizer = Gurobi.Optimizer,
        TimeLimit=100,
        OutputFlag=0,
        Presolve=-1, # automatic setting (-1), off (0), conservative (1), or aggressive (2)
        t0_ = Dict{AbstractID,Float64}(), # dictionary of initial times. Default is empty
        tF_ = Dict{AbstractID,Float64}(), # dictionary of initial times. Default is empty
        Mm = 10000, # for big M constraints
        cost_model = SumOfMakeSpans(),
        job_shop=milp_model.job_shop,
        kwargs...
    )

    # println("NBS TIME LIMIT: TimeLimit = $TimeLimit")
    model = Model(with_optimizer(optimizer,
        TimeLimit=TimeLimit,
        OutputFlag=OutputFlag,
        Presolve=Presolve
        ));

    G = get_graph(project_schedule);
    (missing_successors, missing_predecessors, n_eligible_successors,
        n_eligible_predecessors, n_required_successors, n_required_predecessors,
        upstream_vertices, non_upstream_vertices
        ) = preprocess_project_schedule(project_schedule)
    Δt = map(v->get_path_spec(project_schedule, v).min_path_duration, vertices(G))

    @variable(model, t0[1:nv(G)] >= 0.0); # initial times for all nodes
    @variable(model, tF[1:nv(G)] >= 0.0); # final times for all nodes

    # Precedence relationships
    Xa = SparseMatrixCSC{VariableRef,Int}(nv(G),nv(G),ones(Int,nv(G)+1),Int[],VariableRef[])
    # set all initial times that are provided
    for (id,t) in t0_
        v = get_vtx(project_schedule, id)
        @constraint(model, t0[v] >= t)
    end
    for (id,t) in tF_
        v = get_vtx(project_schedule, id)
        @constraint(model, tF[v] >= t)
    end
    # Precedence constraints and duration constraints for existing nodes and edges
    for v in vertices(G)
        @constraint(model, tF[v] >= t0[v] + Δt[v]) # NOTE Δt may change for some nodes
        for v2 in outneighbors(G,v)
            Xa[v,v2] = @variable(model, binary=true) # TODO remove this (MUST UPDATE n_eligible_successors, etc. accordingly)
            @constraint(model, Xa[v,v2] == 1) #TODO this edge already exists--no reason to encode it as a decision variable
            @constraint(model, t0[v2] >= tF[v]) # NOTE DO NOT CHANGE TO EQUALITY CONSTRAINT. Making this an equality constraint causes the solver to return a higher final value in some cases (e.g., toy problems 2,3,7). Why? Maybe the Big-M constraint forces it to bump up. I though the equality constraint might speed up the solver.
        end
    end

    # Big M constraints
    for v in vertices(G)
        node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
        potential_match = false
        if outdegree(G,v) < n_eligible_successors[v] # NOTE: Trying this out to save time on formulation
            for v2 in non_upstream_vertices[v] # for v2 in vertices(G)
                if indegree(G,v2) < n_eligible_predecessors[v2]
                    node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
                    for (template, val) in missing_successors[v]
                        if !matches_template(template, typeof(node2)) # possible to add an edge
                            continue
                        end
                        for (template2, val2) in missing_predecessors[v2]
                            if !matches_template(template2, typeof(node)) # possible to add an edge
                                continue
                            end
                            if (val > 0 && val2 > 0)
                                potential_match = true
                                new_node = align_with_successor(node,node2)
                                dt_min = generate_path_spec(project_schedule,problem_spec,new_node).min_path_duration
                                Xa[v,v2] = @variable(model, binary=true) # initialize a new binary variable in the sparse adjacency matrix
                                @constraint(model, tF[v] - (t0[v] + dt_min) >= -Mm*(1 - Xa[v,v2]))
                                @constraint(model, t0[v2] - tF[v] >= -Mm*(1 - Xa[v,v2]))
                                break
                            end
                        end
                    end
                end
            end
        end
        if potential_match == false && job_shop == false
            @constraint(model, tF[v] == t0[v] + Δt[v]) # adding this constraint may provide some speedup
        end
    end

    # In the sparse implementation, these constraints must come after all possible edges are defined by a VariableRef
    @constraint(model, Xa * ones(nv(G)) .<= n_eligible_successors);
    @constraint(model, Xa * ones(nv(G)) .>= n_required_successors);
    @constraint(model, Xa' * ones(nv(G)) .<= n_eligible_predecessors);
    @constraint(model, Xa' * ones(nv(G)) .>= n_required_predecessors);
    for i in 1:nv(G)
        for j in i:nv(G)
            # prevent self-edges and cycles
            @constraint(model, Xa[i,j] + Xa[j,i] <= 1)
        end
    end

    # "Job-shop" constraints specifying that no station may be double-booked. A station
    # can only support a single COLLECT or DEPOSIT operation at a time, meaning that all
    # the windows for these operations cannot overlap. In the constraints below, t1 and t2
    # represent the intervals for the COLLECT or DEPOSIT operations of tasks j and j2,
    # respectively. If eny of the operations for these two tasks require use of the same
    # station, we introduce a 2D binary variable y. if y = [1,0], the operation for task
    # j must occur before the operation for task j2. The opposite is true for y == [0,1].
    # We use the big M method here as well to tightly enforce the binary constraints.
    # job_shop_variables = Dict{Tuple{Int,Int},JuMP.VariableRef}();
    Xj = SparseMatrixCSC{VariableRef,Int}(nv(G),nv(G),ones(Int,nv(G)+1),Int[],VariableRef[])
    if job_shop
        for v in 1:nv(G)
            node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
            for v2 in non_upstream_vertices[v] #v+1:nv(G)
                if v2 > v && ~(v in upstream_vertices[v2]) && ~(has_edge(G,v,v2) || has_edge(G,v2,v))
                    node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
                    common_resources = intersect(resources_reserved(node),resources_reserved(node2))
                    if length(common_resources) > 0
                        println("MILP FORMULATION: adding a job shop constraint between ",v, " (",string(node),") and ", v2, " (",string(node2),")")
                        # @show common_resources
                        # Big M constraints
                        Xj[v,v2] = @variable(model, binary=true) #
                        Xj[v2,v] = @variable(model, binary=true) # need both directions to have a valid adjacency matrix
                        @constraint(model, Xj[v,v2] + Xj[v2,v] == 1)
                        @constraint(model, t0[v2] - tF[v] >= -Mm*(1 - Xj[v,v2]))
                        @constraint(model, t0[v] - tF[v2] >= -Mm*(1 - Xj[v2,v]))
                    end
                end
            end
        end
    end

    # Full adjacency matrix
    @expression(model, X, Xa .+ Xj) # Make X an expression rather than a variable

    # Formulate Objective
    # if cost_model <: SumOfMakeSpans
    #     terminal_vtxs = project_schedule.terminal_vtxs
    #     @variable(model, T[1:length(terminal_vtxs)])
    #     for (i,project_head) in enumerate(terminal_vtxs)
    #         for v in project_head
    #             @constraint(model, T[i] >= tF[v])
    #         end
    #     end
    #     cost1 = @expression(model, sum(map(v->tF[v]*get(project_schedule.weights,v,0.0), terminal_vtxs)))
    # elseif cost_model <: MakeSpan
    #     @variable(model, T)
    #     @constraint(model, T .>= tF)
    #     cost1 = @expression(model, T)
    # end
    # # sparsity_cost = @expression(model, (0.5/(nv(G)^2))*sum(Xa)) # cost term to encourage sparse X. Otherwise the solver may add pointless edges
    # # sparsity_cost = @expression(model, (0.5/(nv(G)^2))*sum(X)) # cost term to encourage sparse X. Otherwise the solver may add pointless edges
    # # @objective(model, Min, cost1 + sparsity_cost)
    # @objective(model, Min, cost1)
    milp = SparseAdjacencyMILP(model,Xa,Xj, milp_model.job_shop) #, job_shop_variables
    cost1 = get_objective_expr(milp,cost_model,milp.model,project_schedule,tF)
    @objective(milp.model, Min, cost1)
    milp
end
function get_objective_expr(milp,f::SumOfMakeSpans,model,project_schedule,tF)
    terminal_vtxs = project_schedule.terminal_vtxs
    @variable(model, T[1:length(terminal_vtxs)])
    for (i,project_head) in enumerate(terminal_vtxs)
        for v in project_head
            @constraint(model, T[i] >= tF[v])
        end
    end
    cost1 = @expression(model, sum(map(v->tF[v]*get(project_schedule.weights,v,0.0), terminal_vtxs)))
end
function get_objective_expr(milp,f::MakeSpan,model,project_schedule,tF)
    @variable(model, T)
    @constraint(model, T .>= tF) # TODO Maybe the number of constraints here causes a slowdown that could be addressed by only adding constraints on terminal nodes?
    cost1 = @expression(model, T)
end

export
    GreedyAssignment

"""
    GreedyAssignment - baseline for task assignment. It works by maintaining two
    "open" sets of vertices: one containing vertices that are eligible for a
"""
@with_kw struct GreedyAssignment{C} <: TaskGraphsMILP
    schedule::OperatingSchedule   = OperatingSchedule()
    problem_spec::ProblemSpec   = ProblemSpec()
    cost_model::C               = SumOfMakeSpans()
    # X::M                        = sparse(zeros(Int,nv(project_schedule),nv(project_schedule)))
    t0::Vector{Int}             = zeros(Int,nv(schedule))
    # cost::Float64               = Inf
    # lower_bound::Float64        = Inf
end
exclude_solutions!(model::GreedyAssignment) = nothing # exclude most recent solution in order to get next best solution
JuMP.termination_status(model::GreedyAssignment)    = MOI.OPTIMAL
JuMP.primal_status(model::GreedyAssignment)         = MOI.FEASIBLE_POINT
get_assignment_matrix(model::GreedyAssignment)      = adjacency_matrix(get_graph(model.schedule))
function JuMP.objective_function(model::GreedyAssignment{SumOfMakeSpans})
    t0,tF,slack,local_slack = process_schedule(model.schedule,model.t0)
    return sum(tF[model.schedule.terminal_vtxs] .* map(v->model.schedule.weights[v], model.schedule.terminal_vtxs))
end
function JuMP.objective_function(model::GreedyAssignment{MakeSpan})
    t0,tF,slack,local_slack = process_schedule(model.schedule,model.t0)
    return maximum(tF[model.schedule.terminal_vtxs])
end
function JuMP.objective_function(model::GreedyAssignment)
    println("UNKNOWN COST FUNCTION!")
    return Inf
end
JuMP.objective_bound(model::GreedyAssignment)       = objective_function(model)
JuMP.value(c::Real) = c
function formulate_milp(milp_model::GreedyAssignment,project_schedule::OperatingSchedule,problem_spec::ProblemSpec;
        t0_ = Dict{AbstractID,Float64}(),
        cost_model = SumOfMakeSpans(),
        kwargs...
    )

    model = GreedyAssignment(
        schedule = project_schedule,
        problem_spec = problem_spec,
        cost_model = cost_model
    )
    for (id,t) in t0_
        v = get_vtx(project_schedule, id)
        model.t0[v] = t
    end
    model
end

function construct_schedule_distance_matrix(project_schedule,problem_spec)
    G = get_graph(project_schedule);
    (missing_successors, missing_predecessors, n_eligible_successors, n_eligible_predecessors,
        n_required_successors, n_required_predecessors, upstream_vertices, non_upstream_vertices) = preprocess_project_schedule(project_schedule)
    D = Inf * ones(nv(G),nv(G))
    for v in vertices(G)
        if outdegree(G,v) < n_eligible_successors[v]
            node = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v))
            # for v2 in vertices(G)
            for v2 in non_upstream_vertices[v]
                if indegree(G,v2) < n_eligible_predecessors[v2]
                    node2 = get_node_from_id(project_schedule, get_vtx_id(project_schedule, v2))
                    for (template, val) in missing_successors[v]
                        if !matches_template(template, typeof(node2)) # possible to add an edge
                            continue
                        end
                        for (template2, val2) in missing_predecessors[v2]
                            if !matches_template(template2, typeof(node)) # possible to add an edge
                                continue
                            end
                            if (val > 0 && val2 > 0)
                                new_node = align_with_successor(node,node2)
                                D[v,v2] = generate_path_spec(project_schedule,problem_spec,new_node).min_path_duration
                            end
                        end
                    end
                end
            end
        end
    end
    D
end

function update_greedy_sets!(model,G,C,Ai,Ao,v,n_required_predecessors,n_eligible_successors)
    if indegree(G,v) >= n_required_predecessors[v]
        push!(C,v)
        for v2 in inneighbors(G,v)
            if !(v2 in C)
                setdiff!(C,v)
            end
        end
    else
        push!(Ai,v)
        for v2 in inneighbors(G,v)
            if !(v2 in C)
                setdiff!(Ai,v)
            end
        end
    end
    if (outdegree(G,v) < n_eligible_successors[v]) && (v in C)
        push!(Ao,v)
    end
end

function select_next_edge(model,D,Ao,Ai)
    c = Inf
    a = -1
    b = -2
    for v in sort(collect(Ao))
        for v2 in sort(collect(Ai))
            if D[v,v2] < c
                c = D[v,v2]
                a = v
                b = v2
            end
        end
    end
    a,b
    if a < 0 || b < 0
        println("debugging edge selection for model ",typeof(model))
        for v in Ai
            node = get_node_from_vtx(model.schedule,v)
            println(string("node ",string(node), " needs assignment"))
            @show required_predecessors(node)
            @show indegree(model.schedule,v)
        end
    end
    a,b
end

function JuMP.optimize!(model::GreedyAssignment)

    project_schedule    = model.schedule
    problem_spec        = model.problem_spec
    G                   = get_graph(project_schedule);
    (_, _, n_eligible_successors, _, _, n_required_predecessors, _, _) = preprocess_project_schedule(project_schedule)
    D = construct_schedule_distance_matrix(project_schedule,problem_spec)

    C = Set{Int}()
    Ai = Set{Int}()
    Ao = Set{Int}()
    for v in topological_sort(G)
        update_greedy_sets!(model,G,C,Ai,Ao,v,n_required_predecessors,n_eligible_successors)
    end
    # while length(C) < nv(G)
    while length(Ai) > 0
        v,v2 = select_next_edge(model,D,Ao,Ai)
        setdiff!(Ao,v)
        setdiff!(Ai,v2)
        add_edge!(G,v,v2)
        for v in topological_sort(G)
            update_greedy_sets!(model,G,C,Ai,Ao,v,n_required_predecessors,n_eligible_successors)
        end
    end
    set_leaf_operation_vtxs!(project_schedule)
    propagate_valid_ids!(project_schedule,problem_spec)
    model
end

export
    propagate_valid_ids!

function propagate_valid_ids!(project_schedule::OperatingSchedule,problem_spec::ProblemSpec)
    G = get_graph(project_schedule)
    @assert(is_cyclic(G) == false, "is_cyclic(G)") # string(sparse(adj_matrix))
    # Propagate valid IDs through the schedule
    for v in topological_sort(G)
        # if !get_path_spec(project_schedule, v).fixed
            node_id = get_vtx_id(project_schedule, v)
            node = get_node_from_id(project_schedule, node_id)
            for v2 in inneighbors(G,v)
                node = align_with_predecessor(node,get_node_from_vtx(project_schedule, v2))
            end
            for v2 in outneighbors(G,v)
                node = align_with_successor(node,get_node_from_vtx(project_schedule, v2))
            end
            path_spec = get_path_spec(project_schedule, v)
            if path_spec.fixed
                replace_in_schedule!(project_schedule, path_spec, node, node_id)
            else
                replace_in_schedule!(project_schedule, problem_spec, node, node_id)
            end
        # end
    end
    # project_schedule
    return true
end

"""
    update_project_schedule!

Args:
- solver
- project_schedule
- adj_matrix - adjacency_matrix encoding the edges that need to be added to
    the project schedule

Adds all required edges to the project graph and modifies all nodes to
reflect the appropriate valid IDs (e.g., `Action` nodes are populated with
the correct `RobotID`s)
Returns `false` if the new edges cause cycles in the project graph.
"""
function update_project_schedule!(solver,sched::OperatingSchedule,problem_spec,adj_matrix)
    mtx = adjacency_matrix(sched)
    val = update_project_schedule!(sched,problem_spec,adj_matrix)
    @log_info(1,solver,"Assignment: Adding edges \n",
        map(idx->string("\t",
                string(get_node_from_vtx(sched,idx.I[1]))," → ",
                string(get_node_from_vtx(sched,idx.I[2])),"\n"
                ),
            findall(adj_matrix .- mtx .!= 0))...
    )
    val
end
function update_project_schedule!(sched::OperatingSchedule,problem_spec,adj_matrix)
    # Add all new edges to project sched
    G = get_graph(sched)
    # remove existing edges first, so that there is no carryover between consecutive MILP iterations
    for e in collect(edges(G))
        rem_edge!(G, e)
    end
    # add all edges encoded by adjacency matrix
    for v in vertices(G)
        for v2 in vertices(G)
            if adj_matrix[v,v2] >= 1
                add_edge!(G,v,v2)
            end
        end
    end
    try
        propagate_valid_ids!(sched,problem_spec)
        @assert validate(sched)
    catch e
        if isa(e, AssertionError)
            showerror(stdout,e,catch_backtrace())
        else
            rethrow(e)
        end
        return false
    end
    return true
end

"""
    update_project_schedule!(solver,milp_model::M,project_schedule,problem_spec,
        adj_matrix) where {M<:TaskGraphsMILP}

Args:
- milp_model <: TaskGraphsMILP
- project_schedule::OperatingSchedule
- problem_spec::ProblemSpec
- adj_matrix : an adjacency_matrix or (in the case where
    `milp_model::AssignmentMILP`), an assignment matrix

Adds all required edges to the schedule graph and modifies all nodes to
reflect the appropriate valid IDs (e.g., `Action` nodes are populated with
the correct `RobotID`s)
Returns `false` if the new edges cause cycles in the project graph.
"""
function update_project_schedule!(solver,model::TaskGraphsMILP,sched,prob_spec,
        adj_matrix=get_assignment_matrix(model),
    )
    update_project_schedule!(solver,sched,prob_spec,adj_matrix)
end
function update_project_schedule!(solver,model::AssignmentMILP,sched,prob_spec,
        assignment_matrix=get_assignment_matrix(model),
    )
    adj_matrix = adj_mat_from_assignment_mat(sched,assignment_matrix)
    update_project_schedule!(solver,sched,prob_spec,adj_matrix)
end
