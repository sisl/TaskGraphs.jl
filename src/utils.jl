export
    initialize_random_2D_task_graph_env,
    construct_factory_distance_matrix,
    construct_random_project_spec,
    compute_lower_time_bound


"""
    initialize_random_2D_task_graph_env(G,N;d=[20,20])

    d = [20,20] - dimensions of factor floor
    r₀ - indices of initial robot positions
    s₀ - indices of initial object locations
    sₜ - indices of destination object locations
"""
function initialize_random_2D_task_graph_env(N,M;d=[20,20])
    x₀ = Set{Vector{Int}}() # all possible grid locations
    for i in 1:d[1]
        for j in 1:d[2]
            push!(x₀,Vector{Int}([i,j]))
        end
    end
    ##### Random Problem Initialization #####
    # initial robot locations
    r₀ = Vector{Vector{Int}}(undef, N)
    for i in 1:N
        r₀[i] = pop!(x₀)
    end
    # initial object locations - somewhere in factory
    s₀ = Vector{Vector{Int}}(undef, M)
    for i in 1:M
        s₀[i] = pop!(x₀)
    end
    # final object locations - depends on where the child objects "appear"
    sₜ = Vector{Vector{Int}}(undef, M)
    for i in 1:M
        # if length(outneighbors(G,v)) > 0
        #     v2 = outneighbors(G,v)[1]
        #     sₜ[v] = s₀[v2]
        # else
        sₜ[i] = pop!(x₀)
        # end
    end
    r₀,s₀,sₜ
end

"""
    `construct_factory_distance_matrix(r₀,oₒ,sₒ;dist::Function=(x1,x2)->norm(x2-x1,1))`

    r₀ - vector of initial robot locations.
    sₒ - vector of initial object locations.
    sₜ - vector of station locations (object i must be brough to station i from its initial location)
"""
function construct_factory_distance_matrix(r₀,s₀,sₜ;dist::Function=(x1,x2)->norm(x2-x1,1))
    N = size(r₀,1)
    M = size(s₀,1)
    # Construct distance matrix
    D = zeros(N+M,N+M)
    for i in 1:N
        for j in 1:N
            # distance from robot to robot (not relevant)
            D[i,j] = dist(r₀[i],r₀[j])
        end
        for j in 1:M
            # distance from robot to object + object to station
            D[i,j+N] = dist(r₀[i],s₀[j]) + dist(s₀[j],sₜ[j])
        end
    end
    for i in 1:M
        for j in 1:M
            # distance from robot to object + object to station
            D[i+N,j+N] = dist(sₜ[i],s₀[j]) + dist(s₀[j],sₜ[j])
        end
    end
    # ensure symmetric matrix
    for i in 1:size(D,1)
        for j in 1:size(D,2)
            D[i,j] = max(D[i,j],D[j,i])
            D[j,i] = D[i,j]
        end
    end
    D
end

"""
    construct_random_project_spec(M::Int;max_children=1)

    max_parents - determines the max number of inputs that an operation may have
    depth_bias ∈ [0,1] - hyperparameter for tuning depth.
        If `depth_bias` == 1.0, the project_spec graph will always be depth
        balanced (all paths through the tree will be of the same length). For
        depth_bias == 0.0, the graph will be as "strung out" as possible.
"""
function construct_random_project_spec(M::Int;max_parents=1,depth_bias=1.0,Δt_min::Int=0,Δt_max::Int=0)
    project_spec = ProjectSpec()
    # fill with random operations going backwards
    i = M-1
    frontier = PriorityQueue{Int,Int}([M=>1])
    while i > 0
        depth = 1
        while true
            if (rand() > depth_bias) && (depth < length(frontier))
                depth += 1
            else
                break
            end
        end
        pairs = Vector{Pair{Int,Int}}()
        pair = Pair{Int,Int}(0,0)
        for d in 1:depth
            push!(pairs, peek(frontier))
            dequeue!(frontier)
        end
        for p in pairs[1:end-1]
            enqueue!(frontier,p)
        end
        output_id = pairs[end].first
        station_id = output_id
        input_ids = collect(max(1,1+i-rand(1:max_parents)):i)
        i = i - length(input_ids)
        # Δt = Δt_min + (Δt_max-Δt_min)*rand()
        Δt=rand(Δt_min:Δt_max)
        add_operation!(project_spec,construct_operation(station_id, input_ids, [output_id], Δt))
        for id in input_ids
            enqueue!(frontier, id=>M-i)
        end
    end
    project_spec
end

"""
    compute_lower_time_bound(G,D,r₀,s₀,sₜ,Δt)

    Given a task graph planning problem, computes a lower bound on completion
    time for each task.

    G - the task graph with that encodes dependencies between the M atomic tasks
    D - the distance matrix, where D[i,j] specifies the time required for agent i
        to accomplish task j
    Δt- the process time vector, where Δt[j] encodes the amount of time required
        for task j to be available after all prereqs of j are satisfied
"""
function compute_lower_time_bound(G,D,Δt)
    M = nv(G)
    N = size(D,1) - M

    # split distance matrix into appropriate blocks
    Drr = D[1:N,1:N]
    Drs = D[1:N,N+1:N+M]
    Dsr = Drs'
    Dss = D[N+1:N+M,N+1:N+M]

    # compute lower bound
    t_low = zeros(nv(G))
    root_node = map(e->e.dst, collect(edges(dfs_tree(G,1))))[end]
    bfs_traversal = bfs_tree(G,root_node;dir=:in)
    ids = map(e->e.dst, collect(edges(bfs_traversal)))
    append!(ids, root_node)
    for v in ids
        t_low[v] = Δt[v] + minimum(Drs[:,v])
        for v2 in inneighbors(G,v)
            t_low[v] = max(t_low[v], t_low[v2] + Δt[v] + minimum(Dss[inneighbors(G,v),v]))
        end
    end
    t_low
end


"""
    TODO: HungarianMethod

    Not done yet...
"""
function hungarian_method(Cp)
    C = copy(Cp)
    assignment = collect(1:size(C,1))
    solved = false
    # idxs = map(i->(mod(i-1,size(C,1))+1,div(i-1,size(C,2))+1),sortperm(C[:]))
    idxs = Set{Tuple{Int}}()
    idxs_i = Set{Int}()
    idxs_j = Set{Int}()
    for i in 1:size(C,1)
        for j in sortperm(C[i,:])
            if C[i,j] > 0
                C[i,:] .-= C[i,j]
                break
            end
        end
    end
    for j in 1:size(C,2)
        for i in sortperm(C[:,j])
            if C[i,j] > 0
                C[:,j] .-= C[i,j]
                break
            end
        end
    end
    if (rank(C .== 0) == size(C,1))
        solved = true
    end
    while !solved
        k = argmin(C[:])
        i = mod(k-1,size(C,1))+1
        j = div(i-1,size(C,2))+1
    end
    return assignment, solved
end
