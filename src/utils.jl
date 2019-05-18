export
    initialize_random_2D_task_graph_env,
    construct_factory_distance_matrix


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
