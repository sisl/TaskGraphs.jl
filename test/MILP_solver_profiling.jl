
using LightGraphs, MetaGraphs
using Parameters
using LinearAlgebra
using Compose
using Colors
using TaskGraphs
using GraphPlottingBFS
using DataStructures
using JuMP
using GLPK
using Gurobi

using CSV
using DataFrames

data_path = joinpath(dirname(pathof(TaskGraphs)),"..","test","profiling")
file_list = readdir(data_path)
num_csv_files = 0
for filename in file_list
    if splitext(filename)[end] == ".csv"
        global num_csv_files += 1
    end
end
# filename = joinpath(data_path, string("MILP_profiling_", num_csv_files, ".csv"))
filename = joinpath(data_path, string("MILP_profiling", ".csv"))
# df = DataFrame(
#     N = Int[],
#     M = Int[],
#     max_parents=Int[],
#     depth_bias=Float64[],
#     solve_time=Float64[],
#     termination_status=Int[]
#     )

TRIALS_PER_SETTING = 3;
iteration = 1;
# run tests and push results into table
for N in [10,20,40,100,200]
    for M in Vector{Int}([N/2, N, 2*N, 4*N])
        for max_parents in [3, 5, 10]
            for depth_bias in [0.1, 0.4, 0.8, 1.0]
                global iteration
                for trial in 1:TRIALS_PER_SETTING + (iteration == 1)
                    df = DataFrame(
                        N = Int[],
                        M = Int[],
                        max_parents=Int[],
                        depth_bias=Float64[],
                        solve_time=Float64[],
                        termination_status=Int[]
                        )
                    r₀,s₀,sₜ = initialize_random_2D_task_graph_env(N,M;d=[40,40])
                    Drs, Dss = cached_pickup_and_delivery_distances(r₀,s₀,sₜ)
                    project_spec = construct_random_project_spec(M;max_parents=max_parents,depth_bias=depth_bias,Δt_min=0,Δt_max=0)
                    delivery_graph = construct_delivery_graph(project_spec,M)
                    G = delivery_graph.graph
                    # initialize vector of operation times
                    Δt = zeros(nv(G)) # Δt[j] is the wait time for the object j to appear once all inputs have been satisfied
                    for op in project_spec.operations
                        for id in get_output_ids(op)
                            Δt[id] = duration(op)
                        end
                    end
                    # set initial conditions
                    to0_ = Dict{Int,Float64}()
                    for v in vertices(G)
                        if is_leaf_node(G,v)
                            to0_[v] = 0.0
                        end
                    end
                    tr0_ = Dict{Int,Float64}()
                    for i in 1:N
                        tr0_[i] = 0.0
                    end
                    # formulate MILP
                    model = formulate_JuMP_optimization_problem(
                        G,Drs,Dss,Δt,to0_,tr0_,
                        Gurobi.Optimizer;TimeLimit=10);
                    # Solve!
                    start_time = time()
                    optimize!(model)
                    solve_time = time() - start_time

                    global iteration
                    if iteration > 1
                        row_dict = Dict(
                            :N => N,
                            :M => M,
                            :max_parents => max_parents,
                            :depth_bias => depth_bias,
                            :solve_time => solve_time,
                            :termination_status => Int(termination_status(model))
                            )
                        push!(df, row_dict)
                        file = open(filename;append=true)
                        df |> CSV.write(file;append=true)
                        close(file)
                        for k in sort(collect(keys(row_dict)))
                            print(string(k),": ",row_dict[k],", ")
                        end
                        print("\n")
                    end
                    iteration += 1
                end
            end
        end
    end
end

file = open(filename;write=true)
df |> CSV.write(file)
close(file)
