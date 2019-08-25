module TaskGraphTests

using LightGraphs, MetaGraphs, GraphUtils
using TaskGraphs
using Test
using Logging
using Gurobi

# Set logging level
global_logger(SimpleLogger(stderr, Logging.Debug))
# Define package tests
function run_tests()
    G = DiGraph(3)
    add_edge!(G,1,2)
    @test get_all_root_nodes(G) == Set([2,3])

    @time @testset "TaskGraphs Package Tests" begin
        @time @testset "TaskGraphTests" begin
            N = 4                   # num robots
            M = 10                  # num delivery tasks
            r₀,s₀,sₜ = initialize_random_2D_task_graph_env(N,M;d=[20,20])
            D = construct_factory_distance_matrix(r₀,s₀,sₜ)

            project_spec = ProjectSpec()
            add_operation!(project_spec,construct_operation(1, [1,2,3],[4], 0))
            add_operation!(project_spec,construct_operation(2, [5,6]  ,[7], 0))
            add_operation!(project_spec,construct_operation(3, [4,7]  ,[8], 0))
            add_operation!(project_spec,construct_operation(4, [8,9]  ,[10],0))
            @test has_edge(project_spec.graph, 1, 3)
            @test has_edge(project_spec.graph, 2, 3)
            @test has_edge(project_spec.graph, 3, 4)

            # compute lower bound on task graph
            # t_lower = fill!(Vector{Int}(undef,nv(task_graph.graph)),typemax(Int))
            # env_model = HighLevelEnvModel()
            # populate_lower_time_bound!(task_graph,env_state,env_model,t_lower,length(task_graph.operations))
            # @show t_lower

            N = 20                  # num robots
            M = 60                  # num delivery tasks
            # project_spec = construct_random_project_spec(M;max_parents=3,depth_bias=1.0,Δt_min=0,Δt_max=0)
            project_spec1 = construct_random_project_spec(Int(M/2);max_parents=3,depth_bias=0.25,Δt_min=0,Δt_max=0)
            project_spec2 = construct_random_project_spec(Int(M/2);max_parents=3,depth_bias=0.25,Δt_min=0,Δt_max=0)
            project_spec = combine_project_specs([project_spec1, project_spec2])

            delivery_graph = construct_delivery_graph(project_spec,M)
            r₀,s₀,sₜ,pts = initialize_random_2D_task_graph_env(N,M;d=[400,400])
            Drs, Dss = cached_pickup_and_delivery_distances(pts[r₀],pts[s₀],pts[sₜ])
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

            model = formulate_JuMP_optimization_problem(G,Drs,Dss,Δt,to0_,tr0_,Gurobi.Optimizer;OutputFlag=0);
            spec = TaskGraphProblemSpec(N,M,G,Drs,Dss,Δt,tr0_,to0_)
        end
        @time @testset "RandomInitializationTests" begin
            construct_random_project_spec(20)
        end
    end
end
end
