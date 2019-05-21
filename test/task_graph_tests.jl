module TaskGraphTests

using LightGraphs, MetaGraphs
using TaskGraphs
using Test
using Logging
# Set logging level
global_logger(SimpleLogger(stderr, Logging.Debug))
# Define package tests
function run_tests()
    initial_env_state = EnvState(
        robot_states = [AgentState(x=i,b=100,t=0) for i in 1:10],
        object_states = [ObjectState(x=i) for i in 11:20]
        )
    env_state = copy(initial_env_state)
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

        end
        @time @testset "RandomInitializationTests" begin
            construct_random_project_spec(20)
        end
        # @time @testset "SimpleTransitionTests" begin
        #     let
        #         let
        #             env = nothing
        #             env_state = copy(initial_env_state)
        #             initial_state = AgentState(x=1,b=100,t=0,o=-1)
        #             action = GO(2)
        #             state = transition(env, initial_state, action)
        #             @test state.x == action.x
        #             @test state.b == initial_state.b - 1
        #             @test state.o == initial_state.o
        #             @test state.t == initial_state.t + 1
        #             # collect and deposit object
        #             action = GO(env_state.object_states[1].x)
        #             state = transition(env, initial_state, action)
        #             @test state.x == action.x
        #             action = COLLECT(1)
        #             state = transition(env, state, action)
        #             @test state.o == action.o
        #             action = DEPOSIT(1)
        #             state = transition(env, state, action)
        #             @test state.o == ObjectID()
        #             action = CHARGE(100)
        #             state = transition(env, state, action)
        #             @test state.b == action.b
        #         end
        #     end
        # end
        # @time @testset "EnvTests" begin
        #     let
        #         env = nothing
        #         env_state = copy(initial_env_state)
        #         @test length(env_state.action_queue) == length(env_state.robot_states)
        #         agent_id = 1
        #         object_id = 1
        #         action = GO(env_state.object_states[object_id].x)
        #         env_state = transition(env,env_state,agent_id,action)
        #         agent_state = env_state.robot_states[agent_id]
        #         @test agent_state.x == action.x
        #         action = COLLECT(object_id)
        #         env_state = transition(env,env_state,agent_id,action)
        #         agent_state = env_state.robot_states[agent_id]
        #         object_state = env_state.object_states[object_id]
        #         @test agent_state.o == object_id
        #         @test object_state.r == agent_id
        #         action = GO(1)
        #         env_state = transition(env,env_state,agent_id,action)
        #         agent_state = env_state.robot_states[agent_id]
        #         object_state = env_state.object_states[object_id]
        #         @test agent_state.x == action.x
        #         @test object_state.x == agent_state.x
        #         action = DEPOSIT(object_id)
        #         env_state = transition(env,env_state,agent_id,action)
        #         agent_state = env_state.robot_states[agent_id]
        #         object_state = env_state.object_states[object_id]
        #         @test agent_state.o == ObjectID()
        #         @test object_state.r == RobotID()
        #         action = GO_AND_CHARGE(env.charger_ids[1],100)
        #         env_state = transition(env,env_state,agent_id,action)
        #         agent_state = env_state.robot_states[agent_id]
        #         @test agent_state.x == action.x
        #         @test agent_state.b == action.b
        #         object_id = 2
        #         action = COLLECT_AND_DELIVER(2,env_state.object_states[object_id].x,agent_state.x)
        #         env_state = transition(env,env_state,agent_id,action)
        #         agent_state = env_state.robot_states[agent_id]
        #         object_state = env_state.object_states[object_id]
        #         @test agent_state.x == object_state.x == action.x2
        #         @test agent_state.o == ObjectID()
        #         @test object_state.r == RobotID()
        #     end
        # end
    end
end
end
