module TaskGraphTests

using MultiAgentFactory
using LightGraphs, MetaGraphs
using NearestNeighbors
using Main.TaskGraphs
using Test
using Logging
# Set logging level
global_logger(SimpleLogger(stderr, Logging.Debug))
# Define package tests
function run_tests()
    env, grid_env = initialize_tiny_env_and_grid_env()
    station_ids = Vector{Int}()
    for station in get_stations(env)
       push!(station_ids, knn(grid_env.kdtree, get_position(station),1)[1][1])
    end
    charger_ids = Vector{Int}([1,nv(grid_env.G)])
    # initialize env and env_state
    env_model = HighLevelEnvModel(station_ids,charger_ids,grid_env.G)
    initial_env_state = EnvState(
        robot_states = [AgentState(x=i,b=100,t=0) for i in 1:10],
        object_states = [ObjectState(x=nv(env_model.graph)-i) for i in 1:20]
        )
    env_state = copy(initial_env_state)
    @time @testset "TaskGraphs Package Tests" begin
        @time @testset "TaskGraphTests" begin
            task_graph = TaskGraph()
            add_operation!(task_graph,Operation(
                    Set{PlanningPredicate}([OBJECT_AT(1,1),OBJECT_AT(2,1)]),
                    Set{PlanningPredicate}([OBJECT_AT(3,1)]),
                    8))
            add_operation!(task_graph,Operation(
                    Set{PlanningPredicate}([OBJECT_AT(3,8)]),
                    Set{PlanningPredicate}([OBJECT_AT(4,8)]),
                    6))
            add_operation!(task_graph,Operation(
                    Set{PlanningPredicate}([OBJECT_AT(5,3)]),
                    Set{PlanningPredicate}([OBJECT_AT(6,3)]),
                    2))
            @test has_edge(task_graph.graph, 1, 2)
            add_operation!(task_graph,Operation(
                    Set{PlanningPredicate}([OBJECT_AT(4,3),OBJECT_AT(6,3)]),
                    Set{PlanningPredicate}([OBJECT_AT(7,3)]),
                    4))
            @test has_edge(task_graph.graph, 2, 4)
            @test has_edge(task_graph.graph, 3, 4)

            # compute lower bound on task graph
            t_lower = fill!(Vector{Int}(undef,nv(task_graph.graph)),
                typemax(Int))
            populate_lower_time_bound!(task_graph,env_state,env_model,t_lower,length(task_graph.operations))
            @show t_lower

        end
#         @time @testset "SimpleTransitionTests" begin
#             let
#                 let
#                     env_state = copy(initial_env_state)
#                     initial_state = AgentState(x=1,b=100,t=0,o=-1)
#                     action = GO(2)
#                     state = transition(env, initial_state, action)
#                     @test state.x == action.x
#                     @test state.b == initial_state.b - 1
#                     @test state.o == initial_state.o
#                     @test state.t == initial_state.t + 1
#                     # collect and deposit object
#                     action = GO(env_state.object_states[1].x)
#                     state = transition(env, initial_state, action)
#                     @test state.x == action.x
#                     action = COLLECT(1)
#                     state = transition(env, state, action)
#                     @test state.o == action.o
#                     action = DEPOSIT(1)
#                     state = transition(env, state, action)
#                     @test state.o == ObjectID()
#                     action = CHARGE(100)
#                     state = transition(env, state, action)
#                     @test state.b == action.b
#                 end
#             end
#         end
#         @time @testset "EnvTests" begin
#             let
#                 env_state = copy(initial_env_state)
#                 @test length(env_state.action_queue) == length(env_state.robot_states)
#                 agent_id = 1
#                 object_id = 1
#                 action = GO(env_state.object_states[object_id].x)
#                 env_state = transition(env,env_state,agent_id,action)
#                 agent_state = env_state.robot_states[agent_id]
#                 @test agent_state.x == action.x
#                 action = COLLECT(object_id)
#                 env_state = transition(env,env_state,agent_id,action)
#                 agent_state = env_state.robot_states[agent_id]
#                 object_state = env_state.object_states[object_id]
#                 @test agent_state.o == object_id
#                 @test object_state.r == agent_id
#                 action = GO(1)
#                 env_state = transition(env,env_state,agent_id,action)
#                 agent_state = env_state.robot_states[agent_id]
#                 object_state = env_state.object_states[object_id]
#                 @test agent_state.x == action.x
#                 @test object_state.x == agent_state.x
#                 action = DEPOSIT(object_id)
#                 env_state = transition(env,env_state,agent_id,action)
#                 agent_state = env_state.robot_states[agent_id]
#                 object_state = env_state.object_states[object_id]
#                 @test agent_state.o == ObjectID()
#                 @test object_state.r == RobotID()
#                 action = GO_AND_CHARGE(env.charger_ids[1],100)
#                 env_state = transition(env,env_state,agent_id,action)
#                 agent_state = env_state.robot_states[agent_id]
#                 @test agent_state.x == action.x
#                 @test agent_state.b == action.b
#                 object_id = 2
#                 action = COLLECT_AND_DELIVER(2,env_state.object_states[object_id].x,agent_state.x)
#                 env_state = transition(env,env_state,agent_id,action)
#                 agent_state = env_state.robot_states[agent_id]
#                 object_state = env_state.object_states[object_id]
#                 @test agent_state.x == object_state.x == action.x2
#                 @test agent_state.o == ObjectID()
#                 @test object_state.r == RobotID()
#             end
#         end
    end
end
end

# using Pkg; Pkg.activate("."); using LightGraphs, MetaGraphs, NearestNeighbors, MultiAgentFactory
# include("src/task_graphs/task_graphs.jl"); using Main.TaskGraphs, Main.TaskGraphTests
# Main.TaskGraphTests.run_tests()
# env, grid_env = initialize_tiny_env_and_grid_env()
# station_ids = Vector{Int}()
# for station in get_stations(env)
#    push!(station_ids, knn(grid_env.kdtree, get_position(station),1)[1][1])
# end
# charger_ids = Vector{Int}([1,nv(grid_env.G)])
# env = HighLevelEnvModel(station_ids,charger_ids,grid_env.G)
# env_state = EnvState([AgentState(x=i,b=100,t=0) for i in 1:10],[ObjectState(x=nv(env.graph)-i) for i in 1:20])
# copy(env_state)
# state = AgentState(x=1,b=100,t=0)
# state = copy(state)
# state = transition(env, state, GO(4))
# state = transition(env, state, GO(40))
# state = transition(env, state, GO(1))
# state = transition(env, state, GO_AND_CHARGE(env.charger_ids[end],100))
