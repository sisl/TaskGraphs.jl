using TaskGraphs
using Test
using Logging

include("task_graph_tests.jl")

using Main.TaskGraphTests

Main.TaskGraphTests.run_tests()

global_logger(SimpleLogger(stderr, Logging.Debug))
