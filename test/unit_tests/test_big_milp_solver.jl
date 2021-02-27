# Big MILP solver
let
    using Gurobi
    TaskGraphs.set_default_milp_optimizer!(Gurobi.Optimizer)

    for (i, f) in enumerate([
        pctapf_problem_1,
        pctapf_problem_2,
        pctapf_problem_3,
        pctapf_problem_4,
        pctapf_problem_5,
        # pctapf_problem_6,
        # pctapf_problem_7,
        pctapf_problem_8,
        ])
        for cost_model in [MakeSpan()]
            let
                costs = Float64[]
                for solver in [
                        NBSSolver(assignment_model = TaskGraphsMILPSolver(AssignmentMILP())),
                        BigMILPSolver(EXTRA_T=2),
                        ]
                    set_iteration_limit!(solver,1)
                    @show f
                    pc_tapf = f(solver;cost_function=cost_model,verbose=false);
                    env, cost = solve!(solver,pc_tapf)
                    # @show convert_to_vertex_lists(get_route_plan(env))
                    push!(costs, cost[1])
                    @test validate(env.schedule)
                    @test validate(env.schedule,convert_to_vertex_lists(get_route_plan(env)))
                    @test cost != typemax(Int)
                    @test cost != Inf
                end
                # @show costs
                @test all(costs .== costs[1])
            end
        end
    end

end
let
    TaskGraphs.clear_default_optimizer_attributes!()
    solver = TaskGraphs.BigMILPSolver(EXTRA_T=2)
    prob = pctapf_problem_1(solver)
    env, cost = solve!(solver,prob)
    validate(env.schedule)
    validate(env.schedule,convert_to_vertex_lists(get_route_plan(env)))

    env, cost = solve!(NBSSolver(),prob)
    convert_to_vertex_lists(env.route_plan)
    validate(env.schedule)

    milp = TaskGraphs.formulate_big_milp(prob,solver.EXTRA_T);
    optimize!(milp)
    @test termination_status(milp) == MOI.OPTIMAL
    robot_paths = TaskGraphs.extract_robot_paths(prob,milp)

    object_paths = TaskGraphs.extract_object_paths(prob,milp)

    # @test robot_paths[RobotID(1)][1:7] == [1, 5, 9, 13, 17, 21, 25]
    # @test robot_paths[RobotID(2)][1:7] == [4, 8, 12, 16, 20, 24, 28]

    # @test object_paths[ObjectID(1)][1:7] == [9, 9, 9, 13, 17, 17, 17,] 
    # @test object_paths[ObjectID(2)][1:7] == [12, 12, 12, 16, 20, 24, 28,]
    # @test object_paths[ObjectID(3)][1:7] == [21, 21, 21, 21, 21, 21, 25,]

    # robot_path = robot_paths[RobotID(1)]
    # object_path = object_paths[ObjectID(1)]
    # TaskGraphs.extract_solution(prob,milp)
end
let
    TaskGraphs.clear_default_optimizer_attributes!()
    solver = TaskGraphs.BigMILPSolver(EXTRA_T=2)
    prob = pctapf_problem_1(solver)

    graph = DiGraph(3)
    add_edge!(graph,1,1)
    add_edge!(graph,2,2)
    add_edge!(graph,3,3)
    add_edge!(graph,1,2)
    add_edge!(graph,2,3)
    # add_edge!(graph,3,4)
    G = TaskGraphs.construct_gadget_graph(graph,2)
    GraphPlottingBFS._title_string(n::Tuple{Int,Int}) = "$(n[2]),$(n[1])"
    GraphPlottingBFS.draw_node(G::TaskGraphs.GadgetGraph,v,args...;kwargs...) = draw_node(node_val(get_node(G,v)),args...;kwargs...)
    display_graph(G)


end