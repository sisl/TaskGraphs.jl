let
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_8(;
        verbose=false);
    let
        model = formulate_JuMP_optimization_problem(problem_spec,Gurobi.Optimizer;cost_model=:MakeSpan)
        optimize!(model)
        optimal = (termination_status(model) == MathOptInterface.OPTIMAL)
        optimal_TA_cost = Int(round(value(objective_function(model))));
        @test optimal == true
        @test optimal_TA_cost == 8
    end
    let
        model = formulate_JuMP_optimization_problem(problem_spec,Gurobi.Optimizer;cost_model=:SumOfMakeSpans)
        optimize!(model)
        optimal = (termination_status(model) == MathOptInterface.OPTIMAL)
        optimal_TA_cost = Int(round(value(objective_function(model))));
        @test optimal == true
        @test optimal_TA_cost == 16
    end
end
# Station Sharing
let
    for i in 0:2
        project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_9(;
            verbose=false,Δt_op=0,Δt_collect=[i,0],Δt_deliver=[0,0]
            );
        model = formulate_JuMP_optimization_problem(problem_spec,Gurobi.Optimizer;cost_model=:SumOfMakeSpans)
        optimize!(model)
        optimal = (termination_status(model) == MathOptInterface.OPTIMAL)
        optimal_TA_cost = Int(round(value(objective_function(model))));
        @show optimal
        @show optimal_TA_cost
    end
end
