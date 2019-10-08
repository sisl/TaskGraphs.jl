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
