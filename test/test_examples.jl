# Root nodes
let
    # This project has only two delivery tasks, both of which converge into a
    # single final project. Here we test that the cost models function correctly,
    # treating the tasks as part of the same project head rather than as two
    # separate project heads.
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_4(;
        verbose=false);
    let
        model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;cost_model=MakeSpan)
        optimize!(model)
        optimal = (termination_status(model) == MathOptInterface.OPTIMAL)
        optimal_TA_cost = Int(round(value(objective_function(model))));
        @test optimal == true
        @test optimal_TA_cost == 2
    end
    let
        model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;cost_model=SumOfMakeSpans)
        optimize!(model)
        optimal = (termination_status(model) == MathOptInterface.OPTIMAL)
        optimal_TA_cost = Int(round(value(objective_function(model))));
        @test optimal == true
        @test optimal_TA_cost == 2
    end
end
# Cost models
let
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_8(;
        verbose=false);
    let
        model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;cost_model=MakeSpan)
        optimize!(model)
        optimal = (termination_status(model) == MathOptInterface.OPTIMAL)
        optimal_TA_cost = Int(round(value(objective_function(model))));
        @test optimal == true
        @test optimal_TA_cost == 8
    end
    let
        model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;cost_model=SumOfMakeSpans)
        optimize!(model)
        optimal = (termination_status(model) == MathOptInterface.OPTIMAL)
        optimal_TA_cost = Int(round(value(objective_function(model))));
        @test optimal == true
        @test optimal_TA_cost == 16
    end
end
# Station Sharing
let
    for (i,true_cost) in zip([0,1,2],[7,8,9])
        let
            project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_9(;
                verbose=false,Δt_op=0,Δt_collect=[i,0],Δt_deliver=[0,0]
                );
            model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;cost_model=SumOfMakeSpans)
            optimize!(model)
            optimal = (termination_status(model) == MathOptInterface.OPTIMAL)
            optimal_TA_cost = Int(round(value(objective_function(model))));
            @test optimal_TA_cost == true_cost
            @test optimal == true
        end
    end
    for (i,true_cost) in zip([0,1,2],[7,8,9])
        let
            project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_9(;
                verbose=false,Δt_op=0,Δt_collect=[i,0],Δt_deliver=[0,0]
                );
            model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;cost_model=SumOfMakeSpans)
            optimize!(model)
            optimal = (termination_status(model) == MathOptInterface.OPTIMAL)
            optimal_TA_cost = Int(round(value(objective_function(model))));
            @test optimal_TA_cost == true_cost
            @test optimal == true
        end
    end
end
# Job shop constraints
let
    project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_9(;
        verbose=false,Δt_op=0,Δt_collect=[0,0],Δt_deliver=[0,0]
        );
    model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;cost_model=SumOfMakeSpans)
    optimize!(model)
    @test termination_status(model) == MathOptInterface.OPTIMAL
    assignment_matrix = get_assignment_matrix(model);
    assignments = map(j->findfirst(assignment_matrix[:,j] .== 1),1:problem_spec.M);
    schedule = construct_project_schedule(project_spec, problem_spec, robot_ICs, assignments)
    n_edges = ne(get_graph(schedule))
    add_job_shop_constraints!(schedule,problem_spec,model)
    @test ne(get_graph(schedule)) > n_edges
end
