# Root nodes
for (f,costs) in [
        (initialize_toy_problem_4,[2,2]),
        (initialize_toy_problem_8,[8,16])
    ]
    for (i,cost_model) in enumerate([MakeSpan,SumOfMakeSpans])
        let
            _, problem_spec, _, _, _ = f(;verbose=false);
            let
                model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;cost_model=cost_model)
                optimize!(model)
                optimal = (termination_status(model) == MathOptInterface.OPTIMAL)
                optimal_TA_cost = Int(round(value(objective_function(model))));
                @test optimal == true
                @test optimal_TA_cost == costs[i]
            end
        end
    end
end
# Station Sharing
let
    for (i, dt) in enumerate([0,1,2])
        for (cost_model, costs) in [
            (MakeSpan,[4,5,6])
            (SumOfMakeSpans,[7,8,9])
            ]
            let
                # i = 0
                # cost_model = SumOfMakeSpans
                project_spec, problem_spec, robot_ICs, assignments, env_graph = initialize_toy_problem_9(;
                    verbose=false,Δt_op=0,Δt_collect=[dt,0],Δt_deliver=[0,0]
                    );
                model = formulate_optimization_problem(problem_spec,Gurobi.Optimizer;cost_model=cost_model)
                optimize!(model)
                optimal = (termination_status(model) == MathOptInterface.OPTIMAL)
                optimal_TA_cost = Int(round(value(objective_function(model))));
                # @show optimal_TA_cost
                @show i, cost_model
                @test optimal == true
                @test optimal_TA_cost == costs[i]
            end
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
