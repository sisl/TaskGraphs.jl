solver_configs = [
    (
        solver = TaskGraphsMILPSolver(AssignmentMILP()),
        results_path = "AssignmentMILP",
    ),
    (
        solver = TaskGraphsMILPSolver(SparseAdjacencyMILP()),
        results_path = "SparseAdjacencyMILP",
    ),
    (
        solver = TaskGraphsMILPSolver(FastSparseAdjacencyMILP()),
        results_path = "FastSparseAdjacencyMILP",
    ),
]

base_dir = joinpath("/scratch/task_graphs_experiments","assignment_problems")
config = (
    problem_dir = joinpath(base_dir,"problem_instances"),
    feats = [
        RunTime(),
        FeasibleFlag(),
        OptimalFlag(),
        OptimalityGap(),
        SolutionCost(),
    ]
)
