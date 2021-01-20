# Core Types
## Planning Predicates
```@docs
OBJECT_AT
BOT_AT
BOT_COLLECT
BOT_DEPOSIT
TEAM_ACTION
```
## Scheduling 
```@docs
ProjectSpec
ProblemSpec
ScheduleNode
OperatingSchedule
```

## Path Planning
```@docs
SearchEnv
EnvState
```

## Problem Types
```@docs
AbstractPC_MAPF
AbstractPC_TAPF
PC_TAPF
PC_TA
PC_MAPF
C_PC_MAPF
C_PC_TAPF
RepeatedAbstractPC_TAPF
RepeatedPC_TAPF
ProjectRequest
ReplannerModel
ReplannerConfig
DeferUntilCompletion
ReassignFreeRobots
MergeAndBalance
ConstrainedMergeAndBalance
ReplanningProfilerCache
FullReplanner
ReplannerWithBackup
replan!
```

## Task Assignment Solvers
```@docs
TaskGraphsMILP
TaskGraphsMILPSolver
AssignmentMILP
SparseAdjacencyMILP
GreedyAssignment
formulate_milp
```

## Route Planners
```@docs
AStarSC
ISPS
PIBTPlanner
CBSSolver
```

## PC_TAPF Solvers
```@docs
NBSSolver
<!-- SolverLogger -->
solve!
formulate_assignment_problem
update_assignment_problem!
solve_assignment_problem!
construct_cost_model
plan_route!
```

## Profiling Tools
```@docs
TaskGraphsProblemLoader
write_problem
load_problem
run_profiling
warmup
```