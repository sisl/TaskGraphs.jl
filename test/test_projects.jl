
using TaskGraphs
using LightGraphs, MetaGraphs, GraphUtils

# Construct and modify project schedules
let
    P = OperatingSchedule()
    add_to_schedule!(P, ROBOT_AT(1,1),RobotID(1))
end
