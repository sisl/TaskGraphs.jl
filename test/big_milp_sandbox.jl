using TaskGraphs, CRCBS, GraphUtils, LightGraphs, JuMP, Gurobi
Revise.includet("/home/kylebrown/.julia/dev/TaskGraphs/src/helpers/render_tools.jl")
using Test

vtx_grid = initialize_dense_vtx_grid(3,3)
env = construct_factory_env_from_vtx_grid(vtx_grid)
# convert to graph to ensure undirected edges
env_graph = TaskGraphs.convert_env_graph_to_undirected(env.graph)
edge_list = sort(collect(edges(env_graph));by=e->(e.src,e.dst))
@assert all(map(e->e.src<e.dst, edge_list))


# solver = NBSSolver()
# prob = pctapf_problem_1(solver)
vtx_grid = VtxGrid(reshape([1,2,3,4],(1,4)))
env_graph = construct_factory_env_from_vtx_grid(vtx_grid)
# r0 = [1,4]
# s0 = [2,3]
# sF = [1,4]
r0 = [1,4]
s0 = [2]
sF = [1]
Δt_op = 0
project_spec, robot_ICs = pctapf_problem(r0,s0,sF)
# add_operation!(project_spec,construct_operation(project_spec,-1,[1],[2],Δt_op))
# add_operation!(project_spec,construct_operation(project_spec,-1,[2],  [], Δt_op))
add_operation!(project_spec,construct_operation(project_spec,-1,[1],  [], Δt_op))
def = SimpleProblemDef(project_spec,r0,s0,sF)
project_spec, problem_spec, _, _, robot_ICs = construct_task_graphs_problem(
    def,env_graph)
solver = NBSSolver()
prob = pctapf_problem(solver,project_spec,problem_spec,robot_ICs,env_graph)
model = Model(with_optimizer(Gurobi.Optimizer))
milp_model = TaskGraphs.formulate_big_milp(prob,3,model)
optimize!(model)
robot_paths = TaskGraphs.extract_robot_paths(prob,milp_model)
object_paths = TaskGraphs.extract_object_paths(prob,milp_model)
robot_path_vtxs = Set(union(values(robot_paths)...))
object_path_vtxs = Set(union(values(object_paths)...))
object_path_vtxs = Set(findall(Bool.(round.(value.(milp_model.object_flows[ObjectID(1)])))))

plot_graph_bfs(milp_model.G.G;
    color_function = (G,v,x,y,r)-> begin
        if v in object_path_vtxs
            if v in robot_path_vtxs
                return "red"
            else
                return "orange"
            end
        elseif v in robot_path_vtxs
            return "cyan"
        else
            return "gray"
        end
    end
    )
