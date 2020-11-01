using TaskGraphs, CRCBS, DataFrames, TOML
include(joinpath(pathof(TaskGraphs),"..","helpers/render_tools.jl"))

base_dir = "/scratch/task_graphs_experiments"
# Collaborative
base_problem_dir    = joinpath(base_dir,"collaborative_problem_instances/final")
# base_results_dir    = joinpath(base_dir,"sparse_adjacency_solver/collaborative_transport/results/")
# These results are currently in the slides
base_results_dir    = joinpath(base_dir,
    "sparse_adjacency_solver/final/meta_env_repaired/results")

feats = Dict(
        :optimality_gap=>Int,
        :optimal=>Bool,
        :feasible=>Bool,
        :valid_flag=>Bool,
        :time=>Float64,
        :N=>Int,
        :M=>Int,
        :ratio=>Int,
        :problem_id=>String,
        )

solver_feats = [
    :full_solver=>[
        :optimality_gap,
        :optimal,
        :feasible,
        :time,
        :N,
        :M,
        :ratio,
        :problem_id,
    ],
    :assignment_only=>[
        :optimality_gap,
        :optimal,
        :feasible,
        :time,
        :N,
        :M,
        :ratio,
        :problem_id,
    ],
    :low_level_search_without_repair=>[
        :time,
        :N,
        :M,
        :ratio,
        :valid_flag,
        :problem_id,
    ]
]
df_dict = Dict(
    k=>DataFrame([f=>Vector{feats[f]}() for f in v]...) for (k,v) in solver_feats
)

task_size_distributions = [
                    ( 1=>1.0, 2=>0.0, 4=>0.0 ),
                    ( 1=>1.0, 2=>1.0, 4=>0.0 ),
                    ( 1=>1.0, 2=>1.0, 4=>1.0 ),
                    ( 1=>0.0, 2=>1.0, 4=>1.0 ),
                    ]
ratio_dict = Dict((1,0,0)=>1,(1,1,0)=>2,(1,1,1)=>3,(0,1,1)=>4,)

for (solver_name,df) in df_dict
    results_path = joinpath(base_results_dir,string(solver_name))
    dfkeys = map(Symbol,names(df))
    for results_name in readdir(results_path)
        prob_id = parse(Int,results_name[8:end-5])
        # @show prob_id
        config_file = joinpath(base_problem_dir,"config$(prob_id).toml")
        config = to_symbol_dict(TOML.parsefile(config_file))
        # @show config[:ratio]
        # prob_file = joinpath(base_problem_dir,"problem$(prob_id).toml")
        # prob_dict = to_symbol_dict(TOML.parsefile(prob_file))
        # @show prob_dict[:shapes]
        # shape_sums = map(sum,prob_dict[:shapes])
        # ratio = (sum(shape_sums .== 2),sum(shape_sums.==3),sum(shape_sums.==4))
        # r = ratio
        # if ratio[1] == length(shape_sums)
        #     r = (1,0,0)
        # elseif ratio[1] > 0 && ratio[2] > 0 && ratio[3] == 0
        #     r = (1,1,0)
        # elseif ratio[1] > 0 && ratio[2] > 0 && ratio[3] > 0
        #     r = (1,1,1)
        # elseif ratio[1] == 0 && ratio[2] > 0 && ratio[3] > 0
        #     r = (0,1,1)
        # end
        #
        # @show ratio
        # @show [r...]
        results_file = joinpath(results_path,results_name)
        toml_dict = to_symbol_dict(TOML.parsefile(results_file))
        toml_dict[:ratio] = ratio_dict[tuple(config[:ratio]...)]
        toml_dict[:N] = config[:N]
        toml_dict[:M] = config[:M]
        results_dict = filter(p->p.first in dfkeys, toml_dict)
        results_dict[:problem_id] = results_name
        push!(df,results_dict)
    end
end

# prob_path = "/scratch/task_graphs_experiments/problem_instances/"
# results_path = "/scratch/task_graphs_experiments/results/"

# df_list = Vector{DataFrame}([df_dict[k] for (k,v) in solver_feats])
# for df in values(df_dict)
#     if "size_i" in names(df)
#         df.N = df.size_i .- df.size_j
#         df.M = df.size_j
#     end
# end

plt = get_box_plot_group_plot(df_dict[:assignment_only];
    obj=:time,
    outer_key=:ratio,
    inner_key=:M,
    title="Full Solver",
    ymax=300
    )
# display(plt);
pgfsave("plt.tikz",plt)

for (k,df) in df_dict
    for r in sort(unique(df.ratio))
        for m in sort(unique(df.M))
            sub = df[.&(df.ratio .== r,df.M .== m),:]
            non_feasible = 0
            if "feasible" in names(df)
                non_feasible =  sum(sub.feasible .== false)
            else
                non_feasible = sum(df.valid_flag .== false)
            end
            @show k, r, m, non_feasible
        end
    end
end

# plot_histories_pgf(df_list;
#     y_key=:makespans,
#     x_key=:none,
#     include_keys = [:num_projects=>30,:M=>10,:arrival_interval=>40],
#     xlabel = "time",
#     ylabel = "makespans",
#     ytick_show = true,
#     ymode="linear",
#     lines=false
# )
#
# plt = PGFPlots.GroupPlot(3,5)
# for (n_vals, m_vals) in [
#     ([10,],[20,30,40,]),
#     ([15,],[30,40,50,]),
#     ([20,],[40,50,60,]),
#     ([25,],[50,60,70,]),
#     ([30,],[60,70,80,]),
#     ]
#     gp = group_history_plot(df_list;
#         y_key = :makespans,
#         x_key=:none,
#         n_key = :M,
#         n_vals = n_vals,
#         m_key = :arrival_interval,
#         m_vals = m_vals,
#         ymode="log",
#         lines=false,
#     )
#     append!(plt.axes,gp.axes)
# end
# display(plt)
#
# for (df,planner_config) in zip(df_list,planner_configs)
#     plt = PGFPlots.GroupPlot(3,5)
#     for (n_vals, m_vals) in [
#         ([10,],[20,30,40,]),
#         ([15,],[30,40,50,]),
#         ([20,],[40,50,60,]),
#         ([25,],[50,60,70,]),
#         ([30,],[60,70,80,]),
#         ]
#         gp = group_history_plot([df];
#             y_key = :primary_runtimes,
#             x_key=:none,
#             n_key = :M,
#             n_vals = n_vals,
#             m_key = :arrival_interval,
#             m_vals = m_vals,
#             ymode="log", #"linear"
#             lines=false,
#         )
#         gp2 = group_history_plot([df];
#             y_key = :backup_runtimes,
#             x_key=:none,
#             n_key = :M,
#             n_vals = n_vals,
#             m_key = :arrival_interval,
#             m_vals = m_vals,
#             ymode="log",
#             lines=false,
#             colors=["blue"],
#         )
#         for (ax,ax2) in zip(gp.axes,gp2.axes)
#             append!(ax.plots,ax2.plots)
#         end
#
#         append!(plt.axes,gp.axes)
#     end
#     display(plt)
#     save("/home/kylebrown/Desktop/$(planner_config.planner_name)_runtimes.pdf",plt)
# end
