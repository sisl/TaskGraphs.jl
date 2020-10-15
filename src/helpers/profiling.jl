
CRCBS.extract_feature(solver::NBSSolver,::LowLevelIterationCount, args...) = max_iterations(route_planner(solver))

export
    ObjectPathSummaries

struct ObjectPathSummaries <: FeatureExtractor{Dict{Int,Vector{}}} end
function get_object_path_summaries end
function CRCBS.extract_feature(solver,::ObjectPathSummaries,pc_mapf,env,timer_results)
    summaries = get_object_path_summaries(env)
    to_string_dict(to_string_dict(Dict(k=>to_string_dict(v) for (k,v) in summaries)))
end

extract_object_data(node,t0,tF) = Dict{Symbol,Union{Vector{Int},Int}}()
extract_object_data(node::OBJECT_AT,t0,tF) = Dict{Symbol,Union{Vector{Int},Int}}(
    :object_id=>get_id(get_object_id(node)),
    :start_vtx=>get_id(get_initial_location_id(node)),
    :start_time=>t0,
    )
extract_object_data(node::COLLECT,t0,tF) = Dict{Symbol,Union{Vector{Int},Int}}(
    :object_id=>get_id(get_object_id(node)),
    :start_vtx=>get_id(get_initial_location_id(node)),
    :robot_ids=>map(get_id,get_robot_ids(node)),
    :collect_time=>tF,
    )
extract_object_data(node::CARRY,t0,tF) = Dict{Symbol,Union{Vector{Int},Int}}(
    :object_id=>get_id(get_object_id(node)),
    :start_vtx=>get_id(get_initial_location_id(node)),
    :end_vtx=>get_id(get_destination_location_id(node)),
    :robot_ids=>map(get_id,get_robot_ids(node)),
    :collect_time=>t0,
    )
extract_object_data(node::DEPOSIT,t0,tF) = Dict{Symbol,Union{Vector{Int},Int}}(
    :object_id=>get_id(get_object_id(node)),
    :end_vtx=>get_id(get_destination_location_id(node)),
    :robot_ids=>map(get_id,get_robot_ids(node)),
    :deposit_time=>tF,
    )
function get_object_path_summaries(env::SearchEnv)
    summaries = Dict{Int,Dict{Symbol,Union{Vector{Int},Int}}}()
    for v in vertices(env.schedule)
        node = get_node_from_vtx(env.schedule,v)
        dict = extract_object_data(node,env.cache.t0[v],env.cache.tF[v])
        if haskey(dict,:object_id)
            object_id = dict[:object_id]
            merge!(get!(summaries,object_id,valtype(summaries)()),dict)
        end
    end
    summaries
end

function reconstruct_object_paths(robot_paths,object_path_summaries)
    object_paths = Dict{Int,Vector{Int}}()
    object_intervals = Dict{Int,Tuple{Int,Int}}()
    for object_summary in values(object_path_summaries)
        id = object_summary["object_id"]
        t0 = object_summary["start_time"]
        tC = object_summary["collect_time"]
        tF = object_summary["deposit_time"]
        v0 = object_summary["start_vtx"]
        vF = object_summary["end_vtx"]
        robot_ids = object_summary["robot_ids"]
        p1 = map(t->v0,t0:tC-1)
        p2 = robot_paths[robot_ids[1]][tC+1:tF+1]
        object_paths[id] = vcat(p1,p2)
        object_intervals[id] = (t0,tF)
    end
    return object_paths, object_intervals
end

export
    AbstractProblemLoader,
    add_env!,
    get_env!,
    problem_type,
    PCTAPF_Loader,
    PCTA_Loader,
    PCMAPF_Loader,
    ReplanningProblemLoader

abstract type AbstractProblemLoader end

function add_env!(loader::AbstractProblemLoader,env_id::String,
    env=read_env(env_id))
    if haskey(loader.envs,env_id)
    else
        loader.envs[env_id] = env
        loader.prob_specs[env_id] = ProblemSpec(graph=env)
    end
    loader
end
function get_env!(loader::AbstractProblemLoader,env_id::String)
    if !haskey(loader.envs,env_id)
        loader.envs[env_id] = read_env(env_id)
    end
    return loader.envs[env_id]
end

"""
    TaskGraphsProblemLoader{T}

Helper cache for loading problems of type `T`
"""
struct TaskGraphsProblemLoader{T}
    envs::Dict{String,GridFactoryEnvironment}
    prob_specs::Dict{String,ProblemSpec}
    TaskGraphsProblemLoader{T}() = new(
        Dict{String,GridFactoryEnvironment}(),
        Dict{String,ProblemSpec}()
    )
end
problem_type(::TaskGraphsProblemLoader{T}) where {T} = T
const PCTAPF_Loader = TaskGraphsProblemLoader{PC_TAPF}
const PCMAPF_Loader = TaskGraphsProblemLoader{PC_MAPF}
const PCTA_Loader   = TaskGraphsProblemLoader{PC_TA}
const ReplanningProblemLoader = TaskGraphsProblemLoader{PC_TA}

# """
#     PCTAPF_Loader
#
# Helper cache for loading `PCTAPF` problems
# """
# struct PCTAPF_Loader
#     envs::Dict{String,GridFactoryEnvironment}
#     prob_specs::Dict{String,ProblemSpec}
#     PCTAPF_Loader() = new(
#         Dict{String,GridFactoryEnvironment}(),
#         Dict{String,ProblemSpec}()
#     )
# end
# problem_type(::PCTAPF_Loader) = PC_TAPF
#
# """
#     PCMAPF_Loader
#
# Helper cache for loading `PCMAPF` problems
# """
# struct PCMAPF_Loader
#     envs::Dict{String,GridFactoryEnvironment}
#     prob_specs::Dict{String,ProblemSpec}
#     PCMAPF_Loader() = new(
#         Dict{String,GridFactoryEnvironment}(),
#         Dict{String,ProblemSpec}()
#     )
# end
# problem_type(::PCMAPF_Loader) = PC_MAPF
#
# """
#     PCTA_Loader
#
# Helper cache for loading `PCTA` (assignment) problems
# """
# struct PCTA_Loader
#     envs::Dict{String,GridFactoryEnvironment}
#     prob_specs::Dict{String,ProblemSpec}
#     PCTA_Loader() = new(
#         Dict{String,GridFactoryEnvironment}(),
#         Dict{String,ProblemSpec}()
#     )
# end
# problem_type(::PCTA_Loader) = PC_TA
#
# """
#     ReplanningProblemLoader
#
# Helper cache for loading replanning problems
# """
# struct ReplanningProblemLoader
#     envs::Dict{String,GridFactoryEnvironment}
#     prob_specs::Dict{String,ProblemSpec}
#     ReplanningProblemLoader() = new(
#         Dict{String,GridFactoryEnvironment}(),
#         Dict{String,ProblemSpec}()
#     )
# end

"""
Currently only impemented for PC_TAPF and PC_TA
"""
function CRCBS.load_problem(loader::TaskGraphsProblemLoader,solver,prob_file)
    toml_dict = TOML.parse(prob_file)
    def = read_problem_def(toml_dict)
    env_id = toml_dict["env_id"]
    env = get_env!(loader,env_id)
    problem_type(loader)(solver,def,env)
end
