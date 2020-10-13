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
