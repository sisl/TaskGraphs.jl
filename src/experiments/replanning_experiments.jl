
export
    ReplanningProblemLoader,
    add_env!,
    get_env!

"""
    ReplanningProblemLoader

Helper cache for loading replanning problems
"""
struct ReplanningProblemLoader
    envs::Dict{String,GridFactoryEnvironment}
    prob_specs::Dict{String,ProblemSpec}
    ReplanningProblemLoader() = new(
        Dict{String,GridFactoryEnvironment}(),
        Dict{String,ProblemSpec}()
    )
end
function add_env!(loader::ReplanningProblemLoader,env_id::String,
    env=read_env(env_id))
    if haskey(loader.envs,env_id)
    else
        loader.envs[env_id] = env
        loader.prob_specs[env_id] = ProblemSpec(graph=env)
    end
    loader
end
function get_env!(loader::ReplanningProblemLoader,env_id::String)
    if !haskey(loader.envs,env_id)
        loader.envs[env_id] = read_env(env_id)
    end
    return loader.envs[env_id]
end

export SimpleReplanningRequest

"""
    SimpleReplanningRequest

Intermediate representation of a `ProjectRequest` (useful for I/O)
"""
struct SimpleReplanningRequest
    def::SimpleProblemDef
    t_request::Int
    t_arrival::Int
end
function TOML.parse(def::SimpleReplanningRequest)
    toml_dict = TOML.parse(def.def)
    toml_dict["t_request"] = def.t_request
    toml_dict["t_arrival"] = def.t_arrival
    toml_dict
end

export read_simple_request
function read_simple_request(toml_dict)
    SimpleReplanningRequest(
        read_problem_def(toml_dict),
        toml_dict["t_request"],
        toml_dict["t_arrival"]
    )
end
read_simple_request(path::String) = read_simple_request(TOML.parse(path))

function ProjectRequest(def::SimpleReplanningRequest,prob_spec)
    ProjectRequest(
        schedule = construct_partial_project_schedule(
            def.def.project_spec,prob_spec),
        t_request = def.t_request,
        t_arrival = def.t_arrival
        )
end

export SimpleRepeatedProblemDef

"""
    SimpleRepeatedProblemDef

Intermediate representation of a `RepeatedPC_TAPF` (useful for I/O)
"""
@with_kw struct SimpleRepeatedProblemDef
    requests::Vector{SimpleReplanningRequest} = Vector{SimpleReplanningRequest}()
    r0::Vector{Int} = Int[]
    env_id::String = ""
end

export write_simple_repeated_problem_def
# problem_path/
#   config.toml - # robot initial positions, env_id
#   stages/
#       stage1/
#           problem.toml
#       ...
#       stageN/
#           problem.toml
function write_simple_repeated_problem_def(path,def::SimpleRepeatedProblemDef)
    toml_dict = Dict("r0" => def.r0,"env_id" => def.env_id)
    mkpath(path)
    open(joinpath(path,"def.toml"),"w") do io
        TOML.print(io,toml_dict)
    end
    for (i,stage_def) in enumerate(def.requests)
        p = joinpath(path,@sprintf("stage%3.3i",i))
        mkpath(p)
        toml_dict = TOML.parse(stage_def)
        open(joinpath(p,"problem.toml"),"w") do io
            TOML.print(io,toml_dict)
        end
    end
    return true
end

export read_simple_repeated_problem_def
function read_simple_repeated_problem_def(path)
    toml_dict = TOML.parsefile(joinpath(path,"def.toml"))
    prob_def = SimpleRepeatedProblemDef(
        SimpleReplanningRequest[],
        toml_dict["r0"],
        toml_dict["env_id"],
    )
    for p in readdir(path;join=true,sort=true)
        isdir(p) ? nothing : continue
        prob_dict = TOML.parsefile(joinpath(p,"problem.toml"))
        push!(prob_def.requests,read_simple_request(prob_dict))
    end
    prob_def
end
function RepeatedPC_TAPF(simple_def::SimpleRepeatedProblemDef,solver,loader::ReplanningProblemLoader)
    env         = loader.envs[simple_def.env_id]
    prob_spec   = ProblemSpec(loader.prob_specs[simple_def.env_id],
        r0=simple_def.r0,N=length(simple_def.r0))
    sched = construct_partial_project_schedule(
        [ROBOT_AT(r,x) for (r,x) in enumerate(simple_def.r0)],prob_spec)
    search_env = construct_search_env(solver,sched,prob_spec,env)
    r_pc_tapf = RepeatedPC_TAPF(
        search_env,
        map(req->ProjectRequest(req,prob_spec), simple_def.requests)
    )
end

"""
    instantiate_random_repeated_pctapf_problem(env,config)

Instantiate a random `RepeatedPC_TAPF` problem based on the parameters of config.
"""
function random_pctapf_sequence(env::GridFactoryEnvironment,config;
        num_projects        = get(config,:num_projects,10),
        kwargs...)
    projects = map(i->random_pctapf_def(env,config;kwargs...), 1:num_projects)
end

function random_repeated_pctapf_def(env,config;
        arrival_interval    = get(config,:arrival_interval,40),
        warning_time        = get(config,:warning_time,0),
        N                   = get(config,:N,0),
        env_id              = get(config,:env_id,""),
        kwargs...)
    projects = random_pctapf_sequence(env,config;N=0,kwargs...)
    requests = map(i->SimpleReplanningRequest(
        projects[i],
        i*arrival_interval, # request
        i*arrival_interval + warning_time # arrival time
        ), 1:length(projects))
    r0,_,_ = get_random_problem_instantiation(N,0,[],[],get_free_zones(env))
    prob_def = SimpleRepeatedProblemDef(
        requests = requests,
        r0 = r0,
        env_id = env_id
        )
end

export write_repeated_pctapf_problems!

padded_problem_name(number,name="problem",ext=".toml") = @sprintf("%s%4.4i%s",name,number,ext)
function write_repeated_pctapf_problems!(loader::ReplanningProblemLoader,config::Dict,base_path::String,prob_counter::Int=1)
    env = get_env!(loader,config[:env_id])
    num_trials = get(config,:num_trials,1)
    for i in 1:num_trials
        prob_name = padded_problem_name(prob_counter,"problem","")
        prob_path = joinpath(base_path,prob_name)
        mkpath(prob_path)
        open(joinpath(prob_path,"config.toml"),"w") do io
            TOML.print(io,config)
        end
        prob_def = random_repeated_pctapf_def(env,config)
        write_simple_repeated_problem_def(prob_path,prob_def)
        prob_counter += 1
    end
    return prob_counter
end
function write_repeated_pctapf_problems!(loader::ReplanningProblemLoader,configs::Vector{D},base_path::String,prob_counter::Int=1) where {D<:Dict}
    for config in configs
        prob_counter = write_repeated_pctapf_problems!(loader,config,base_path,prob_counter)
    end
    return prob_counter
end

export compile_replanning_results!

function compile_replanning_results!(
        cache::ReplanningProfilerCache,solver,env,
        timer_results,prob,stage,request)
    push!(cache.schedules,deepcopy(env.schedule))
    # store project ids
    for v in vertices(request.schedule)
        id = get_vtx_id(request.schedule,v)
        cache.project_ids[id] = stage
    end
    # store results
    push!(cache.stage_results, compile_results(solver,cache.features,prob,env,timer_results))
    # store final results
    if stage == length(prob.requests)
        merge!(cache.final_results, compile_results(solver,cache.final_features,prob,env,timer_results))
    end
    return cache
end

export write_replanning_results

function write_replanning_results(cache::ReplanningProfilerCache,results_path)
    mkpath(results_path)
    for (i,results) in enumerate(cache.stage_results)
        path = joinpath(results_path,padded_problem_name(i,"stage"))
        open(path,"w") do io
            TOML.print(io,results)
        end
    end
    open(joinpath(results_path,"final_results.toml"),"w") do io
        TOML.print(io,cache.final_results)
    end
    return results_path
end

function write_replanning_results(
    loader::ReplanningProblemLoader,
    planner::ReplannerWithBackup,
    results_path,
    primary_prefix="primary_planner",
    backup_prefix="backup_planner",
    )
    write_replanning_results(planner.primary_planner.cache,
        joinpath(results_path,primary_prefix))
    write_replanning_results(planner.backup_planner.cache,
        joinpath(results_path,backup_prefix))
    return results_path
end

export profile_replanner!

function is_problem_file(path)
    if isdir(path) # each replanning problem is in a directory
        prob_name = splitdir(path)[end]
        if !isnothing(findfirst("problem",prob_name))
            return true
        end
    end
    return false
end
function profile_replanner!(
    loader::ReplanningProblemLoader,
    planner::ReplannerWithBackup,
    base_problem_dir,
    base_results_dir,
    )
    for f in readdir(base_problem_dir;join=true)
        if is_problem_file(f)
            problem_name = splitdir(f)[end]
            outpath = joinpath(base_results_dir,problem_name)
            ispath(outpath) ? continue : nothing # skip over this if results already exist
            @log_info(-1,0,"PROFILING: testing on problem ",problem_name)
            simple_prob_def = read_simple_repeated_problem_def(f)
            prob = RepeatedPC_TAPF(simple_prob_def,planner.primary_planner.solver,loader)
            search_env, planner = profile_replanner!(planner,prob)
            @log_info(-1,0,"PROFILING: writing results for problem ",problem_name)
            write_replanning_results(loader,planner,outpath)
        end
    end
end

function profile_replanner!(planner::FullReplanner,prob::RepeatedAbstractPC_TAPF)
    profile_replanner!(planner.solver,planner.replanner,prob,planner.cache)
end
function profile_replanner!(solver,replan_model,prob::RepeatedAbstractPC_TAPF,
        cache = ReplanningProfilerCache()
    )
    env = prob.env
    for (stage,request) in enumerate(prob.requests)
        @log_info(1,solver,"REPLANNING: Stage ",stage)
        remap_object_ids!(request.schedule,env.schedule)
        base_env = replan!(solver,replan_model,env,request)
        reset_solver!(solver)
        # env, cost = solve!(solver,base_env)
        env, timer_results = profile_solver!(solver,construct_routing_problem(prob,base_env))
        compile_replanning_results!(cache,solver,env,timer_results,prob,stage,request)
        @log_info(1,solver,"Stage ",stage," - ","route planner iterations: ",
            iterations(route_planner(solver)))
    end
    return env, cache
end

function profile_replanner!(planner::ReplannerWithBackup,prob::RepeatedAbstractPC_TAPF)
    reset_cache!(planner)
    plannerA = planner.primary_planner
    plannerB = planner.backup_planner
    env = prob.env
    for (stage,request) in enumerate(prob.requests)
        remap_object_ids!(request.schedule,env.schedule)

        base_envB = replan!(plannerB,env,request)
        envB, resultsB = profile_solver!(plannerB.solver,construct_routing_problem(prob,base_envB))
        compile_replanning_results!(plannerB.cache,plannerB.solver,envB,
            resultsB,prob,stage,request)

        base_envA = replan!(plannerA,env,request)
        envA, resultsA = profile_solver!(plannerA.solver,construct_routing_problem(prob,base_envA))
        compile_replanning_results!(plannerA.cache,plannerA.solver,envA,
            resultsA,prob,stage,request)

        # if failed_status(plannerA) == false
        if feasible_status(plannerA)
            @log_info(-1,0,"REPLANNING: ",
            "Primary planner succeeded at stage $stage.")
            env = envA
        elseif feasible_status(plannerB)
            @log_info(-1,0,"REPLANNING: ",
            "Primary planner failed at stage $stage. Proceeding with backup plan.")
            env = envB
        else
            @log_info(-1,0,"REPLANNING:",
                "Both primary and backup planners failed at stage $stage.",
                " Returning early.")
            break
        end
    end
    return env, planner
end

export warmup

function warmup(planner::ReplannerWithBackup,loader::ReplanningProblemLoader,
        base_dir            = joinpath("/tmp","warmup"),
        base_problem_dir    = joinpath(base_dir,"problem_instances"),
        base_results_dir    = joinpath(base_dir,"results"),
        config = Dict(
            :N => 10,
            :M => 5,
            :num_projects => 3,
            :env_id => sort(collect(keys(loader.envs)))[1],
            ),
    )
    Random.seed!(0)
    write_repeated_pctapf_problems!(loader,config,base_problem_dir)
    env = get_env!(loader,config[:env_id])
    # turn off time constraints for warm up
    set_real_time_flag!(planner,false)
    profile_replanner!(loader,planner,base_problem_dir,base_results_dir)
    # turn real time constraints back on
    set_real_time_flag!(planner,true)
    run(`rm -rf $base_dir`) # remove dummy warmup folder
    reset_cache!(planner)
    return planner
end

export product_config_dicts

function product_config_dicts(configs...)
    map(dicts->merge(dicts...),Base.Iterators.product(configs...))[:]
end

export
    replanning_config_1,
    replanning_config_2,
    replanning_config_3

"""
    replanning_config_1

Returns a vector of config dictionaries, which can be used to generate random
problem instances for profiling.
"""
function replanning_config_1()
    base_configs = [
        Dict(
            :warning_time=>20,
            :commit_threshold=>10,
            :fallback_commit_threshold=>10,
            :num_trials => 4,
            :max_parents => 3,
            :depth_bias => 0.4,
            :dt_min => 0,
            :dt_max => 0,
            :dt_collect => 0,
            :dt_deliver => 0,
            # :task_sizes => (1=>1.0,2=>0.0,4=>0.0),
            :env_id=>"env_2",
            )
    ]
    stream_configs = [
        Dict(:N=>30, :M=>10, :num_projects=>10, :arrival_interval=>40, ),
        Dict(:N=>30, :M=>15, :num_projects=>10, :arrival_interval=>50, ),
        Dict(:N=>30, :M=>20, :num_projects=>10, :arrival_interval=>60, ),
        Dict(:N=>30, :M=>25, :num_projects=>10, :arrival_interval=>70, ),
        Dict(:N=>30, :M=>30, :num_projects=>10, :arrival_interval=>80, ),
    ]
    product_config_dicts(base_configs,stream_configs)
end
function replanning_config_2()
    base_configs = [
        Dict(
            :warning_time=>4,
            :commit_threshold=>4,
            :fallback_commit_threshold=>4,
            :num_trials => 1,
            :max_parents => 3,
            :depth_bias => 0.4,
            :dt_min => 0,
            :dt_max => 0,
            :dt_collect => 0,
            :dt_deliver => 0,
            # :task_sizes => (1=>1.0,2=>0.0,4=>0.0),
            :save_paths => false,
            :env_id=>"env_2",
            )
    ]
    stream_configs = [
        Dict(:N=>2, :M=>3, :num_projects=>4, :arrival_interval=>10, ),
    ]
    product_config_dicts(base_configs,stream_configs)
end
function replanning_config_3()
    base_configs = [
        Dict(
            :warning_time=>0,
            :commit_threshold=>10,
            :fallback_commit_threshold=>10,
            :num_trials => 4,
            :max_parents => 3,
            :depth_bias => 0.4,
            :dt_min => 0,
            :dt_max => 0,
            :dt_collect => 0,
            :dt_deliver => 0,
            # :task_sizes => (1=>1.0,2=>0.0,4=>0.0),
            :env_id=>"env_2",
            )
    ]
    robot_configs = [
        Dict(:N=>30)
    ]
    project_configs = [
        Dict(:num_projects=>10),
        Dict(:num_projects=>20),
        Dict(:num_projects=>30),
    ]
    stream_configs = [
        Dict(:M=>10, :arrival_interval=>20, ),
        Dict(:M=>10, :arrival_interval=>30, ),
        Dict(:M=>10, :arrival_interval=>40, ),

        Dict(:M=>15, :arrival_interval=>30, ),
        Dict(:M=>15, :arrival_interval=>40, ),
        Dict(:M=>15, :arrival_interval=>50, ),

        Dict(:M=>20, :arrival_interval=>40, ),
        Dict(:M=>20, :arrival_interval=>50, ),
        Dict(:M=>20, :arrival_interval=>60, ),

        Dict(:M=>25, :arrival_interval=>50, ),
        Dict(:M=>25, :arrival_interval=>60, ),
        Dict(:M=>25, :arrival_interval=>70, ),

        Dict(:M=>30, :arrival_interval=>60, ),
        Dict(:M=>30, :arrival_interval=>70, ),
        Dict(:M=>30, :arrival_interval=>80, ),
    ]
    product_config_dicts(base_configs,robot_configs,project_configs,stream_configs)
end

export setup_replanning_experiments

function setup_replanning_experiments(base_dir)
    loader = ReplanningProblemLoader()
    add_env!(loader,"env_2",init_env_2())
    # get probem config
    problem_configs = replanning_config_1()
    # define paths to problems and results
    base_problem_dir    = joinpath(base_dir,"problem_instances")
    base_results_dir    = joinpath(base_dir,"results")
    # write problems
    Random.seed!(0)
    # Problem generation and profiling
    reset_task_id_counter!()
    reset_operation_id_counter!()
    write_repeated_pctapf_problems!(loader,problem_configs,base_problem_dir)
    # set up planner and profiling features
    feats = [
        RunTime(),IterationCount(),TimeOutStatus(),IterationMaxOutStatus(),
        SolutionCost(),OptimalityGap(),OptimalFlag(),FeasibleFlag(),NumConflicts(),
        ObjectPathSummaries(),
        ]
    final_feats = [SolutionCost(),NumConflicts(),RobotPaths()]
    # Primary planner
    path_finder = DefaultAStarSC()
    set_iteration_limit!(path_finder,5000)
    primary_route_planner = CBSSolver(ISPS(path_finder))
    set_iteration_limit!(primary_route_planner,1000)
    primary_planner = FullReplanner(
        solver = NBSSolver(path_planner=primary_route_planner),
        replanner = MergeAndBalance(),
        cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
        )
    set_verbosity!(primary_planner.solver,0)
    # Backup planner
    backup_planner = FullReplanner(
        solver = NBSSolver(
            assignment_model = TaskGraphsMILPSolver(GreedyAssignment()),
            path_planner = PIBTPlanner{NTuple{3,Float64}}()
            ),
        replanner = MergeAndBalance(),
        cache = ReplanningProfilerCache(features=feats,final_features=final_feats)
        )
    set_iteration_limit!(backup_planner,1)
    set_iteration_limit!(route_planner(backup_planner.solver),5000)
    set_verbosity!(backup_planner.solver,0)
    # Full solver
    planner = ReplannerWithBackup(primary_planner,backup_planner)
    return loader, planner, base_results_dir, base_problem_dir
end
