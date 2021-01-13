using TaskGraphs
using Documenter

makedocs(;
    modules=[TaskGraphs],
    authors="kylebrown <kylejbrown17@gmail.com> and contributors",
    repo="https://github.com/kylejbrown17/TaskGraphs.jl/blob/{commit}{path}#L{line}",
    sitename="TaskGraphs.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://kylejbrown17.github.io/TaskGraphs.jl",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
    ],
)

deploydocs(;
    repo="github.com/kylejbrown17/TaskGraphs.jl",
)
