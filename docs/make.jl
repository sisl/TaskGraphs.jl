using TaskGraphs
using Documenter

makedocs(;
    modules=[TaskGraphs],
    repo="https://github.com/sisl/TaskGraphs.jl",
    sitename="TaskGraphs.jl",
    format=Documenter.HTML(),
    pages=[
        "Home" => "index.md",
        "Getting Started" => "getting_started.md",
        "Profiling" => "profiling.md",
        "Core Types and Methods" => "library.md",
        "API Reference" => "reference.md",
    ],
)

deploydocs(;
    repo="github.com/sisl/TaskGraphs.jl",
    devbranch = "master",
    devurl = "latest",
)
