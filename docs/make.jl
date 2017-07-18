using Documenter,RobotOS
makedocs(modules=[RobotOS],
        doctest=false, clean=true,
        format =:html,
        authors="Josh Langsfeld",
        sitename="RobotOS.jl",
        pages = Any[
        "Home" => "index.md"
               ]
               )

deploydocs(
    deps=Deps.pip("mkdocs","python-markdown-math"),
    repo="github.com/jdlangs/RobotOS.jl",
    branch = "gh-pages",
    latest = "master",
    target="build",
    osname="linux",
    julia="0.6",
    make=nothing)
