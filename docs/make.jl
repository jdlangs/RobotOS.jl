using Documenter,RobotOS

makedocs(
    modules=[RobotOS],
    authors="Josh Langsfeld",
    )

deploydocs(
    deps=Deps.pip("mkdocs"),
    repo="github.com/jdlangs/RobotOS.jl",
    branch = "gh-pages",
    latest = "master",
    target="build",
    osname="linux",
    julia="0.6",
    )
