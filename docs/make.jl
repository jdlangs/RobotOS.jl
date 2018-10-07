using Documenter,RobotOS

makedocs(
    modules=[RobotOS],
    authors="Josh Langsfeld",
    )

deploydocs(
    target="site",
    repo="github.com/jdlangs/RobotOS.jl",
    branch = "gh-pages",
    latest = "master",
    osname="linux",
    julia="1.0",
    deps=Deps.pip("mkdocs"),
    )
