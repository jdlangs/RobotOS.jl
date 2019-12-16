using Clang
using PyCall
import LightXML

function ament_pkg_prefix(pkg)
    ament_index_python = pyimport("ament_index_python")
    prefix = try
        ament_index_python.get_package_prefix(pkg)
    catch ex
        if ex isa PyCall.PyError && ex.T == ament_index_python.PackageNotFoundError
            throw(ErrorException("Package '$pkg' not found! Check your environment sourcing and/or AMENT_PREFIX_PATH environment variable."))
        else
            rethrow(ex)
        end
    end
    prefix
end

function ament_pkg_include_dir(pkg)
    joinpath(ament_pkg_prefix(pkg), "include", pkg)
end

function ros_pkg_dependencies(pkg)
    pkgxml = LightXML.parse_file(joinpath(ament_pkg_prefix(pkg), "share", pkg, "package.xml"))
    xmlroot = LightXML.root(pkgxml)
    dep_elems = [xmlroot["depend"]; xmlroot["build_depend"]]
    [LightXML.content(elem) for elem in dep_elems]
end

function wrap_rospkg(pkg)
    @info "Generating wrapper for ROS package '$pkg'"
    include_dir = ament_pkg_include_dir(pkg)
    headers = [joinpath(include_dir, header) for header in readdir(include_dir) if endswith(header, ".h")]

    pkgdeps = ros_pkg_dependencies(pkg)
    dep_include_dirs = [ament_pkg_include_dir(pkg) for dep in pkgdeps]

    # create a work context
    ctx = DefaultContext()

    # parse headers
    parse_headers!(ctx, headers,
                   includes=[CLANG_INCLUDE; dep_include_dirs],
                  )

    # settings
    ctx.libname = "lib" * pkg
    ctx.options["is_function_strictly_typed"] = true
    ctx.options["is_struct_mutable"] = false

    # write output
    api_file = joinpath(@__DIR__, "lib$(pkg)_api.jl")
    api_stream = open(api_file, "w")

    for trans_unit in ctx.trans_units
        root_cursor = getcursor(trans_unit)
        push!(ctx.cursor_stack, root_cursor)
        header = spelling(root_cursor)
        @info "    wrapping header: $header ..."
        # loop over all of the child cursors and wrap them, if appropriate.
        ctx.children = children(root_cursor)
        for (i, child) in enumerate(ctx.children)
            child_name = name(child)
            child_header = filename(child)
            ctx.children_index = i
            # choose which cursor to wrap
            if startswith(child_name, "__")
                continue  # skip compiler definitions
            end
            if child_name in keys(ctx.common_buffer)
                continue  # already wrapped
            end
            if child_header != header
                continue  # skip if cursor filename is not in the headers to be wrapped
            end

            wrap!(ctx, child)
        end
        @info "    writing $(api_file)"
        println(api_stream, "# Julia wrapper for header: $(basename(header))")
        println(api_stream, "# Automatically generated using Clang.jl\n")
        print_buffer(api_stream, ctx.api_buffer)
        empty!(ctx.api_buffer)  # clean up api_buffer for the next header
    end
    close(api_stream)

    # write "common" definitions: types, typealiases, etc.
    common_file = joinpath(@__DIR__, "lib$(pkg)_common.jl")
    @info "    writing $(common_file)"
    open(common_file, "w") do f
        println(f, "# Automatically generated using Clang.jl\n")
        print_buffer(f, dump_to_buffer(ctx.common_buffer))
    end
end
