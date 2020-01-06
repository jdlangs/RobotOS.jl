# Julia wrapper for ROS package 'rcl' API
# Automatically generated using Clang.jl, do not edit manually
# Use `wrap_rospkg` in `clang_wrap.jl` for regeneration

# -----------------------------------------
# Generated wrapper for allocator.h
# -----------------------------------------

# -----------------------
# end allocator.h
# -----------------------

# -----------------------------------------
# Generated wrapper for arguments.h
# -----------------------------------------

function rcl_get_zero_initialized_arguments()
    ccall((:rcl_get_zero_initialized_arguments, LIBRCL), rcl_arguments_t, ())
end

function rcl_parse_arguments(argc::Cint, argv, allocator::rcl_allocator_t, args_output)
    ccall((:rcl_parse_arguments, LIBRCL), rcl_ret_t, (Cint, Ptr{Cstring}, rcl_allocator_t, Ptr{rcl_arguments_t}), argc, argv, allocator, args_output)
end

function rcl_arguments_get_count_unparsed(args)
    ccall((:rcl_arguments_get_count_unparsed, LIBRCL), Cint, (Ptr{rcl_arguments_t},), args)
end

function rcl_arguments_get_unparsed(args, allocator::rcl_allocator_t, output_unparsed_indices)
    ccall((:rcl_arguments_get_unparsed, LIBRCL), rcl_ret_t, (Ptr{rcl_arguments_t}, rcl_allocator_t, Ptr{Ptr{Cint}}), args, allocator, output_unparsed_indices)
end

function rcl_arguments_get_count_unparsed_ros(args)
    ccall((:rcl_arguments_get_count_unparsed_ros, LIBRCL), Cint, (Ptr{rcl_arguments_t},), args)
end

function rcl_arguments_get_unparsed_ros(args, allocator::rcl_allocator_t, output_unparsed_ros_indices)
    ccall((:rcl_arguments_get_unparsed_ros, LIBRCL), rcl_ret_t, (Ptr{rcl_arguments_t}, rcl_allocator_t, Ptr{Ptr{Cint}}), args, allocator, output_unparsed_ros_indices)
end

function rcl_arguments_get_param_files_count(args)
    ccall((:rcl_arguments_get_param_files_count, LIBRCL), Cint, (Ptr{rcl_arguments_t},), args)
end

function rcl_arguments_get_param_files(arguments, allocator::rcl_allocator_t, parameter_files)
    ccall((:rcl_arguments_get_param_files, LIBRCL), rcl_ret_t, (Ptr{rcl_arguments_t}, rcl_allocator_t, Ptr{Ptr{Cstring}}), arguments, allocator, parameter_files)
end

#function rcl_arguments_get_param_overrides(arguments, parameter_overrides)
    #ccall((:rcl_arguments_get_param_overrides, LIBRCL), rcl_ret_t, (Ptr{rcl_arguments_t}, Ptr{Ptr{rcl_params_t}}), arguments, parameter_overrides)
#end

function rcl_remove_ros_arguments(argv, args, allocator::rcl_allocator_t, nonros_argc, nonros_argv)
    ccall((:rcl_remove_ros_arguments, LIBRCL), rcl_ret_t, (Ptr{Cstring}, Ptr{rcl_arguments_t}, rcl_allocator_t, Ptr{Cint}, Ptr{Ptr{Cstring}}), argv, args, allocator, nonros_argc, nonros_argv)
end

function rcl_arguments_copy(args, args_out)
    ccall((:rcl_arguments_copy, LIBRCL), rcl_ret_t, (Ptr{rcl_arguments_t}, Ptr{rcl_arguments_t}), args, args_out)
end

function rcl_arguments_fini(args)
    ccall((:rcl_arguments_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_arguments_t},), args)
end

# -----------------------
# end arguments.h
# -----------------------

# -----------------------------------------
# Generated wrapper for client.h
# -----------------------------------------

function rcl_get_zero_initialized_client()
    ccall((:rcl_get_zero_initialized_client, LIBRCL), rcl_client_t, ())
end

function rcl_client_init(client, node, type_support, service_name, options)
    ccall((:rcl_client_init, LIBRCL), rcl_ret_t, (Ptr{rcl_client_t}, Ptr{rcl_node_t}, Ptr{rosidl_service_type_support_t}, Cstring, Ptr{rcl_client_options_t}), client, node, type_support, service_name, options)
end

function rcl_client_fini(client, node)
    ccall((:rcl_client_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_client_t}, Ptr{rcl_node_t}), client, node)
end

function rcl_client_get_default_options()
    ccall((:rcl_client_get_default_options, LIBRCL), rcl_client_options_t, ())
end

function rcl_send_request(client, ros_request, sequence_number)
    ccall((:rcl_send_request, LIBRCL), rcl_ret_t, (Ptr{rcl_client_t}, Ptr{Cvoid}, Ptr{Int64}), client, ros_request, sequence_number)
end

function rcl_take_response(client, request_header, ros_response)
    ccall((:rcl_take_response, LIBRCL), rcl_ret_t, (Ptr{rcl_client_t}, Ptr{rmw_request_id_t}, Ptr{Cvoid}), client, request_header, ros_response)
end

function rcl_client_get_service_name(client)
    ccall((:rcl_client_get_service_name, LIBRCL), Cstring, (Ptr{rcl_client_t},), client)
end

function rcl_client_get_options(client)
    ccall((:rcl_client_get_options, LIBRCL), Ptr{rcl_client_options_t}, (Ptr{rcl_client_t},), client)
end

function rcl_client_get_rmw_handle(client)
    ccall((:rcl_client_get_rmw_handle, LIBRCL), Ptr{rmw_client_t}, (Ptr{rcl_client_t},), client)
end

function rcl_client_is_valid(client)
    ccall((:rcl_client_is_valid, LIBRCL), Bool, (Ptr{rcl_client_t},), client)
end

# -----------------------
# end client.h
# -----------------------

# -----------------------------------------
# Generated wrapper for context.h
# -----------------------------------------

function rcl_get_zero_initialized_context()
    ccall((:rcl_get_zero_initialized_context, LIBRCL), rcl_context_t, ())
end

function rcl_context_fini(context)
    ccall((:rcl_context_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_context_t},), context)
end

function rcl_context_get_init_options(context)
    ccall((:rcl_context_get_init_options, LIBRCL), Ptr{rcl_init_options_t}, (Ptr{rcl_context_t},), context)
end

function rcl_context_get_instance_id(context)
    ccall((:rcl_context_get_instance_id, LIBRCL), rcl_context_instance_id_t, (Ptr{rcl_context_t},), context)
end

function rcl_context_is_valid(context)
    ccall((:rcl_context_is_valid, LIBRCL), Bool, (Ptr{rcl_context_t},), context)
end

function rcl_context_get_rmw_context(context)
    ccall((:rcl_context_get_rmw_context, LIBRCL), Ptr{rmw_context_t}, (Ptr{rcl_context_t},), context)
end

# -----------------------
# end context.h
# -----------------------

# -----------------------------------------
# Generated wrapper for error_handling.h
# -----------------------------------------

# -----------------------
# end error_handling.h
# -----------------------

# -----------------------------------------
# Generated wrapper for event.h
# -----------------------------------------

function rcl_get_zero_initialized_event()
    ccall((:rcl_get_zero_initialized_event, LIBRCL), rcl_event_t, ())
end

function rcl_publisher_event_init(event, publisher, event_type::rcl_publisher_event_type_t)
    ccall((:rcl_publisher_event_init, LIBRCL), rcl_ret_t, (Ptr{rcl_event_t}, Ptr{rcl_publisher_t}, rcl_publisher_event_type_t), event, publisher, event_type)
end

function rcl_subscription_event_init(event, subscription, event_type::rcl_subscription_event_type_t)
    ccall((:rcl_subscription_event_init, LIBRCL), rcl_ret_t, (Ptr{rcl_event_t}, Ptr{rcl_subscription_t}, rcl_subscription_event_type_t), event, subscription, event_type)
end

function rcl_take_event(event, event_info)
    ccall((:rcl_take_event, LIBRCL), rcl_ret_t, (Ptr{rcl_event_t}, Ptr{Cvoid}), event, event_info)
end

function rcl_event_fini(event)
    ccall((:rcl_event_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_event_t},), event)
end

function rcl_event_get_rmw_handle(event)
    ccall((:rcl_event_get_rmw_handle, LIBRCL), Ptr{rmw_event_t}, (Ptr{rcl_event_t},), event)
end

# -----------------------
# end event.h
# -----------------------

# -----------------------------------------
# Generated wrapper for expand_topic_name.h
# -----------------------------------------

function rcl_expand_topic_name(input_topic_name, node_name, node_namespace, substitutions, allocator::rcl_allocator_t, output_topic_name)
    ccall((:rcl_expand_topic_name, LIBRCL), rcl_ret_t, (Cstring, Cstring, Cstring, Ptr{rcutils_string_map_t}, rcl_allocator_t, Ptr{Cstring}), input_topic_name, node_name, node_namespace, substitutions, allocator, output_topic_name)
end

function rcl_get_default_topic_name_substitutions(string_map)
    ccall((:rcl_get_default_topic_name_substitutions, LIBRCL), rcl_ret_t, (Ptr{rcutils_string_map_t},), string_map)
end

# -----------------------
# end expand_topic_name.h
# -----------------------

# -----------------------------------------
# Generated wrapper for graph.h
# -----------------------------------------

function rcl_get_publisher_names_and_types_by_node(node, allocator, no_demangle::Bool, node_name, node_namespace, topic_names_and_types)
    ccall((:rcl_get_publisher_names_and_types_by_node, LIBRCL), rcl_ret_t, (Ptr{rcl_node_t}, Ptr{rcl_allocator_t}, Bool, Cstring, Cstring, Ptr{rcl_names_and_types_t}), node, allocator, no_demangle, node_name, node_namespace, topic_names_and_types)
end

function rcl_get_subscriber_names_and_types_by_node(node, allocator, no_demangle::Bool, node_name, node_namespace, topic_names_and_types)
    ccall((:rcl_get_subscriber_names_and_types_by_node, LIBRCL), rcl_ret_t, (Ptr{rcl_node_t}, Ptr{rcl_allocator_t}, Bool, Cstring, Cstring, Ptr{rcl_names_and_types_t}), node, allocator, no_demangle, node_name, node_namespace, topic_names_and_types)
end

function rcl_get_service_names_and_types_by_node(node, allocator, node_name, node_namespace, service_names_and_types)
    ccall((:rcl_get_service_names_and_types_by_node, LIBRCL), rcl_ret_t, (Ptr{rcl_node_t}, Ptr{rcl_allocator_t}, Cstring, Cstring, Ptr{rcl_names_and_types_t}), node, allocator, node_name, node_namespace, service_names_and_types)
end

function rcl_get_client_names_and_types_by_node(node, allocator, node_name, node_namespace, service_names_and_types)
    ccall((:rcl_get_client_names_and_types_by_node, LIBRCL), rcl_ret_t, (Ptr{rcl_node_t}, Ptr{rcl_allocator_t}, Cstring, Cstring, Ptr{rcl_names_and_types_t}), node, allocator, node_name, node_namespace, service_names_and_types)
end

function rcl_get_topic_names_and_types(node, allocator, no_demangle::Bool, topic_names_and_types)
    ccall((:rcl_get_topic_names_and_types, LIBRCL), rcl_ret_t, (Ptr{rcl_node_t}, Ptr{rcl_allocator_t}, Bool, Ptr{rcl_names_and_types_t}), node, allocator, no_demangle, topic_names_and_types)
end

function rcl_get_service_names_and_types(node, allocator, service_names_and_types)
    ccall((:rcl_get_service_names_and_types, LIBRCL), rcl_ret_t, (Ptr{rcl_node_t}, Ptr{rcl_allocator_t}, Ptr{rcl_names_and_types_t}), node, allocator, service_names_and_types)
end

function rcl_names_and_types_init(names_and_types, size::Csize_t, allocator)
    ccall((:rcl_names_and_types_init, LIBRCL), rcl_ret_t, (Ptr{rcl_names_and_types_t}, Csize_t, Ptr{rcl_allocator_t}), names_and_types, size, allocator)
end

function rcl_names_and_types_fini(names_and_types)
    ccall((:rcl_names_and_types_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_names_and_types_t},), names_and_types)
end

function rcl_get_node_names(node, allocator::rcl_allocator_t, node_names, node_namespaces)
    ccall((:rcl_get_node_names, LIBRCL), rcl_ret_t, (Ptr{rcl_node_t}, rcl_allocator_t, Ptr{rcutils_string_array_t}, Ptr{rcutils_string_array_t}), node, allocator, node_names, node_namespaces)
end

function rcl_count_publishers(node, topic_name, count)
    ccall((:rcl_count_publishers, LIBRCL), rcl_ret_t, (Ptr{rcl_node_t}, Cstring, Ptr{Csize_t}), node, topic_name, count)
end

function rcl_count_subscribers(node, topic_name, count)
    ccall((:rcl_count_subscribers, LIBRCL), rcl_ret_t, (Ptr{rcl_node_t}, Cstring, Ptr{Csize_t}), node, topic_name, count)
end

function rcl_service_server_is_available(node, client, is_available)
    ccall((:rcl_service_server_is_available, LIBRCL), rcl_ret_t, (Ptr{rcl_node_t}, Ptr{rcl_client_t}, Ptr{Bool}), node, client, is_available)
end

# -----------------------
# end graph.h
# -----------------------

# -----------------------------------------
# Generated wrapper for guard_condition.h
# -----------------------------------------

function rcl_get_zero_initialized_guard_condition()
    ccall((:rcl_get_zero_initialized_guard_condition, LIBRCL), rcl_guard_condition_t, ())
end

function rcl_guard_condition_init(guard_condition, context, options::rcl_guard_condition_options_t)
    ccall((:rcl_guard_condition_init, LIBRCL), rcl_ret_t, (Ptr{rcl_guard_condition_t}, Ptr{rcl_context_t}, rcl_guard_condition_options_t), guard_condition, context, options)
end

function rcl_guard_condition_init_from_rmw(guard_condition, rmw_guard_condition, context, options::rcl_guard_condition_options_t)
    ccall((:rcl_guard_condition_init_from_rmw, LIBRCL), rcl_ret_t, (Ptr{rcl_guard_condition_t}, Ptr{rmw_guard_condition_t}, Ptr{rcl_context_t}, rcl_guard_condition_options_t), guard_condition, rmw_guard_condition, context, options)
end

function rcl_guard_condition_fini(guard_condition)
    ccall((:rcl_guard_condition_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_guard_condition_t},), guard_condition)
end

function rcl_guard_condition_get_default_options()
    ccall((:rcl_guard_condition_get_default_options, LIBRCL), rcl_guard_condition_options_t, ())
end

function rcl_trigger_guard_condition(guard_condition)
    ccall((:rcl_trigger_guard_condition, LIBRCL), rcl_ret_t, (Ptr{rcl_guard_condition_t},), guard_condition)
end

function rcl_guard_condition_get_options(guard_condition)
    ccall((:rcl_guard_condition_get_options, LIBRCL), Ptr{rcl_guard_condition_options_t}, (Ptr{rcl_guard_condition_t},), guard_condition)
end

function rcl_guard_condition_get_rmw_handle(guard_condition)
    ccall((:rcl_guard_condition_get_rmw_handle, LIBRCL), Ptr{rmw_guard_condition_t}, (Ptr{rcl_guard_condition_t},), guard_condition)
end

# -----------------------
# end guard_condition.h
# -----------------------

# -----------------------------------------
# Generated wrapper for init.h
# -----------------------------------------

function rcl_init(argc::Cint, argv, options, context)
    ccall((:rcl_init, LIBRCL), rcl_ret_t, (Cint, Ptr{Cstring}, Ptr{rcl_init_options_t}, Ptr{rcl_context_t}), argc, argv, options, context)
end

function rcl_shutdown(context)
    ccall((:rcl_shutdown, LIBRCL), rcl_ret_t, (Ptr{rcl_context_t},), context)
end

# -----------------------
# end init.h
# -----------------------

# -----------------------------------------
# Generated wrapper for init_options.h
# -----------------------------------------

function rcl_get_zero_initialized_init_options()
    ccall((:rcl_get_zero_initialized_init_options, LIBRCL), rcl_init_options_t, ())
end

function rcl_init_options_init(init_options, allocator::rcl_allocator_t)
    ccall((:rcl_init_options_init, LIBRCL), rcl_ret_t, (Ptr{rcl_init_options_t}, rcl_allocator_t), init_options, allocator)
end

function rcl_init_options_copy(src, dst)
    ccall((:rcl_init_options_copy, LIBRCL), rcl_ret_t, (Ptr{rcl_init_options_t}, Ptr{rcl_init_options_t}), src, dst)
end

function rcl_init_options_fini(init_options)
    ccall((:rcl_init_options_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_init_options_t},), init_options)
end

function rcl_init_options_get_rmw_init_options(init_options)
    ccall((:rcl_init_options_get_rmw_init_options, LIBRCL), Ptr{rmw_init_options_t}, (Ptr{rcl_init_options_t},), init_options)
end

# -----------------------
# end init_options.h
# -----------------------

# -----------------------------------------
# Generated wrapper for lexer.h
# -----------------------------------------

function rcl_lexer_analyze(text, lexeme, length)
    ccall((:rcl_lexer_analyze, LIBRCL), rcl_ret_t, (Cstring, Ptr{rcl_lexeme_t}, Ptr{Csize_t}), text, lexeme, length)
end

# -----------------------
# end lexer.h
# -----------------------

# -----------------------------------------
# Generated wrapper for lexer_lookahead.h
# -----------------------------------------

function rcl_get_zero_initialized_lexer_lookahead2()
    ccall((:rcl_get_zero_initialized_lexer_lookahead2, LIBRCL), rcl_lexer_lookahead2_t, ())
end

function rcl_lexer_lookahead2_init(buffer, text, allocator::rcl_allocator_t)
    ccall((:rcl_lexer_lookahead2_init, LIBRCL), rcl_ret_t, (Ptr{rcl_lexer_lookahead2_t}, Cstring, rcl_allocator_t), buffer, text, allocator)
end

function rcl_lexer_lookahead2_fini(buffer)
    ccall((:rcl_lexer_lookahead2_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_lexer_lookahead2_t},), buffer)
end

function rcl_lexer_lookahead2_peek(buffer, next_type)
    ccall((:rcl_lexer_lookahead2_peek, LIBRCL), rcl_ret_t, (Ptr{rcl_lexer_lookahead2_t}, Ptr{rcl_lexeme_t}), buffer, next_type)
end

function rcl_lexer_lookahead2_peek2(buffer, next_type1, next_type2)
    ccall((:rcl_lexer_lookahead2_peek2, LIBRCL), rcl_ret_t, (Ptr{rcl_lexer_lookahead2_t}, Ptr{rcl_lexeme_t}, Ptr{rcl_lexeme_t}), buffer, next_type1, next_type2)
end

function rcl_lexer_lookahead2_accept(buffer, lexeme_text, lexeme_text_length)
    ccall((:rcl_lexer_lookahead2_accept, LIBRCL), rcl_ret_t, (Ptr{rcl_lexer_lookahead2_t}, Ptr{Cstring}, Ptr{Csize_t}), buffer, lexeme_text, lexeme_text_length)
end

function rcl_lexer_lookahead2_expect(buffer, type::rcl_lexeme_t, lexeme_text, lexeme_text_length)
    ccall((:rcl_lexer_lookahead2_expect, LIBRCL), rcl_ret_t, (Ptr{rcl_lexer_lookahead2_t}, rcl_lexeme_t, Ptr{Cstring}, Ptr{Csize_t}), buffer, type, lexeme_text, lexeme_text_length)
end

function rcl_lexer_lookahead2_get_text(buffer)
    ccall((:rcl_lexer_lookahead2_get_text, LIBRCL), Cstring, (Ptr{rcl_lexer_lookahead2_t},), buffer)
end

# -----------------------
# end lexer_lookahead.h
# -----------------------

# -----------------------------------------
# Generated wrapper for localhost.h
# -----------------------------------------

function rcl_localhost_only()
    ccall((:rcl_localhost_only, LIBRCL), Bool, ())
end

# -----------------------
# end localhost.h
# -----------------------

# -----------------------------------------
# Generated wrapper for logging.h
# -----------------------------------------

function rcl_logging_configure(global_args, allocator)
    ccall((:rcl_logging_configure, LIBRCL), rcl_ret_t, (Ptr{rcl_arguments_t}, Ptr{rcl_allocator_t}), global_args, allocator)
end

function rcl_logging_fini()
    ccall((:rcl_logging_fini, LIBRCL), rcl_ret_t, ())
end

function rcl_logging_rosout_enabled()
    ccall((:rcl_logging_rosout_enabled, LIBRCL), Bool, ())
end

# -----------------------
# end logging.h
# -----------------------

# -----------------------------------------
# Generated wrapper for logging_external_interface.h
# -----------------------------------------

function rcl_logging_external_initialize(config_file, allocator::rcutils_allocator_t)
    ccall((:rcl_logging_external_initialize, LIBRCL), rcl_ret_t, (Cstring, rcutils_allocator_t), config_file, allocator)
end

function rcl_logging_external_shutdown()
    ccall((:rcl_logging_external_shutdown, LIBRCL), rcl_ret_t, ())
end

function rcl_logging_external_log(severity::Cint, name, msg)
    ccall((:rcl_logging_external_log, LIBRCL), Cvoid, (Cint, Cstring, Cstring), severity, name, msg)
end

function rcl_logging_external_set_logger_level(name, level::Cint)
    ccall((:rcl_logging_external_set_logger_level, LIBRCL), rcl_ret_t, (Cstring, Cint), name, level)
end

# -----------------------
# end logging_external_interface.h
# -----------------------

# -----------------------------------------
# Generated wrapper for logging_rosout.h
# -----------------------------------------

function rcl_logging_rosout_init(allocator)
    ccall((:rcl_logging_rosout_init, LIBRCL), rcl_ret_t, (Ptr{rcl_allocator_t},), allocator)
end

function rcl_logging_rosout_fini()
    ccall((:rcl_logging_rosout_fini, LIBRCL), rcl_ret_t, ())
end

function rcl_logging_rosout_init_publisher_for_node(node)
    ccall((:rcl_logging_rosout_init_publisher_for_node, LIBRCL), rcl_ret_t, (Ptr{rcl_node_t},), node)
end

function rcl_logging_rosout_fini_publisher_for_node(node)
    ccall((:rcl_logging_rosout_fini_publisher_for_node, LIBRCL), rcl_ret_t, (Ptr{rcl_node_t},), node)
end

function rcl_logging_rosout_output_handler(location, severity::Cint, name, timestamp::rcutils_time_point_value_t, format, args)
    ccall((:rcl_logging_rosout_output_handler, LIBRCL), Cvoid, (Ptr{rcutils_log_location_t}, Cint, Cstring, rcutils_time_point_value_t, Cstring, Ptr{va_list}), location, severity, name, timestamp, format, args)
end

# -----------------------
# end logging_rosout.h
# -----------------------

# -----------------------------------------
# Generated wrapper for macros.h
# -----------------------------------------

# -----------------------
# end macros.h
# -----------------------

# -----------------------------------------
# Generated wrapper for node.h
# -----------------------------------------

function rcl_get_zero_initialized_node()
    ccall((:rcl_get_zero_initialized_node, LIBRCL), rcl_node_t, ())
end

function rcl_node_init(node, name, namespace_, context, options)
    ccall((:rcl_node_init, LIBRCL), rcl_ret_t, (Ptr{rcl_node_t}, Cstring, Cstring, Ptr{rcl_context_t}, Ptr{rcl_node_options_t}), node, name, namespace_, context, options)
end

function rcl_node_fini(node)
    ccall((:rcl_node_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_node_t},), node)
end

function rcl_node_is_valid(node)
    ccall((:rcl_node_is_valid, LIBRCL), Bool, (Ptr{rcl_node_t},), node)
end

function rcl_node_is_valid_except_context(node)
    ccall((:rcl_node_is_valid_except_context, LIBRCL), Bool, (Ptr{rcl_node_t},), node)
end

function rcl_node_get_name(node)
    ccall((:rcl_node_get_name, LIBRCL), Cstring, (Ptr{rcl_node_t},), node)
end

function rcl_node_get_namespace(node)
    ccall((:rcl_node_get_namespace, LIBRCL), Cstring, (Ptr{rcl_node_t},), node)
end

function rcl_node_get_fully_qualified_name(node)
    ccall((:rcl_node_get_fully_qualified_name, LIBRCL), Cstring, (Ptr{rcl_node_t},), node)
end

function rcl_node_get_options(node)
    ccall((:rcl_node_get_options, LIBRCL), Ptr{rcl_node_options_t}, (Ptr{rcl_node_t},), node)
end

function rcl_node_get_domain_id(node, domain_id)
    ccall((:rcl_node_get_domain_id, LIBRCL), rcl_ret_t, (Ptr{rcl_node_t}, Ptr{Csize_t}), node, domain_id)
end

function rcl_node_assert_liveliness(node)
    ccall((:rcl_node_assert_liveliness, LIBRCL), rcl_ret_t, (Ptr{rcl_node_t},), node)
end

function rcl_node_get_rmw_handle(node)
    ccall((:rcl_node_get_rmw_handle, LIBRCL), Ptr{rmw_node_t}, (Ptr{rcl_node_t},), node)
end

function rcl_node_get_rcl_instance_id(node)
    ccall((:rcl_node_get_rcl_instance_id, LIBRCL), UInt64, (Ptr{rcl_node_t},), node)
end

function rcl_node_get_graph_guard_condition(node)
    ccall((:rcl_node_get_graph_guard_condition, LIBRCL), Ptr{rcl_guard_condition_t}, (Ptr{rcl_node_t},), node)
end

function rcl_node_get_logger_name(node)
    ccall((:rcl_node_get_logger_name, LIBRCL), Cstring, (Ptr{rcl_node_t},), node)
end

# -----------------------
# end node.h
# -----------------------

# -----------------------------------------
# Generated wrapper for node_options.h
# -----------------------------------------

function rcl_node_get_default_options()
    ccall((:rcl_node_get_default_options, LIBRCL), rcl_node_options_t, ())
end

function rcl_node_options_copy(options, options_out)
    ccall((:rcl_node_options_copy, LIBRCL), rcl_ret_t, (Ptr{rcl_node_options_t}, Ptr{rcl_node_options_t}), options, options_out)
end

function rcl_node_options_fini(options)
    ccall((:rcl_node_options_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_node_options_t},), options)
end

# -----------------------
# end node_options.h
# -----------------------

# -----------------------------------------
# Generated wrapper for publisher.h
# -----------------------------------------

function rcl_get_zero_initialized_publisher()
    ccall((:rcl_get_zero_initialized_publisher, LIBRCL), rcl_publisher_t, ())
end

function rcl_publisher_init(publisher, node, type_support, topic_name, options)
    ccall((:rcl_publisher_init, LIBRCL), rcl_ret_t, (Ptr{rcl_publisher_t}, Ptr{rcl_node_t}, Ptr{rosidl_message_type_support_t}, Cstring, Ptr{rcl_publisher_options_t}), publisher, node, type_support, topic_name, options)
end

function rcl_publisher_fini(publisher, node)
    ccall((:rcl_publisher_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_publisher_t}, Ptr{rcl_node_t}), publisher, node)
end

function rcl_publisher_get_default_options()
    ccall((:rcl_publisher_get_default_options, LIBRCL), rcl_publisher_options_t, ())
end

function rcl_borrow_loaned_message(publisher, type_support, ros_message)
    ccall((:rcl_borrow_loaned_message, LIBRCL), rcl_ret_t, (Ptr{rcl_publisher_t}, Ptr{rosidl_message_type_support_t}, Ptr{Ptr{Cvoid}}), publisher, type_support, ros_message)
end

function rcl_return_loaned_message_from_publisher(publisher, loaned_message)
    ccall((:rcl_return_loaned_message_from_publisher, LIBRCL), rcl_ret_t, (Ptr{rcl_publisher_t}, Ptr{Cvoid}), publisher, loaned_message)
end

function rcl_publish(publisher, ros_message, allocation)
    ccall((:rcl_publish, LIBRCL), rcl_ret_t, (Ptr{rcl_publisher_t}, Ptr{Cvoid}, Ptr{rmw_publisher_allocation_t}), publisher, ros_message, allocation)
end

function rcl_publish_serialized_message(publisher, serialized_message, allocation)
    ccall((:rcl_publish_serialized_message, LIBRCL), rcl_ret_t, (Ptr{rcl_publisher_t}, Ptr{rcl_serialized_message_t}, Ptr{rmw_publisher_allocation_t}), publisher, serialized_message, allocation)
end

function rcl_publish_loaned_message(publisher, ros_message, allocation)
    ccall((:rcl_publish_loaned_message, LIBRCL), rcl_ret_t, (Ptr{rcl_publisher_t}, Ptr{Cvoid}, Ptr{rmw_publisher_allocation_t}), publisher, ros_message, allocation)
end

function rcl_publisher_assert_liveliness(publisher)
    ccall((:rcl_publisher_assert_liveliness, LIBRCL), rcl_ret_t, (Ptr{rcl_publisher_t},), publisher)
end

function rcl_publisher_get_topic_name(publisher)
    ccall((:rcl_publisher_get_topic_name, LIBRCL), Cstring, (Ptr{rcl_publisher_t},), publisher)
end

function rcl_publisher_get_options(publisher)
    ccall((:rcl_publisher_get_options, LIBRCL), Ptr{rcl_publisher_options_t}, (Ptr{rcl_publisher_t},), publisher)
end

function rcl_publisher_get_rmw_handle(publisher)
    ccall((:rcl_publisher_get_rmw_handle, LIBRCL), Ptr{rmw_publisher_t}, (Ptr{rcl_publisher_t},), publisher)
end

function rcl_publisher_get_context(publisher)
    ccall((:rcl_publisher_get_context, LIBRCL), Ptr{rcl_context_t}, (Ptr{rcl_publisher_t},), publisher)
end

function rcl_publisher_is_valid(publisher)
    ccall((:rcl_publisher_is_valid, LIBRCL), Bool, (Ptr{rcl_publisher_t},), publisher)
end

function rcl_publisher_is_valid_except_context(publisher)
    ccall((:rcl_publisher_is_valid_except_context, LIBRCL), Bool, (Ptr{rcl_publisher_t},), publisher)
end

function rcl_publisher_get_subscription_count(publisher, subscription_count)
    ccall((:rcl_publisher_get_subscription_count, LIBRCL), rmw_ret_t, (Ptr{rcl_publisher_t}, Ptr{Csize_t}), publisher, subscription_count)
end

function rcl_publisher_get_actual_qos(publisher)
    ccall((:rcl_publisher_get_actual_qos, LIBRCL), Ptr{rmw_qos_profile_t}, (Ptr{rcl_publisher_t},), publisher)
end

function rcl_publisher_can_loan_messages(publisher)
    ccall((:rcl_publisher_can_loan_messages, LIBRCL), Bool, (Ptr{rcl_publisher_t},), publisher)
end

# -----------------------
# end publisher.h
# -----------------------

# -----------------------------------------
# Generated wrapper for rcl.h
# -----------------------------------------

# -----------------------
# end rcl.h
# -----------------------

# -----------------------------------------
# Generated wrapper for remap.h
# -----------------------------------------

function rcl_get_zero_initialized_remap()
    ccall((:rcl_get_zero_initialized_remap, LIBRCL), rcl_remap_t, ())
end

function rcl_remap_topic_name(local_arguments, global_arguments, topic_name, node_name, node_namespace, allocator::rcl_allocator_t, output_name)
    ccall((:rcl_remap_topic_name, LIBRCL), rcl_ret_t, (Ptr{rcl_arguments_t}, Ptr{rcl_arguments_t}, Cstring, Cstring, Cstring, rcl_allocator_t, Ptr{Cstring}), local_arguments, global_arguments, topic_name, node_name, node_namespace, allocator, output_name)
end

function rcl_remap_service_name(local_arguments, global_arguments, service_name, node_name, node_namespace, allocator::rcl_allocator_t, output_name)
    ccall((:rcl_remap_service_name, LIBRCL), rcl_ret_t, (Ptr{rcl_arguments_t}, Ptr{rcl_arguments_t}, Cstring, Cstring, Cstring, rcl_allocator_t, Ptr{Cstring}), local_arguments, global_arguments, service_name, node_name, node_namespace, allocator, output_name)
end

function rcl_remap_node_name(local_arguments, global_arguments, node_name, allocator::rcl_allocator_t, output_name)
    ccall((:rcl_remap_node_name, LIBRCL), rcl_ret_t, (Ptr{rcl_arguments_t}, Ptr{rcl_arguments_t}, Cstring, rcl_allocator_t, Ptr{Cstring}), local_arguments, global_arguments, node_name, allocator, output_name)
end

function rcl_remap_node_namespace(local_arguments, global_arguments, node_name, allocator::rcl_allocator_t, output_namespace)
    ccall((:rcl_remap_node_namespace, LIBRCL), rcl_ret_t, (Ptr{rcl_arguments_t}, Ptr{rcl_arguments_t}, Cstring, rcl_allocator_t, Ptr{Cstring}), local_arguments, global_arguments, node_name, allocator, output_namespace)
end

function rcl_remap_fini(remap)
    ccall((:rcl_remap_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_remap_t},), remap)
end

# -----------------------
# end remap.h
# -----------------------

# -----------------------------------------
# Generated wrapper for security_directory.h
# -----------------------------------------

function rcl_get_secure_root(node_name, node_namespace, allocator)
    ccall((:rcl_get_secure_root, LIBRCL), Cstring, (Cstring, Cstring, Ptr{rcl_allocator_t}), node_name, node_namespace, allocator)
end

# -----------------------
# end security_directory.h
# -----------------------

# -----------------------------------------
# Generated wrapper for service.h
# -----------------------------------------

function rcl_get_zero_initialized_service()
    ccall((:rcl_get_zero_initialized_service, LIBRCL), rcl_service_t, ())
end

function rcl_service_init(service, node, type_support, service_name, options)
    ccall((:rcl_service_init, LIBRCL), rcl_ret_t, (Ptr{rcl_service_t}, Ptr{rcl_node_t}, Ptr{rosidl_service_type_support_t}, Cstring, Ptr{rcl_service_options_t}), service, node, type_support, service_name, options)
end

function rcl_service_fini(service, node)
    ccall((:rcl_service_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_service_t}, Ptr{rcl_node_t}), service, node)
end

function rcl_service_get_default_options()
    ccall((:rcl_service_get_default_options, LIBRCL), rcl_service_options_t, ())
end

function rcl_take_request(service, request_header, ros_request)
    ccall((:rcl_take_request, LIBRCL), rcl_ret_t, (Ptr{rcl_service_t}, Ptr{rmw_request_id_t}, Ptr{Cvoid}), service, request_header, ros_request)
end

function rcl_send_response(service, response_header, ros_response)
    ccall((:rcl_send_response, LIBRCL), rcl_ret_t, (Ptr{rcl_service_t}, Ptr{rmw_request_id_t}, Ptr{Cvoid}), service, response_header, ros_response)
end

function rcl_service_get_service_name(service)
    ccall((:rcl_service_get_service_name, LIBRCL), Cstring, (Ptr{rcl_service_t},), service)
end

function rcl_service_get_options(service)
    ccall((:rcl_service_get_options, LIBRCL), Ptr{rcl_service_options_t}, (Ptr{rcl_service_t},), service)
end

function rcl_service_get_rmw_handle(service)
    ccall((:rcl_service_get_rmw_handle, LIBRCL), Ptr{rmw_service_t}, (Ptr{rcl_service_t},), service)
end

function rcl_service_is_valid(service)
    ccall((:rcl_service_is_valid, LIBRCL), Bool, (Ptr{rcl_service_t},), service)
end

# -----------------------
# end service.h
# -----------------------

# -----------------------------------------
# Generated wrapper for subscription.h
# -----------------------------------------

function rcl_get_zero_initialized_subscription()
    ccall((:rcl_get_zero_initialized_subscription, LIBRCL), rcl_subscription_t, ())
end

function rcl_subscription_init(subscription, node, type_support, topic_name, options)
    ccall((:rcl_subscription_init, LIBRCL), rcl_ret_t, (Ptr{rcl_subscription_t}, Ptr{rcl_node_t}, Ptr{rosidl_message_type_support_t}, Cstring, Ptr{rcl_subscription_options_t}), subscription, node, type_support, topic_name, options)
end

function rcl_subscription_fini(subscription, node)
    ccall((:rcl_subscription_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_subscription_t}, Ptr{rcl_node_t}), subscription, node)
end

function rcl_subscription_get_default_options()
    ccall((:rcl_subscription_get_default_options, LIBRCL), rcl_subscription_options_t, ())
end

function rcl_take(subscription, ros_message, message_info, allocation)
    ccall((:rcl_take, LIBRCL), rcl_ret_t, (Ptr{rcl_subscription_t}, Ptr{Cvoid}, Ptr{rmw_message_info_t}, Ptr{rmw_subscription_allocation_t}), subscription, ros_message, message_info, allocation)
end

function rcl_take_serialized_message(subscription, serialized_message, message_info, allocation)
    ccall((:rcl_take_serialized_message, LIBRCL), rcl_ret_t, (Ptr{rcl_subscription_t}, Ptr{rcl_serialized_message_t}, Ptr{rmw_message_info_t}, Ptr{rmw_subscription_allocation_t}), subscription, serialized_message, message_info, allocation)
end

function rcl_take_loaned_message(subscription, loaned_message, message_info, allocation)
    ccall((:rcl_take_loaned_message, LIBRCL), rcl_ret_t, (Ptr{rcl_subscription_t}, Ptr{Ptr{Cvoid}}, Ptr{rmw_message_info_t}, Ptr{rmw_subscription_allocation_t}), subscription, loaned_message, message_info, allocation)
end

function rcl_return_loaned_message_from_subscription(subscription, loaned_message)
    ccall((:rcl_return_loaned_message_from_subscription, LIBRCL), rcl_ret_t, (Ptr{rcl_subscription_t}, Ptr{Cvoid}), subscription, loaned_message)
end

function rcl_subscription_get_topic_name(subscription)
    ccall((:rcl_subscription_get_topic_name, LIBRCL), Cstring, (Ptr{rcl_subscription_t},), subscription)
end

function rcl_subscription_get_options(subscription)
    ccall((:rcl_subscription_get_options, LIBRCL), Ptr{rcl_subscription_options_t}, (Ptr{rcl_subscription_t},), subscription)
end

function rcl_subscription_get_rmw_handle(subscription)
    ccall((:rcl_subscription_get_rmw_handle, LIBRCL), Ptr{rmw_subscription_t}, (Ptr{rcl_subscription_t},), subscription)
end

function rcl_subscription_is_valid(subscription)
    ccall((:rcl_subscription_is_valid, LIBRCL), Bool, (Ptr{rcl_subscription_t},), subscription)
end

function rcl_subscription_get_publisher_count(subscription, publisher_count)
    ccall((:rcl_subscription_get_publisher_count, LIBRCL), rmw_ret_t, (Ptr{rcl_subscription_t}, Ptr{Csize_t}), subscription, publisher_count)
end

function rcl_subscription_get_actual_qos(subscription)
    ccall((:rcl_subscription_get_actual_qos, LIBRCL), Ptr{rmw_qos_profile_t}, (Ptr{rcl_subscription_t},), subscription)
end

function rcl_subscription_can_loan_messages(subscription)
    ccall((:rcl_subscription_can_loan_messages, LIBRCL), Bool, (Ptr{rcl_subscription_t},), subscription)
end

# -----------------------
# end subscription.h
# -----------------------

# -----------------------------------------
# Generated wrapper for time.h
# -----------------------------------------

function rcl_clock_valid(clock)
    ccall((:rcl_clock_valid, LIBRCL), Bool, (Ptr{rcl_clock_t},), clock)
end

function rcl_clock_init(clock_type::rcl_clock_type_t, clock, allocator)
    ccall((:rcl_clock_init, LIBRCL), rcl_ret_t, (rcl_clock_type_t, Ptr{rcl_clock_t}, Ptr{rcl_allocator_t}), clock_type, clock, allocator)
end

function rcl_clock_fini(clock)
    ccall((:rcl_clock_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_clock_t},), clock)
end

function rcl_ros_clock_init(clock, allocator)
    ccall((:rcl_ros_clock_init, LIBRCL), rcl_ret_t, (Ptr{rcl_clock_t}, Ptr{rcl_allocator_t}), clock, allocator)
end

function rcl_ros_clock_fini(clock)
    ccall((:rcl_ros_clock_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_clock_t},), clock)
end

function rcl_steady_clock_init(clock, allocator)
    ccall((:rcl_steady_clock_init, LIBRCL), rcl_ret_t, (Ptr{rcl_clock_t}, Ptr{rcl_allocator_t}), clock, allocator)
end

function rcl_steady_clock_fini(clock)
    ccall((:rcl_steady_clock_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_clock_t},), clock)
end

function rcl_system_clock_init(clock, allocator)
    ccall((:rcl_system_clock_init, LIBRCL), rcl_ret_t, (Ptr{rcl_clock_t}, Ptr{rcl_allocator_t}), clock, allocator)
end

function rcl_system_clock_fini(clock)
    ccall((:rcl_system_clock_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_clock_t},), clock)
end

function rcl_difference_times(start, finish, delta)
    ccall((:rcl_difference_times, LIBRCL), rcl_ret_t, (Ptr{rcl_time_point_t}, Ptr{rcl_time_point_t}, Ptr{rcl_duration_t}), start, finish, delta)
end

function rcl_clock_get_now(clock, time_point_value)
    ccall((:rcl_clock_get_now, LIBRCL), rcl_ret_t, (Ptr{rcl_clock_t}, Ptr{rcl_time_point_value_t}), clock, time_point_value)
end

function rcl_enable_ros_time_override(clock)
    ccall((:rcl_enable_ros_time_override, LIBRCL), rcl_ret_t, (Ptr{rcl_clock_t},), clock)
end

function rcl_disable_ros_time_override(clock)
    ccall((:rcl_disable_ros_time_override, LIBRCL), rcl_ret_t, (Ptr{rcl_clock_t},), clock)
end

function rcl_is_enabled_ros_time_override(clock, is_enabled)
    ccall((:rcl_is_enabled_ros_time_override, LIBRCL), rcl_ret_t, (Ptr{rcl_clock_t}, Ptr{Bool}), clock, is_enabled)
end

function rcl_set_ros_time_override(clock, time_value::rcl_time_point_value_t)
    ccall((:rcl_set_ros_time_override, LIBRCL), rcl_ret_t, (Ptr{rcl_clock_t}, rcl_time_point_value_t), clock, time_value)
end

function rcl_clock_add_jump_callback(clock, threshold::rcl_jump_threshold_t, callback::rcl_jump_callback_t, user_data)
    ccall((:rcl_clock_add_jump_callback, LIBRCL), rcl_ret_t, (Ptr{rcl_clock_t}, rcl_jump_threshold_t, rcl_jump_callback_t, Ptr{Cvoid}), clock, threshold, callback, user_data)
end

function rcl_clock_remove_jump_callback(clock, callback::rcl_jump_callback_t, user_data)
    ccall((:rcl_clock_remove_jump_callback, LIBRCL), rcl_ret_t, (Ptr{rcl_clock_t}, rcl_jump_callback_t, Ptr{Cvoid}), clock, callback, user_data)
end

# -----------------------
# end time.h
# -----------------------

# -----------------------------------------
# Generated wrapper for timer.h
# -----------------------------------------

function rcl_get_zero_initialized_timer()
    ccall((:rcl_get_zero_initialized_timer, LIBRCL), rcl_timer_t, ())
end

function rcl_timer_init(timer, clock, context, period::Int64, callback::rcl_timer_callback_t, allocator::rcl_allocator_t)
    ccall((:rcl_timer_init, LIBRCL), rcl_ret_t, (Ptr{rcl_timer_t}, Ptr{rcl_clock_t}, Ptr{rcl_context_t}, Int64, rcl_timer_callback_t, rcl_allocator_t), timer, clock, context, period, callback, allocator)
end

function rcl_timer_fini(timer)
    ccall((:rcl_timer_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_timer_t},), timer)
end

function rcl_timer_call(timer)
    ccall((:rcl_timer_call, LIBRCL), rcl_ret_t, (Ptr{rcl_timer_t},), timer)
end

function rcl_timer_clock(timer, clock)
    ccall((:rcl_timer_clock, LIBRCL), rcl_ret_t, (Ptr{rcl_timer_t}, Ptr{Ptr{rcl_clock_t}}), timer, clock)
end

function rcl_timer_is_ready(timer, is_ready)
    ccall((:rcl_timer_is_ready, LIBRCL), rcl_ret_t, (Ptr{rcl_timer_t}, Ptr{Bool}), timer, is_ready)
end

function rcl_timer_get_time_until_next_call(timer, time_until_next_call)
    ccall((:rcl_timer_get_time_until_next_call, LIBRCL), rcl_ret_t, (Ptr{rcl_timer_t}, Ptr{Int64}), timer, time_until_next_call)
end

function rcl_timer_get_time_since_last_call(timer, time_since_last_call)
    ccall((:rcl_timer_get_time_since_last_call, LIBRCL), rcl_ret_t, (Ptr{rcl_timer_t}, Ptr{Int64}), timer, time_since_last_call)
end

function rcl_timer_get_period(timer, period)
    ccall((:rcl_timer_get_period, LIBRCL), rcl_ret_t, (Ptr{rcl_timer_t}, Ptr{Int64}), timer, period)
end

function rcl_timer_exchange_period(timer, new_period::Int64, old_period)
    ccall((:rcl_timer_exchange_period, LIBRCL), rcl_ret_t, (Ptr{rcl_timer_t}, Int64, Ptr{Int64}), timer, new_period, old_period)
end

function rcl_timer_get_callback(timer)
    ccall((:rcl_timer_get_callback, LIBRCL), rcl_timer_callback_t, (Ptr{rcl_timer_t},), timer)
end

function rcl_timer_exchange_callback(timer, new_callback::rcl_timer_callback_t)
    ccall((:rcl_timer_exchange_callback, LIBRCL), rcl_timer_callback_t, (Ptr{rcl_timer_t}, rcl_timer_callback_t), timer, new_callback)
end

function rcl_timer_cancel(timer)
    ccall((:rcl_timer_cancel, LIBRCL), rcl_ret_t, (Ptr{rcl_timer_t},), timer)
end

function rcl_timer_is_canceled(timer, is_canceled)
    ccall((:rcl_timer_is_canceled, LIBRCL), rcl_ret_t, (Ptr{rcl_timer_t}, Ptr{Bool}), timer, is_canceled)
end

function rcl_timer_reset(timer)
    ccall((:rcl_timer_reset, LIBRCL), rcl_ret_t, (Ptr{rcl_timer_t},), timer)
end

function rcl_timer_get_allocator(timer)
    ccall((:rcl_timer_get_allocator, LIBRCL), Ptr{rcl_allocator_t}, (Ptr{rcl_timer_t},), timer)
end

function rcl_timer_get_guard_condition(timer)
    ccall((:rcl_timer_get_guard_condition, LIBRCL), Ptr{rcl_guard_condition_t}, (Ptr{rcl_timer_t},), timer)
end

# -----------------------
# end timer.h
# -----------------------

# -----------------------------------------
# Generated wrapper for types.h
# -----------------------------------------

# -----------------------
# end types.h
# -----------------------

# -----------------------------------------
# Generated wrapper for validate_topic_name.h
# -----------------------------------------

function rcl_validate_topic_name(topic_name, validation_result, invalid_index)
    ccall((:rcl_validate_topic_name, LIBRCL), rcl_ret_t, (Cstring, Ptr{Cint}, Ptr{Csize_t}), topic_name, validation_result, invalid_index)
end

function rcl_validate_topic_name_with_size(topic_name, topic_name_length::Csize_t, validation_result, invalid_index)
    ccall((:rcl_validate_topic_name_with_size, LIBRCL), rcl_ret_t, (Cstring, Csize_t, Ptr{Cint}, Ptr{Csize_t}), topic_name, topic_name_length, validation_result, invalid_index)
end

function rcl_topic_name_validation_result_string(validation_result::Cint)
    ccall((:rcl_topic_name_validation_result_string, LIBRCL), Cstring, (Cint,), validation_result)
end

# -----------------------
# end validate_topic_name.h
# -----------------------

# -----------------------------------------
# Generated wrapper for visibility_control.h
# -----------------------------------------

# -----------------------
# end visibility_control.h
# -----------------------

# -----------------------------------------
# Generated wrapper for wait.h
# -----------------------------------------

function rcl_get_zero_initialized_wait_set()
    ccall((:rcl_get_zero_initialized_wait_set, LIBRCL), rcl_wait_set_t, ())
end

function rcl_wait_set_init(wait_set, number_of_subscriptions::Csize_t, number_of_guard_conditions::Csize_t, number_of_timers::Csize_t, number_of_clients::Csize_t, number_of_services::Csize_t, number_of_events::Csize_t, context, allocator::rcl_allocator_t)
    ccall((:rcl_wait_set_init, LIBRCL), rcl_ret_t, (Ptr{rcl_wait_set_t}, Csize_t, Csize_t, Csize_t, Csize_t, Csize_t, Csize_t, Ptr{rcl_context_t}, rcl_allocator_t), wait_set, number_of_subscriptions, number_of_guard_conditions, number_of_timers, number_of_clients, number_of_services, number_of_events, context, allocator)
end

function rcl_wait_set_fini(wait_set)
    ccall((:rcl_wait_set_fini, LIBRCL), rcl_ret_t, (Ptr{rcl_wait_set_t},), wait_set)
end

function rcl_wait_set_get_allocator(wait_set, allocator)
    ccall((:rcl_wait_set_get_allocator, LIBRCL), rcl_ret_t, (Ptr{rcl_wait_set_t}, Ptr{rcl_allocator_t}), wait_set, allocator)
end

function rcl_wait_set_add_subscription(wait_set, subscription, index)
    ccall((:rcl_wait_set_add_subscription, LIBRCL), rcl_ret_t, (Ptr{rcl_wait_set_t}, Ptr{rcl_subscription_t}, Ptr{Csize_t}), wait_set, subscription, index)
end

function rcl_wait_set_clear(wait_set)
    ccall((:rcl_wait_set_clear, LIBRCL), rcl_ret_t, (Ptr{rcl_wait_set_t},), wait_set)
end

function rcl_wait_set_resize(wait_set, subscriptions_size::Csize_t, guard_conditions_size::Csize_t, timers_size::Csize_t, clients_size::Csize_t, services_size::Csize_t, events_size::Csize_t)
    ccall((:rcl_wait_set_resize, LIBRCL), rcl_ret_t, (Ptr{rcl_wait_set_t}, Csize_t, Csize_t, Csize_t, Csize_t, Csize_t, Csize_t), wait_set, subscriptions_size, guard_conditions_size, timers_size, clients_size, services_size, events_size)
end

function rcl_wait_set_add_guard_condition(wait_set, guard_condition, index)
    ccall((:rcl_wait_set_add_guard_condition, LIBRCL), rcl_ret_t, (Ptr{rcl_wait_set_t}, Ptr{rcl_guard_condition_t}, Ptr{Csize_t}), wait_set, guard_condition, index)
end

function rcl_wait_set_add_timer(wait_set, timer, index)
    ccall((:rcl_wait_set_add_timer, LIBRCL), rcl_ret_t, (Ptr{rcl_wait_set_t}, Ptr{rcl_timer_t}, Ptr{Csize_t}), wait_set, timer, index)
end

function rcl_wait_set_add_client(wait_set, client, index)
    ccall((:rcl_wait_set_add_client, LIBRCL), rcl_ret_t, (Ptr{rcl_wait_set_t}, Ptr{rcl_client_t}, Ptr{Csize_t}), wait_set, client, index)
end

function rcl_wait_set_add_service(wait_set, service, index)
    ccall((:rcl_wait_set_add_service, LIBRCL), rcl_ret_t, (Ptr{rcl_wait_set_t}, Ptr{rcl_service_t}, Ptr{Csize_t}), wait_set, service, index)
end

function rcl_wait_set_add_event(wait_set, event, index)
    ccall((:rcl_wait_set_add_event, LIBRCL), rcl_ret_t, (Ptr{rcl_wait_set_t}, Ptr{rcl_event_t}, Ptr{Csize_t}), wait_set, event, index)
end

function rcl_wait(wait_set, timeout::Int64)
    ccall((:rcl_wait, LIBRCL), rcl_ret_t, (Ptr{rcl_wait_set_t}, Int64), wait_set, timeout)
end

# -----------------------
# end wait.h
# -----------------------
