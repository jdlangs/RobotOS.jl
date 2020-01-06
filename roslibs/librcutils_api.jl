# Julia wrapper for ROS package 'rcutils' API
# Automatically generated using Clang.jl, do not edit manually
# Use `wrap_rospkg` in `clang_wrap.jl` for regeneration

# -----------------------------------------
# Generated wrapper for allocator.h
# -----------------------------------------

function rcutils_get_zero_initialized_allocator()
    ccall((:rcutils_get_zero_initialized_allocator, LIBRCUTILS), rcutils_allocator_t, ())
end

function rcutils_get_default_allocator()
    ccall((:rcutils_get_default_allocator, LIBRCUTILS), rcutils_allocator_t, ())
end

function rcutils_allocator_is_valid(allocator)
    ccall((:rcutils_allocator_is_valid, LIBRCUTILS), Bool, (Ptr{rcutils_allocator_t},), allocator)
end

function rcutils_reallocf(pointer, size::Csize_t, allocator)
    ccall((:rcutils_reallocf, LIBRCUTILS), Ptr{Cvoid}, (Ptr{Cvoid}, Csize_t, Ptr{rcutils_allocator_t}), pointer, size, allocator)
end

# -----------------------
# end allocator.h
# -----------------------

# -----------------------------------------
# Generated wrapper for cmdline_parser.h
# -----------------------------------------

function rcutils_cli_option_exist(_begin, _end, option)
    ccall((:rcutils_cli_option_exist, LIBRCUTILS), Bool, (Ptr{Cstring}, Ptr{Cstring}, Cstring), _begin, _end, option)
end

function rcutils_cli_get_option(_begin, _end, option)
    ccall((:rcutils_cli_get_option, LIBRCUTILS), Cstring, (Ptr{Cstring}, Ptr{Cstring}, Cstring), _begin, _end, option)
end

# -----------------------
# end cmdline_parser.h
# -----------------------

# -----------------------------------------
# Generated wrapper for error_handling.h
# -----------------------------------------

function rcutils_initialize_error_handling_thread_local_storage(allocator::rcutils_allocator_t)
    ccall((:rcutils_initialize_error_handling_thread_local_storage, LIBRCUTILS), rcutils_ret_t, (rcutils_allocator_t,), allocator)
end

function rcutils_set_error_state(error_string, file, line_number::Csize_t)
    ccall((:rcutils_set_error_state, LIBRCUTILS), Cvoid, (Cstring, Cstring, Csize_t), error_string, file, line_number)
end

function rcutils_error_is_set()
    ccall((:rcutils_error_is_set, LIBRCUTILS), Bool, ())
end

function rcutils_get_error_state()
    ccall((:rcutils_get_error_state, LIBRCUTILS), Ptr{rcutils_error_state_t}, ())
end

function rcutils_get_error_string()
    ccall((:rcutils_get_error_string, LIBRCUTILS), rcutils_error_string_t, ())
end

function rcutils_reset_error()
    ccall((:rcutils_reset_error, LIBRCUTILS), Cvoid, ())
end

# -----------------------
# end error_handling.h
# -----------------------

# -----------------------------------------
# Generated wrapper for filesystem.h
# -----------------------------------------

function rcutils_get_cwd(buffer, max_length::Csize_t)
    ccall((:rcutils_get_cwd, LIBRCUTILS), Bool, (Cstring, Csize_t), buffer, max_length)
end

function rcutils_is_directory(abs_path)
    ccall((:rcutils_is_directory, LIBRCUTILS), Bool, (Cstring,), abs_path)
end

function rcutils_is_file(abs_path)
    ccall((:rcutils_is_file, LIBRCUTILS), Bool, (Cstring,), abs_path)
end

function rcutils_exists(abs_path)
    ccall((:rcutils_exists, LIBRCUTILS), Bool, (Cstring,), abs_path)
end

function rcutils_is_readable(abs_path)
    ccall((:rcutils_is_readable, LIBRCUTILS), Bool, (Cstring,), abs_path)
end

function rcutils_is_writable(abs_path)
    ccall((:rcutils_is_writable, LIBRCUTILS), Bool, (Cstring,), abs_path)
end

function rcutils_is_readable_and_writable(abs_path)
    ccall((:rcutils_is_readable_and_writable, LIBRCUTILS), Bool, (Cstring,), abs_path)
end

function rcutils_join_path(left_hand_path, right_hand_path, allocator::rcutils_allocator_t)
    ccall((:rcutils_join_path, LIBRCUTILS), Cstring, (Cstring, Cstring, rcutils_allocator_t), left_hand_path, right_hand_path, allocator)
end

function rcutils_to_native_path(path, allocator::rcutils_allocator_t)
    ccall((:rcutils_to_native_path, LIBRCUTILS), Cstring, (Cstring, rcutils_allocator_t), path, allocator)
end

function rcutils_mkdir(abs_path)
    ccall((:rcutils_mkdir, LIBRCUTILS), Bool, (Cstring,), abs_path)
end

# -----------------------
# end filesystem.h
# -----------------------

# -----------------------------------------
# Generated wrapper for find.h
# -----------------------------------------

function rcutils_find(str, delimiter::UInt8)
    ccall((:rcutils_find, LIBRCUTILS), Csize_t, (Cstring, UInt8), str, delimiter)
end

function rcutils_findn(str, delimiter::UInt8, string_length::Csize_t)
    ccall((:rcutils_findn, LIBRCUTILS), Csize_t, (Cstring, UInt8, Csize_t), str, delimiter, string_length)
end

function rcutils_find_last(str, delimiter::UInt8)
    ccall((:rcutils_find_last, LIBRCUTILS), Csize_t, (Cstring, UInt8), str, delimiter)
end

function rcutils_find_lastn(str, delimiter::UInt8, string_length::Csize_t)
    ccall((:rcutils_find_lastn, LIBRCUTILS), Csize_t, (Cstring, UInt8, Csize_t), str, delimiter, string_length)
end

# -----------------------
# end find.h
# -----------------------

# -----------------------------------------
# Generated wrapper for format_string.h
# -----------------------------------------

# -----------------------
# end format_string.h
# -----------------------

# -----------------------------------------
# Generated wrapper for get_env.h
# -----------------------------------------

function rcutils_get_env(env_name, env_value)
    ccall((:rcutils_get_env, LIBRCUTILS), Cstring, (Cstring, Ptr{Cstring}), env_name, env_value)
end

function rcutils_get_home_dir()
    ccall((:rcutils_get_home_dir, LIBRCUTILS), Cstring, ())
end

# -----------------------
# end get_env.h
# -----------------------

# -----------------------------------------
# Generated wrapper for isalnum_no_locale.h
# -----------------------------------------

function rcutils_isalnum_no_locale()
    ccall((:rcutils_isalnum_no_locale, LIBRCUTILS), Cint, ())
end

# -----------------------
# end isalnum_no_locale.h
# -----------------------

# -----------------------------------------
# Generated wrapper for logging.h
# -----------------------------------------

function rcutils_logging_initialize_with_allocator(allocator::rcutils_allocator_t)
    ccall((:rcutils_logging_initialize_with_allocator, LIBRCUTILS), rcutils_ret_t, (rcutils_allocator_t,), allocator)
end

function rcutils_logging_initialize()
    ccall((:rcutils_logging_initialize, LIBRCUTILS), rcutils_ret_t, ())
end

function rcutils_logging_shutdown()
    ccall((:rcutils_logging_shutdown, LIBRCUTILS), rcutils_ret_t, ())
end

function rcutils_logging_severity_level_from_string(severity_string, allocator::rcutils_allocator_t, severity)
    ccall((:rcutils_logging_severity_level_from_string, LIBRCUTILS), rcutils_ret_t, (Cstring, rcutils_allocator_t, Ptr{Cint}), severity_string, allocator, severity)
end

function rcutils_logging_get_output_handler()
    ccall((:rcutils_logging_get_output_handler, LIBRCUTILS), rcutils_logging_output_handler_t, ())
end

function rcutils_logging_set_output_handler(_function::rcutils_logging_output_handler_t)
    ccall((:rcutils_logging_set_output_handler, LIBRCUTILS), Cvoid, (rcutils_logging_output_handler_t,), _function)
end

function rcutils_logging_format_message(location, severity::Cint, name, timestamp::rcutils_time_point_value_t, msg, logging_output)
    ccall((:rcutils_logging_format_message, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_log_location_t}, Cint, Cstring, rcutils_time_point_value_t, Cstring, Ptr{rcutils_char_array_t}), location, severity, name, timestamp, msg, logging_output)
end

function rcutils_logging_get_default_logger_level()
    ccall((:rcutils_logging_get_default_logger_level, LIBRCUTILS), Cint, ())
end

function rcutils_logging_set_default_logger_level(level::Cint)
    ccall((:rcutils_logging_set_default_logger_level, LIBRCUTILS), Cvoid, (Cint,), level)
end

function rcutils_logging_get_logger_level(name)
    ccall((:rcutils_logging_get_logger_level, LIBRCUTILS), Cint, (Cstring,), name)
end

function rcutils_logging_get_logger_leveln(name, name_length::Csize_t)
    ccall((:rcutils_logging_get_logger_leveln, LIBRCUTILS), Cint, (Cstring, Csize_t), name, name_length)
end

function rcutils_logging_set_logger_level(name, level::Cint)
    ccall((:rcutils_logging_set_logger_level, LIBRCUTILS), rcutils_ret_t, (Cstring, Cint), name, level)
end

function rcutils_logging_logger_is_enabled_for(name, severity::Cint)
    ccall((:rcutils_logging_logger_is_enabled_for, LIBRCUTILS), Bool, (Cstring, Cint), name, severity)
end

function rcutils_logging_get_logger_effective_level(name)
    ccall((:rcutils_logging_get_logger_effective_level, LIBRCUTILS), Cint, (Cstring,), name)
end

function rcutils_logging_console_output_handler(location, severity::Cint, name, timestamp::rcutils_time_point_value_t, format, args)
    ccall((:rcutils_logging_console_output_handler, LIBRCUTILS), Cvoid, (Ptr{rcutils_log_location_t}, Cint, Cstring, rcutils_time_point_value_t, Cstring, Ptr{va_list}), location, severity, name, timestamp, format, args)
end

# -----------------------
# end logging.h
# -----------------------

# -----------------------------------------
# Generated wrapper for logging_macros.h
# -----------------------------------------

# -----------------------
# end logging_macros.h
# -----------------------

# -----------------------------------------
# Generated wrapper for macros.h
# -----------------------------------------

# -----------------------
# end macros.h
# -----------------------

# -----------------------------------------
# Generated wrapper for process.h
# -----------------------------------------

function rcutils_get_pid()
    ccall((:rcutils_get_pid, LIBRCUTILS), Cint, ())
end

function rcutils_get_executable_name(allocator::rcutils_allocator_t)
    ccall((:rcutils_get_executable_name, LIBRCUTILS), Cstring, (rcutils_allocator_t,), allocator)
end

# -----------------------
# end process.h
# -----------------------

# -----------------------------------------
# Generated wrapper for repl_str.h
# -----------------------------------------

function rcutils_repl_str(str, from, to, allocator)
    ccall((:rcutils_repl_str, LIBRCUTILS), Cstring, (Cstring, Cstring, Cstring, Ptr{rcutils_allocator_t}), str, from, to, allocator)
end

# -----------------------
# end repl_str.h
# -----------------------

# -----------------------------------------
# Generated wrapper for snprintf.h
# -----------------------------------------

# -----------------------
# end snprintf.h
# -----------------------

# -----------------------------------------
# Generated wrapper for split.h
# -----------------------------------------

function rcutils_split(str, delimiter::UInt8, allocator::rcutils_allocator_t, string_array)
    ccall((:rcutils_split, LIBRCUTILS), rcutils_ret_t, (Cstring, UInt8, rcutils_allocator_t, Ptr{rcutils_string_array_t}), str, delimiter, allocator, string_array)
end

function rcutils_split_last(str, delimiter::UInt8, allocator::rcutils_allocator_t, string_array)
    ccall((:rcutils_split_last, LIBRCUTILS), rcutils_ret_t, (Cstring, UInt8, rcutils_allocator_t, Ptr{rcutils_string_array_t}), str, delimiter, allocator, string_array)
end

# -----------------------
# end split.h
# -----------------------

# -----------------------------------------
# Generated wrapper for stdatomic_helper.h
# -----------------------------------------

function rcutils_atomic_load_bool(a_bool)
    ccall((:rcutils_atomic_load_bool, LIBRCUTILS), Bool, (Ptr{atomic_bool},), a_bool)
end

function rcutils_atomic_load_int64_t(a_int64_t)
    ccall((:rcutils_atomic_load_int64_t, LIBRCUTILS), Int64, (Ptr{atomic_int_least64_t},), a_int64_t)
end

function rcutils_atomic_load_uint64_t(a_uint64_t)
    ccall((:rcutils_atomic_load_uint64_t, LIBRCUTILS), UInt64, (Ptr{atomic_uint_least64_t},), a_uint64_t)
end

function rcutils_atomic_load_uintptr_t(a_uintptr_t)
    ccall((:rcutils_atomic_load_uintptr_t, LIBRCUTILS), Csize_t, (Ptr{atomic_uintptr_t},), a_uintptr_t)
end

function rcutils_atomic_compare_exchange_strong_uint_least64_t(a_uint_least64_t, expected, desired::UInt64)
    ccall((:rcutils_atomic_compare_exchange_strong_uint_least64_t, LIBRCUTILS), Bool, (Ptr{atomic_uint_least64_t}, Ptr{UInt64}, UInt64), a_uint_least64_t, expected, desired)
end

function rcutils_atomic_exchange_bool(a_bool, desired::Bool)
    ccall((:rcutils_atomic_exchange_bool, LIBRCUTILS), Bool, (Ptr{atomic_bool}, Bool), a_bool, desired)
end

function rcutils_atomic_exchange_int64_t(a_int64_t, desired::Int64)
    ccall((:rcutils_atomic_exchange_int64_t, LIBRCUTILS), Int64, (Ptr{atomic_int_least64_t}, Int64), a_int64_t, desired)
end

function rcutils_atomic_exchange_uint64_t(a_uint64_t, desired::UInt64)
    ccall((:rcutils_atomic_exchange_uint64_t, LIBRCUTILS), UInt64, (Ptr{atomic_uint_least64_t}, UInt64), a_uint64_t, desired)
end

function rcutils_atomic_exchange_uintptr_t(a_uintptr_t, desired::Csize_t)
    ccall((:rcutils_atomic_exchange_uintptr_t, LIBRCUTILS), Csize_t, (Ptr{atomic_uintptr_t}, Csize_t), a_uintptr_t, desired)
end

function rcutils_atomic_fetch_add_uint64_t(a_uint64_t, arg::UInt64)
    ccall((:rcutils_atomic_fetch_add_uint64_t, LIBRCUTILS), UInt64, (Ptr{atomic_uint_least64_t}, UInt64), a_uint64_t, arg)
end

# -----------------------
# end stdatomic_helper.h
# -----------------------

# -----------------------------------------
# Generated wrapper for strdup.h
# -----------------------------------------

function rcutils_strdup(str, allocator::rcutils_allocator_t)
    ccall((:rcutils_strdup, LIBRCUTILS), Cstring, (Cstring, rcutils_allocator_t), str, allocator)
end

function rcutils_strndup(str, string_length::Csize_t, allocator::rcutils_allocator_t)
    ccall((:rcutils_strndup, LIBRCUTILS), Cstring, (Cstring, Csize_t, rcutils_allocator_t), str, string_length, allocator)
end

# -----------------------
# end strdup.h
# -----------------------

# -----------------------------------------
# Generated wrapper for time.h
# -----------------------------------------

function rcutils_system_time_now(now)
    ccall((:rcutils_system_time_now, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_time_point_value_t},), now)
end

function rcutils_steady_time_now(now)
    ccall((:rcutils_steady_time_now, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_time_point_value_t},), now)
end

function rcutils_time_point_value_as_nanoseconds_string(time_point, str, str_size::Csize_t)
    ccall((:rcutils_time_point_value_as_nanoseconds_string, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_time_point_value_t}, Cstring, Csize_t), time_point, str, str_size)
end

function rcutils_time_point_value_as_seconds_string(time_point, str, str_size::Csize_t)
    ccall((:rcutils_time_point_value_as_seconds_string, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_time_point_value_t}, Cstring, Csize_t), time_point, str, str_size)
end

# -----------------------
# end time.h
# -----------------------

# -----------------------------------------
# Generated wrapper for types.h
# -----------------------------------------

# -----------------------
# end types.h
# -----------------------

# -----------------------------------------
# Generated wrapper for visibility_control.h
# -----------------------------------------

# -----------------------
# end visibility_control.h
# -----------------------

# -----------------------------------------
# Generated wrapper for stdatomic.h
# -----------------------------------------

# -----------------------
# end stdatomic.h
# -----------------------

# -----------------------------------------
# Generated wrapper for stdatomic.h
# -----------------------------------------

# -----------------------
# end stdatomic.h
# -----------------------

# -----------------------------------------
# Generated wrapper for array_list.h
# -----------------------------------------

function rcutils_get_zero_initialized_array_list()
    ccall((:rcutils_get_zero_initialized_array_list, LIBRCUTILS), rcutils_array_list_t, ())
end

function rcutils_array_list_init(array_list, initial_capacity::Csize_t, data_size::Csize_t, allocator)
    ccall((:rcutils_array_list_init, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_array_list_t}, Csize_t, Csize_t, Ptr{rcutils_allocator_t}), array_list, initial_capacity, data_size, allocator)
end

function rcutils_array_list_fini(array_list)
    ccall((:rcutils_array_list_fini, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_array_list_t},), array_list)
end

function rcutils_array_list_add(array_list, data)
    ccall((:rcutils_array_list_add, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_array_list_t}, Ptr{Cvoid}), array_list, data)
end

function rcutils_array_list_set(array_list, index::Csize_t, data)
    ccall((:rcutils_array_list_set, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_array_list_t}, Csize_t, Ptr{Cvoid}), array_list, index, data)
end

function rcutils_array_list_remove(array_list, index::Csize_t)
    ccall((:rcutils_array_list_remove, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_array_list_t}, Csize_t), array_list, index)
end

function rcutils_array_list_get(array_list, index::Csize_t, data)
    ccall((:rcutils_array_list_get, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_array_list_t}, Csize_t, Ptr{Cvoid}), array_list, index, data)
end

function rcutils_array_list_get_size(array_list, size)
    ccall((:rcutils_array_list_get_size, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_array_list_t}, Ptr{Csize_t}), array_list, size)
end

# -----------------------
# end array_list.h
# -----------------------

# -----------------------------------------
# Generated wrapper for char_array.h
# -----------------------------------------

function rcutils_get_zero_initialized_char_array()
    ccall((:rcutils_get_zero_initialized_char_array, LIBRCUTILS), rcutils_char_array_t, ())
end

function rcutils_char_array_init(char_array, buffer_capacity::Csize_t, allocator)
    ccall((:rcutils_char_array_init, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_char_array_t}, Csize_t, Ptr{rcutils_allocator_t}), char_array, buffer_capacity, allocator)
end

function rcutils_char_array_fini(char_array)
    ccall((:rcutils_char_array_fini, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_char_array_t},), char_array)
end

function rcutils_char_array_resize(char_array, new_size::Csize_t)
    ccall((:rcutils_char_array_resize, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_char_array_t}, Csize_t), char_array, new_size)
end

function rcutils_char_array_expand_as_needed(char_array, new_size::Csize_t)
    ccall((:rcutils_char_array_expand_as_needed, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_char_array_t}, Csize_t), char_array, new_size)
end

function rcutils_char_array_strncat(char_array, src, n::Csize_t)
    ccall((:rcutils_char_array_strncat, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_char_array_t}, Cstring, Csize_t), char_array, src, n)
end

function rcutils_char_array_strcat(char_array, src)
    ccall((:rcutils_char_array_strcat, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_char_array_t}, Cstring), char_array, src)
end

function rcutils_char_array_memcpy(char_array, src, n::Csize_t)
    ccall((:rcutils_char_array_memcpy, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_char_array_t}, Cstring, Csize_t), char_array, src, n)
end

function rcutils_char_array_strcpy(char_array, src)
    ccall((:rcutils_char_array_strcpy, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_char_array_t}, Cstring), char_array, src)
end

# -----------------------
# end char_array.h
# -----------------------

# -----------------------------------------
# Generated wrapper for hash_map.h
# -----------------------------------------

function rcutils_hash_map_string_hash_func(key_str)
    ccall((:rcutils_hash_map_string_hash_func, LIBRCUTILS), Csize_t, (Ptr{Cvoid},), key_str)
end

function rcutils_hash_map_string_cmp_func(val1, val2)
    ccall((:rcutils_hash_map_string_cmp_func, LIBRCUTILS), Cint, (Ptr{Cvoid}, Ptr{Cvoid}), val1, val2)
end

function rcutils_get_zero_initialized_hash_map()
    ccall((:rcutils_get_zero_initialized_hash_map, LIBRCUTILS), rcutils_hash_map_t, ())
end

function rcutils_hash_map_init(hash_map, initial_capacity::Csize_t, key_size::Csize_t, data_size::Csize_t, key_hashing_func::rcutils_hash_map_key_hasher_t, key_cmp_func::rcutils_hash_map_key_cmp_t, allocator)
    ccall((:rcutils_hash_map_init, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_hash_map_t}, Csize_t, Csize_t, Csize_t, rcutils_hash_map_key_hasher_t, rcutils_hash_map_key_cmp_t, Ptr{rcutils_allocator_t}), hash_map, initial_capacity, key_size, data_size, key_hashing_func, key_cmp_func, allocator)
end

function rcutils_hash_map_fini(hash_map)
    ccall((:rcutils_hash_map_fini, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_hash_map_t},), hash_map)
end

function rcutils_hash_map_get_capacity(hash_map, capacity)
    ccall((:rcutils_hash_map_get_capacity, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_hash_map_t}, Ptr{Csize_t}), hash_map, capacity)
end

function rcutils_hash_map_get_size(hash_map, size)
    ccall((:rcutils_hash_map_get_size, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_hash_map_t}, Ptr{Csize_t}), hash_map, size)
end

function rcutils_hash_map_set(hash_map, key, value)
    ccall((:rcutils_hash_map_set, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_hash_map_t}, Ptr{Cvoid}, Ptr{Cvoid}), hash_map, key, value)
end

function rcutils_hash_map_unset(hash_map, key)
    ccall((:rcutils_hash_map_unset, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_hash_map_t}, Ptr{Cvoid}), hash_map, key)
end

function rcutils_hash_map_key_exists(hash_map, key)
    ccall((:rcutils_hash_map_key_exists, LIBRCUTILS), Bool, (Ptr{rcutils_hash_map_t}, Ptr{Cvoid}), hash_map, key)
end

function rcutils_hash_map_get(hash_map, key, data)
    ccall((:rcutils_hash_map_get, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_hash_map_t}, Ptr{Cvoid}, Ptr{Cvoid}), hash_map, key, data)
end

function rcutils_hash_map_get_next_key_and_data(hash_map, previous_key, key, data)
    ccall((:rcutils_hash_map_get_next_key_and_data, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_hash_map_t}, Ptr{Cvoid}, Ptr{Cvoid}, Ptr{Cvoid}), hash_map, previous_key, key, data)
end

# -----------------------
# end hash_map.h
# -----------------------

# -----------------------------------------
# Generated wrapper for rcutils_ret.h
# -----------------------------------------

# -----------------------
# end rcutils_ret.h
# -----------------------

# -----------------------------------------
# Generated wrapper for string_array.h
# -----------------------------------------

function rcutils_get_zero_initialized_string_array()
    ccall((:rcutils_get_zero_initialized_string_array, LIBRCUTILS), rcutils_string_array_t, ())
end

function rcutils_string_array_init(string_array, size::Csize_t, allocator)
    ccall((:rcutils_string_array_init, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_string_array_t}, Csize_t, Ptr{rcutils_allocator_t}), string_array, size, allocator)
end

function rcutils_string_array_fini(string_array)
    ccall((:rcutils_string_array_fini, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_string_array_t},), string_array)
end

function rcutils_string_array_cmp(lhs, rhs, res)
    ccall((:rcutils_string_array_cmp, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_string_array_t}, Ptr{rcutils_string_array_t}, Ptr{Cint}), lhs, rhs, res)
end

# -----------------------
# end string_array.h
# -----------------------

# -----------------------------------------
# Generated wrapper for string_map.h
# -----------------------------------------

function rcutils_get_zero_initialized_string_map()
    ccall((:rcutils_get_zero_initialized_string_map, LIBRCUTILS), rcutils_string_map_t, ())
end

function rcutils_string_map_init(string_map, initial_capacity::Csize_t, allocator::rcutils_allocator_t)
    ccall((:rcutils_string_map_init, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_string_map_t}, Csize_t, rcutils_allocator_t), string_map, initial_capacity, allocator)
end

function rcutils_string_map_fini(string_map)
    ccall((:rcutils_string_map_fini, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_string_map_t},), string_map)
end

function rcutils_string_map_get_capacity(string_map, capacity)
    ccall((:rcutils_string_map_get_capacity, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_string_map_t}, Ptr{Csize_t}), string_map, capacity)
end

function rcutils_string_map_get_size(string_map, size)
    ccall((:rcutils_string_map_get_size, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_string_map_t}, Ptr{Csize_t}), string_map, size)
end

function rcutils_string_map_reserve(string_map, capacity::Csize_t)
    ccall((:rcutils_string_map_reserve, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_string_map_t}, Csize_t), string_map, capacity)
end

function rcutils_string_map_clear(string_map)
    ccall((:rcutils_string_map_clear, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_string_map_t},), string_map)
end

function rcutils_string_map_set(string_map, key, value)
    ccall((:rcutils_string_map_set, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_string_map_t}, Cstring, Cstring), string_map, key, value)
end

function rcutils_string_map_set_no_resize(string_map, key, value)
    ccall((:rcutils_string_map_set_no_resize, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_string_map_t}, Cstring, Cstring), string_map, key, value)
end

function rcutils_string_map_unset(string_map, key)
    ccall((:rcutils_string_map_unset, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_string_map_t}, Cstring), string_map, key)
end

function rcutils_string_map_key_exists(string_map, key)
    ccall((:rcutils_string_map_key_exists, LIBRCUTILS), Bool, (Ptr{rcutils_string_map_t}, Cstring), string_map, key)
end

function rcutils_string_map_key_existsn(string_map, key, key_length::Csize_t)
    ccall((:rcutils_string_map_key_existsn, LIBRCUTILS), Bool, (Ptr{rcutils_string_map_t}, Cstring, Csize_t), string_map, key, key_length)
end

function rcutils_string_map_get(string_map, key)
    ccall((:rcutils_string_map_get, LIBRCUTILS), Cstring, (Ptr{rcutils_string_map_t}, Cstring), string_map, key)
end

function rcutils_string_map_getn(string_map, key, key_length::Csize_t)
    ccall((:rcutils_string_map_getn, LIBRCUTILS), Cstring, (Ptr{rcutils_string_map_t}, Cstring, Csize_t), string_map, key, key_length)
end

function rcutils_string_map_get_next_key(string_map, key)
    ccall((:rcutils_string_map_get_next_key, LIBRCUTILS), Cstring, (Ptr{rcutils_string_map_t}, Cstring), string_map, key)
end

function rcutils_string_map_copy(src_string_map, dst_string_map)
    ccall((:rcutils_string_map_copy, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_string_map_t}, Ptr{rcutils_string_map_t}), src_string_map, dst_string_map)
end

# -----------------------
# end string_map.h
# -----------------------

# -----------------------------------------
# Generated wrapper for uint8_array.h
# -----------------------------------------

function rcutils_get_zero_initialized_uint8_array()
    ccall((:rcutils_get_zero_initialized_uint8_array, LIBRCUTILS), rcutils_uint8_array_t, ())
end

function rcutils_uint8_array_init(uint8_array, buffer_capacity::Csize_t, allocator)
    ccall((:rcutils_uint8_array_init, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_uint8_array_t}, Csize_t, Ptr{rcutils_allocator_t}), uint8_array, buffer_capacity, allocator)
end

function rcutils_uint8_array_fini(uint8_array)
    ccall((:rcutils_uint8_array_fini, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_uint8_array_t},), uint8_array)
end

function rcutils_uint8_array_resize(uint8_array, new_size::Csize_t)
    ccall((:rcutils_uint8_array_resize, LIBRCUTILS), rcutils_ret_t, (Ptr{rcutils_uint8_array_t}, Csize_t), uint8_array, new_size)
end

# -----------------------
# end uint8_array.h
# -----------------------
