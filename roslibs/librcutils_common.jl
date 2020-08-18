# Julia wrapper for ROS package 'rcutils' common definitions
# Automatically generated using Clang.jl, do not edit manually
# Use `wrap_rospkg` in `clang_wrap.jl` for regeneration

# Skipping MacroDefinition: RCUTILS_CHECK_ALLOCATOR ( allocator , fail_statement ) if ( ! rcutils_allocator_is_valid ( allocator ) ) { fail_statement ; }
# Skipping MacroDefinition: RCUTILS_CHECK_ALLOCATOR_WITH_MSG ( allocator , msg , fail_statement ) if ( ! rcutils_allocator_is_valid ( allocator ) ) { RCUTILS_SET_ERROR_MSG ( msg ) ; fail_statement ; }

struct rcutils_allocator_t
    allocate::Ptr{Cvoid}
    deallocate::Ptr{Cvoid}
    reallocate::Ptr{Cvoid}
    zero_allocate::Ptr{Cvoid}
    state::Ptr{Cvoid}
end

# Skipping MacroDefinition: RCUTILS_SAFE_FWRITE_TO_STDERR ( msg ) do { fwrite ( msg , sizeof ( char ) , strlen ( msg ) , stderr ) ; } while ( 0 )

const RCUTILS_ERROR_STATE_LINE_NUMBER_STR_MAX_LENGTH = 20
const RCUTILS_ERROR_FORMATTING_CHARACTERS = 6
const RCUTILS_ERROR_MESSAGE_MAX_LENGTH = 1024
const RCUTILS_ERROR_STATE_MESSAGE_MAX_LENGTH = 768
const RCUTILS_ERROR_STATE_FILE_MAX_LENGTH = (((RCUTILS_ERROR_MESSAGE_MAX_LENGTH - RCUTILS_ERROR_STATE_MESSAGE_MAX_LENGTH) - RCUTILS_ERROR_STATE_LINE_NUMBER_STR_MAX_LENGTH) - RCUTILS_ERROR_FORMATTING_CHARACTERS) - 1

# Skipping MacroDefinition: RCUTILS_CHECK_ARGUMENT_FOR_NULL ( argument , error_return_type ) RCUTILS_CHECK_FOR_NULL_WITH_MSG ( argument , # argument " argument is null" , return error_return_type )
# Skipping MacroDefinition: RCUTILS_CHECK_FOR_NULL_WITH_MSG ( value , msg , error_statement ) do { if ( NULL == value ) { RCUTILS_SET_ERROR_MSG ( msg ) ; error_statement ; } } while ( 0 )
# Skipping MacroDefinition: RCUTILS_SET_ERROR_MSG ( msg ) do { rcutils_set_error_state ( msg , __FILE__ , __LINE__ ) ; } while ( 0 )
# Skipping MacroDefinition: RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING ( format_string , ... ) do { char output_msg [ RCUTILS_ERROR_MESSAGE_MAX_LENGTH ] ; int ret = rcutils_snprintf ( output_msg , sizeof ( output_msg ) , format_string , __VA_ARGS__ ) ; if ( ret < 0 ) { RCUTILS_SAFE_FWRITE_TO_STDERR ( "Failed to call snprintf for error message formatting\n" ) ; } else { RCUTILS_SET_ERROR_MSG ( output_msg ) ; } } while ( 0 )

struct rcutils_error_string_t
    str::NTuple{1024, UInt8}
end

struct rcutils_error_state_t
    message::NTuple{768, UInt8}
    file::NTuple{229, UInt8}
    line_number::UInt64
end

# Skipping MacroDefinition: rcutils_format_string ( allocator , format_string , ... ) rcutils_format_string_limit ( allocator , 2048 , format_string , __VA_ARGS__ )

const RCUTILS_LOGGING_SEPARATOR_CHAR = '.'
const RCUTILS_LOGGING_SEPARATOR_STRING = "."

@cenum RCUTILS_LOG_SEVERITY::UInt32 begin
    RCUTILS_LOG_SEVERITY_UNSET = 0
    RCUTILS_LOG_SEVERITY_DEBUG = 10
    RCUTILS_LOG_SEVERITY_INFO = 20
    RCUTILS_LOG_SEVERITY_WARN = 30
    RCUTILS_LOG_SEVERITY_ERROR = 40
    RCUTILS_LOG_SEVERITY_FATAL = 50
end


const RCUTILS_DEFAULT_LOGGER_DEFAULT_LEVEL = RCUTILS_LOG_SEVERITY_INFO

# Skipping MacroDefinition: RCUTILS_LIKELY ( x ) __builtin_expect ( ( x ) , 1 )
# Skipping MacroDefinition: RCUTILS_UNLIKELY ( x ) __builtin_expect ( ( x ) , 0 )
# Skipping MacroDefinition: RCUTILS_LOGGING_AUTOINIT if ( RCUTILS_UNLIKELY ( ! g_rcutils_logging_initialized ) ) { rcutils_ret_t ret = rcutils_logging_initialize ( ) ; if ( ret != RCUTILS_RET_OK ) { RCUTILS_SAFE_FWRITE_TO_STDERR ( "[rcutils|" __FILE__ ":" RCUTILS_STRINGIFY ( __LINE__ ) "] error initializing logging: " ) ; RCUTILS_SAFE_FWRITE_TO_STDERR ( rcutils_get_error_string ( ) . str ) ; RCUTILS_SAFE_FWRITE_TO_STDERR ( "\n" ) ; rcutils_reset_error ( ) ; } }

struct rcutils_log_location_t
    function_name::Cstring
    file_name::Cstring
    line_number::Csize_t
end

const rcutils_logging_output_handler_t = Ptr{Cvoid}
const RCUTILS_LOG_MIN_SEVERITY_DEBUG = 0
const RCUTILS_LOG_MIN_SEVERITY_INFO = 1
const RCUTILS_LOG_MIN_SEVERITY_WARN = 2
const RCUTILS_LOG_MIN_SEVERITY_ERROR = 3
const RCUTILS_LOG_MIN_SEVERITY_FATAL = 4
const RCUTILS_LOG_MIN_SEVERITY_NONE = 5
const RCUTILS_LOG_MIN_SEVERITY = RCUTILS_LOG_MIN_SEVERITY_DEBUG

# Skipping MacroDefinition: RCUTILS_LOG_COND_NAMED ( severity , condition_before , condition_after , name , ... ) do { RCUTILS_LOGGING_AUTOINIT static rcutils_log_location_t __rcutils_logging_location = { __func__ , __FILE__ , __LINE__ } ; if ( rcutils_logging_logger_is_enabled_for ( name , severity ) ) { condition_before rcutils_log ( & __rcutils_logging_location , severity , name , __VA_ARGS__ ) ; condition_after } } while ( 0 )
# Skipping MacroDefinition: RCUTILS_LOG_CONDITION_ONCE_BEFORE { static int __rcutils_logging_once = 0 ; if ( RCUTILS_UNLIKELY ( 0 == __rcutils_logging_once ) ) { __rcutils_logging_once = 1 ;
# Skipping MacroDefinition: RCUTILS_LOG_CONDITION_ONCE_AFTER } }
# Skipping MacroDefinition: RCUTILS_LOG_CONDITION_EXPRESSION_BEFORE ( expression ) if ( expression ) {
# Skipping MacroDefinition: RCUTILS_LOG_CONDITION_FUNCTION_BEFORE ( function ) if ( ( * function ) ( ) ) {
# Skipping MacroDefinition: RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE { static bool __rcutils_logging_first = true ; if ( RCUTILS_UNLIKELY ( true == __rcutils_logging_first ) ) { __rcutils_logging_first = false ; } else {
# Skipping MacroDefinition: RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER } }
# Skipping MacroDefinition: RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) { static rcutils_duration_value_t __rcutils_logging_duration = RCUTILS_MS_TO_NS ( ( rcutils_duration_value_t ) duration ) ; static rcutils_time_point_value_t __rcutils_logging_last_logged = 0 ; rcutils_time_point_value_t __rcutils_logging_now = 0 ; bool __rcutils_logging_condition = true ; if ( get_time_point_value ( & __rcutils_logging_now ) != RCUTILS_RET_OK ) { rcutils_log ( & __rcutils_logging_location , RCUTILS_LOG_SEVERITY_ERROR , "" , "%s() at %s:%d getting current steady time failed\n" , __func__ , __FILE__ , __LINE__ ) ; } else { __rcutils_logging_condition = __rcutils_logging_now >= __rcutils_logging_last_logged + __rcutils_logging_duration ; } if ( RCUTILS_LIKELY ( __rcutils_logging_condition ) ) { __rcutils_logging_last_logged = __rcutils_logging_now ;
# Skipping MacroDefinition: RCUTILS_LOG_CONDITION_THROTTLE_AFTER } }
# Skipping MacroDefinition: RCUTILS_LOG_DEBUG ( ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_DEBUG , RCUTILS_LOG_CONDITION_EMPTY , RCUTILS_LOG_CONDITION_EMPTY , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_DEBUG_NAMED ( name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_DEBUG , RCUTILS_LOG_CONDITION_EMPTY , RCUTILS_LOG_CONDITION_EMPTY , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_DEBUG_ONCE ( ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_DEBUG , RCUTILS_LOG_CONDITION_ONCE_BEFORE , RCUTILS_LOG_CONDITION_ONCE_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_DEBUG_ONCE_NAMED ( name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_DEBUG , RCUTILS_LOG_CONDITION_ONCE_BEFORE , RCUTILS_LOG_CONDITION_ONCE_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_DEBUG_EXPRESSION ( expression , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_DEBUG , RCUTILS_LOG_CONDITION_EXPRESSION_BEFORE ( expression ) , RCUTILS_LOG_CONDITION_EXPRESSION_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_DEBUG_EXPRESSION_NAMED ( expression , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_DEBUG , RCUTILS_LOG_CONDITION_EXPRESSION_BEFORE ( expression ) , RCUTILS_LOG_CONDITION_EXPRESSION_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_DEBUG_FUNCTION ( function , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_DEBUG , RCUTILS_LOG_CONDITION_FUNCTION_BEFORE ( function ) , RCUTILS_LOG_CONDITION_FUNCTION_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_DEBUG_FUNCTION_NAMED ( function , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_DEBUG , RCUTILS_LOG_CONDITION_FUNCTION_BEFORE ( function ) , RCUTILS_LOG_CONDITION_FUNCTION_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_DEBUG_SKIPFIRST ( ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_DEBUG , RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_DEBUG_SKIPFIRST_NAMED ( name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_DEBUG , RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_DEBUG_THROTTLE ( get_time_point_value , duration , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_DEBUG , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) , RCUTILS_LOG_CONDITION_THROTTLE_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_DEBUG_SKIPFIRST_THROTTLE ( get_time_point_value , duration , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_DEBUG , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_THROTTLE_AFTER RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_DEBUG_THROTTLE_NAMED ( get_time_point_value , duration , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_DEBUG , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) , RCUTILS_LOG_CONDITION_THROTTLE_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_DEBUG_SKIPFIRST_THROTTLE_NAMED ( get_time_point_value , duration , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_DEBUG , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_THROTTLE_AFTER RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_INFO ( ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_INFO , RCUTILS_LOG_CONDITION_EMPTY , RCUTILS_LOG_CONDITION_EMPTY , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_INFO_NAMED ( name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_INFO , RCUTILS_LOG_CONDITION_EMPTY , RCUTILS_LOG_CONDITION_EMPTY , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_INFO_ONCE ( ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_INFO , RCUTILS_LOG_CONDITION_ONCE_BEFORE , RCUTILS_LOG_CONDITION_ONCE_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_INFO_ONCE_NAMED ( name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_INFO , RCUTILS_LOG_CONDITION_ONCE_BEFORE , RCUTILS_LOG_CONDITION_ONCE_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_INFO_EXPRESSION ( expression , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_INFO , RCUTILS_LOG_CONDITION_EXPRESSION_BEFORE ( expression ) , RCUTILS_LOG_CONDITION_EXPRESSION_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_INFO_EXPRESSION_NAMED ( expression , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_INFO , RCUTILS_LOG_CONDITION_EXPRESSION_BEFORE ( expression ) , RCUTILS_LOG_CONDITION_EXPRESSION_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_INFO_FUNCTION ( function , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_INFO , RCUTILS_LOG_CONDITION_FUNCTION_BEFORE ( function ) , RCUTILS_LOG_CONDITION_FUNCTION_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_INFO_FUNCTION_NAMED ( function , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_INFO , RCUTILS_LOG_CONDITION_FUNCTION_BEFORE ( function ) , RCUTILS_LOG_CONDITION_FUNCTION_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_INFO_SKIPFIRST ( ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_INFO , RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_INFO_SKIPFIRST_NAMED ( name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_INFO , RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_INFO_THROTTLE ( get_time_point_value , duration , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_INFO , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) , RCUTILS_LOG_CONDITION_THROTTLE_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_INFO_SKIPFIRST_THROTTLE ( get_time_point_value , duration , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_INFO , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_THROTTLE_AFTER RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_INFO_THROTTLE_NAMED ( get_time_point_value , duration , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_INFO , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) , RCUTILS_LOG_CONDITION_THROTTLE_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_INFO_SKIPFIRST_THROTTLE_NAMED ( get_time_point_value , duration , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_INFO , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_THROTTLE_AFTER RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_WARN ( ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_WARN , RCUTILS_LOG_CONDITION_EMPTY , RCUTILS_LOG_CONDITION_EMPTY , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_WARN_NAMED ( name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_WARN , RCUTILS_LOG_CONDITION_EMPTY , RCUTILS_LOG_CONDITION_EMPTY , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_WARN_ONCE ( ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_WARN , RCUTILS_LOG_CONDITION_ONCE_BEFORE , RCUTILS_LOG_CONDITION_ONCE_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_WARN_ONCE_NAMED ( name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_WARN , RCUTILS_LOG_CONDITION_ONCE_BEFORE , RCUTILS_LOG_CONDITION_ONCE_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_WARN_EXPRESSION ( expression , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_WARN , RCUTILS_LOG_CONDITION_EXPRESSION_BEFORE ( expression ) , RCUTILS_LOG_CONDITION_EXPRESSION_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_WARN_EXPRESSION_NAMED ( expression , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_WARN , RCUTILS_LOG_CONDITION_EXPRESSION_BEFORE ( expression ) , RCUTILS_LOG_CONDITION_EXPRESSION_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_WARN_FUNCTION ( function , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_WARN , RCUTILS_LOG_CONDITION_FUNCTION_BEFORE ( function ) , RCUTILS_LOG_CONDITION_FUNCTION_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_WARN_FUNCTION_NAMED ( function , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_WARN , RCUTILS_LOG_CONDITION_FUNCTION_BEFORE ( function ) , RCUTILS_LOG_CONDITION_FUNCTION_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_WARN_SKIPFIRST ( ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_WARN , RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_WARN_SKIPFIRST_NAMED ( name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_WARN , RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_WARN_THROTTLE ( get_time_point_value , duration , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_WARN , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) , RCUTILS_LOG_CONDITION_THROTTLE_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_WARN_SKIPFIRST_THROTTLE ( get_time_point_value , duration , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_WARN , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_THROTTLE_AFTER RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_WARN_THROTTLE_NAMED ( get_time_point_value , duration , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_WARN , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) , RCUTILS_LOG_CONDITION_THROTTLE_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_WARN_SKIPFIRST_THROTTLE_NAMED ( get_time_point_value , duration , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_WARN , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_THROTTLE_AFTER RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_ERROR ( ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_ERROR , RCUTILS_LOG_CONDITION_EMPTY , RCUTILS_LOG_CONDITION_EMPTY , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_ERROR_NAMED ( name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_ERROR , RCUTILS_LOG_CONDITION_EMPTY , RCUTILS_LOG_CONDITION_EMPTY , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_ERROR_ONCE ( ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_ERROR , RCUTILS_LOG_CONDITION_ONCE_BEFORE , RCUTILS_LOG_CONDITION_ONCE_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_ERROR_ONCE_NAMED ( name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_ERROR , RCUTILS_LOG_CONDITION_ONCE_BEFORE , RCUTILS_LOG_CONDITION_ONCE_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_ERROR_EXPRESSION ( expression , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_ERROR , RCUTILS_LOG_CONDITION_EXPRESSION_BEFORE ( expression ) , RCUTILS_LOG_CONDITION_EXPRESSION_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_ERROR_EXPRESSION_NAMED ( expression , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_ERROR , RCUTILS_LOG_CONDITION_EXPRESSION_BEFORE ( expression ) , RCUTILS_LOG_CONDITION_EXPRESSION_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_ERROR_FUNCTION ( function , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_ERROR , RCUTILS_LOG_CONDITION_FUNCTION_BEFORE ( function ) , RCUTILS_LOG_CONDITION_FUNCTION_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_ERROR_FUNCTION_NAMED ( function , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_ERROR , RCUTILS_LOG_CONDITION_FUNCTION_BEFORE ( function ) , RCUTILS_LOG_CONDITION_FUNCTION_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_ERROR_SKIPFIRST ( ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_ERROR , RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_ERROR_SKIPFIRST_NAMED ( name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_ERROR , RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_ERROR_THROTTLE ( get_time_point_value , duration , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_ERROR , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) , RCUTILS_LOG_CONDITION_THROTTLE_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_ERROR_SKIPFIRST_THROTTLE ( get_time_point_value , duration , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_ERROR , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_THROTTLE_AFTER RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_ERROR_THROTTLE_NAMED ( get_time_point_value , duration , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_ERROR , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) , RCUTILS_LOG_CONDITION_THROTTLE_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_ERROR_SKIPFIRST_THROTTLE_NAMED ( get_time_point_value , duration , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_ERROR , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_THROTTLE_AFTER RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_FATAL ( ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_FATAL , RCUTILS_LOG_CONDITION_EMPTY , RCUTILS_LOG_CONDITION_EMPTY , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_FATAL_NAMED ( name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_FATAL , RCUTILS_LOG_CONDITION_EMPTY , RCUTILS_LOG_CONDITION_EMPTY , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_FATAL_ONCE ( ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_FATAL , RCUTILS_LOG_CONDITION_ONCE_BEFORE , RCUTILS_LOG_CONDITION_ONCE_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_FATAL_ONCE_NAMED ( name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_FATAL , RCUTILS_LOG_CONDITION_ONCE_BEFORE , RCUTILS_LOG_CONDITION_ONCE_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_FATAL_EXPRESSION ( expression , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_FATAL , RCUTILS_LOG_CONDITION_EXPRESSION_BEFORE ( expression ) , RCUTILS_LOG_CONDITION_EXPRESSION_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_FATAL_EXPRESSION_NAMED ( expression , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_FATAL , RCUTILS_LOG_CONDITION_EXPRESSION_BEFORE ( expression ) , RCUTILS_LOG_CONDITION_EXPRESSION_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_FATAL_FUNCTION ( function , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_FATAL , RCUTILS_LOG_CONDITION_FUNCTION_BEFORE ( function ) , RCUTILS_LOG_CONDITION_FUNCTION_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_FATAL_FUNCTION_NAMED ( function , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_FATAL , RCUTILS_LOG_CONDITION_FUNCTION_BEFORE ( function ) , RCUTILS_LOG_CONDITION_FUNCTION_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_FATAL_SKIPFIRST ( ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_FATAL , RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_FATAL_SKIPFIRST_NAMED ( name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_FATAL , RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_FATAL_THROTTLE ( get_time_point_value , duration , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_FATAL , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) , RCUTILS_LOG_CONDITION_THROTTLE_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_FATAL_SKIPFIRST_THROTTLE ( get_time_point_value , duration , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_FATAL , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_THROTTLE_AFTER RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , NULL , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_FATAL_THROTTLE_NAMED ( get_time_point_value , duration , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_FATAL , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) , RCUTILS_LOG_CONDITION_THROTTLE_AFTER , name , __VA_ARGS__ )
# Skipping MacroDefinition: RCUTILS_LOG_FATAL_SKIPFIRST_THROTTLE_NAMED ( get_time_point_value , duration , name , ... ) RCUTILS_LOG_COND_NAMED ( RCUTILS_LOG_SEVERITY_FATAL , RCUTILS_LOG_CONDITION_THROTTLE_BEFORE ( get_time_point_value , duration ) RCUTILS_LOG_CONDITION_SKIPFIRST_BEFORE , RCUTILS_LOG_CONDITION_THROTTLE_AFTER RCUTILS_LOG_CONDITION_SKIPFIRST_AFTER , name , __VA_ARGS__ )

const RclLogFilter = Ptr{Cvoid}

# Skipping MacroDefinition: RCUTILS_WARN_UNUSED __attribute__ ( ( warn_unused_result ) )
# Skipping MacroDefinition: RCUTILS_STRINGIFY_IMPL ( x ) # x
# Skipping MacroDefinition: RCUTILS_STRINGIFY ( x ) RCUTILS_STRINGIFY_IMPL ( x )
# Skipping MacroDefinition: RCUTILS_UNUSED ( x ) ( void ) ( x )
# Skipping MacroDefinition: RCUTILS_ATTRIBUTE_PRINTF_FORMAT ( format_string_index , first_to_check_index ) __attribute__ ( ( format ( printf , format_string_index , first_to_check_index ) ) )
# Skipping MacroDefinition: my__has_feature ( ... ) __has_feature ( __VAR_ARGS__ )
# Skipping MacroDefinition: rcutils_atomic_load ( object , out ) ( out ) = atomic_load ( object )
# Skipping MacroDefinition: rcutils_atomic_compare_exchange_strong ( object , out , expected , desired ) ( out ) = atomic_compare_exchange_strong ( object , expected , desired )
# Skipping MacroDefinition: rcutils_atomic_exchange ( object , out , desired ) ( out ) = atomic_exchange ( object , desired )
# Skipping MacroDefinition: rcutils_atomic_store ( object , desired ) atomic_store ( object , desired )
# Skipping MacroDefinition: rcutils_atomic_fetch_add ( object , out , arg ) ( out ) = atomic_fetch_add ( object , arg )
# Skipping MacroDefinition: RCUTILS_S_TO_NS ( seconds ) ( seconds * ( 1000LL * 1000LL * 1000LL ) )
# Skipping MacroDefinition: RCUTILS_MS_TO_NS ( milliseconds ) ( milliseconds * ( 1000LL * 1000LL ) )
# Skipping MacroDefinition: RCUTILS_US_TO_NS ( microseconds ) ( microseconds * 1000LL )
# Skipping MacroDefinition: RCUTILS_NS_TO_S ( nanoseconds ) ( nanoseconds / ( 1000LL * 1000LL * 1000LL ) )
# Skipping MacroDefinition: RCUTILS_NS_TO_MS ( nanoseconds ) ( nanoseconds / ( 1000LL * 1000LL ) )
# Skipping MacroDefinition: RCUTILS_NS_TO_US ( nanoseconds ) ( nanoseconds / 1000LL )

#const RCUTILS_STEADY_TIME = rcutils_steady_time_now
const rcutils_time_point_value_t = Int64
const rcutils_duration_value_t = Int64

# Skipping MacroDefinition: RCUTILS_EXPORT __attribute__ ( ( visibility ( "default" ) ) )
# Skipping MacroDefinition: RCUTILS_PUBLIC __attribute__ ( ( visibility ( "default" ) ) )
# Skipping MacroDefinition: RCUTILS_LOCAL __attribute__ ( ( visibility ( "hidden" ) ) )
# Skipping MacroDefinition: ATOMIC_VAR_INIT ( value ) ( value )
# Skipping MacroDefinition: atomic_init ( obj , value ) __c11_atomic_init ( obj , value )
# Skipping MacroDefinition: atomic_thread_fence ( order ) __c11_atomic_thread_fence ( order )
# Skipping MacroDefinition: atomic_signal_fence ( order ) __c11_atomic_signal_fence ( order )
# Skipping MacroDefinition: atomic_is_lock_free ( obj ) __c11_atomic_is_lock_free ( sizeof ( obj ) )
# Skipping MacroDefinition: atomic_compare_exchange_strong_explicit ( object , expected , desired , success , failure ) __c11_atomic_compare_exchange_strong ( object , expected , desired , success , failure )
# Skipping MacroDefinition: atomic_compare_exchange_weak_explicit ( object , expected , desired , success , failure ) __c11_atomic_compare_exchange_weak ( object , expected , desired , success , failure )
# Skipping MacroDefinition: atomic_exchange_explicit ( object , desired , order ) __c11_atomic_exchange ( object , desired , order )
# Skipping MacroDefinition: atomic_fetch_add_explicit ( object , operand , order ) __c11_atomic_fetch_add ( object , operand , order )
# Skipping MacroDefinition: atomic_fetch_and_explicit ( object , operand , order ) __c11_atomic_fetch_and ( object , operand , order )
# Skipping MacroDefinition: atomic_fetch_or_explicit ( object , operand , order ) __c11_atomic_fetch_or ( object , operand , order )
# Skipping MacroDefinition: atomic_fetch_sub_explicit ( object , operand , order ) __c11_atomic_fetch_sub ( object , operand , order )
# Skipping MacroDefinition: atomic_fetch_xor_explicit ( object , operand , order ) __c11_atomic_fetch_xor ( object , operand , order )
# Skipping MacroDefinition: atomic_load_explicit ( object , order ) __c11_atomic_load ( object , order )
# Skipping MacroDefinition: atomic_store_explicit ( object , desired , order ) __c11_atomic_store ( object , desired , order )
# Skipping MacroDefinition: atomic_compare_exchange_strong ( object , expected , desired ) atomic_compare_exchange_strong_explicit ( object , expected , desired , memory_order_seq_cst , memory_order_seq_cst )
# Skipping MacroDefinition: atomic_compare_exchange_weak ( object , expected , desired ) atomic_compare_exchange_weak_explicit ( object , expected , desired , memory_order_seq_cst , memory_order_seq_cst )
# Skipping MacroDefinition: atomic_exchange ( object , desired ) atomic_exchange_explicit ( object , desired , memory_order_seq_cst )
# Skipping MacroDefinition: atomic_fetch_add ( object , operand ) atomic_fetch_add_explicit ( object , operand , memory_order_seq_cst )
# Skipping MacroDefinition: atomic_fetch_and ( object , operand ) atomic_fetch_and_explicit ( object , operand , memory_order_seq_cst )
# Skipping MacroDefinition: atomic_fetch_or ( object , operand ) atomic_fetch_or_explicit ( object , operand , memory_order_seq_cst )
# Skipping MacroDefinition: atomic_fetch_sub ( object , operand ) atomic_fetch_sub_explicit ( object , operand , memory_order_seq_cst )
# Skipping MacroDefinition: atomic_fetch_xor ( object , operand ) atomic_fetch_xor_explicit ( object , operand , memory_order_seq_cst )
# Skipping MacroDefinition: atomic_load ( object ) atomic_load_explicit ( object , memory_order_seq_cst )
# Skipping MacroDefinition: atomic_store ( object , desired ) atomic_store_explicit ( object , desired , memory_order_seq_cst )
# Skipping MacroDefinition: ATOMIC_FLAG_INIT ATOMIC_VAR_INIT ( 0 )
# Skipping MacroDefinition: atomic_flag_clear_explicit ( object , order ) atomic_store_explicit ( object , 0 , order )
# Skipping MacroDefinition: atomic_flag_test_and_set_explicit ( object , order ) atomic_compare_exchange_strong_explicit ( object , 0 , 1 , order , order )
# Skipping MacroDefinition: atomic_flag_clear ( object ) atomic_flag_clear_explicit ( object , memory_order_seq_cst )
# Skipping MacroDefinition: atomic_flag_test_and_set ( object ) atomic_flag_test_and_set_explicit ( object , memory_order_seq_cst )

@cenum memory_order::UInt32 begin
    memory_order_relaxed = 0
    memory_order_consume = 1
    memory_order_acquire = 2
    memory_order_release = 3
    memory_order_acq_rel = 4
    memory_order_seq_cst = 5
end


const atomic_bool = Cvoid
const atomic_char = Cvoid
const atomic_schar = Cvoid
const atomic_uchar = Cvoid
const atomic_short = Cvoid
const atomic_ushort = Cvoid
const atomic_int = Cvoid
const atomic_uint = Cvoid
const atomic_long = Cvoid
const atomic_ulong = Cvoid
const atomic_llong = Cvoid
const atomic_ullong = Cvoid
const atomic_wchar_t = Cvoid
const atomic_int_least8_t = Cvoid
const atomic_uint_least8_t = Cvoid
const atomic_int_least16_t = Cvoid
const atomic_uint_least16_t = Cvoid
const atomic_int_least32_t = Cvoid
const atomic_uint_least32_t = Cvoid
const atomic_int_least64_t = Cvoid
const atomic_uint_least64_t = Cvoid
const atomic_int_fast8_t = Cvoid
const atomic_uint_fast8_t = Cvoid
const atomic_int_fast16_t = Cvoid
const atomic_uint_fast16_t = Cvoid
const atomic_int_fast32_t = Cvoid
const atomic_uint_fast32_t = Cvoid
const atomic_int_fast64_t = Cvoid
const atomic_uint_fast64_t = Cvoid
const atomic_intptr_t = Cvoid
const atomic_uintptr_t = Cvoid
const atomic_size_t = Cvoid
const atomic_ptrdiff_t = Cvoid
const atomic_intmax_t = Cvoid
const atomic_uintmax_t = Cvoid
const atomic_flag = atomic_bool

# Skipping MacroDefinition: ATOMIC_VAR_INIT ( value ) { . __val = ( value ) }
# Skipping MacroDefinition: atomic_init ( obj , value ) do { ( obj ) -> __val = ( value ) ; \
#} while ( 0 )
# Skipping MacroDefinition: atomic_thread_fence ( order ) MemoryBarrier ( )
# Skipping MacroDefinition: atomic_signal_fence ( order ) _ReadWriteBarrier ( )
# Skipping MacroDefinition: atomic_is_lock_free ( obj ) ( sizeof ( ( obj ) -> __val ) <= sizeof ( void * ) )
# Skipping MacroDefinition: rcutils_win32_atomic_compare_exchange_strong ( object , out , expected , desired ) __pragma ( warning ( push ) ) __pragma ( warning ( disable : 4244 ) ) __pragma ( warning ( disable : 4047 ) ) __pragma ( warning ( disable : 4024 ) ) do { switch ( sizeof ( out ) ) { case sizeof ( uint64_t ) : out = InterlockedCompareExchange64 ( ( LONGLONG * ) object , desired , * expected ) ; break ; case sizeof ( uint32_t ) : out = _InterlockedCompareExchange ( ( LONG * ) object , desired , * expected ) ; break ; case sizeof ( uint16_t ) : out = _InterlockedCompareExchange16 ( ( SHORT * ) object , desired , * expected ) ; break ; case sizeof ( uint8_t ) : out = _InterlockedCompareExchange8 ( ( char * ) object , desired , * expected ) ; break ; default : RCUTILS_LOG_ERROR_NAMED ( _RCUTILS_PACKAGE_NAME , "Unsupported integer type in atomic_compare_exchange_strong" ) ; exit ( - 1 ) ; break ; } } while ( 0 ) ; __pragma ( warning ( pop ) )
# Skipping MacroDefinition: rcutils_win32_atomic_compare_exchange_weak ( object , out , expected , desired ) rcutils_win32_atomic_compare_exchange_strong ( object , out , expected , desired )
# Skipping MacroDefinition: rcutils_win32_atomic_exchange ( object , out , desired ) __pragma ( warning ( push ) ) __pragma ( warning ( disable : 4244 ) ) __pragma ( warning ( disable : 4047 ) ) __pragma ( warning ( disable : 4024 ) ) do { switch ( sizeof ( out ) ) { case sizeof ( uint64_t ) : out = InterlockedExchange64 ( ( LONGLONG * ) object , desired ) ; break ; case sizeof ( uint32_t ) : out = _InterlockedExchange ( ( LONG * ) object , desired ) ; break ; case sizeof ( uint16_t ) : out = _InterlockedExchange16 ( ( SHORT * ) object , desired ) ; break ; case sizeof ( uint8_t ) : out = _InterlockedExchange8 ( ( char * ) object , desired ) ; break ; default : RCUTILS_LOG_ERROR_NAMED ( _RCUTILS_PACKAGE_NAME , "Unsupported integer type in atomic_exchange_strong" ) ; exit ( - 1 ) ; break ; } } while ( 0 ) ; __pragma ( warning ( pop ) )
# Skipping MacroDefinition: rcutils_win32_atomic_fetch_add ( object , out , operand ) __pragma ( warning ( push ) ) __pragma ( warning ( disable : 4244 ) ) __pragma ( warning ( disable : 4047 ) ) __pragma ( warning ( disable : 4024 ) ) do { switch ( sizeof ( out ) ) { case sizeof ( uint64_t ) : out = InterlockedExchangeAdd64 ( ( LONGLONG * ) object , operand ) ; break ; case sizeof ( uint32_t ) : out = _InterlockedExchangeAdd ( ( LONG * ) object , operand ) ; break ; case sizeof ( uint16_t ) : out = _InterlockedExchangeAdd16 ( ( SHORT * ) object , operand ) ; break ; case sizeof ( uint8_t ) : out = _InterlockedExchangeAdd8 ( ( char * ) object , operand ) ; break ; default : RCUTILS_LOG_ERROR_NAMED ( _RCUTILS_PACKAGE_NAME , "Unsupported integer type in atomic_fetch_add" ) ; exit ( - 1 ) ; break ; } } while ( 0 ) ; __pragma ( warning ( pop ) )
# Skipping MacroDefinition: rcutils_win32_atomic_fetch_and ( object , out , operand ) __pragma ( warning ( push ) ) __pragma ( warning ( disable : 4244 ) ) __pragma ( warning ( disable : 4047 ) ) __pragma ( warning ( disable : 4024 ) ) do { switch ( sizeof ( out ) ) { case sizeof ( uint64_t ) : out = InterlockedAnd64 ( ( LONGLONG * ) object , operand ) ; break ; case sizeof ( uint32_t ) : out = _InterlockedAnd ( ( LONG * ) object , operand ) ; break ; case sizeof ( uint16_t ) : out = _InterlockedAnd16 ( ( SHORT * ) object , operand ) ; break ; case sizeof ( uint8_t ) : out = _InterlockedAnd8 ( ( char * ) object , operand ) ; break ; default : RCUTILS_LOG_ERROR_NAMED ( _RCUTILS_PACKAGE_NAME , "Unsupported integer type in atomic_fetch_and" ) ; exit ( - 1 ) ; break ; } } while ( 0 ) ; __pragma ( warning ( pop ) )
# Skipping MacroDefinition: rcutils_win32_atomic_fetch_or ( object , out , operand ) __pragma ( warning ( push ) ) __pragma ( warning ( disable : 4244 ) ) __pragma ( warning ( disable : 4047 ) ) __pragma ( warning ( disable : 4024 ) ) do { switch ( sizeof ( out ) ) { case sizeof ( uint64_t ) : out = InterlockedOr64 ( ( LONGLONG * ) object , operand ) ; break ; case sizeof ( uint32_t ) : out = _InterlockedOr ( ( LONG * ) object , operand ) ; break ; case sizeof ( uint16_t ) : out = _InterlockedOr16 ( ( SHORT * ) object , operand ) ; break ; case sizeof ( uint8_t ) : out = _InterlockedOr8 ( ( char * ) object , operand ) ; break ; default : RCUTILS_LOG_ERROR_NAMED ( _RCUTILS_PACKAGE_NAME , "Unsupported integer type in atomic_fetch_or" ) ; exit ( - 1 ) ; break ; } } while ( 0 ) ; __pragma ( warning ( pop ) )
# Skipping MacroDefinition: rcutils_win32_atomic_fetch_sub ( object , out , operand ) rcutils_win32_atomic_fetch_add ( object , out , - ( operand ) )
# Skipping MacroDefinition: rcutils_win32_atomic_fetch_xor ( object , out , operand ) __pragma ( warning ( push ) ) __pragma ( warning ( disable : 4244 ) ) __pragma ( warning ( disable : 4047 ) ) __pragma ( warning ( disable : 4024 ) ) do { switch ( sizeof ( out ) ) { case sizeof ( uint64_t ) : out = InterlockedXor64 ( ( LONGLONG * ) object , operand ) ; break ; case sizeof ( uint32_t ) : out = _InterlockedXor ( ( LONG * ) object , operand ) ; break ; case sizeof ( uint16_t ) : out = _InterlockedXor16 ( ( SHORT * ) object , operand ) ; break ; case sizeof ( uint8_t ) : out = _InterlockedXor8 ( ( char * ) object , operand ) ; break ; default : RCUTILS_LOG_ERROR_NAMED ( _RCUTILS_PACKAGE_NAME , "Unsupported integer type in atomic_fetch_xor" ) ; exit ( - 1 ) ; break ; } } while ( 0 ) ; __pragma ( warning ( pop ) )
# Skipping MacroDefinition: rcutils_win32_atomic_load ( object , out ) __pragma ( warning ( push ) ) __pragma ( warning ( disable : 4244 ) ) __pragma ( warning ( disable : 4047 ) ) __pragma ( warning ( disable : 4024 ) ) do { switch ( sizeof ( out ) ) { case sizeof ( uint64_t ) : out = InterlockedExchangeAdd64 ( ( LONGLONG * ) object , 0 ) ; break ; case sizeof ( uint32_t ) : out = _InterlockedExchangeAdd ( ( LONG * ) object , 0 ) ; break ; case sizeof ( uint16_t ) : out = _InterlockedExchangeAdd16 ( ( SHORT * ) object , 0 ) ; break ; case sizeof ( uint8_t ) : out = _InterlockedExchangeAdd8 ( ( char * ) object , 0 ) ; break ; default : RCUTILS_LOG_ERROR_NAMED ( _RCUTILS_PACKAGE_NAME , "Unsupported integer type in atomic_load" ) ; exit ( - 1 ) ; break ; } } while ( 0 ) ; __pragma ( warning ( pop ) )
# Skipping MacroDefinition: rcutils_win32_atomic_store ( object , desired ) do { MemoryBarrier ( ) ; ( object ) -> __val = ( desired ) ; MemoryBarrier ( ) ; } while ( 0 )
# Skipping MacroDefinition: ARRAY_LIST_VALIDATE_ARRAY_LIST ( array_list ) RCUTILS_CHECK_ARGUMENT_FOR_NULL ( array_list , RCUTILS_RET_INVALID_ARGUMENT ) ; if ( NULL == array_list -> impl ) { RCUTILS_SET_ERROR_MSG ( "array_list is not initialized" ) ; return RCUTILS_RET_NOT_INITIALIZED ; }

const rcutils_array_list_impl_t = Cvoid

struct rcutils_array_list_t
    impl::Ptr{rcutils_array_list_impl_t}
end

struct rcutils_char_array_t
    buffer::Cstring
    owns_buffer::Bool
    buffer_length::Csize_t
    buffer_capacity::Csize_t
    allocator::rcutils_allocator_t
end

# Skipping MacroDefinition: HASH_MAP_VALIDATE_HASH_MAP ( map ) RCUTILS_CHECK_ARGUMENT_FOR_NULL ( map , RCUTILS_RET_INVALID_ARGUMENT ) ; if ( NULL == map -> impl ) { RCUTILS_SET_ERROR_MSG ( "map is not initialized" ) ; return RCUTILS_RET_NOT_INITIALIZED ; }

const rcutils_hash_map_impl_t = Cvoid

struct rcutils_hash_map_t
    impl::Ptr{rcutils_hash_map_impl_t}
end

const rcutils_hash_map_key_hasher_t = Ptr{Cvoid}
const rcutils_hash_map_key_cmp_t = Ptr{Cvoid}
const RCUTILS_RET_OK = 0
const RCUTILS_RET_WARN = 1
const RCUTILS_RET_ERROR = 2
const RCUTILS_RET_BAD_ALLOC = 10
const RCUTILS_RET_INVALID_ARGUMENT = 11
const RCUTILS_RET_NOT_ENOUGH_SPACE = 12
const RCUTILS_RET_NOT_INITIALIZED = 13
const RCUTILS_RET_NOT_FOUND = 14
const RCUTILS_RET_STRING_MAP_ALREADY_INIT = 30
const RCUTILS_RET_STRING_MAP_INVALID = 31
const RCUTILS_RET_STRING_KEY_NOT_FOUND = 32
const RCUTILS_RET_LOGGING_SEVERITY_MAP_INVALID = 40
const RCUTILS_RET_LOGGING_SEVERITY_STRING_INVALID = 41
const RCUTILS_RET_HASH_MAP_NO_MORE_ENTRIES = 50
const rcutils_ret_t = Cint

struct rcutils_string_array_t
    size::Csize_t
    data::Ptr{Cstring}
    allocator::rcutils_allocator_t
end

const rcutils_string_map_impl_t = Cvoid

struct rcutils_string_map_t
    impl::Ptr{rcutils_string_map_impl_t}
end

struct rcutils_uint8_array_t
    buffer::Ptr{UInt8}
    buffer_length::Csize_t
    buffer_capacity::Csize_t
    allocator::rcutils_allocator_t
end
