#Debugging helper utils
_debug_output = false
_debug_indent = 0
debug(d::Bool) = global _debug_output = d
macro debug(expr, other...)
    :(if _debug_output
        print(repeat("\t", _debug_indent))
        println($expr,$(other...)) 
      end)
end
macro debug_addindent()
    :(global _debug_indent += 1)
end
macro debug_subindent()
    :(global _debug_indent -= 1)
end

