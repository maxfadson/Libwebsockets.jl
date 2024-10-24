module Libwebsockets

using libwebsockets_jll

to_c_type(::Type{<:AbstractString}) = Cstring
to_c_type(t::Type{<:Union{Array,Ref}}) = Ptr{eltype(t)}

const IS_LIBC_MUSL = occursin("musl", Sys.MACHINE)

if Sys.isapple() && Sys.ARCH === :aarch64
    include("../lib/aarch64-apple-darwin20.jl")
elseif Sys.isapple() && Sys.ARCH === :x86_64 &&
    include("../lib/x86_64-apple-darwin14.jl")
elseif Sys.islinux() && Sys.ARCH === :x86_64 && !IS_LIBC_MUSL
    include("../lib/x86_64-linux-gnu.jl")
else
    error("Unknown platform: $(Sys.MACHINE)")
end

end # module
