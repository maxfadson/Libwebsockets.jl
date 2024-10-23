# prologue.jl

function OSSL_provider_init(handle::Ptr{Cvoid}, in::Ptr{Cvoid}, out::Ptr{Ptr{Cvoid}}, provctx::Ptr{Ptr{Cvoid}})
    ccall((:OSSL_provider_init, "libssl"), Cint,
          (Ptr{Cvoid}, Ptr{Cvoid}, Ptr{Ptr{Cvoid}}, Ptr{Ptr{Cvoid}}),
          handle, in, out, provctx)
end