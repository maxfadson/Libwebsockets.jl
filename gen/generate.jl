using Pkg
using Pkg.Artifacts
using Clang.Generators
using Clang.Generators.JLLEnvs
using libwebsockets_jll
# using OpenSSL_jll

include("rewrite.jl")

cd(@__DIR__)

artifact_toml = joinpath(dirname(pathof(libwebsockets_jll)), "..", "Artifacts.toml")
artifact_dir = Pkg.Artifacts.ensure_artifact_installed("libwebsockets", artifact_toml)

include_dir = joinpath(artifact_dir, "include") |> normpath

config_h = joinpath(include_dir, "lws_config.h")
websockets_h = joinpath(include_dir, "libwebsockets.h")

# openssl_include_dir = joinpath(OpenSSL_jll.artifact_dir, "include")

options = load_options(joinpath(@__DIR__, "generator.toml"))

const platforms = String[
    "aarch64-apple-darwin20",
    "x86_64-apple-darwin14",
    "x86_64-linux-gnu",
]

for target in platforms
    @info "processing $target"

    options["general"]["output_file_path"] = joinpath(@__DIR__, "..", "lib", "$target.jl")

    args = get_default_args(target)
    push!(args, "-I$include_dir")
    # push!(args, "-I$openssl_include_dir")

    headers = [config_h, websockets_h]

    ctx = create_context(headers, args, options)
    rewrite!(ctx.dag)
    build!(ctx)
end

