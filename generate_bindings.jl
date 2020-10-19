using Clang
using Clang.LibClang.Clang_jll

const CHIPMUNK_INCLUDE = "Chipmunk2D/include/chipmunk"

const CHIPMUNK_HEADERS = [
    "Chipmunk2D/include/chipmunk/chipmunk.h",
    "Chipmunk2D/include/chipmunk/chipmunk_structs.h",
    "Chipmunk2D/include/chipmunk/chipmunk_types.h",
]

const EXCLUDED_CURSOR_NAMES = Set([
    "cpcalloc",
    "cpfree",
    "cprealloc",

    "cpfsqrt",
    "cpfsin",
    "cpfcos",
    "cpfacos",
    "cpfatan2",
    "cpfmod",
    "cpfexp",
    "cpfpow",
    "cpffloor",
    "cpfceil",
    "CPFLOAT_MIN",
])

wrap_context = init(
    headers=CHIPMUNK_HEADERS,
    output_file=joinpath(@__DIR__, "libchipmunk_api.jl"),
    common_file=joinpath(@__DIR__, "libchipmunk_common.jl"),
    clang_includes=vcat(CHIPMUNK_INCLUDE, CLANG_INCLUDE),
    clang_args=["-I", joinpath(CHIPMUNK_INCLUDE, "..")],

    cursor_wrapped=((cursor_name, cursor) -> begin
        !in(cursor_name, EXCLUDED_CURSOR_NAMES)
    end),

    header_wrapped=((root, current) -> begin
        # current == root
        startswith(current, "Chipmunk2D/include/chipmunk/")
    end),

    header_library=(header_name -> "libchipmunk_name"),
    clang_diagnostics=true,
)

run(wrap_context)
