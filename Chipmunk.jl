module Chipmunk

using CEnum
using Libdl

libchipmunk = dlopen("Chipmunk2D/build/src/libchipmunk.dylib")
const libchipmunk_name = "libchipmunk"

include("libchipmunk_common.jl")
include("libchipmunk_api.jl")

end # module Chipmunk
