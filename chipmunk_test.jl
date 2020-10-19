include("Chipmunk.jl")

space = Chipmunk.cpSpaceNew()
Chipmunk.cpSpaceSetGravity(space, Chipmunk.cpVect(0, 10))
body = Chipmunk.cpBodyNew(1, 1)
Chipmunk.cpSpaceAddBody(space, body)

for i = 1:10
    Chipmunk.cpSpaceStep(space, 1 / 60)

    p = Chipmunk.cpBodyGetPosition(body)
    v = Chipmunk.cpBodyGetVelocity(body)

    println("position = ($(p.x), $(p.y)), velocity = ($(v.x), $(v.y))")
end
