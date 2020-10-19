# Automatically generated using Clang.jl


# Skipping MacroDefinition: cpAssertSoft ( __condition__ , ... ) if ( ! ( __condition__ ) ) { cpMessage ( # __condition__ , __FILE__ , __LINE__ , 1 , 0 , __VA_ARGS__ ) ; abort ( ) ; }
# Skipping MacroDefinition: cpAssertWarn ( __condition__ , ... ) if ( ! ( __condition__ ) ) cpMessage ( # __condition__ , __FILE__ , __LINE__ , 0 , 0 , __VA_ARGS__ )
# Skipping MacroDefinition: cpAssertHard ( __condition__ , ... ) if ( ! ( __condition__ ) ) { cpMessage ( # __condition__ , __FILE__ , __LINE__ , 1 , 1 , __VA_ARGS__ ) ; abort ( ) ; }

const CP_USE_DOUBLES = 1

# Skipping MacroDefinition: INFINITY ( __builtin_inf ( ) )
# Skipping MacroDefinition: CP_PI ( ( cpFloat ) 3.14159265358979323846264338327950288 )

const cpTrue = 1
const cpFalse = 0

# Skipping MacroDefinition: CP_NO_GROUP ( ( cpGroup ) 0 )
# Skipping MacroDefinition: CP_ALL_CATEGORIES ( ~ ( cpBitmask ) 0 )
# Skipping MacroDefinition: CP_WILDCARD_COLLISION_TYPE ( ~ ( cpCollisionType ) 0 )

const CP_BUFFER_BYTES = 32 * 1024
const CP_MAX_CONTACTS_PER_ARBITER = 2

# Skipping MacroDefinition: CP_ARBITER_GET_SHAPES ( __arb__ , __a__ , __b__ ) cpShape * __a__ , * __b__ ; cpArbiterGetShapes ( __arb__ , & __a__ , & __b__ ) ;
# Skipping MacroDefinition: CP_ARBITER_GET_BODIES ( __arb__ , __a__ , __b__ ) cpBody * __a__ , * __b__ ; cpArbiterGetBodies ( __arb__ , & __a__ , & __b__ ) ;

const CP_VERSION_MAJOR = 7
const CP_VERSION_MINOR = 0
const CP_VERSION_RELEASE = 3

# Skipping MacroDefinition: CP_CONVEX_HULL ( __count__ , __verts__ , __count_var__ , __verts_var__ ) cpVect * __verts_var__ = ( cpVect * ) alloca ( __count__ * sizeof ( cpVect ) ) ; int __count_var__ = cpConvexHull ( __count__ , __verts__ , __verts_var__ , NULL , 0.0 ) ;

const cpFloat = Cdouble
const cpHashValue = Csize_t
const cpCollisionID = UInt32
const cpBool = Cuchar
const cpDataPointer = Ptr{Cvoid}
const cpCollisionType = Csize_t
const cpGroup = Csize_t
const cpBitmask = UInt32
const cpTimestamp = UInt32

struct cpArbiterFwd end
struct cpArrayFwd end
struct cpBodyFwd end
struct cpCollisionHandlerFwd end
struct cpConstraintClassFwd end
struct cpConstraintFwd end
struct cpContactFwd end
struct cpContactBufferHeaderFwd end
struct cpHashSetFwd end
struct cpShapeClassFwd end
struct cpShapeFwd end
struct cpSpaceFwd end
struct cpSpatialIndexClassFwd end
struct cpSpatialIndexFwd end
struct cpSplittingPlaneFwd end

struct cpVect
    x::cpFloat
    y::cpFloat
end

struct cpTransform
    a::cpFloat
    b::cpFloat
    c::cpFloat
    d::cpFloat
    tx::cpFloat
    ty::cpFloat
end

struct cpMat2x2
    a::cpFloat
    b::cpFloat
    c::cpFloat
    d::cpFloat
end

struct cpArray
    num::Cint
    max::Cint
    arr::Ptr{Ptr{Cvoid}}
end

const cpHashSet = Cvoid
const cpBodyVelocityFunc = Ptr{Cvoid}
const cpBodyPositionFunc = Ptr{Cvoid}
const cpSpatialIndexDestroyImpl = Ptr{Cvoid}
const cpSpatialIndexCountImpl = Ptr{Cvoid}
const cpSpatialIndexEachImpl = Ptr{Cvoid}
const cpSpatialIndexContainsImpl = Ptr{Cvoid}
const cpSpatialIndexInsertImpl = Ptr{Cvoid}
const cpSpatialIndexRemoveImpl = Ptr{Cvoid}
const cpSpatialIndexReindexImpl = Ptr{Cvoid}
const cpSpatialIndexReindexObjectImpl = Ptr{Cvoid}
const cpSpatialIndexReindexQueryImpl = Ptr{Cvoid}
const cpSpatialIndexQueryImpl = Ptr{Cvoid}
const cpSpatialIndexSegmentQueryImpl = Ptr{Cvoid}

struct cpSpatialIndexClass
    destroy::cpSpatialIndexDestroyImpl
    count::cpSpatialIndexCountImpl
    each::cpSpatialIndexEachImpl
    contains::cpSpatialIndexContainsImpl
    insert::cpSpatialIndexInsertImpl
    remove::cpSpatialIndexRemoveImpl
    reindex::cpSpatialIndexReindexImpl
    reindexObject::cpSpatialIndexReindexObjectImpl
    reindexQuery::cpSpatialIndexReindexQueryImpl
    query::cpSpatialIndexQueryImpl
    segmentQuery::cpSpatialIndexSegmentQueryImpl
end

const cpSpatialIndexBBFunc = Ptr{Cvoid}

struct cpSpatialIndex
    klass::Ptr{cpSpatialIndexClassFwd}
    bbfunc::cpSpatialIndexBBFunc
    staticIndex::Ptr{cpSpatialIndexFwd}
    dynamicIndex::Ptr{cpSpatialIndexFwd}
end

const cpContactBufferHeader = Cvoid
const cpCollisionBeginFunc = Ptr{Cvoid}
const cpCollisionPreSolveFunc = Ptr{Cvoid}
const cpCollisionPostSolveFunc = Ptr{Cvoid}
const cpCollisionSeparateFunc = Ptr{Cvoid}

struct cpCollisionHandler
    typeA::cpCollisionType
    typeB::cpCollisionType
    beginFunc::cpCollisionBeginFunc
    preSolveFunc::cpCollisionPreSolveFunc
    postSolveFunc::cpCollisionPostSolveFunc
    separateFunc::cpCollisionSeparateFunc
    userData::cpDataPointer
end

struct cpSpace
    iterations::Cint
    gravity::cpVect
    damping::cpFloat
    idleSpeedThreshold::cpFloat
    sleepTimeThreshold::cpFloat
    collisionSlop::cpFloat
    collisionBias::cpFloat
    collisionPersistence::cpTimestamp
    userData::cpDataPointer
    stamp::cpTimestamp
    curr_dt::cpFloat
    dynamicBodies::Ptr{cpArrayFwd}
    staticBodies::Ptr{cpArrayFwd}
    rousedBodies::Ptr{cpArrayFwd}
    sleepingComponents::Ptr{cpArrayFwd}
    shapeIDCounter::cpHashValue
    staticShapes::Ptr{cpSpatialIndexFwd}
    dynamicShapes::Ptr{cpSpatialIndexFwd}
    constraints::Ptr{cpArrayFwd}
    arbiters::Ptr{cpArrayFwd}
    contactBuffersHead::Ptr{cpContactBufferHeaderFwd}
    cachedArbiters::Ptr{cpHashSetFwd}
    pooledArbiters::Ptr{cpArrayFwd}
    allocatedBuffers::Ptr{cpArrayFwd}
    locked::UInt32
    usesWildcards::cpBool
    collisionHandlers::Ptr{cpHashSetFwd}
    defaultHandler::cpCollisionHandler
    skipPostStep::cpBool
    postStepCallbacks::Ptr{cpArrayFwd}
    staticBody::Ptr{cpBodyFwd}
end

struct cpBodySleeping
    root::Ptr{cpBodyFwd}
    next::Ptr{cpBodyFwd}
    idleTime::cpFloat
end

struct cpBody
    velocity_func::cpBodyVelocityFunc
    position_func::cpBodyPositionFunc
    m::cpFloat
    m_inv::cpFloat
    i::cpFloat
    i_inv::cpFloat
    cog::cpVect
    p::cpVect
    v::cpVect
    f::cpVect
    a::cpFloat
    w::cpFloat
    t::cpFloat
    transform::cpTransform
    userData::cpDataPointer
    v_bias::cpVect
    w_bias::cpFloat
    space::Ptr{cpSpaceFwd}
    shapeList::Ptr{cpShapeFwd}
    arbiterList::Ptr{cpArbiterFwd}
    constraintList::Ptr{cpConstraintFwd}
    sleeping::cpBodySleeping
end

@cenum cpShapeType::UInt32 begin
    CP_CIRCLE_SHAPE = 0
    CP_SEGMENT_SHAPE = 1
    CP_POLY_SHAPE = 2
    CP_NUM_SHAPES = 3
end

const cpShapeCacheDataImpl = Ptr{Cvoid}
const cpShapeDestroyImpl = Ptr{Cvoid}
const cpShapePointQueryImpl = Ptr{Cvoid}
const cpShapeSegmentQueryImpl = Ptr{Cvoid}

struct cpShapeClass
    type::cpShapeType
    cacheData::cpShapeCacheDataImpl
    destroy::cpShapeDestroyImpl
    pointQuery::cpShapePointQueryImpl
    segmentQuery::cpShapeSegmentQueryImpl
end

struct cpShapeMassInfo
    m::cpFloat
    i::cpFloat
    cog::cpVect
    area::cpFloat
end

struct cpBB
    l::cpFloat
    b::cpFloat
    r::cpFloat
    t::cpFloat
end

struct cpShapeFilter
    group::cpGroup
    categories::cpBitmask
    mask::cpBitmask
end

struct cpShape
    klass::Ptr{cpShapeClassFwd}
    space::Ptr{cpSpaceFwd}
    body::Ptr{cpBodyFwd}
    massInfo::cpShapeMassInfo
    bb::cpBB
    sensor::cpBool
    e::cpFloat
    u::cpFloat
    surfaceV::cpVect
    userData::cpDataPointer
    type::cpCollisionType
    filter::cpShapeFilter
    next::Ptr{cpShapeFwd}
    prev::Ptr{cpShapeFwd}
    hashid::cpHashValue
end

struct cpArbiterThread
    next::Ptr{cpArbiterFwd}
    prev::Ptr{cpArbiterFwd}
end

struct cpContact
    r1::cpVect
    r2::cpVect
    nMass::cpFloat
    tMass::cpFloat
    bounce::cpFloat
    jnAcc::cpFloat
    jtAcc::cpFloat
    jBias::cpFloat
    bias::cpFloat
    hash::cpHashValue
end

@cenum cpArbiterState::UInt32 begin
    CP_ARBITER_STATE_FIRST_COLLISION = 0
    CP_ARBITER_STATE_NORMAL = 1
    CP_ARBITER_STATE_IGNORE = 2
    CP_ARBITER_STATE_CACHED = 3
    CP_ARBITER_STATE_INVALIDATED = 4
end


struct cpArbiter
    e::cpFloat
    u::cpFloat
    surface_vr::cpVect
    data::cpDataPointer
    a::Ptr{cpShapeFwd}
    b::Ptr{cpShapeFwd}
    body_a::Ptr{cpBodyFwd}
    body_b::Ptr{cpBodyFwd}
    thread_a::cpArbiterThread
    thread_b::cpArbiterThread
    count::Cint
    contacts::Ptr{cpContactFwd}
    n::cpVect
    handler::Ptr{cpCollisionHandlerFwd}
    handlerA::Ptr{cpCollisionHandlerFwd}
    handlerB::Ptr{cpCollisionHandlerFwd}
    swapped::cpBool
    stamp::cpTimestamp
    state::cpArbiterState
end

const cpConstraintPreStepImpl = Ptr{Cvoid}
const cpConstraintApplyCachedImpulseImpl = Ptr{Cvoid}
const cpConstraintApplyImpulseImpl = Ptr{Cvoid}
const cpConstraintGetImpulseImpl = Ptr{Cvoid}

struct cpConstraintClass
    preStep::cpConstraintPreStepImpl
    applyCachedImpulse::cpConstraintApplyCachedImpulseImpl
    applyImpulse::cpConstraintApplyImpulseImpl
    getImpulse::cpConstraintGetImpulseImpl
end

const cpConstraintPreSolveFunc = Ptr{Cvoid}
const cpConstraintPostSolveFunc = Ptr{Cvoid}

struct cpConstraint
    klass::Ptr{cpConstraintClassFwd}
    space::Ptr{cpSpaceFwd}
    a::Ptr{cpBodyFwd}
    b::Ptr{cpBodyFwd}
    next_a::Ptr{cpConstraintFwd}
    next_b::Ptr{cpConstraintFwd}
    maxForce::cpFloat
    errorBias::cpFloat
    maxBias::cpFloat
    collideBodies::cpBool
    preSolve::cpConstraintPreSolveFunc
    postSolve::cpConstraintPostSolveFunc
    userData::cpDataPointer
end

struct cpCircleShape
    shape::cpShape
    c::cpVect
    tc::cpVect
    r::cpFloat
end

struct cpSegmentShape
    shape::cpShape
    a::cpVect
    b::cpVect
    n::cpVect
    ta::cpVect
    tb::cpVect
    tn::cpVect
    r::cpFloat
    a_tangent::cpVect
    b_tangent::cpVect
end

struct cpSplittingPlane
    v0::cpVect
    n::cpVect
end

struct cpPolyShape
    shape::cpShape
    r::cpFloat
    count::Cint
    planes::Ptr{cpSplittingPlaneFwd}
    _planes::NTuple{12, cpSplittingPlane}
end

struct cpPinJoint
    constraint::cpConstraint
    anchorA::cpVect
    anchorB::cpVect
    dist::cpFloat
    r1::cpVect
    r2::cpVect
    n::cpVect
    nMass::cpFloat
    jnAcc::cpFloat
    bias::cpFloat
end

struct cpSlideJoint
    constraint::cpConstraint
    anchorA::cpVect
    anchorB::cpVect
    min::cpFloat
    max::cpFloat
    r1::cpVect
    r2::cpVect
    n::cpVect
    nMass::cpFloat
    jnAcc::cpFloat
    bias::cpFloat
end

struct cpPivotJoint
    constraint::cpConstraint
    anchorA::cpVect
    anchorB::cpVect
    r1::cpVect
    r2::cpVect
    k::cpMat2x2
    jAcc::cpVect
    bias::cpVect
end

struct cpGrooveJoint
    constraint::cpConstraint
    grv_n::cpVect
    grv_a::cpVect
    grv_b::cpVect
    anchorB::cpVect
    grv_tn::cpVect
    clamp::cpFloat
    r1::cpVect
    r2::cpVect
    k::cpMat2x2
    jAcc::cpVect
    bias::cpVect
end

const cpDampedSpringForceFunc = Ptr{Cvoid}

struct cpDampedSpring
    constraint::cpConstraint
    anchorA::cpVect
    anchorB::cpVect
    restLength::cpFloat
    stiffness::cpFloat
    damping::cpFloat
    springForceFunc::cpDampedSpringForceFunc
    target_vrn::cpFloat
    v_coef::cpFloat
    r1::cpVect
    r2::cpVect
    nMass::cpFloat
    n::cpVect
    jAcc::cpFloat
end

const cpDampedRotarySpringTorqueFunc = Ptr{Cvoid}

struct cpDampedRotarySpring
    constraint::cpConstraint
    restAngle::cpFloat
    stiffness::cpFloat
    damping::cpFloat
    springTorqueFunc::cpDampedRotarySpringTorqueFunc
    target_wrn::cpFloat
    w_coef::cpFloat
    iSum::cpFloat
    jAcc::cpFloat
end

struct cpRotaryLimitJoint
    constraint::cpConstraint
    min::cpFloat
    max::cpFloat
    iSum::cpFloat
    bias::cpFloat
    jAcc::cpFloat
end

struct cpRatchetJoint
    constraint::cpConstraint
    angle::cpFloat
    phase::cpFloat
    ratchet::cpFloat
    iSum::cpFloat
    bias::cpFloat
    jAcc::cpFloat
end

struct cpGearJoint
    constraint::cpConstraint
    phase::cpFloat
    ratio::cpFloat
    ratio_inv::cpFloat
    iSum::cpFloat
    bias::cpFloat
    jAcc::cpFloat
end

const cpSimpleMotorJoint = Cvoid

struct cpContactPointSetPoints
    pointA::cpVect
    pointB::cpVect
    distance::cpFloat
end

struct cpContactPointSet
    count::Cint
    normal::cpVect
    points::cpContactPointSetPoints
end

const cpSpatialIndexIteratorFunc = Ptr{Cvoid}
const cpSpatialIndexQueryFunc = Ptr{Cvoid}
const cpSpatialIndexSegmentQueryFunc = Ptr{Cvoid}
const cpSpaceHash = Cvoid
const cpBBTree = Cvoid
const cpBBTreeVelocityFunc = Ptr{Cvoid}
const cpSweep1D = Cvoid

@cenum cpBodyType::UInt32 begin
    CP_BODY_TYPE_DYNAMIC = 0
    CP_BODY_TYPE_KINEMATIC = 1
    CP_BODY_TYPE_STATIC = 2
end

const cpBodyShapeIteratorFunc = Ptr{Cvoid}
const cpBodyConstraintIteratorFunc = Ptr{Cvoid}
const cpBodyArbiterIteratorFunc = Ptr{Cvoid}

struct cpPointQueryInfo
    shape::Ptr{cpShapeFwd}
    point::cpVect
    distance::cpFloat
    gradient::cpVect
end

struct cpSegmentQueryInfo
    shape::Ptr{cpShapeFwd}
    point::cpVect
    normal::cpVect
    alpha::cpFloat
end

struct cpSimpleMotor
    constraint::cpConstraint
    rate::cpFloat
    iSum::cpFloat
    jAcc::cpFloat
end

const cpPostStepFunc = Ptr{Cvoid}
const cpSpacePointQueryFunc = Ptr{Cvoid}
const cpSpaceSegmentQueryFunc = Ptr{Cvoid}
const cpSpaceBBQueryFunc = Ptr{Cvoid}
const cpSpaceShapeQueryFunc = Ptr{Cvoid}
const cpSpaceBodyIteratorFunc = Ptr{Cvoid}
const cpSpaceShapeIteratorFunc = Ptr{Cvoid}
const cpSpaceConstraintIteratorFunc = Ptr{Cvoid}

struct cpSpaceDebugColor
    r::Cfloat
    g::Cfloat
    b::Cfloat
    a::Cfloat
end

const cpSpaceDebugDrawCircleImpl = Ptr{Cvoid}
const cpSpaceDebugDrawSegmentImpl = Ptr{Cvoid}
const cpSpaceDebugDrawFatSegmentImpl = Ptr{Cvoid}
const cpSpaceDebugDrawPolygonImpl = Ptr{Cvoid}
const cpSpaceDebugDrawDotImpl = Ptr{Cvoid}
const cpSpaceDebugDrawColorForShapeImpl = Ptr{Cvoid}

@cenum cpSpaceDebugDrawFlags::UInt32 begin
    CP_SPACE_DEBUG_DRAW_SHAPES = 1
    CP_SPACE_DEBUG_DRAW_CONSTRAINTS = 2
    CP_SPACE_DEBUG_DRAW_COLLISION_POINTS = 4
end

struct cpSpaceDebugDrawOptions
    drawCircle::cpSpaceDebugDrawCircleImpl
    drawSegment::cpSpaceDebugDrawSegmentImpl
    drawFatSegment::cpSpaceDebugDrawFatSegmentImpl
    drawPolygon::cpSpaceDebugDrawPolygonImpl
    drawDot::cpSpaceDebugDrawDotImpl
    flags::cpSpaceDebugDrawFlags
    shapeOutlineColor::cpSpaceDebugColor
    colorForShape::cpSpaceDebugDrawColorForShapeImpl
    constraintColor::cpSpaceDebugColor
    collisionPointColor::cpSpaceDebugColor
    data::cpDataPointer
end

const cpSpacePointQueryBlock = Cvoid
const cpSpaceSegmentQueryBlock = Cvoid
const cpSpaceBBQueryBlock = Cvoid
const cpSpaceShapeQueryBlock = Cvoid
const CP_POLY_SHAPE_INLINE_ALLOC = 6

struct cpCollisionInfo
    a::Ptr{cpShapeFwd}
    b::Ptr{cpShapeFwd}
    id::cpCollisionID
    n::cpVect
    count::Cint
    arr::Ptr{cpContactFwd}
end

const cpSpaceArbiterApplyImpulseFunc = Ptr{Cvoid}

struct cpPostStepCallback
    func::cpPostStepFunc
    key::Ptr{Cvoid}
    data::Ptr{Cvoid}
end
