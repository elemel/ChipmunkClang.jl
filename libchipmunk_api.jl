# Julia wrapper for header: chipmunk.h
# Automatically generated using Clang.jl


function cpfmax(a, b)
    ccall((:cpfmax, libchipmunk_name), cpFloat, (cpFloat, cpFloat), a, b)
end

function cpfmin(a, b)
    ccall((:cpfmin, libchipmunk_name), cpFloat, (cpFloat, cpFloat), a, b)
end

function cpfabs(f)
    ccall((:cpfabs, libchipmunk_name), cpFloat, (cpFloat,), f)
end

function cpfclamp(f, min, max)
    ccall((:cpfclamp, libchipmunk_name), cpFloat, (cpFloat, cpFloat, cpFloat), f, min, max)
end

function cpfclamp01(f)
    ccall((:cpfclamp01, libchipmunk_name), cpFloat, (cpFloat,), f)
end

function cpflerp(f1, f2, t)
    ccall((:cpflerp, libchipmunk_name), cpFloat, (cpFloat, cpFloat, cpFloat), f1, f2, t)
end

function cpflerpconst(f1, f2, d)
    ccall((:cpflerpconst, libchipmunk_name), cpFloat, (cpFloat, cpFloat, cpFloat), f1, f2, d)
end

function cpv(x, y)
    ccall((:cpv, libchipmunk_name), cpVect, (cpFloat, cpFloat), x, y)
end

function cpveql(v1, v2)
    ccall((:cpveql, libchipmunk_name), cpBool, (cpVect, cpVect), v1, v2)
end

function cpvadd(v1, v2)
    ccall((:cpvadd, libchipmunk_name), cpVect, (cpVect, cpVect), v1, v2)
end

function cpvsub(v1, v2)
    ccall((:cpvsub, libchipmunk_name), cpVect, (cpVect, cpVect), v1, v2)
end

function cpvneg(v)
    ccall((:cpvneg, libchipmunk_name), cpVect, (cpVect,), v)
end

function cpvmult(v, s)
    ccall((:cpvmult, libchipmunk_name), cpVect, (cpVect, cpFloat), v, s)
end

function cpvdot(v1, v2)
    ccall((:cpvdot, libchipmunk_name), cpFloat, (cpVect, cpVect), v1, v2)
end

function cpvcross(v1, v2)
    ccall((:cpvcross, libchipmunk_name), cpFloat, (cpVect, cpVect), v1, v2)
end

function cpvperp(v)
    ccall((:cpvperp, libchipmunk_name), cpVect, (cpVect,), v)
end

function cpvrperp(v)
    ccall((:cpvrperp, libchipmunk_name), cpVect, (cpVect,), v)
end

function cpvproject(v1, v2)
    ccall((:cpvproject, libchipmunk_name), cpVect, (cpVect, cpVect), v1, v2)
end

function cpvforangle(a)
    ccall((:cpvforangle, libchipmunk_name), cpVect, (cpFloat,), a)
end

function cpvtoangle(v)
    ccall((:cpvtoangle, libchipmunk_name), cpFloat, (cpVect,), v)
end

function cpvrotate(v1, v2)
    ccall((:cpvrotate, libchipmunk_name), cpVect, (cpVect, cpVect), v1, v2)
end

function cpvunrotate(v1, v2)
    ccall((:cpvunrotate, libchipmunk_name), cpVect, (cpVect, cpVect), v1, v2)
end

function cpvlengthsq(v)
    ccall((:cpvlengthsq, libchipmunk_name), cpFloat, (cpVect,), v)
end

function cpvlength(v)
    ccall((:cpvlength, libchipmunk_name), cpFloat, (cpVect,), v)
end

function cpvlerp(v1, v2, t)
    ccall((:cpvlerp, libchipmunk_name), cpVect, (cpVect, cpVect, cpFloat), v1, v2, t)
end

function cpvnormalize(v)
    ccall((:cpvnormalize, libchipmunk_name), cpVect, (cpVect,), v)
end

function cpvslerp(v1, v2, t)
    ccall((:cpvslerp, libchipmunk_name), cpVect, (cpVect, cpVect, cpFloat), v1, v2, t)
end

function cpvslerpconst(v1, v2, a)
    ccall((:cpvslerpconst, libchipmunk_name), cpVect, (cpVect, cpVect, cpFloat), v1, v2, a)
end

function cpvclamp(v, len)
    ccall((:cpvclamp, libchipmunk_name), cpVect, (cpVect, cpFloat), v, len)
end

function cpvlerpconst(v1, v2, d)
    ccall((:cpvlerpconst, libchipmunk_name), cpVect, (cpVect, cpVect, cpFloat), v1, v2, d)
end

function cpvdist(v1, v2)
    ccall((:cpvdist, libchipmunk_name), cpFloat, (cpVect, cpVect), v1, v2)
end

function cpvdistsq(v1, v2)
    ccall((:cpvdistsq, libchipmunk_name), cpFloat, (cpVect, cpVect), v1, v2)
end

function cpvnear(v1, v2, dist)
    ccall((:cpvnear, libchipmunk_name), cpBool, (cpVect, cpVect, cpFloat), v1, v2, dist)
end

function cpMat2x2New(a, b, c, d)
    ccall((:cpMat2x2New, libchipmunk_name), cpMat2x2, (cpFloat, cpFloat, cpFloat, cpFloat), a, b, c, d)
end

function cpMat2x2Transform(m, v)
    ccall((:cpMat2x2Transform, libchipmunk_name), cpVect, (cpMat2x2, cpVect), m, v)
end

function cpBBNew(l, b, r, t)
    ccall((:cpBBNew, libchipmunk_name), cpBB, (cpFloat, cpFloat, cpFloat, cpFloat), l, b, r, t)
end

function cpBBNewForExtents(c, hw, hh)
    ccall((:cpBBNewForExtents, libchipmunk_name), cpBB, (cpVect, cpFloat, cpFloat), c, hw, hh)
end

function cpBBNewForCircle(p, r)
    ccall((:cpBBNewForCircle, libchipmunk_name), cpBB, (cpVect, cpFloat), p, r)
end

function cpBBIntersects(a, b)
    ccall((:cpBBIntersects, libchipmunk_name), cpBool, (cpBB, cpBB), a, b)
end

function cpBBContainsBB(bb, other)
    ccall((:cpBBContainsBB, libchipmunk_name), cpBool, (cpBB, cpBB), bb, other)
end

function cpBBContainsVect(bb, v)
    ccall((:cpBBContainsVect, libchipmunk_name), cpBool, (cpBB, cpVect), bb, v)
end

function cpBBMerge(a, b)
    ccall((:cpBBMerge, libchipmunk_name), cpBB, (cpBB, cpBB), a, b)
end

function cpBBExpand(bb, v)
    ccall((:cpBBExpand, libchipmunk_name), cpBB, (cpBB, cpVect), bb, v)
end

function cpBBCenter(bb)
    ccall((:cpBBCenter, libchipmunk_name), cpVect, (cpBB,), bb)
end

function cpBBArea(bb)
    ccall((:cpBBArea, libchipmunk_name), cpFloat, (cpBB,), bb)
end

function cpBBMergedArea(a, b)
    ccall((:cpBBMergedArea, libchipmunk_name), cpFloat, (cpBB, cpBB), a, b)
end

function cpBBSegmentQuery(bb, a, b)
    ccall((:cpBBSegmentQuery, libchipmunk_name), cpFloat, (cpBB, cpVect, cpVect), bb, a, b)
end

function cpBBIntersectsSegment(bb, a, b)
    ccall((:cpBBIntersectsSegment, libchipmunk_name), cpBool, (cpBB, cpVect, cpVect), bb, a, b)
end

function cpBBClampVect(bb, v)
    ccall((:cpBBClampVect, libchipmunk_name), cpVect, (cpBB, cpVect), bb, v)
end

function cpBBWrapVect(bb, v)
    ccall((:cpBBWrapVect, libchipmunk_name), cpVect, (cpBB, cpVect), bb, v)
end

function cpBBOffset(bb, v)
    ccall((:cpBBOffset, libchipmunk_name), cpBB, (cpBB, cpVect), bb, v)
end

function cpTransformNew(a, b, c, d, tx, ty)
    ccall((:cpTransformNew, libchipmunk_name), cpTransform, (cpFloat, cpFloat, cpFloat, cpFloat, cpFloat, cpFloat), a, b, c, d, tx, ty)
end

function cpTransformNewTranspose(a, c, tx, b, d, ty)
    ccall((:cpTransformNewTranspose, libchipmunk_name), cpTransform, (cpFloat, cpFloat, cpFloat, cpFloat, cpFloat, cpFloat), a, c, tx, b, d, ty)
end

function cpTransformInverse(t)
    ccall((:cpTransformInverse, libchipmunk_name), cpTransform, (cpTransform,), t)
end

function cpTransformMult(t1, t2)
    ccall((:cpTransformMult, libchipmunk_name), cpTransform, (cpTransform, cpTransform), t1, t2)
end

function cpTransformPoint(t, p)
    ccall((:cpTransformPoint, libchipmunk_name), cpVect, (cpTransform, cpVect), t, p)
end

function cpTransformVect(t, v)
    ccall((:cpTransformVect, libchipmunk_name), cpVect, (cpTransform, cpVect), t, v)
end

function cpTransformbBB(t, bb)
    ccall((:cpTransformbBB, libchipmunk_name), cpBB, (cpTransform, cpBB), t, bb)
end

function cpTransformTranslate(translate)
    ccall((:cpTransformTranslate, libchipmunk_name), cpTransform, (cpVect,), translate)
end

function cpTransformScale(scaleX, scaleY)
    ccall((:cpTransformScale, libchipmunk_name), cpTransform, (cpFloat, cpFloat), scaleX, scaleY)
end

function cpTransformRotate(radians)
    ccall((:cpTransformRotate, libchipmunk_name), cpTransform, (cpFloat,), radians)
end

function cpTransformRigid(translate, radians)
    ccall((:cpTransformRigid, libchipmunk_name), cpTransform, (cpVect, cpFloat), translate, radians)
end

function cpTransformRigidInverse(t)
    ccall((:cpTransformRigidInverse, libchipmunk_name), cpTransform, (cpTransform,), t)
end

function cpTransformWrap(outer, inner)
    ccall((:cpTransformWrap, libchipmunk_name), cpTransform, (cpTransform, cpTransform), outer, inner)
end

function cpTransformWrapInverse(outer, inner)
    ccall((:cpTransformWrapInverse, libchipmunk_name), cpTransform, (cpTransform, cpTransform), outer, inner)
end

function cpTransformOrtho(bb)
    ccall((:cpTransformOrtho, libchipmunk_name), cpTransform, (cpBB,), bb)
end

function cpTransformBoneScale(v0, v1)
    ccall((:cpTransformBoneScale, libchipmunk_name), cpTransform, (cpVect, cpVect), v0, v1)
end

function cpTransformAxialScale(axis, pivot, scale)
    ccall((:cpTransformAxialScale, libchipmunk_name), cpTransform, (cpVect, cpVect, cpFloat), axis, pivot, scale)
end

function cpSpaceHashAlloc()
    ccall((:cpSpaceHashAlloc, libchipmunk_name), Ptr{cpSpaceHash}, ())
end

function cpSpaceHashInit(hash, celldim, numcells, bbfunc, staticIndex)
    ccall((:cpSpaceHashInit, libchipmunk_name), Ptr{cpSpatialIndex}, (Ptr{cpSpaceHash}, cpFloat, Cint, cpSpatialIndexBBFunc, Ptr{cpSpatialIndex}), hash, celldim, numcells, bbfunc, staticIndex)
end

function cpSpaceHashNew(celldim, cells, bbfunc, staticIndex)
    ccall((:cpSpaceHashNew, libchipmunk_name), Ptr{cpSpatialIndex}, (cpFloat, Cint, cpSpatialIndexBBFunc, Ptr{cpSpatialIndex}), celldim, cells, bbfunc, staticIndex)
end

function cpSpaceHashResize(hash, celldim, numcells)
    ccall((:cpSpaceHashResize, libchipmunk_name), Cvoid, (Ptr{cpSpaceHash}, cpFloat, Cint), hash, celldim, numcells)
end

function cpBBTreeAlloc()
    ccall((:cpBBTreeAlloc, libchipmunk_name), Ptr{cpBBTree}, ())
end

function cpBBTreeInit(tree, bbfunc, staticIndex)
    ccall((:cpBBTreeInit, libchipmunk_name), Ptr{cpSpatialIndex}, (Ptr{cpBBTree}, cpSpatialIndexBBFunc, Ptr{cpSpatialIndex}), tree, bbfunc, staticIndex)
end

function cpBBTreeNew(bbfunc, staticIndex)
    ccall((:cpBBTreeNew, libchipmunk_name), Ptr{cpSpatialIndex}, (cpSpatialIndexBBFunc, Ptr{cpSpatialIndex}), bbfunc, staticIndex)
end

function cpBBTreeOptimize(index)
    ccall((:cpBBTreeOptimize, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex},), index)
end

function cpBBTreeSetVelocityFunc(index, func)
    ccall((:cpBBTreeSetVelocityFunc, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, cpBBTreeVelocityFunc), index, func)
end

function cpSweep1DAlloc()
    ccall((:cpSweep1DAlloc, libchipmunk_name), Ptr{cpSweep1D}, ())
end

function cpSweep1DInit(sweep, bbfunc, staticIndex)
    ccall((:cpSweep1DInit, libchipmunk_name), Ptr{cpSpatialIndex}, (Ptr{cpSweep1D}, cpSpatialIndexBBFunc, Ptr{cpSpatialIndex}), sweep, bbfunc, staticIndex)
end

function cpSweep1DNew(bbfunc, staticIndex)
    ccall((:cpSweep1DNew, libchipmunk_name), Ptr{cpSpatialIndex}, (cpSpatialIndexBBFunc, Ptr{cpSpatialIndex}), bbfunc, staticIndex)
end

function cpSpatialIndexFree(index)
    ccall((:cpSpatialIndexFree, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex},), index)
end

function cpSpatialIndexCollideStatic(dynamicIndex, staticIndex, func, data)
    ccall((:cpSpatialIndexCollideStatic, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, Ptr{cpSpatialIndex}, cpSpatialIndexQueryFunc, Ptr{Cvoid}), dynamicIndex, staticIndex, func, data)
end

function cpSpatialIndexDestroy(index)
    ccall((:cpSpatialIndexDestroy, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex},), index)
end

function cpSpatialIndexCount(index)
    ccall((:cpSpatialIndexCount, libchipmunk_name), Cint, (Ptr{cpSpatialIndex},), index)
end

function cpSpatialIndexEach(index, func, data)
    ccall((:cpSpatialIndexEach, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, cpSpatialIndexIteratorFunc, Ptr{Cvoid}), index, func, data)
end

function cpSpatialIndexContains(index, obj, hashid)
    ccall((:cpSpatialIndexContains, libchipmunk_name), cpBool, (Ptr{cpSpatialIndex}, Ptr{Cvoid}, cpHashValue), index, obj, hashid)
end

function cpSpatialIndexInsert(index, obj, hashid)
    ccall((:cpSpatialIndexInsert, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, Ptr{Cvoid}, cpHashValue), index, obj, hashid)
end

function cpSpatialIndexRemove(index, obj, hashid)
    ccall((:cpSpatialIndexRemove, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, Ptr{Cvoid}, cpHashValue), index, obj, hashid)
end

function cpSpatialIndexReindex(index)
    ccall((:cpSpatialIndexReindex, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex},), index)
end

function cpSpatialIndexReindexObject(index, obj, hashid)
    ccall((:cpSpatialIndexReindexObject, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, Ptr{Cvoid}, cpHashValue), index, obj, hashid)
end

function cpSpatialIndexQuery(index, obj, bb, func, data)
    ccall((:cpSpatialIndexQuery, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, Ptr{Cvoid}, cpBB, cpSpatialIndexQueryFunc, Ptr{Cvoid}), index, obj, bb, func, data)
end

function cpSpatialIndexSegmentQuery(index, obj, a, b, t_exit, func, data)
    ccall((:cpSpatialIndexSegmentQuery, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, Ptr{Cvoid}, cpVect, cpVect, cpFloat, cpSpatialIndexSegmentQueryFunc, Ptr{Cvoid}), index, obj, a, b, t_exit, func, data)
end

function cpSpatialIndexReindexQuery(index, func, data)
    ccall((:cpSpatialIndexReindexQuery, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, cpSpatialIndexQueryFunc, Ptr{Cvoid}), index, func, data)
end

function cpArbiterGetRestitution(arb)
    ccall((:cpArbiterGetRestitution, libchipmunk_name), cpFloat, (Ptr{cpArbiter},), arb)
end

function cpArbiterSetRestitution(arb, restitution)
    ccall((:cpArbiterSetRestitution, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, cpFloat), arb, restitution)
end

function cpArbiterGetFriction(arb)
    ccall((:cpArbiterGetFriction, libchipmunk_name), cpFloat, (Ptr{cpArbiter},), arb)
end

function cpArbiterSetFriction(arb, friction)
    ccall((:cpArbiterSetFriction, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, cpFloat), arb, friction)
end

function cpArbiterGetSurfaceVelocity(arb)
    ccall((:cpArbiterGetSurfaceVelocity, libchipmunk_name), cpVect, (Ptr{cpArbiter},), arb)
end

function cpArbiterSetSurfaceVelocity(arb, vr)
    ccall((:cpArbiterSetSurfaceVelocity, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, cpVect), arb, vr)
end

function cpArbiterGetUserData(arb)
    ccall((:cpArbiterGetUserData, libchipmunk_name), cpDataPointer, (Ptr{cpArbiter},), arb)
end

function cpArbiterSetUserData(arb, userData)
    ccall((:cpArbiterSetUserData, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, cpDataPointer), arb, userData)
end

function cpArbiterTotalImpulse(arb)
    ccall((:cpArbiterTotalImpulse, libchipmunk_name), cpVect, (Ptr{cpArbiter},), arb)
end

function cpArbiterTotalKE(arb)
    ccall((:cpArbiterTotalKE, libchipmunk_name), cpFloat, (Ptr{cpArbiter},), arb)
end

function cpArbiterIgnore(arb)
    ccall((:cpArbiterIgnore, libchipmunk_name), cpBool, (Ptr{cpArbiter},), arb)
end

function cpArbiterGetShapes(arb, a, b)
    ccall((:cpArbiterGetShapes, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, Ptr{Ptr{cpShape}}, Ptr{Ptr{cpShape}}), arb, a, b)
end

function cpArbiterGetBodies(arb, a, b)
    ccall((:cpArbiterGetBodies, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, Ptr{Ptr{cpBody}}, Ptr{Ptr{cpBody}}), arb, a, b)
end

function cpArbiterGetContactPointSet(arb)
    ccall((:cpArbiterGetContactPointSet, libchipmunk_name), cpContactPointSet, (Ptr{cpArbiter},), arb)
end

function cpArbiterSetContactPointSet(arb, set)
    ccall((:cpArbiterSetContactPointSet, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, Ptr{cpContactPointSet}), arb, set)
end

function cpArbiterIsFirstContact(arb)
    ccall((:cpArbiterIsFirstContact, libchipmunk_name), cpBool, (Ptr{cpArbiter},), arb)
end

function cpArbiterIsRemoval(arb)
    ccall((:cpArbiterIsRemoval, libchipmunk_name), cpBool, (Ptr{cpArbiter},), arb)
end

function cpArbiterGetCount(arb)
    ccall((:cpArbiterGetCount, libchipmunk_name), Cint, (Ptr{cpArbiter},), arb)
end

function cpArbiterGetNormal(arb)
    ccall((:cpArbiterGetNormal, libchipmunk_name), cpVect, (Ptr{cpArbiter},), arb)
end

function cpArbiterGetPointA(arb, i)
    ccall((:cpArbiterGetPointA, libchipmunk_name), cpVect, (Ptr{cpArbiter}, Cint), arb, i)
end

function cpArbiterGetPointB(arb, i)
    ccall((:cpArbiterGetPointB, libchipmunk_name), cpVect, (Ptr{cpArbiter}, Cint), arb, i)
end

function cpArbiterGetDepth(arb, i)
    ccall((:cpArbiterGetDepth, libchipmunk_name), cpFloat, (Ptr{cpArbiter}, Cint), arb, i)
end

function cpArbiterCallWildcardBeginA(arb, space)
    ccall((:cpArbiterCallWildcardBeginA, libchipmunk_name), cpBool, (Ptr{cpArbiter}, Ptr{cpSpace}), arb, space)
end

function cpArbiterCallWildcardBeginB(arb, space)
    ccall((:cpArbiterCallWildcardBeginB, libchipmunk_name), cpBool, (Ptr{cpArbiter}, Ptr{cpSpace}), arb, space)
end

function cpArbiterCallWildcardPreSolveA(arb, space)
    ccall((:cpArbiterCallWildcardPreSolveA, libchipmunk_name), cpBool, (Ptr{cpArbiter}, Ptr{cpSpace}), arb, space)
end

function cpArbiterCallWildcardPreSolveB(arb, space)
    ccall((:cpArbiterCallWildcardPreSolveB, libchipmunk_name), cpBool, (Ptr{cpArbiter}, Ptr{cpSpace}), arb, space)
end

function cpArbiterCallWildcardPostSolveA(arb, space)
    ccall((:cpArbiterCallWildcardPostSolveA, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, Ptr{cpSpace}), arb, space)
end

function cpArbiterCallWildcardPostSolveB(arb, space)
    ccall((:cpArbiterCallWildcardPostSolveB, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, Ptr{cpSpace}), arb, space)
end

function cpArbiterCallWildcardSeparateA(arb, space)
    ccall((:cpArbiterCallWildcardSeparateA, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, Ptr{cpSpace}), arb, space)
end

function cpArbiterCallWildcardSeparateB(arb, space)
    ccall((:cpArbiterCallWildcardSeparateB, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, Ptr{cpSpace}), arb, space)
end

function cpBodyAlloc()
    ccall((:cpBodyAlloc, libchipmunk_name), Ptr{cpBody}, ())
end

function cpBodyInit(body, mass, moment)
    ccall((:cpBodyInit, libchipmunk_name), Ptr{cpBody}, (Ptr{cpBody}, cpFloat, cpFloat), body, mass, moment)
end

function cpBodyNew(mass, moment)
    ccall((:cpBodyNew, libchipmunk_name), Ptr{cpBody}, (cpFloat, cpFloat), mass, moment)
end

function cpBodyNewKinematic()
    ccall((:cpBodyNewKinematic, libchipmunk_name), Ptr{cpBody}, ())
end

function cpBodyNewStatic()
    ccall((:cpBodyNewStatic, libchipmunk_name), Ptr{cpBody}, ())
end

function cpBodyDestroy(body)
    ccall((:cpBodyDestroy, libchipmunk_name), Cvoid, (Ptr{cpBody},), body)
end

function cpBodyFree(body)
    ccall((:cpBodyFree, libchipmunk_name), Cvoid, (Ptr{cpBody},), body)
end

function cpBodyActivate(body)
    ccall((:cpBodyActivate, libchipmunk_name), Cvoid, (Ptr{cpBody},), body)
end

function cpBodyActivateStatic(body, filter)
    ccall((:cpBodyActivateStatic, libchipmunk_name), Cvoid, (Ptr{cpBody}, Ptr{cpShape}), body, filter)
end

function cpBodySleep(body)
    ccall((:cpBodySleep, libchipmunk_name), Cvoid, (Ptr{cpBody},), body)
end

function cpBodySleepWithGroup(body, group)
    ccall((:cpBodySleepWithGroup, libchipmunk_name), Cvoid, (Ptr{cpBody}, Ptr{cpBody}), body, group)
end

function cpBodyIsSleeping(body)
    ccall((:cpBodyIsSleeping, libchipmunk_name), cpBool, (Ptr{cpBody},), body)
end

function cpBodyGetType(body)
    ccall((:cpBodyGetType, libchipmunk_name), cpBodyType, (Ptr{cpBody},), body)
end

function cpBodySetType(body, type)
    ccall((:cpBodySetType, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpBodyType), body, type)
end

function cpBodyGetSpace(body)
    ccall((:cpBodyGetSpace, libchipmunk_name), Ptr{cpSpace}, (Ptr{cpBody},), body)
end

function cpBodyGetMass(body)
    ccall((:cpBodyGetMass, libchipmunk_name), cpFloat, (Ptr{cpBody},), body)
end

function cpBodySetMass(body, m)
    ccall((:cpBodySetMass, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpFloat), body, m)
end

function cpBodyGetMoment(body)
    ccall((:cpBodyGetMoment, libchipmunk_name), cpFloat, (Ptr{cpBody},), body)
end

function cpBodySetMoment(body, i)
    ccall((:cpBodySetMoment, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpFloat), body, i)
end

function cpBodyGetPosition(body)
    ccall((:cpBodyGetPosition, libchipmunk_name), cpVect, (Ptr{cpBody},), body)
end

function cpBodySetPosition(body, pos)
    ccall((:cpBodySetPosition, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect), body, pos)
end

function cpBodyGetCenterOfGravity(body)
    ccall((:cpBodyGetCenterOfGravity, libchipmunk_name), cpVect, (Ptr{cpBody},), body)
end

function cpBodySetCenterOfGravity(body, cog)
    ccall((:cpBodySetCenterOfGravity, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect), body, cog)
end

function cpBodyGetVelocity(body)
    ccall((:cpBodyGetVelocity, libchipmunk_name), cpVect, (Ptr{cpBody},), body)
end

function cpBodySetVelocity(body, velocity)
    ccall((:cpBodySetVelocity, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect), body, velocity)
end

function cpBodyGetForce(body)
    ccall((:cpBodyGetForce, libchipmunk_name), cpVect, (Ptr{cpBody},), body)
end

function cpBodySetForce(body, force)
    ccall((:cpBodySetForce, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect), body, force)
end

function cpBodyGetAngle(body)
    ccall((:cpBodyGetAngle, libchipmunk_name), cpFloat, (Ptr{cpBody},), body)
end

function cpBodySetAngle(body, a)
    ccall((:cpBodySetAngle, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpFloat), body, a)
end

function cpBodyGetAngularVelocity(body)
    ccall((:cpBodyGetAngularVelocity, libchipmunk_name), cpFloat, (Ptr{cpBody},), body)
end

function cpBodySetAngularVelocity(body, angularVelocity)
    ccall((:cpBodySetAngularVelocity, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpFloat), body, angularVelocity)
end

function cpBodyGetTorque(body)
    ccall((:cpBodyGetTorque, libchipmunk_name), cpFloat, (Ptr{cpBody},), body)
end

function cpBodySetTorque(body, torque)
    ccall((:cpBodySetTorque, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpFloat), body, torque)
end

function cpBodyGetRotation(body)
    ccall((:cpBodyGetRotation, libchipmunk_name), cpVect, (Ptr{cpBody},), body)
end

function cpBodyGetUserData(body)
    ccall((:cpBodyGetUserData, libchipmunk_name), cpDataPointer, (Ptr{cpBody},), body)
end

function cpBodySetUserData(body, userData)
    ccall((:cpBodySetUserData, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpDataPointer), body, userData)
end

function cpBodySetVelocityUpdateFunc(body, velocityFunc)
    ccall((:cpBodySetVelocityUpdateFunc, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpBodyVelocityFunc), body, velocityFunc)
end

function cpBodySetPositionUpdateFunc(body, positionFunc)
    ccall((:cpBodySetPositionUpdateFunc, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpBodyPositionFunc), body, positionFunc)
end

function cpBodyUpdateVelocity(body, gravity, damping, dt)
    ccall((:cpBodyUpdateVelocity, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect, cpFloat, cpFloat), body, gravity, damping, dt)
end

function cpBodyUpdatePosition(body, dt)
    ccall((:cpBodyUpdatePosition, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpFloat), body, dt)
end

function cpBodyLocalToWorld(body, point)
    ccall((:cpBodyLocalToWorld, libchipmunk_name), cpVect, (Ptr{cpBody}, cpVect), body, point)
end

function cpBodyWorldToLocal(body, point)
    ccall((:cpBodyWorldToLocal, libchipmunk_name), cpVect, (Ptr{cpBody}, cpVect), body, point)
end

function cpBodyApplyForceAtWorldPoint(body, force, point)
    ccall((:cpBodyApplyForceAtWorldPoint, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect, cpVect), body, force, point)
end

function cpBodyApplyForceAtLocalPoint(body, force, point)
    ccall((:cpBodyApplyForceAtLocalPoint, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect, cpVect), body, force, point)
end

function cpBodyApplyImpulseAtWorldPoint(body, impulse, point)
    ccall((:cpBodyApplyImpulseAtWorldPoint, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect, cpVect), body, impulse, point)
end

function cpBodyApplyImpulseAtLocalPoint(body, impulse, point)
    ccall((:cpBodyApplyImpulseAtLocalPoint, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect, cpVect), body, impulse, point)
end

function cpBodyGetVelocityAtWorldPoint(body, point)
    ccall((:cpBodyGetVelocityAtWorldPoint, libchipmunk_name), cpVect, (Ptr{cpBody}, cpVect), body, point)
end

function cpBodyGetVelocityAtLocalPoint(body, point)
    ccall((:cpBodyGetVelocityAtLocalPoint, libchipmunk_name), cpVect, (Ptr{cpBody}, cpVect), body, point)
end

function cpBodyKineticEnergy(body)
    ccall((:cpBodyKineticEnergy, libchipmunk_name), cpFloat, (Ptr{cpBody},), body)
end

function cpBodyEachShape(body, func, data)
    ccall((:cpBodyEachShape, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpBodyShapeIteratorFunc, Ptr{Cvoid}), body, func, data)
end

function cpBodyEachConstraint(body, func, data)
    ccall((:cpBodyEachConstraint, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpBodyConstraintIteratorFunc, Ptr{Cvoid}), body, func, data)
end

function cpBodyEachArbiter(body, func, data)
    ccall((:cpBodyEachArbiter, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpBodyArbiterIteratorFunc, Ptr{Cvoid}), body, func, data)
end

function cpShapeFilterNew(group, categories, mask)
    ccall((:cpShapeFilterNew, libchipmunk_name), cpShapeFilter, (cpGroup, cpBitmask, cpBitmask), group, categories, mask)
end

function cpShapeDestroy(shape)
    ccall((:cpShapeDestroy, libchipmunk_name), Cvoid, (Ptr{cpShape},), shape)
end

function cpShapeFree(shape)
    ccall((:cpShapeFree, libchipmunk_name), Cvoid, (Ptr{cpShape},), shape)
end

function cpShapeCacheBB(shape)
    ccall((:cpShapeCacheBB, libchipmunk_name), cpBB, (Ptr{cpShape},), shape)
end

function cpShapeUpdate(shape, transform)
    ccall((:cpShapeUpdate, libchipmunk_name), cpBB, (Ptr{cpShape}, cpTransform), shape, transform)
end

function cpShapePointQuery(shape, p, out)
    ccall((:cpShapePointQuery, libchipmunk_name), cpFloat, (Ptr{cpShape}, cpVect, Ptr{cpPointQueryInfo}), shape, p, out)
end

function cpShapeSegmentQuery(shape, a, b, radius, info)
    ccall((:cpShapeSegmentQuery, libchipmunk_name), cpBool, (Ptr{cpShape}, cpVect, cpVect, cpFloat, Ptr{cpSegmentQueryInfo}), shape, a, b, radius, info)
end

function cpShapesCollide(a, b)
    ccall((:cpShapesCollide, libchipmunk_name), cpContactPointSet, (Ptr{cpShape}, Ptr{cpShape}), a, b)
end

function cpShapeGetSpace(shape)
    ccall((:cpShapeGetSpace, libchipmunk_name), Ptr{cpSpace}, (Ptr{cpShape},), shape)
end

function cpShapeGetBody(shape)
    ccall((:cpShapeGetBody, libchipmunk_name), Ptr{cpBody}, (Ptr{cpShape},), shape)
end

function cpShapeSetBody(shape, body)
    ccall((:cpShapeSetBody, libchipmunk_name), Cvoid, (Ptr{cpShape}, Ptr{cpBody}), shape, body)
end

function cpShapeGetMass(shape)
    ccall((:cpShapeGetMass, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpShapeSetMass(shape, mass)
    ccall((:cpShapeSetMass, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpFloat), shape, mass)
end

function cpShapeGetDensity(shape)
    ccall((:cpShapeGetDensity, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpShapeSetDensity(shape, density)
    ccall((:cpShapeSetDensity, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpFloat), shape, density)
end

function cpShapeGetMoment(shape)
    ccall((:cpShapeGetMoment, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpShapeGetArea(shape)
    ccall((:cpShapeGetArea, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpShapeGetCenterOfGravity(shape)
    ccall((:cpShapeGetCenterOfGravity, libchipmunk_name), cpVect, (Ptr{cpShape},), shape)
end

function cpShapeGetBB(shape)
    ccall((:cpShapeGetBB, libchipmunk_name), cpBB, (Ptr{cpShape},), shape)
end

function cpShapeGetSensor(shape)
    ccall((:cpShapeGetSensor, libchipmunk_name), cpBool, (Ptr{cpShape},), shape)
end

function cpShapeSetSensor(shape, sensor)
    ccall((:cpShapeSetSensor, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpBool), shape, sensor)
end

function cpShapeGetElasticity(shape)
    ccall((:cpShapeGetElasticity, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpShapeSetElasticity(shape, elasticity)
    ccall((:cpShapeSetElasticity, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpFloat), shape, elasticity)
end

function cpShapeGetFriction(shape)
    ccall((:cpShapeGetFriction, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpShapeSetFriction(shape, friction)
    ccall((:cpShapeSetFriction, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpFloat), shape, friction)
end

function cpShapeGetSurfaceVelocity(shape)
    ccall((:cpShapeGetSurfaceVelocity, libchipmunk_name), cpVect, (Ptr{cpShape},), shape)
end

function cpShapeSetSurfaceVelocity(shape, surfaceVelocity)
    ccall((:cpShapeSetSurfaceVelocity, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpVect), shape, surfaceVelocity)
end

function cpShapeGetUserData(shape)
    ccall((:cpShapeGetUserData, libchipmunk_name), cpDataPointer, (Ptr{cpShape},), shape)
end

function cpShapeSetUserData(shape, userData)
    ccall((:cpShapeSetUserData, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpDataPointer), shape, userData)
end

function cpShapeGetCollisionType(shape)
    ccall((:cpShapeGetCollisionType, libchipmunk_name), cpCollisionType, (Ptr{cpShape},), shape)
end

function cpShapeSetCollisionType(shape, collisionType)
    ccall((:cpShapeSetCollisionType, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpCollisionType), shape, collisionType)
end

function cpShapeGetFilter(shape)
    ccall((:cpShapeGetFilter, libchipmunk_name), cpShapeFilter, (Ptr{cpShape},), shape)
end

function cpShapeSetFilter(shape, filter)
    ccall((:cpShapeSetFilter, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpShapeFilter), shape, filter)
end

function cpCircleShapeAlloc()
    ccall((:cpCircleShapeAlloc, libchipmunk_name), Ptr{cpCircleShape}, ())
end

function cpCircleShapeInit(circle, body, radius, offset)
    ccall((:cpCircleShapeInit, libchipmunk_name), Ptr{cpCircleShape}, (Ptr{cpCircleShape}, Ptr{cpBody}, cpFloat, cpVect), circle, body, radius, offset)
end

function cpCircleShapeNew(body, radius, offset)
    ccall((:cpCircleShapeNew, libchipmunk_name), Ptr{cpShape}, (Ptr{cpBody}, cpFloat, cpVect), body, radius, offset)
end

function cpCircleShapeGetOffset(shape)
    ccall((:cpCircleShapeGetOffset, libchipmunk_name), cpVect, (Ptr{cpShape},), shape)
end

function cpCircleShapeGetRadius(shape)
    ccall((:cpCircleShapeGetRadius, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpSegmentShapeAlloc()
    ccall((:cpSegmentShapeAlloc, libchipmunk_name), Ptr{cpSegmentShape}, ())
end

function cpSegmentShapeInit(seg, body, a, b, radius)
    ccall((:cpSegmentShapeInit, libchipmunk_name), Ptr{cpSegmentShape}, (Ptr{cpSegmentShape}, Ptr{cpBody}, cpVect, cpVect, cpFloat), seg, body, a, b, radius)
end

function cpSegmentShapeNew(body, a, b, radius)
    ccall((:cpSegmentShapeNew, libchipmunk_name), Ptr{cpShape}, (Ptr{cpBody}, cpVect, cpVect, cpFloat), body, a, b, radius)
end

function cpSegmentShapeSetNeighbors(shape, prev, next)
    ccall((:cpSegmentShapeSetNeighbors, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpVect, cpVect), shape, prev, next)
end

function cpSegmentShapeGetA(shape)
    ccall((:cpSegmentShapeGetA, libchipmunk_name), cpVect, (Ptr{cpShape},), shape)
end

function cpSegmentShapeGetB(shape)
    ccall((:cpSegmentShapeGetB, libchipmunk_name), cpVect, (Ptr{cpShape},), shape)
end

function cpSegmentShapeGetNormal(shape)
    ccall((:cpSegmentShapeGetNormal, libchipmunk_name), cpVect, (Ptr{cpShape},), shape)
end

function cpSegmentShapeGetRadius(shape)
    ccall((:cpSegmentShapeGetRadius, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpPolyShapeAlloc()
    ccall((:cpPolyShapeAlloc, libchipmunk_name), Ptr{cpPolyShape}, ())
end

function cpPolyShapeInit(poly, body, count, verts, transform, radius)
    ccall((:cpPolyShapeInit, libchipmunk_name), Ptr{cpPolyShape}, (Ptr{cpPolyShape}, Ptr{cpBody}, Cint, Ptr{cpVect}, cpTransform, cpFloat), poly, body, count, verts, transform, radius)
end

function cpPolyShapeInitRaw(poly, body, count, verts, radius)
    ccall((:cpPolyShapeInitRaw, libchipmunk_name), Ptr{cpPolyShape}, (Ptr{cpPolyShape}, Ptr{cpBody}, Cint, Ptr{cpVect}, cpFloat), poly, body, count, verts, radius)
end

function cpPolyShapeNew(body, count, verts, transform, radius)
    ccall((:cpPolyShapeNew, libchipmunk_name), Ptr{cpShape}, (Ptr{cpBody}, Cint, Ptr{cpVect}, cpTransform, cpFloat), body, count, verts, transform, radius)
end

function cpPolyShapeNewRaw(body, count, verts, radius)
    ccall((:cpPolyShapeNewRaw, libchipmunk_name), Ptr{cpShape}, (Ptr{cpBody}, Cint, Ptr{cpVect}, cpFloat), body, count, verts, radius)
end

function cpBoxShapeInit(poly, body, width, height, radius)
    ccall((:cpBoxShapeInit, libchipmunk_name), Ptr{cpPolyShape}, (Ptr{cpPolyShape}, Ptr{cpBody}, cpFloat, cpFloat, cpFloat), poly, body, width, height, radius)
end

function cpBoxShapeInit2(poly, body, box, radius)
    ccall((:cpBoxShapeInit2, libchipmunk_name), Ptr{cpPolyShape}, (Ptr{cpPolyShape}, Ptr{cpBody}, cpBB, cpFloat), poly, body, box, radius)
end

function cpBoxShapeNew(body, width, height, radius)
    ccall((:cpBoxShapeNew, libchipmunk_name), Ptr{cpShape}, (Ptr{cpBody}, cpFloat, cpFloat, cpFloat), body, width, height, radius)
end

function cpBoxShapeNew2(body, box, radius)
    ccall((:cpBoxShapeNew2, libchipmunk_name), Ptr{cpShape}, (Ptr{cpBody}, cpBB, cpFloat), body, box, radius)
end

function cpPolyShapeGetCount(shape)
    ccall((:cpPolyShapeGetCount, libchipmunk_name), Cint, (Ptr{cpShape},), shape)
end

function cpPolyShapeGetVert(shape, index)
    ccall((:cpPolyShapeGetVert, libchipmunk_name), cpVect, (Ptr{cpShape}, Cint), shape, index)
end

function cpPolyShapeGetRadius(shape)
    ccall((:cpPolyShapeGetRadius, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpConstraintDestroy(constraint)
    ccall((:cpConstraintDestroy, libchipmunk_name), Cvoid, (Ptr{cpConstraint},), constraint)
end

function cpConstraintFree(constraint)
    ccall((:cpConstraintFree, libchipmunk_name), Cvoid, (Ptr{cpConstraint},), constraint)
end

function cpConstraintGetSpace(constraint)
    ccall((:cpConstraintGetSpace, libchipmunk_name), Ptr{cpSpace}, (Ptr{cpConstraint},), constraint)
end

function cpConstraintGetBodyA(constraint)
    ccall((:cpConstraintGetBodyA, libchipmunk_name), Ptr{cpBody}, (Ptr{cpConstraint},), constraint)
end

function cpConstraintGetBodyB(constraint)
    ccall((:cpConstraintGetBodyB, libchipmunk_name), Ptr{cpBody}, (Ptr{cpConstraint},), constraint)
end

function cpConstraintGetMaxForce(constraint)
    ccall((:cpConstraintGetMaxForce, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpConstraintSetMaxForce(constraint, maxForce)
    ccall((:cpConstraintSetMaxForce, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, maxForce)
end

function cpConstraintGetErrorBias(constraint)
    ccall((:cpConstraintGetErrorBias, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpConstraintSetErrorBias(constraint, errorBias)
    ccall((:cpConstraintSetErrorBias, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, errorBias)
end

function cpConstraintGetMaxBias(constraint)
    ccall((:cpConstraintGetMaxBias, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpConstraintSetMaxBias(constraint, maxBias)
    ccall((:cpConstraintSetMaxBias, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, maxBias)
end

function cpConstraintGetCollideBodies(constraint)
    ccall((:cpConstraintGetCollideBodies, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpConstraintSetCollideBodies(constraint, collideBodies)
    ccall((:cpConstraintSetCollideBodies, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpBool), constraint, collideBodies)
end

function cpConstraintGetPreSolveFunc(constraint)
    ccall((:cpConstraintGetPreSolveFunc, libchipmunk_name), cpConstraintPreSolveFunc, (Ptr{cpConstraint},), constraint)
end

function cpConstraintSetPreSolveFunc(constraint, preSolveFunc)
    ccall((:cpConstraintSetPreSolveFunc, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpConstraintPreSolveFunc), constraint, preSolveFunc)
end

function cpConstraintGetPostSolveFunc(constraint)
    ccall((:cpConstraintGetPostSolveFunc, libchipmunk_name), cpConstraintPostSolveFunc, (Ptr{cpConstraint},), constraint)
end

function cpConstraintSetPostSolveFunc(constraint, postSolveFunc)
    ccall((:cpConstraintSetPostSolveFunc, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpConstraintPostSolveFunc), constraint, postSolveFunc)
end

function cpConstraintGetUserData(constraint)
    ccall((:cpConstraintGetUserData, libchipmunk_name), cpDataPointer, (Ptr{cpConstraint},), constraint)
end

function cpConstraintSetUserData(constraint, userData)
    ccall((:cpConstraintSetUserData, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpDataPointer), constraint, userData)
end

function cpConstraintGetImpulse(constraint)
    ccall((:cpConstraintGetImpulse, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpConstraintIsPinJoint(constraint)
    ccall((:cpConstraintIsPinJoint, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpPinJointAlloc()
    ccall((:cpPinJointAlloc, libchipmunk_name), Ptr{cpPinJoint}, ())
end

function cpPinJointInit(joint, a, b, anchorA, anchorB)
    ccall((:cpPinJointInit, libchipmunk_name), Ptr{cpPinJoint}, (Ptr{cpPinJoint}, Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect), joint, a, b, anchorA, anchorB)
end

function cpPinJointNew(a, b, anchorA, anchorB)
    ccall((:cpPinJointNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect), a, b, anchorA, anchorB)
end

function cpPinJointGetAnchorA(constraint)
    ccall((:cpPinJointGetAnchorA, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpPinJointSetAnchorA(constraint, anchorA)
    ccall((:cpPinJointSetAnchorA, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorA)
end

function cpPinJointGetAnchorB(constraint)
    ccall((:cpPinJointGetAnchorB, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpPinJointSetAnchorB(constraint, anchorB)
    ccall((:cpPinJointSetAnchorB, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorB)
end

function cpPinJointGetDist(constraint)
    ccall((:cpPinJointGetDist, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpPinJointSetDist(constraint, dist)
    ccall((:cpPinJointSetDist, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, dist)
end

function cpConstraintIsSlideJoint(constraint)
    ccall((:cpConstraintIsSlideJoint, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpSlideJointAlloc()
    ccall((:cpSlideJointAlloc, libchipmunk_name), Ptr{cpSlideJoint}, ())
end

function cpSlideJointInit(joint, a, b, anchorA, anchorB, min, max)
    ccall((:cpSlideJointInit, libchipmunk_name), Ptr{cpSlideJoint}, (Ptr{cpSlideJoint}, Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect, cpFloat, cpFloat), joint, a, b, anchorA, anchorB, min, max)
end

function cpSlideJointNew(a, b, anchorA, anchorB, min, max)
    ccall((:cpSlideJointNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect, cpFloat, cpFloat), a, b, anchorA, anchorB, min, max)
end

function cpSlideJointGetAnchorA(constraint)
    ccall((:cpSlideJointGetAnchorA, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpSlideJointSetAnchorA(constraint, anchorA)
    ccall((:cpSlideJointSetAnchorA, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorA)
end

function cpSlideJointGetAnchorB(constraint)
    ccall((:cpSlideJointGetAnchorB, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpSlideJointSetAnchorB(constraint, anchorB)
    ccall((:cpSlideJointSetAnchorB, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorB)
end

function cpSlideJointGetMin(constraint)
    ccall((:cpSlideJointGetMin, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpSlideJointSetMin(constraint, min)
    ccall((:cpSlideJointSetMin, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, min)
end

function cpSlideJointGetMax(constraint)
    ccall((:cpSlideJointGetMax, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpSlideJointSetMax(constraint, max)
    ccall((:cpSlideJointSetMax, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, max)
end

function cpConstraintIsPivotJoint(constraint)
    ccall((:cpConstraintIsPivotJoint, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpPivotJointAlloc()
    ccall((:cpPivotJointAlloc, libchipmunk_name), Ptr{cpPivotJoint}, ())
end

function cpPivotJointInit(joint, a, b, anchorA, anchorB)
    ccall((:cpPivotJointInit, libchipmunk_name), Ptr{cpPivotJoint}, (Ptr{cpPivotJoint}, Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect), joint, a, b, anchorA, anchorB)
end

function cpPivotJointNew(a, b, pivot)
    ccall((:cpPivotJointNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpVect), a, b, pivot)
end

function cpPivotJointNew2(a, b, anchorA, anchorB)
    ccall((:cpPivotJointNew2, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect), a, b, anchorA, anchorB)
end

function cpPivotJointGetAnchorA(constraint)
    ccall((:cpPivotJointGetAnchorA, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpPivotJointSetAnchorA(constraint, anchorA)
    ccall((:cpPivotJointSetAnchorA, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorA)
end

function cpPivotJointGetAnchorB(constraint)
    ccall((:cpPivotJointGetAnchorB, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpPivotJointSetAnchorB(constraint, anchorB)
    ccall((:cpPivotJointSetAnchorB, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorB)
end

function cpConstraintIsGrooveJoint(constraint)
    ccall((:cpConstraintIsGrooveJoint, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpGrooveJointAlloc()
    ccall((:cpGrooveJointAlloc, libchipmunk_name), Ptr{cpGrooveJoint}, ())
end

function cpGrooveJointInit(joint, a, b, groove_a, groove_b, anchorB)
    ccall((:cpGrooveJointInit, libchipmunk_name), Ptr{cpGrooveJoint}, (Ptr{cpGrooveJoint}, Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect, cpVect), joint, a, b, groove_a, groove_b, anchorB)
end

function cpGrooveJointNew(a, b, groove_a, groove_b, anchorB)
    ccall((:cpGrooveJointNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect, cpVect), a, b, groove_a, groove_b, anchorB)
end

function cpGrooveJointGetGrooveA(constraint)
    ccall((:cpGrooveJointGetGrooveA, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpGrooveJointSetGrooveA(constraint, grooveA)
    ccall((:cpGrooveJointSetGrooveA, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, grooveA)
end

function cpGrooveJointGetGrooveB(constraint)
    ccall((:cpGrooveJointGetGrooveB, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpGrooveJointSetGrooveB(constraint, grooveB)
    ccall((:cpGrooveJointSetGrooveB, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, grooveB)
end

function cpGrooveJointGetAnchorB(constraint)
    ccall((:cpGrooveJointGetAnchorB, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpGrooveJointSetAnchorB(constraint, anchorB)
    ccall((:cpGrooveJointSetAnchorB, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorB)
end

function cpConstraintIsDampedSpring(constraint)
    ccall((:cpConstraintIsDampedSpring, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpDampedSpringAlloc()
    ccall((:cpDampedSpringAlloc, libchipmunk_name), Ptr{cpDampedSpring}, ())
end

function cpDampedSpringInit(joint, a, b, anchorA, anchorB, restLength, stiffness, damping)
    ccall((:cpDampedSpringInit, libchipmunk_name), Ptr{cpDampedSpring}, (Ptr{cpDampedSpring}, Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect, cpFloat, cpFloat, cpFloat), joint, a, b, anchorA, anchorB, restLength, stiffness, damping)
end

function cpDampedSpringNew(a, b, anchorA, anchorB, restLength, stiffness, damping)
    ccall((:cpDampedSpringNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect, cpFloat, cpFloat, cpFloat), a, b, anchorA, anchorB, restLength, stiffness, damping)
end

function cpDampedSpringGetAnchorA(constraint)
    ccall((:cpDampedSpringGetAnchorA, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpDampedSpringSetAnchorA(constraint, anchorA)
    ccall((:cpDampedSpringSetAnchorA, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorA)
end

function cpDampedSpringGetAnchorB(constraint)
    ccall((:cpDampedSpringGetAnchorB, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpDampedSpringSetAnchorB(constraint, anchorB)
    ccall((:cpDampedSpringSetAnchorB, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorB)
end

function cpDampedSpringGetRestLength(constraint)
    ccall((:cpDampedSpringGetRestLength, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpDampedSpringSetRestLength(constraint, restLength)
    ccall((:cpDampedSpringSetRestLength, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, restLength)
end

function cpDampedSpringGetStiffness(constraint)
    ccall((:cpDampedSpringGetStiffness, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpDampedSpringSetStiffness(constraint, stiffness)
    ccall((:cpDampedSpringSetStiffness, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, stiffness)
end

function cpDampedSpringGetDamping(constraint)
    ccall((:cpDampedSpringGetDamping, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpDampedSpringSetDamping(constraint, damping)
    ccall((:cpDampedSpringSetDamping, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, damping)
end

function cpDampedSpringGetSpringForceFunc(constraint)
    ccall((:cpDampedSpringGetSpringForceFunc, libchipmunk_name), cpDampedSpringForceFunc, (Ptr{cpConstraint},), constraint)
end

function cpDampedSpringSetSpringForceFunc(constraint, springForceFunc)
    ccall((:cpDampedSpringSetSpringForceFunc, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpDampedSpringForceFunc), constraint, springForceFunc)
end

function cpConstraintIsDampedRotarySpring(constraint)
    ccall((:cpConstraintIsDampedRotarySpring, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpDampedRotarySpringAlloc()
    ccall((:cpDampedRotarySpringAlloc, libchipmunk_name), Ptr{cpDampedRotarySpring}, ())
end

function cpDampedRotarySpringInit(joint, a, b, restAngle, stiffness, damping)
    ccall((:cpDampedRotarySpringInit, libchipmunk_name), Ptr{cpDampedRotarySpring}, (Ptr{cpDampedRotarySpring}, Ptr{cpBody}, Ptr{cpBody}, cpFloat, cpFloat, cpFloat), joint, a, b, restAngle, stiffness, damping)
end

function cpDampedRotarySpringNew(a, b, restAngle, stiffness, damping)
    ccall((:cpDampedRotarySpringNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpFloat, cpFloat, cpFloat), a, b, restAngle, stiffness, damping)
end

function cpDampedRotarySpringGetRestAngle(constraint)
    ccall((:cpDampedRotarySpringGetRestAngle, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpDampedRotarySpringSetRestAngle(constraint, restAngle)
    ccall((:cpDampedRotarySpringSetRestAngle, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, restAngle)
end

function cpDampedRotarySpringGetStiffness(constraint)
    ccall((:cpDampedRotarySpringGetStiffness, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpDampedRotarySpringSetStiffness(constraint, stiffness)
    ccall((:cpDampedRotarySpringSetStiffness, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, stiffness)
end

function cpDampedRotarySpringGetDamping(constraint)
    ccall((:cpDampedRotarySpringGetDamping, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpDampedRotarySpringSetDamping(constraint, damping)
    ccall((:cpDampedRotarySpringSetDamping, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, damping)
end

function cpDampedRotarySpringGetSpringTorqueFunc(constraint)
    ccall((:cpDampedRotarySpringGetSpringTorqueFunc, libchipmunk_name), cpDampedRotarySpringTorqueFunc, (Ptr{cpConstraint},), constraint)
end

function cpDampedRotarySpringSetSpringTorqueFunc(constraint, springTorqueFunc)
    ccall((:cpDampedRotarySpringSetSpringTorqueFunc, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpDampedRotarySpringTorqueFunc), constraint, springTorqueFunc)
end

function cpConstraintIsRotaryLimitJoint(constraint)
    ccall((:cpConstraintIsRotaryLimitJoint, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpRotaryLimitJointAlloc()
    ccall((:cpRotaryLimitJointAlloc, libchipmunk_name), Ptr{cpRotaryLimitJoint}, ())
end

function cpRotaryLimitJointInit(joint, a, b, min, max)
    ccall((:cpRotaryLimitJointInit, libchipmunk_name), Ptr{cpRotaryLimitJoint}, (Ptr{cpRotaryLimitJoint}, Ptr{cpBody}, Ptr{cpBody}, cpFloat, cpFloat), joint, a, b, min, max)
end

function cpRotaryLimitJointNew(a, b, min, max)
    ccall((:cpRotaryLimitJointNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpFloat, cpFloat), a, b, min, max)
end

function cpRotaryLimitJointGetMin(constraint)
    ccall((:cpRotaryLimitJointGetMin, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpRotaryLimitJointSetMin(constraint, min)
    ccall((:cpRotaryLimitJointSetMin, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, min)
end

function cpRotaryLimitJointGetMax(constraint)
    ccall((:cpRotaryLimitJointGetMax, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpRotaryLimitJointSetMax(constraint, max)
    ccall((:cpRotaryLimitJointSetMax, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, max)
end

function cpConstraintIsRatchetJoint(constraint)
    ccall((:cpConstraintIsRatchetJoint, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpRatchetJointAlloc()
    ccall((:cpRatchetJointAlloc, libchipmunk_name), Ptr{cpRatchetJoint}, ())
end

function cpRatchetJointInit(joint, a, b, phase, ratchet)
    ccall((:cpRatchetJointInit, libchipmunk_name), Ptr{cpRatchetJoint}, (Ptr{cpRatchetJoint}, Ptr{cpBody}, Ptr{cpBody}, cpFloat, cpFloat), joint, a, b, phase, ratchet)
end

function cpRatchetJointNew(a, b, phase, ratchet)
    ccall((:cpRatchetJointNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpFloat, cpFloat), a, b, phase, ratchet)
end

function cpRatchetJointGetAngle(constraint)
    ccall((:cpRatchetJointGetAngle, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpRatchetJointSetAngle(constraint, angle)
    ccall((:cpRatchetJointSetAngle, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, angle)
end

function cpRatchetJointGetPhase(constraint)
    ccall((:cpRatchetJointGetPhase, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpRatchetJointSetPhase(constraint, phase)
    ccall((:cpRatchetJointSetPhase, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, phase)
end

function cpRatchetJointGetRatchet(constraint)
    ccall((:cpRatchetJointGetRatchet, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpRatchetJointSetRatchet(constraint, ratchet)
    ccall((:cpRatchetJointSetRatchet, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, ratchet)
end

function cpConstraintIsGearJoint(constraint)
    ccall((:cpConstraintIsGearJoint, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpGearJointAlloc()
    ccall((:cpGearJointAlloc, libchipmunk_name), Ptr{cpGearJoint}, ())
end

function cpGearJointInit(joint, a, b, phase, ratio)
    ccall((:cpGearJointInit, libchipmunk_name), Ptr{cpGearJoint}, (Ptr{cpGearJoint}, Ptr{cpBody}, Ptr{cpBody}, cpFloat, cpFloat), joint, a, b, phase, ratio)
end

function cpGearJointNew(a, b, phase, ratio)
    ccall((:cpGearJointNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpFloat, cpFloat), a, b, phase, ratio)
end

function cpGearJointGetPhase(constraint)
    ccall((:cpGearJointGetPhase, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpGearJointSetPhase(constraint, phase)
    ccall((:cpGearJointSetPhase, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, phase)
end

function cpGearJointGetRatio(constraint)
    ccall((:cpGearJointGetRatio, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpGearJointSetRatio(constraint, ratio)
    ccall((:cpGearJointSetRatio, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, ratio)
end

function cpConstraintIsSimpleMotor(constraint)
    ccall((:cpConstraintIsSimpleMotor, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpSimpleMotorAlloc()
    ccall((:cpSimpleMotorAlloc, libchipmunk_name), Ptr{cpSimpleMotor}, ())
end

function cpSimpleMotorInit(joint, a, b, rate)
    ccall((:cpSimpleMotorInit, libchipmunk_name), Ptr{cpSimpleMotor}, (Ptr{cpSimpleMotor}, Ptr{cpBody}, Ptr{cpBody}, cpFloat), joint, a, b, rate)
end

function cpSimpleMotorNew(a, b, rate)
    ccall((:cpSimpleMotorNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpFloat), a, b, rate)
end

function cpSimpleMotorGetRate(constraint)
    ccall((:cpSimpleMotorGetRate, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpSimpleMotorSetRate(constraint, rate)
    ccall((:cpSimpleMotorSetRate, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, rate)
end

function cpSpaceAlloc()
    ccall((:cpSpaceAlloc, libchipmunk_name), Ptr{cpSpace}, ())
end

function cpSpaceInit(space)
    ccall((:cpSpaceInit, libchipmunk_name), Ptr{cpSpace}, (Ptr{cpSpace},), space)
end

function cpSpaceNew()
    ccall((:cpSpaceNew, libchipmunk_name), Ptr{cpSpace}, ())
end

function cpSpaceDestroy(space)
    ccall((:cpSpaceDestroy, libchipmunk_name), Cvoid, (Ptr{cpSpace},), space)
end

function cpSpaceFree(space)
    ccall((:cpSpaceFree, libchipmunk_name), Cvoid, (Ptr{cpSpace},), space)
end

function cpSpaceGetIterations(space)
    ccall((:cpSpaceGetIterations, libchipmunk_name), Cint, (Ptr{cpSpace},), space)
end

function cpSpaceSetIterations(space, iterations)
    ccall((:cpSpaceSetIterations, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Cint), space, iterations)
end

function cpSpaceGetGravity(space)
    ccall((:cpSpaceGetGravity, libchipmunk_name), cpVect, (Ptr{cpSpace},), space)
end

function cpSpaceSetGravity(space, gravity)
    ccall((:cpSpaceSetGravity, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpVect), space, gravity)
end

function cpSpaceGetDamping(space)
    ccall((:cpSpaceGetDamping, libchipmunk_name), cpFloat, (Ptr{cpSpace},), space)
end

function cpSpaceSetDamping(space, damping)
    ccall((:cpSpaceSetDamping, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpFloat), space, damping)
end

function cpSpaceGetIdleSpeedThreshold(space)
    ccall((:cpSpaceGetIdleSpeedThreshold, libchipmunk_name), cpFloat, (Ptr{cpSpace},), space)
end

function cpSpaceSetIdleSpeedThreshold(space, idleSpeedThreshold)
    ccall((:cpSpaceSetIdleSpeedThreshold, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpFloat), space, idleSpeedThreshold)
end

function cpSpaceGetSleepTimeThreshold(space)
    ccall((:cpSpaceGetSleepTimeThreshold, libchipmunk_name), cpFloat, (Ptr{cpSpace},), space)
end

function cpSpaceSetSleepTimeThreshold(space, sleepTimeThreshold)
    ccall((:cpSpaceSetSleepTimeThreshold, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpFloat), space, sleepTimeThreshold)
end

function cpSpaceGetCollisionSlop(space)
    ccall((:cpSpaceGetCollisionSlop, libchipmunk_name), cpFloat, (Ptr{cpSpace},), space)
end

function cpSpaceSetCollisionSlop(space, collisionSlop)
    ccall((:cpSpaceSetCollisionSlop, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpFloat), space, collisionSlop)
end

function cpSpaceGetCollisionBias(space)
    ccall((:cpSpaceGetCollisionBias, libchipmunk_name), cpFloat, (Ptr{cpSpace},), space)
end

function cpSpaceSetCollisionBias(space, collisionBias)
    ccall((:cpSpaceSetCollisionBias, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpFloat), space, collisionBias)
end

function cpSpaceGetCollisionPersistence(space)
    ccall((:cpSpaceGetCollisionPersistence, libchipmunk_name), cpTimestamp, (Ptr{cpSpace},), space)
end

function cpSpaceSetCollisionPersistence(space, collisionPersistence)
    ccall((:cpSpaceSetCollisionPersistence, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpTimestamp), space, collisionPersistence)
end

function cpSpaceGetUserData(space)
    ccall((:cpSpaceGetUserData, libchipmunk_name), cpDataPointer, (Ptr{cpSpace},), space)
end

function cpSpaceSetUserData(space, userData)
    ccall((:cpSpaceSetUserData, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpDataPointer), space, userData)
end

function cpSpaceGetStaticBody(space)
    ccall((:cpSpaceGetStaticBody, libchipmunk_name), Ptr{cpBody}, (Ptr{cpSpace},), space)
end

function cpSpaceGetCurrentTimeStep(space)
    ccall((:cpSpaceGetCurrentTimeStep, libchipmunk_name), cpFloat, (Ptr{cpSpace},), space)
end

function cpSpaceIsLocked(space)
    ccall((:cpSpaceIsLocked, libchipmunk_name), cpBool, (Ptr{cpSpace},), space)
end

function cpSpaceAddDefaultCollisionHandler(space)
    ccall((:cpSpaceAddDefaultCollisionHandler, libchipmunk_name), Ptr{cpCollisionHandler}, (Ptr{cpSpace},), space)
end

function cpSpaceAddCollisionHandler(space, a, b)
    ccall((:cpSpaceAddCollisionHandler, libchipmunk_name), Ptr{cpCollisionHandler}, (Ptr{cpSpace}, cpCollisionType, cpCollisionType), space, a, b)
end

function cpSpaceAddWildcardHandler(space, type)
    ccall((:cpSpaceAddWildcardHandler, libchipmunk_name), Ptr{cpCollisionHandler}, (Ptr{cpSpace}, cpCollisionType), space, type)
end

function cpSpaceAddShape(space, shape)
    ccall((:cpSpaceAddShape, libchipmunk_name), Ptr{cpShape}, (Ptr{cpSpace}, Ptr{cpShape}), space, shape)
end

function cpSpaceAddBody(space, body)
    ccall((:cpSpaceAddBody, libchipmunk_name), Ptr{cpBody}, (Ptr{cpSpace}, Ptr{cpBody}), space, body)
end

function cpSpaceAddConstraint(space, constraint)
    ccall((:cpSpaceAddConstraint, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpSpace}, Ptr{cpConstraint}), space, constraint)
end

function cpSpaceRemoveShape(space, shape)
    ccall((:cpSpaceRemoveShape, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Ptr{cpShape}), space, shape)
end

function cpSpaceRemoveBody(space, body)
    ccall((:cpSpaceRemoveBody, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Ptr{cpBody}), space, body)
end

function cpSpaceRemoveConstraint(space, constraint)
    ccall((:cpSpaceRemoveConstraint, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Ptr{cpConstraint}), space, constraint)
end

function cpSpaceContainsShape(space, shape)
    ccall((:cpSpaceContainsShape, libchipmunk_name), cpBool, (Ptr{cpSpace}, Ptr{cpShape}), space, shape)
end

function cpSpaceContainsBody(space, body)
    ccall((:cpSpaceContainsBody, libchipmunk_name), cpBool, (Ptr{cpSpace}, Ptr{cpBody}), space, body)
end

function cpSpaceContainsConstraint(space, constraint)
    ccall((:cpSpaceContainsConstraint, libchipmunk_name), cpBool, (Ptr{cpSpace}, Ptr{cpConstraint}), space, constraint)
end

function cpSpaceAddPostStepCallback(space, func, key, data)
    ccall((:cpSpaceAddPostStepCallback, libchipmunk_name), cpBool, (Ptr{cpSpace}, cpPostStepFunc, Ptr{Cvoid}, Ptr{Cvoid}), space, func, key, data)
end

function cpSpacePointQuery(space, point, maxDistance, filter, func, data)
    ccall((:cpSpacePointQuery, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpVect, cpFloat, cpShapeFilter, cpSpacePointQueryFunc, Ptr{Cvoid}), space, point, maxDistance, filter, func, data)
end

function cpSpacePointQueryNearest(space, point, maxDistance, filter, out)
    ccall((:cpSpacePointQueryNearest, libchipmunk_name), Ptr{cpShape}, (Ptr{cpSpace}, cpVect, cpFloat, cpShapeFilter, Ptr{cpPointQueryInfo}), space, point, maxDistance, filter, out)
end

function cpSpaceSegmentQuery(space, start, _end, radius, filter, func, data)
    ccall((:cpSpaceSegmentQuery, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpVect, cpVect, cpFloat, cpShapeFilter, cpSpaceSegmentQueryFunc, Ptr{Cvoid}), space, start, _end, radius, filter, func, data)
end

function cpSpaceSegmentQueryFirst(space, start, _end, radius, filter, out)
    ccall((:cpSpaceSegmentQueryFirst, libchipmunk_name), Ptr{cpShape}, (Ptr{cpSpace}, cpVect, cpVect, cpFloat, cpShapeFilter, Ptr{cpSegmentQueryInfo}), space, start, _end, radius, filter, out)
end

function cpSpaceBBQuery(space, bb, filter, func, data)
    ccall((:cpSpaceBBQuery, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpBB, cpShapeFilter, cpSpaceBBQueryFunc, Ptr{Cvoid}), space, bb, filter, func, data)
end

function cpSpaceShapeQuery(space, shape, func, data)
    ccall((:cpSpaceShapeQuery, libchipmunk_name), cpBool, (Ptr{cpSpace}, Ptr{cpShape}, cpSpaceShapeQueryFunc, Ptr{Cvoid}), space, shape, func, data)
end

function cpSpaceEachBody(space, func, data)
    ccall((:cpSpaceEachBody, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpSpaceBodyIteratorFunc, Ptr{Cvoid}), space, func, data)
end

function cpSpaceEachShape(space, func, data)
    ccall((:cpSpaceEachShape, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpSpaceShapeIteratorFunc, Ptr{Cvoid}), space, func, data)
end

function cpSpaceEachConstraint(space, func, data)
    ccall((:cpSpaceEachConstraint, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpSpaceConstraintIteratorFunc, Ptr{Cvoid}), space, func, data)
end

function cpSpaceReindexStatic(space)
    ccall((:cpSpaceReindexStatic, libchipmunk_name), Cvoid, (Ptr{cpSpace},), space)
end

function cpSpaceReindexShape(space, shape)
    ccall((:cpSpaceReindexShape, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Ptr{cpShape}), space, shape)
end

function cpSpaceReindexShapesForBody(space, body)
    ccall((:cpSpaceReindexShapesForBody, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Ptr{cpBody}), space, body)
end

function cpSpaceUseSpatialHash(space, dim, count)
    ccall((:cpSpaceUseSpatialHash, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpFloat, Cint), space, dim, count)
end

function cpSpaceStep(space, dt)
    ccall((:cpSpaceStep, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpFloat), space, dt)
end

function cpSpaceDebugDraw(space, options)
    ccall((:cpSpaceDebugDraw, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Ptr{cpSpaceDebugDrawOptions}), space, options)
end

function cpMomentForCircle(m, r1, r2, offset)
    ccall((:cpMomentForCircle, libchipmunk_name), cpFloat, (cpFloat, cpFloat, cpFloat, cpVect), m, r1, r2, offset)
end

function cpAreaForCircle(r1, r2)
    ccall((:cpAreaForCircle, libchipmunk_name), cpFloat, (cpFloat, cpFloat), r1, r2)
end

function cpMomentForSegment(m, a, b, radius)
    ccall((:cpMomentForSegment, libchipmunk_name), cpFloat, (cpFloat, cpVect, cpVect, cpFloat), m, a, b, radius)
end

function cpAreaForSegment(a, b, radius)
    ccall((:cpAreaForSegment, libchipmunk_name), cpFloat, (cpVect, cpVect, cpFloat), a, b, radius)
end

function cpMomentForPoly(m, count, verts, offset, radius)
    ccall((:cpMomentForPoly, libchipmunk_name), cpFloat, (cpFloat, Cint, Ptr{cpVect}, cpVect, cpFloat), m, count, verts, offset, radius)
end

function cpAreaForPoly(count, verts, radius)
    ccall((:cpAreaForPoly, libchipmunk_name), cpFloat, (Cint, Ptr{cpVect}, cpFloat), count, verts, radius)
end

function cpCentroidForPoly(count, verts)
    ccall((:cpCentroidForPoly, libchipmunk_name), cpVect, (Cint, Ptr{cpVect}), count, verts)
end

function cpMomentForBox(m, width, height)
    ccall((:cpMomentForBox, libchipmunk_name), cpFloat, (cpFloat, cpFloat, cpFloat), m, width, height)
end

function cpMomentForBox2(m, box)
    ccall((:cpMomentForBox2, libchipmunk_name), cpFloat, (cpFloat, cpBB), m, box)
end

function cpConvexHull(count, verts, result, first, tol)
    ccall((:cpConvexHull, libchipmunk_name), Cint, (Cint, Ptr{cpVect}, Ptr{cpVect}, Ptr{Cint}, cpFloat), count, verts, result, first, tol)
end

function cpClosetPointOnSegment(p, a, b)
    ccall((:cpClosetPointOnSegment, libchipmunk_name), cpVect, (cpVect, cpVect, cpVect), p, a, b)
end

function cpSpaceEachBody_b(space, block)
    ccall((:cpSpaceEachBody_b, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Cvoid), space, block)
end

function cpSpaceEachShape_b(space, block)
    ccall((:cpSpaceEachShape_b, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Cvoid), space, block)
end

function cpSpaceEachConstraint_b(space, block)
    ccall((:cpSpaceEachConstraint_b, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Cvoid), space, block)
end

function cpBodyEachShape_b(body, block)
    ccall((:cpBodyEachShape_b, libchipmunk_name), Cvoid, (Ptr{cpBody}, Cvoid), body, block)
end

function cpBodyEachConstraint_b(body, block)
    ccall((:cpBodyEachConstraint_b, libchipmunk_name), Cvoid, (Ptr{cpBody}, Cvoid), body, block)
end

function cpBodyEachArbiter_b(body, block)
    ccall((:cpBodyEachArbiter_b, libchipmunk_name), Cvoid, (Ptr{cpBody}, Cvoid), body, block)
end

function cpSpacePointQuery_b(space, point, maxDistance, filter, block)
    ccall((:cpSpacePointQuery_b, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpVect, cpFloat, cpShapeFilter, cpSpacePointQueryBlock), space, point, maxDistance, filter, block)
end

function cpSpaceSegmentQuery_b(space, start, _end, radius, filter, block)
    ccall((:cpSpaceSegmentQuery_b, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpVect, cpVect, cpFloat, cpShapeFilter, cpSpaceSegmentQueryBlock), space, start, _end, radius, filter, block)
end

function cpSpaceBBQuery_b(space, bb, filter, block)
    ccall((:cpSpaceBBQuery_b, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpBB, cpShapeFilter, cpSpaceBBQueryBlock), space, bb, filter, block)
end

function cpSpaceShapeQuery_b(space, shape, block)
    ccall((:cpSpaceShapeQuery_b, libchipmunk_name), cpBool, (Ptr{cpSpace}, Ptr{cpShape}, cpSpaceShapeQueryBlock), space, shape, block)
end
# Julia wrapper for header: chipmunk_structs.h
# Automatically generated using Clang.jl


function cpfmax(a, b)
    ccall((:cpfmax, libchipmunk_name), cpFloat, (cpFloat, cpFloat), a, b)
end

function cpfmin(a, b)
    ccall((:cpfmin, libchipmunk_name), cpFloat, (cpFloat, cpFloat), a, b)
end

function cpfabs(f)
    ccall((:cpfabs, libchipmunk_name), cpFloat, (cpFloat,), f)
end

function cpfclamp(f, min, max)
    ccall((:cpfclamp, libchipmunk_name), cpFloat, (cpFloat, cpFloat, cpFloat), f, min, max)
end

function cpfclamp01(f)
    ccall((:cpfclamp01, libchipmunk_name), cpFloat, (cpFloat,), f)
end

function cpflerp(f1, f2, t)
    ccall((:cpflerp, libchipmunk_name), cpFloat, (cpFloat, cpFloat, cpFloat), f1, f2, t)
end

function cpflerpconst(f1, f2, d)
    ccall((:cpflerpconst, libchipmunk_name), cpFloat, (cpFloat, cpFloat, cpFloat), f1, f2, d)
end

function cpv(x, y)
    ccall((:cpv, libchipmunk_name), cpVect, (cpFloat, cpFloat), x, y)
end

function cpveql(v1, v2)
    ccall((:cpveql, libchipmunk_name), cpBool, (cpVect, cpVect), v1, v2)
end

function cpvadd(v1, v2)
    ccall((:cpvadd, libchipmunk_name), cpVect, (cpVect, cpVect), v1, v2)
end

function cpvsub(v1, v2)
    ccall((:cpvsub, libchipmunk_name), cpVect, (cpVect, cpVect), v1, v2)
end

function cpvneg(v)
    ccall((:cpvneg, libchipmunk_name), cpVect, (cpVect,), v)
end

function cpvmult(v, s)
    ccall((:cpvmult, libchipmunk_name), cpVect, (cpVect, cpFloat), v, s)
end

function cpvdot(v1, v2)
    ccall((:cpvdot, libchipmunk_name), cpFloat, (cpVect, cpVect), v1, v2)
end

function cpvcross(v1, v2)
    ccall((:cpvcross, libchipmunk_name), cpFloat, (cpVect, cpVect), v1, v2)
end

function cpvperp(v)
    ccall((:cpvperp, libchipmunk_name), cpVect, (cpVect,), v)
end

function cpvrperp(v)
    ccall((:cpvrperp, libchipmunk_name), cpVect, (cpVect,), v)
end

function cpvproject(v1, v2)
    ccall((:cpvproject, libchipmunk_name), cpVect, (cpVect, cpVect), v1, v2)
end

function cpvforangle(a)
    ccall((:cpvforangle, libchipmunk_name), cpVect, (cpFloat,), a)
end

function cpvtoangle(v)
    ccall((:cpvtoangle, libchipmunk_name), cpFloat, (cpVect,), v)
end

function cpvrotate(v1, v2)
    ccall((:cpvrotate, libchipmunk_name), cpVect, (cpVect, cpVect), v1, v2)
end

function cpvunrotate(v1, v2)
    ccall((:cpvunrotate, libchipmunk_name), cpVect, (cpVect, cpVect), v1, v2)
end

function cpvlengthsq(v)
    ccall((:cpvlengthsq, libchipmunk_name), cpFloat, (cpVect,), v)
end

function cpvlength(v)
    ccall((:cpvlength, libchipmunk_name), cpFloat, (cpVect,), v)
end

function cpvlerp(v1, v2, t)
    ccall((:cpvlerp, libchipmunk_name), cpVect, (cpVect, cpVect, cpFloat), v1, v2, t)
end

function cpvnormalize(v)
    ccall((:cpvnormalize, libchipmunk_name), cpVect, (cpVect,), v)
end

function cpvslerp(v1, v2, t)
    ccall((:cpvslerp, libchipmunk_name), cpVect, (cpVect, cpVect, cpFloat), v1, v2, t)
end

function cpvslerpconst(v1, v2, a)
    ccall((:cpvslerpconst, libchipmunk_name), cpVect, (cpVect, cpVect, cpFloat), v1, v2, a)
end

function cpvclamp(v, len)
    ccall((:cpvclamp, libchipmunk_name), cpVect, (cpVect, cpFloat), v, len)
end

function cpvlerpconst(v1, v2, d)
    ccall((:cpvlerpconst, libchipmunk_name), cpVect, (cpVect, cpVect, cpFloat), v1, v2, d)
end

function cpvdist(v1, v2)
    ccall((:cpvdist, libchipmunk_name), cpFloat, (cpVect, cpVect), v1, v2)
end

function cpvdistsq(v1, v2)
    ccall((:cpvdistsq, libchipmunk_name), cpFloat, (cpVect, cpVect), v1, v2)
end

function cpvnear(v1, v2, dist)
    ccall((:cpvnear, libchipmunk_name), cpBool, (cpVect, cpVect, cpFloat), v1, v2, dist)
end

function cpMat2x2New(a, b, c, d)
    ccall((:cpMat2x2New, libchipmunk_name), cpMat2x2, (cpFloat, cpFloat, cpFloat, cpFloat), a, b, c, d)
end

function cpMat2x2Transform(m, v)
    ccall((:cpMat2x2Transform, libchipmunk_name), cpVect, (cpMat2x2, cpVect), m, v)
end

function cpBBNew(l, b, r, t)
    ccall((:cpBBNew, libchipmunk_name), cpBB, (cpFloat, cpFloat, cpFloat, cpFloat), l, b, r, t)
end

function cpBBNewForExtents(c, hw, hh)
    ccall((:cpBBNewForExtents, libchipmunk_name), cpBB, (cpVect, cpFloat, cpFloat), c, hw, hh)
end

function cpBBNewForCircle(p, r)
    ccall((:cpBBNewForCircle, libchipmunk_name), cpBB, (cpVect, cpFloat), p, r)
end

function cpBBIntersects(a, b)
    ccall((:cpBBIntersects, libchipmunk_name), cpBool, (cpBB, cpBB), a, b)
end

function cpBBContainsBB(bb, other)
    ccall((:cpBBContainsBB, libchipmunk_name), cpBool, (cpBB, cpBB), bb, other)
end

function cpBBContainsVect(bb, v)
    ccall((:cpBBContainsVect, libchipmunk_name), cpBool, (cpBB, cpVect), bb, v)
end

function cpBBMerge(a, b)
    ccall((:cpBBMerge, libchipmunk_name), cpBB, (cpBB, cpBB), a, b)
end

function cpBBExpand(bb, v)
    ccall((:cpBBExpand, libchipmunk_name), cpBB, (cpBB, cpVect), bb, v)
end

function cpBBCenter(bb)
    ccall((:cpBBCenter, libchipmunk_name), cpVect, (cpBB,), bb)
end

function cpBBArea(bb)
    ccall((:cpBBArea, libchipmunk_name), cpFloat, (cpBB,), bb)
end

function cpBBMergedArea(a, b)
    ccall((:cpBBMergedArea, libchipmunk_name), cpFloat, (cpBB, cpBB), a, b)
end

function cpBBSegmentQuery(bb, a, b)
    ccall((:cpBBSegmentQuery, libchipmunk_name), cpFloat, (cpBB, cpVect, cpVect), bb, a, b)
end

function cpBBIntersectsSegment(bb, a, b)
    ccall((:cpBBIntersectsSegment, libchipmunk_name), cpBool, (cpBB, cpVect, cpVect), bb, a, b)
end

function cpBBClampVect(bb, v)
    ccall((:cpBBClampVect, libchipmunk_name), cpVect, (cpBB, cpVect), bb, v)
end

function cpBBWrapVect(bb, v)
    ccall((:cpBBWrapVect, libchipmunk_name), cpVect, (cpBB, cpVect), bb, v)
end

function cpBBOffset(bb, v)
    ccall((:cpBBOffset, libchipmunk_name), cpBB, (cpBB, cpVect), bb, v)
end

function cpTransformNew(a, b, c, d, tx, ty)
    ccall((:cpTransformNew, libchipmunk_name), cpTransform, (cpFloat, cpFloat, cpFloat, cpFloat, cpFloat, cpFloat), a, b, c, d, tx, ty)
end

function cpTransformNewTranspose(a, c, tx, b, d, ty)
    ccall((:cpTransformNewTranspose, libchipmunk_name), cpTransform, (cpFloat, cpFloat, cpFloat, cpFloat, cpFloat, cpFloat), a, c, tx, b, d, ty)
end

function cpTransformInverse(t)
    ccall((:cpTransformInverse, libchipmunk_name), cpTransform, (cpTransform,), t)
end

function cpTransformMult(t1, t2)
    ccall((:cpTransformMult, libchipmunk_name), cpTransform, (cpTransform, cpTransform), t1, t2)
end

function cpTransformPoint(t, p)
    ccall((:cpTransformPoint, libchipmunk_name), cpVect, (cpTransform, cpVect), t, p)
end

function cpTransformVect(t, v)
    ccall((:cpTransformVect, libchipmunk_name), cpVect, (cpTransform, cpVect), t, v)
end

function cpTransformbBB(t, bb)
    ccall((:cpTransformbBB, libchipmunk_name), cpBB, (cpTransform, cpBB), t, bb)
end

function cpTransformTranslate(translate)
    ccall((:cpTransformTranslate, libchipmunk_name), cpTransform, (cpVect,), translate)
end

function cpTransformScale(scaleX, scaleY)
    ccall((:cpTransformScale, libchipmunk_name), cpTransform, (cpFloat, cpFloat), scaleX, scaleY)
end

function cpTransformRotate(radians)
    ccall((:cpTransformRotate, libchipmunk_name), cpTransform, (cpFloat,), radians)
end

function cpTransformRigid(translate, radians)
    ccall((:cpTransformRigid, libchipmunk_name), cpTransform, (cpVect, cpFloat), translate, radians)
end

function cpTransformRigidInverse(t)
    ccall((:cpTransformRigidInverse, libchipmunk_name), cpTransform, (cpTransform,), t)
end

function cpTransformWrap(outer, inner)
    ccall((:cpTransformWrap, libchipmunk_name), cpTransform, (cpTransform, cpTransform), outer, inner)
end

function cpTransformWrapInverse(outer, inner)
    ccall((:cpTransformWrapInverse, libchipmunk_name), cpTransform, (cpTransform, cpTransform), outer, inner)
end

function cpTransformOrtho(bb)
    ccall((:cpTransformOrtho, libchipmunk_name), cpTransform, (cpBB,), bb)
end

function cpTransformBoneScale(v0, v1)
    ccall((:cpTransformBoneScale, libchipmunk_name), cpTransform, (cpVect, cpVect), v0, v1)
end

function cpTransformAxialScale(axis, pivot, scale)
    ccall((:cpTransformAxialScale, libchipmunk_name), cpTransform, (cpVect, cpVect, cpFloat), axis, pivot, scale)
end

function cpSpaceHashAlloc()
    ccall((:cpSpaceHashAlloc, libchipmunk_name), Ptr{cpSpaceHash}, ())
end

function cpSpaceHashInit(hash, celldim, numcells, bbfunc, staticIndex)
    ccall((:cpSpaceHashInit, libchipmunk_name), Ptr{cpSpatialIndex}, (Ptr{cpSpaceHash}, cpFloat, Cint, cpSpatialIndexBBFunc, Ptr{cpSpatialIndex}), hash, celldim, numcells, bbfunc, staticIndex)
end

function cpSpaceHashNew(celldim, cells, bbfunc, staticIndex)
    ccall((:cpSpaceHashNew, libchipmunk_name), Ptr{cpSpatialIndex}, (cpFloat, Cint, cpSpatialIndexBBFunc, Ptr{cpSpatialIndex}), celldim, cells, bbfunc, staticIndex)
end

function cpSpaceHashResize(hash, celldim, numcells)
    ccall((:cpSpaceHashResize, libchipmunk_name), Cvoid, (Ptr{cpSpaceHash}, cpFloat, Cint), hash, celldim, numcells)
end

function cpBBTreeAlloc()
    ccall((:cpBBTreeAlloc, libchipmunk_name), Ptr{cpBBTree}, ())
end

function cpBBTreeInit(tree, bbfunc, staticIndex)
    ccall((:cpBBTreeInit, libchipmunk_name), Ptr{cpSpatialIndex}, (Ptr{cpBBTree}, cpSpatialIndexBBFunc, Ptr{cpSpatialIndex}), tree, bbfunc, staticIndex)
end

function cpBBTreeNew(bbfunc, staticIndex)
    ccall((:cpBBTreeNew, libchipmunk_name), Ptr{cpSpatialIndex}, (cpSpatialIndexBBFunc, Ptr{cpSpatialIndex}), bbfunc, staticIndex)
end

function cpBBTreeOptimize(index)
    ccall((:cpBBTreeOptimize, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex},), index)
end

function cpBBTreeSetVelocityFunc(index, func)
    ccall((:cpBBTreeSetVelocityFunc, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, cpBBTreeVelocityFunc), index, func)
end

function cpSweep1DAlloc()
    ccall((:cpSweep1DAlloc, libchipmunk_name), Ptr{cpSweep1D}, ())
end

function cpSweep1DInit(sweep, bbfunc, staticIndex)
    ccall((:cpSweep1DInit, libchipmunk_name), Ptr{cpSpatialIndex}, (Ptr{cpSweep1D}, cpSpatialIndexBBFunc, Ptr{cpSpatialIndex}), sweep, bbfunc, staticIndex)
end

function cpSweep1DNew(bbfunc, staticIndex)
    ccall((:cpSweep1DNew, libchipmunk_name), Ptr{cpSpatialIndex}, (cpSpatialIndexBBFunc, Ptr{cpSpatialIndex}), bbfunc, staticIndex)
end

function cpSpatialIndexFree(index)
    ccall((:cpSpatialIndexFree, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex},), index)
end

function cpSpatialIndexCollideStatic(dynamicIndex, staticIndex, func, data)
    ccall((:cpSpatialIndexCollideStatic, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, Ptr{cpSpatialIndex}, cpSpatialIndexQueryFunc, Ptr{Cvoid}), dynamicIndex, staticIndex, func, data)
end

function cpSpatialIndexDestroy(index)
    ccall((:cpSpatialIndexDestroy, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex},), index)
end

function cpSpatialIndexCount(index)
    ccall((:cpSpatialIndexCount, libchipmunk_name), Cint, (Ptr{cpSpatialIndex},), index)
end

function cpSpatialIndexEach(index, func, data)
    ccall((:cpSpatialIndexEach, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, cpSpatialIndexIteratorFunc, Ptr{Cvoid}), index, func, data)
end

function cpSpatialIndexContains(index, obj, hashid)
    ccall((:cpSpatialIndexContains, libchipmunk_name), cpBool, (Ptr{cpSpatialIndex}, Ptr{Cvoid}, cpHashValue), index, obj, hashid)
end

function cpSpatialIndexInsert(index, obj, hashid)
    ccall((:cpSpatialIndexInsert, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, Ptr{Cvoid}, cpHashValue), index, obj, hashid)
end

function cpSpatialIndexRemove(index, obj, hashid)
    ccall((:cpSpatialIndexRemove, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, Ptr{Cvoid}, cpHashValue), index, obj, hashid)
end

function cpSpatialIndexReindex(index)
    ccall((:cpSpatialIndexReindex, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex},), index)
end

function cpSpatialIndexReindexObject(index, obj, hashid)
    ccall((:cpSpatialIndexReindexObject, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, Ptr{Cvoid}, cpHashValue), index, obj, hashid)
end

function cpSpatialIndexQuery(index, obj, bb, func, data)
    ccall((:cpSpatialIndexQuery, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, Ptr{Cvoid}, cpBB, cpSpatialIndexQueryFunc, Ptr{Cvoid}), index, obj, bb, func, data)
end

function cpSpatialIndexSegmentQuery(index, obj, a, b, t_exit, func, data)
    ccall((:cpSpatialIndexSegmentQuery, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, Ptr{Cvoid}, cpVect, cpVect, cpFloat, cpSpatialIndexSegmentQueryFunc, Ptr{Cvoid}), index, obj, a, b, t_exit, func, data)
end

function cpSpatialIndexReindexQuery(index, func, data)
    ccall((:cpSpatialIndexReindexQuery, libchipmunk_name), Cvoid, (Ptr{cpSpatialIndex}, cpSpatialIndexQueryFunc, Ptr{Cvoid}), index, func, data)
end

function cpArbiterGetRestitution(arb)
    ccall((:cpArbiterGetRestitution, libchipmunk_name), cpFloat, (Ptr{cpArbiter},), arb)
end

function cpArbiterSetRestitution(arb, restitution)
    ccall((:cpArbiterSetRestitution, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, cpFloat), arb, restitution)
end

function cpArbiterGetFriction(arb)
    ccall((:cpArbiterGetFriction, libchipmunk_name), cpFloat, (Ptr{cpArbiter},), arb)
end

function cpArbiterSetFriction(arb, friction)
    ccall((:cpArbiterSetFriction, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, cpFloat), arb, friction)
end

function cpArbiterGetSurfaceVelocity(arb)
    ccall((:cpArbiterGetSurfaceVelocity, libchipmunk_name), cpVect, (Ptr{cpArbiter},), arb)
end

function cpArbiterSetSurfaceVelocity(arb, vr)
    ccall((:cpArbiterSetSurfaceVelocity, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, cpVect), arb, vr)
end

function cpArbiterGetUserData(arb)
    ccall((:cpArbiterGetUserData, libchipmunk_name), cpDataPointer, (Ptr{cpArbiter},), arb)
end

function cpArbiterSetUserData(arb, userData)
    ccall((:cpArbiterSetUserData, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, cpDataPointer), arb, userData)
end

function cpArbiterTotalImpulse(arb)
    ccall((:cpArbiterTotalImpulse, libchipmunk_name), cpVect, (Ptr{cpArbiter},), arb)
end

function cpArbiterTotalKE(arb)
    ccall((:cpArbiterTotalKE, libchipmunk_name), cpFloat, (Ptr{cpArbiter},), arb)
end

function cpArbiterIgnore(arb)
    ccall((:cpArbiterIgnore, libchipmunk_name), cpBool, (Ptr{cpArbiter},), arb)
end

function cpArbiterGetShapes(arb, a, b)
    ccall((:cpArbiterGetShapes, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, Ptr{Ptr{cpShape}}, Ptr{Ptr{cpShape}}), arb, a, b)
end

function cpArbiterGetBodies(arb, a, b)
    ccall((:cpArbiterGetBodies, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, Ptr{Ptr{cpBody}}, Ptr{Ptr{cpBody}}), arb, a, b)
end

function cpArbiterGetContactPointSet(arb)
    ccall((:cpArbiterGetContactPointSet, libchipmunk_name), cpContactPointSet, (Ptr{cpArbiter},), arb)
end

function cpArbiterSetContactPointSet(arb, set)
    ccall((:cpArbiterSetContactPointSet, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, Ptr{cpContactPointSet}), arb, set)
end

function cpArbiterIsFirstContact(arb)
    ccall((:cpArbiterIsFirstContact, libchipmunk_name), cpBool, (Ptr{cpArbiter},), arb)
end

function cpArbiterIsRemoval(arb)
    ccall((:cpArbiterIsRemoval, libchipmunk_name), cpBool, (Ptr{cpArbiter},), arb)
end

function cpArbiterGetCount(arb)
    ccall((:cpArbiterGetCount, libchipmunk_name), Cint, (Ptr{cpArbiter},), arb)
end

function cpArbiterGetNormal(arb)
    ccall((:cpArbiterGetNormal, libchipmunk_name), cpVect, (Ptr{cpArbiter},), arb)
end

function cpArbiterGetPointA(arb, i)
    ccall((:cpArbiterGetPointA, libchipmunk_name), cpVect, (Ptr{cpArbiter}, Cint), arb, i)
end

function cpArbiterGetPointB(arb, i)
    ccall((:cpArbiterGetPointB, libchipmunk_name), cpVect, (Ptr{cpArbiter}, Cint), arb, i)
end

function cpArbiterGetDepth(arb, i)
    ccall((:cpArbiterGetDepth, libchipmunk_name), cpFloat, (Ptr{cpArbiter}, Cint), arb, i)
end

function cpArbiterCallWildcardBeginA(arb, space)
    ccall((:cpArbiterCallWildcardBeginA, libchipmunk_name), cpBool, (Ptr{cpArbiter}, Ptr{cpSpace}), arb, space)
end

function cpArbiterCallWildcardBeginB(arb, space)
    ccall((:cpArbiterCallWildcardBeginB, libchipmunk_name), cpBool, (Ptr{cpArbiter}, Ptr{cpSpace}), arb, space)
end

function cpArbiterCallWildcardPreSolveA(arb, space)
    ccall((:cpArbiterCallWildcardPreSolveA, libchipmunk_name), cpBool, (Ptr{cpArbiter}, Ptr{cpSpace}), arb, space)
end

function cpArbiterCallWildcardPreSolveB(arb, space)
    ccall((:cpArbiterCallWildcardPreSolveB, libchipmunk_name), cpBool, (Ptr{cpArbiter}, Ptr{cpSpace}), arb, space)
end

function cpArbiterCallWildcardPostSolveA(arb, space)
    ccall((:cpArbiterCallWildcardPostSolveA, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, Ptr{cpSpace}), arb, space)
end

function cpArbiterCallWildcardPostSolveB(arb, space)
    ccall((:cpArbiterCallWildcardPostSolveB, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, Ptr{cpSpace}), arb, space)
end

function cpArbiterCallWildcardSeparateA(arb, space)
    ccall((:cpArbiterCallWildcardSeparateA, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, Ptr{cpSpace}), arb, space)
end

function cpArbiterCallWildcardSeparateB(arb, space)
    ccall((:cpArbiterCallWildcardSeparateB, libchipmunk_name), Cvoid, (Ptr{cpArbiter}, Ptr{cpSpace}), arb, space)
end

function cpBodyAlloc()
    ccall((:cpBodyAlloc, libchipmunk_name), Ptr{cpBody}, ())
end

function cpBodyInit(body, mass, moment)
    ccall((:cpBodyInit, libchipmunk_name), Ptr{cpBody}, (Ptr{cpBody}, cpFloat, cpFloat), body, mass, moment)
end

function cpBodyNew(mass, moment)
    ccall((:cpBodyNew, libchipmunk_name), Ptr{cpBody}, (cpFloat, cpFloat), mass, moment)
end

function cpBodyNewKinematic()
    ccall((:cpBodyNewKinematic, libchipmunk_name), Ptr{cpBody}, ())
end

function cpBodyNewStatic()
    ccall((:cpBodyNewStatic, libchipmunk_name), Ptr{cpBody}, ())
end

function cpBodyDestroy(body)
    ccall((:cpBodyDestroy, libchipmunk_name), Cvoid, (Ptr{cpBody},), body)
end

function cpBodyFree(body)
    ccall((:cpBodyFree, libchipmunk_name), Cvoid, (Ptr{cpBody},), body)
end

function cpBodyActivate(body)
    ccall((:cpBodyActivate, libchipmunk_name), Cvoid, (Ptr{cpBody},), body)
end

function cpBodyActivateStatic(body, filter)
    ccall((:cpBodyActivateStatic, libchipmunk_name), Cvoid, (Ptr{cpBody}, Ptr{cpShape}), body, filter)
end

function cpBodySleep(body)
    ccall((:cpBodySleep, libchipmunk_name), Cvoid, (Ptr{cpBody},), body)
end

function cpBodySleepWithGroup(body, group)
    ccall((:cpBodySleepWithGroup, libchipmunk_name), Cvoid, (Ptr{cpBody}, Ptr{cpBody}), body, group)
end

function cpBodyIsSleeping(body)
    ccall((:cpBodyIsSleeping, libchipmunk_name), cpBool, (Ptr{cpBody},), body)
end

function cpBodyGetType(body)
    ccall((:cpBodyGetType, libchipmunk_name), cpBodyType, (Ptr{cpBody},), body)
end

function cpBodySetType(body, type)
    ccall((:cpBodySetType, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpBodyType), body, type)
end

function cpBodyGetSpace(body)
    ccall((:cpBodyGetSpace, libchipmunk_name), Ptr{cpSpace}, (Ptr{cpBody},), body)
end

function cpBodyGetMass(body)
    ccall((:cpBodyGetMass, libchipmunk_name), cpFloat, (Ptr{cpBody},), body)
end

function cpBodySetMass(body, m)
    ccall((:cpBodySetMass, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpFloat), body, m)
end

function cpBodyGetMoment(body)
    ccall((:cpBodyGetMoment, libchipmunk_name), cpFloat, (Ptr{cpBody},), body)
end

function cpBodySetMoment(body, i)
    ccall((:cpBodySetMoment, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpFloat), body, i)
end

function cpBodyGetPosition(body)
    ccall((:cpBodyGetPosition, libchipmunk_name), cpVect, (Ptr{cpBody},), body)
end

function cpBodySetPosition(body, pos)
    ccall((:cpBodySetPosition, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect), body, pos)
end

function cpBodyGetCenterOfGravity(body)
    ccall((:cpBodyGetCenterOfGravity, libchipmunk_name), cpVect, (Ptr{cpBody},), body)
end

function cpBodySetCenterOfGravity(body, cog)
    ccall((:cpBodySetCenterOfGravity, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect), body, cog)
end

function cpBodyGetVelocity(body)
    ccall((:cpBodyGetVelocity, libchipmunk_name), cpVect, (Ptr{cpBody},), body)
end

function cpBodySetVelocity(body, velocity)
    ccall((:cpBodySetVelocity, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect), body, velocity)
end

function cpBodyGetForce(body)
    ccall((:cpBodyGetForce, libchipmunk_name), cpVect, (Ptr{cpBody},), body)
end

function cpBodySetForce(body, force)
    ccall((:cpBodySetForce, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect), body, force)
end

function cpBodyGetAngle(body)
    ccall((:cpBodyGetAngle, libchipmunk_name), cpFloat, (Ptr{cpBody},), body)
end

function cpBodySetAngle(body, a)
    ccall((:cpBodySetAngle, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpFloat), body, a)
end

function cpBodyGetAngularVelocity(body)
    ccall((:cpBodyGetAngularVelocity, libchipmunk_name), cpFloat, (Ptr{cpBody},), body)
end

function cpBodySetAngularVelocity(body, angularVelocity)
    ccall((:cpBodySetAngularVelocity, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpFloat), body, angularVelocity)
end

function cpBodyGetTorque(body)
    ccall((:cpBodyGetTorque, libchipmunk_name), cpFloat, (Ptr{cpBody},), body)
end

function cpBodySetTorque(body, torque)
    ccall((:cpBodySetTorque, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpFloat), body, torque)
end

function cpBodyGetRotation(body)
    ccall((:cpBodyGetRotation, libchipmunk_name), cpVect, (Ptr{cpBody},), body)
end

function cpBodyGetUserData(body)
    ccall((:cpBodyGetUserData, libchipmunk_name), cpDataPointer, (Ptr{cpBody},), body)
end

function cpBodySetUserData(body, userData)
    ccall((:cpBodySetUserData, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpDataPointer), body, userData)
end

function cpBodySetVelocityUpdateFunc(body, velocityFunc)
    ccall((:cpBodySetVelocityUpdateFunc, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpBodyVelocityFunc), body, velocityFunc)
end

function cpBodySetPositionUpdateFunc(body, positionFunc)
    ccall((:cpBodySetPositionUpdateFunc, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpBodyPositionFunc), body, positionFunc)
end

function cpBodyUpdateVelocity(body, gravity, damping, dt)
    ccall((:cpBodyUpdateVelocity, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect, cpFloat, cpFloat), body, gravity, damping, dt)
end

function cpBodyUpdatePosition(body, dt)
    ccall((:cpBodyUpdatePosition, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpFloat), body, dt)
end

function cpBodyLocalToWorld(body, point)
    ccall((:cpBodyLocalToWorld, libchipmunk_name), cpVect, (Ptr{cpBody}, cpVect), body, point)
end

function cpBodyWorldToLocal(body, point)
    ccall((:cpBodyWorldToLocal, libchipmunk_name), cpVect, (Ptr{cpBody}, cpVect), body, point)
end

function cpBodyApplyForceAtWorldPoint(body, force, point)
    ccall((:cpBodyApplyForceAtWorldPoint, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect, cpVect), body, force, point)
end

function cpBodyApplyForceAtLocalPoint(body, force, point)
    ccall((:cpBodyApplyForceAtLocalPoint, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect, cpVect), body, force, point)
end

function cpBodyApplyImpulseAtWorldPoint(body, impulse, point)
    ccall((:cpBodyApplyImpulseAtWorldPoint, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect, cpVect), body, impulse, point)
end

function cpBodyApplyImpulseAtLocalPoint(body, impulse, point)
    ccall((:cpBodyApplyImpulseAtLocalPoint, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpVect, cpVect), body, impulse, point)
end

function cpBodyGetVelocityAtWorldPoint(body, point)
    ccall((:cpBodyGetVelocityAtWorldPoint, libchipmunk_name), cpVect, (Ptr{cpBody}, cpVect), body, point)
end

function cpBodyGetVelocityAtLocalPoint(body, point)
    ccall((:cpBodyGetVelocityAtLocalPoint, libchipmunk_name), cpVect, (Ptr{cpBody}, cpVect), body, point)
end

function cpBodyKineticEnergy(body)
    ccall((:cpBodyKineticEnergy, libchipmunk_name), cpFloat, (Ptr{cpBody},), body)
end

function cpBodyEachShape(body, func, data)
    ccall((:cpBodyEachShape, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpBodyShapeIteratorFunc, Ptr{Cvoid}), body, func, data)
end

function cpBodyEachConstraint(body, func, data)
    ccall((:cpBodyEachConstraint, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpBodyConstraintIteratorFunc, Ptr{Cvoid}), body, func, data)
end

function cpBodyEachArbiter(body, func, data)
    ccall((:cpBodyEachArbiter, libchipmunk_name), Cvoid, (Ptr{cpBody}, cpBodyArbiterIteratorFunc, Ptr{Cvoid}), body, func, data)
end

function cpShapeFilterNew(group, categories, mask)
    ccall((:cpShapeFilterNew, libchipmunk_name), cpShapeFilter, (cpGroup, cpBitmask, cpBitmask), group, categories, mask)
end

function cpShapeDestroy(shape)
    ccall((:cpShapeDestroy, libchipmunk_name), Cvoid, (Ptr{cpShape},), shape)
end

function cpShapeFree(shape)
    ccall((:cpShapeFree, libchipmunk_name), Cvoid, (Ptr{cpShape},), shape)
end

function cpShapeCacheBB(shape)
    ccall((:cpShapeCacheBB, libchipmunk_name), cpBB, (Ptr{cpShape},), shape)
end

function cpShapeUpdate(shape, transform)
    ccall((:cpShapeUpdate, libchipmunk_name), cpBB, (Ptr{cpShape}, cpTransform), shape, transform)
end

function cpShapePointQuery(shape, p, out)
    ccall((:cpShapePointQuery, libchipmunk_name), cpFloat, (Ptr{cpShape}, cpVect, Ptr{cpPointQueryInfo}), shape, p, out)
end

function cpShapeSegmentQuery(shape, a, b, radius, info)
    ccall((:cpShapeSegmentQuery, libchipmunk_name), cpBool, (Ptr{cpShape}, cpVect, cpVect, cpFloat, Ptr{cpSegmentQueryInfo}), shape, a, b, radius, info)
end

function cpShapesCollide(a, b)
    ccall((:cpShapesCollide, libchipmunk_name), cpContactPointSet, (Ptr{cpShape}, Ptr{cpShape}), a, b)
end

function cpShapeGetSpace(shape)
    ccall((:cpShapeGetSpace, libchipmunk_name), Ptr{cpSpace}, (Ptr{cpShape},), shape)
end

function cpShapeGetBody(shape)
    ccall((:cpShapeGetBody, libchipmunk_name), Ptr{cpBody}, (Ptr{cpShape},), shape)
end

function cpShapeSetBody(shape, body)
    ccall((:cpShapeSetBody, libchipmunk_name), Cvoid, (Ptr{cpShape}, Ptr{cpBody}), shape, body)
end

function cpShapeGetMass(shape)
    ccall((:cpShapeGetMass, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpShapeSetMass(shape, mass)
    ccall((:cpShapeSetMass, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpFloat), shape, mass)
end

function cpShapeGetDensity(shape)
    ccall((:cpShapeGetDensity, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpShapeSetDensity(shape, density)
    ccall((:cpShapeSetDensity, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpFloat), shape, density)
end

function cpShapeGetMoment(shape)
    ccall((:cpShapeGetMoment, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpShapeGetArea(shape)
    ccall((:cpShapeGetArea, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpShapeGetCenterOfGravity(shape)
    ccall((:cpShapeGetCenterOfGravity, libchipmunk_name), cpVect, (Ptr{cpShape},), shape)
end

function cpShapeGetBB(shape)
    ccall((:cpShapeGetBB, libchipmunk_name), cpBB, (Ptr{cpShape},), shape)
end

function cpShapeGetSensor(shape)
    ccall((:cpShapeGetSensor, libchipmunk_name), cpBool, (Ptr{cpShape},), shape)
end

function cpShapeSetSensor(shape, sensor)
    ccall((:cpShapeSetSensor, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpBool), shape, sensor)
end

function cpShapeGetElasticity(shape)
    ccall((:cpShapeGetElasticity, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpShapeSetElasticity(shape, elasticity)
    ccall((:cpShapeSetElasticity, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpFloat), shape, elasticity)
end

function cpShapeGetFriction(shape)
    ccall((:cpShapeGetFriction, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpShapeSetFriction(shape, friction)
    ccall((:cpShapeSetFriction, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpFloat), shape, friction)
end

function cpShapeGetSurfaceVelocity(shape)
    ccall((:cpShapeGetSurfaceVelocity, libchipmunk_name), cpVect, (Ptr{cpShape},), shape)
end

function cpShapeSetSurfaceVelocity(shape, surfaceVelocity)
    ccall((:cpShapeSetSurfaceVelocity, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpVect), shape, surfaceVelocity)
end

function cpShapeGetUserData(shape)
    ccall((:cpShapeGetUserData, libchipmunk_name), cpDataPointer, (Ptr{cpShape},), shape)
end

function cpShapeSetUserData(shape, userData)
    ccall((:cpShapeSetUserData, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpDataPointer), shape, userData)
end

function cpShapeGetCollisionType(shape)
    ccall((:cpShapeGetCollisionType, libchipmunk_name), cpCollisionType, (Ptr{cpShape},), shape)
end

function cpShapeSetCollisionType(shape, collisionType)
    ccall((:cpShapeSetCollisionType, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpCollisionType), shape, collisionType)
end

function cpShapeGetFilter(shape)
    ccall((:cpShapeGetFilter, libchipmunk_name), cpShapeFilter, (Ptr{cpShape},), shape)
end

function cpShapeSetFilter(shape, filter)
    ccall((:cpShapeSetFilter, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpShapeFilter), shape, filter)
end

function cpCircleShapeAlloc()
    ccall((:cpCircleShapeAlloc, libchipmunk_name), Ptr{cpCircleShape}, ())
end

function cpCircleShapeInit(circle, body, radius, offset)
    ccall((:cpCircleShapeInit, libchipmunk_name), Ptr{cpCircleShape}, (Ptr{cpCircleShape}, Ptr{cpBody}, cpFloat, cpVect), circle, body, radius, offset)
end

function cpCircleShapeNew(body, radius, offset)
    ccall((:cpCircleShapeNew, libchipmunk_name), Ptr{cpShape}, (Ptr{cpBody}, cpFloat, cpVect), body, radius, offset)
end

function cpCircleShapeGetOffset(shape)
    ccall((:cpCircleShapeGetOffset, libchipmunk_name), cpVect, (Ptr{cpShape},), shape)
end

function cpCircleShapeGetRadius(shape)
    ccall((:cpCircleShapeGetRadius, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpSegmentShapeAlloc()
    ccall((:cpSegmentShapeAlloc, libchipmunk_name), Ptr{cpSegmentShape}, ())
end

function cpSegmentShapeInit(seg, body, a, b, radius)
    ccall((:cpSegmentShapeInit, libchipmunk_name), Ptr{cpSegmentShape}, (Ptr{cpSegmentShape}, Ptr{cpBody}, cpVect, cpVect, cpFloat), seg, body, a, b, radius)
end

function cpSegmentShapeNew(body, a, b, radius)
    ccall((:cpSegmentShapeNew, libchipmunk_name), Ptr{cpShape}, (Ptr{cpBody}, cpVect, cpVect, cpFloat), body, a, b, radius)
end

function cpSegmentShapeSetNeighbors(shape, prev, next)
    ccall((:cpSegmentShapeSetNeighbors, libchipmunk_name), Cvoid, (Ptr{cpShape}, cpVect, cpVect), shape, prev, next)
end

function cpSegmentShapeGetA(shape)
    ccall((:cpSegmentShapeGetA, libchipmunk_name), cpVect, (Ptr{cpShape},), shape)
end

function cpSegmentShapeGetB(shape)
    ccall((:cpSegmentShapeGetB, libchipmunk_name), cpVect, (Ptr{cpShape},), shape)
end

function cpSegmentShapeGetNormal(shape)
    ccall((:cpSegmentShapeGetNormal, libchipmunk_name), cpVect, (Ptr{cpShape},), shape)
end

function cpSegmentShapeGetRadius(shape)
    ccall((:cpSegmentShapeGetRadius, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpPolyShapeAlloc()
    ccall((:cpPolyShapeAlloc, libchipmunk_name), Ptr{cpPolyShape}, ())
end

function cpPolyShapeInit(poly, body, count, verts, transform, radius)
    ccall((:cpPolyShapeInit, libchipmunk_name), Ptr{cpPolyShape}, (Ptr{cpPolyShape}, Ptr{cpBody}, Cint, Ptr{cpVect}, cpTransform, cpFloat), poly, body, count, verts, transform, radius)
end

function cpPolyShapeInitRaw(poly, body, count, verts, radius)
    ccall((:cpPolyShapeInitRaw, libchipmunk_name), Ptr{cpPolyShape}, (Ptr{cpPolyShape}, Ptr{cpBody}, Cint, Ptr{cpVect}, cpFloat), poly, body, count, verts, radius)
end

function cpPolyShapeNew(body, count, verts, transform, radius)
    ccall((:cpPolyShapeNew, libchipmunk_name), Ptr{cpShape}, (Ptr{cpBody}, Cint, Ptr{cpVect}, cpTransform, cpFloat), body, count, verts, transform, radius)
end

function cpPolyShapeNewRaw(body, count, verts, radius)
    ccall((:cpPolyShapeNewRaw, libchipmunk_name), Ptr{cpShape}, (Ptr{cpBody}, Cint, Ptr{cpVect}, cpFloat), body, count, verts, radius)
end

function cpBoxShapeInit(poly, body, width, height, radius)
    ccall((:cpBoxShapeInit, libchipmunk_name), Ptr{cpPolyShape}, (Ptr{cpPolyShape}, Ptr{cpBody}, cpFloat, cpFloat, cpFloat), poly, body, width, height, radius)
end

function cpBoxShapeInit2(poly, body, box, radius)
    ccall((:cpBoxShapeInit2, libchipmunk_name), Ptr{cpPolyShape}, (Ptr{cpPolyShape}, Ptr{cpBody}, cpBB, cpFloat), poly, body, box, radius)
end

function cpBoxShapeNew(body, width, height, radius)
    ccall((:cpBoxShapeNew, libchipmunk_name), Ptr{cpShape}, (Ptr{cpBody}, cpFloat, cpFloat, cpFloat), body, width, height, radius)
end

function cpBoxShapeNew2(body, box, radius)
    ccall((:cpBoxShapeNew2, libchipmunk_name), Ptr{cpShape}, (Ptr{cpBody}, cpBB, cpFloat), body, box, radius)
end

function cpPolyShapeGetCount(shape)
    ccall((:cpPolyShapeGetCount, libchipmunk_name), Cint, (Ptr{cpShape},), shape)
end

function cpPolyShapeGetVert(shape, index)
    ccall((:cpPolyShapeGetVert, libchipmunk_name), cpVect, (Ptr{cpShape}, Cint), shape, index)
end

function cpPolyShapeGetRadius(shape)
    ccall((:cpPolyShapeGetRadius, libchipmunk_name), cpFloat, (Ptr{cpShape},), shape)
end

function cpConstraintDestroy(constraint)
    ccall((:cpConstraintDestroy, libchipmunk_name), Cvoid, (Ptr{cpConstraint},), constraint)
end

function cpConstraintFree(constraint)
    ccall((:cpConstraintFree, libchipmunk_name), Cvoid, (Ptr{cpConstraint},), constraint)
end

function cpConstraintGetSpace(constraint)
    ccall((:cpConstraintGetSpace, libchipmunk_name), Ptr{cpSpace}, (Ptr{cpConstraint},), constraint)
end

function cpConstraintGetBodyA(constraint)
    ccall((:cpConstraintGetBodyA, libchipmunk_name), Ptr{cpBody}, (Ptr{cpConstraint},), constraint)
end

function cpConstraintGetBodyB(constraint)
    ccall((:cpConstraintGetBodyB, libchipmunk_name), Ptr{cpBody}, (Ptr{cpConstraint},), constraint)
end

function cpConstraintGetMaxForce(constraint)
    ccall((:cpConstraintGetMaxForce, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpConstraintSetMaxForce(constraint, maxForce)
    ccall((:cpConstraintSetMaxForce, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, maxForce)
end

function cpConstraintGetErrorBias(constraint)
    ccall((:cpConstraintGetErrorBias, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpConstraintSetErrorBias(constraint, errorBias)
    ccall((:cpConstraintSetErrorBias, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, errorBias)
end

function cpConstraintGetMaxBias(constraint)
    ccall((:cpConstraintGetMaxBias, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpConstraintSetMaxBias(constraint, maxBias)
    ccall((:cpConstraintSetMaxBias, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, maxBias)
end

function cpConstraintGetCollideBodies(constraint)
    ccall((:cpConstraintGetCollideBodies, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpConstraintSetCollideBodies(constraint, collideBodies)
    ccall((:cpConstraintSetCollideBodies, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpBool), constraint, collideBodies)
end

function cpConstraintGetPreSolveFunc(constraint)
    ccall((:cpConstraintGetPreSolveFunc, libchipmunk_name), cpConstraintPreSolveFunc, (Ptr{cpConstraint},), constraint)
end

function cpConstraintSetPreSolveFunc(constraint, preSolveFunc)
    ccall((:cpConstraintSetPreSolveFunc, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpConstraintPreSolveFunc), constraint, preSolveFunc)
end

function cpConstraintGetPostSolveFunc(constraint)
    ccall((:cpConstraintGetPostSolveFunc, libchipmunk_name), cpConstraintPostSolveFunc, (Ptr{cpConstraint},), constraint)
end

function cpConstraintSetPostSolveFunc(constraint, postSolveFunc)
    ccall((:cpConstraintSetPostSolveFunc, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpConstraintPostSolveFunc), constraint, postSolveFunc)
end

function cpConstraintGetUserData(constraint)
    ccall((:cpConstraintGetUserData, libchipmunk_name), cpDataPointer, (Ptr{cpConstraint},), constraint)
end

function cpConstraintSetUserData(constraint, userData)
    ccall((:cpConstraintSetUserData, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpDataPointer), constraint, userData)
end

function cpConstraintGetImpulse(constraint)
    ccall((:cpConstraintGetImpulse, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpConstraintIsPinJoint(constraint)
    ccall((:cpConstraintIsPinJoint, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpPinJointAlloc()
    ccall((:cpPinJointAlloc, libchipmunk_name), Ptr{cpPinJoint}, ())
end

function cpPinJointInit(joint, a, b, anchorA, anchorB)
    ccall((:cpPinJointInit, libchipmunk_name), Ptr{cpPinJoint}, (Ptr{cpPinJoint}, Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect), joint, a, b, anchorA, anchorB)
end

function cpPinJointNew(a, b, anchorA, anchorB)
    ccall((:cpPinJointNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect), a, b, anchorA, anchorB)
end

function cpPinJointGetAnchorA(constraint)
    ccall((:cpPinJointGetAnchorA, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpPinJointSetAnchorA(constraint, anchorA)
    ccall((:cpPinJointSetAnchorA, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorA)
end

function cpPinJointGetAnchorB(constraint)
    ccall((:cpPinJointGetAnchorB, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpPinJointSetAnchorB(constraint, anchorB)
    ccall((:cpPinJointSetAnchorB, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorB)
end

function cpPinJointGetDist(constraint)
    ccall((:cpPinJointGetDist, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpPinJointSetDist(constraint, dist)
    ccall((:cpPinJointSetDist, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, dist)
end

function cpConstraintIsSlideJoint(constraint)
    ccall((:cpConstraintIsSlideJoint, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpSlideJointAlloc()
    ccall((:cpSlideJointAlloc, libchipmunk_name), Ptr{cpSlideJoint}, ())
end

function cpSlideJointInit(joint, a, b, anchorA, anchorB, min, max)
    ccall((:cpSlideJointInit, libchipmunk_name), Ptr{cpSlideJoint}, (Ptr{cpSlideJoint}, Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect, cpFloat, cpFloat), joint, a, b, anchorA, anchorB, min, max)
end

function cpSlideJointNew(a, b, anchorA, anchorB, min, max)
    ccall((:cpSlideJointNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect, cpFloat, cpFloat), a, b, anchorA, anchorB, min, max)
end

function cpSlideJointGetAnchorA(constraint)
    ccall((:cpSlideJointGetAnchorA, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpSlideJointSetAnchorA(constraint, anchorA)
    ccall((:cpSlideJointSetAnchorA, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorA)
end

function cpSlideJointGetAnchorB(constraint)
    ccall((:cpSlideJointGetAnchorB, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpSlideJointSetAnchorB(constraint, anchorB)
    ccall((:cpSlideJointSetAnchorB, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorB)
end

function cpSlideJointGetMin(constraint)
    ccall((:cpSlideJointGetMin, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpSlideJointSetMin(constraint, min)
    ccall((:cpSlideJointSetMin, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, min)
end

function cpSlideJointGetMax(constraint)
    ccall((:cpSlideJointGetMax, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpSlideJointSetMax(constraint, max)
    ccall((:cpSlideJointSetMax, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, max)
end

function cpConstraintIsPivotJoint(constraint)
    ccall((:cpConstraintIsPivotJoint, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpPivotJointAlloc()
    ccall((:cpPivotJointAlloc, libchipmunk_name), Ptr{cpPivotJoint}, ())
end

function cpPivotJointInit(joint, a, b, anchorA, anchorB)
    ccall((:cpPivotJointInit, libchipmunk_name), Ptr{cpPivotJoint}, (Ptr{cpPivotJoint}, Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect), joint, a, b, anchorA, anchorB)
end

function cpPivotJointNew(a, b, pivot)
    ccall((:cpPivotJointNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpVect), a, b, pivot)
end

function cpPivotJointNew2(a, b, anchorA, anchorB)
    ccall((:cpPivotJointNew2, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect), a, b, anchorA, anchorB)
end

function cpPivotJointGetAnchorA(constraint)
    ccall((:cpPivotJointGetAnchorA, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpPivotJointSetAnchorA(constraint, anchorA)
    ccall((:cpPivotJointSetAnchorA, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorA)
end

function cpPivotJointGetAnchorB(constraint)
    ccall((:cpPivotJointGetAnchorB, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpPivotJointSetAnchorB(constraint, anchorB)
    ccall((:cpPivotJointSetAnchorB, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorB)
end

function cpConstraintIsGrooveJoint(constraint)
    ccall((:cpConstraintIsGrooveJoint, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpGrooveJointAlloc()
    ccall((:cpGrooveJointAlloc, libchipmunk_name), Ptr{cpGrooveJoint}, ())
end

function cpGrooveJointInit(joint, a, b, groove_a, groove_b, anchorB)
    ccall((:cpGrooveJointInit, libchipmunk_name), Ptr{cpGrooveJoint}, (Ptr{cpGrooveJoint}, Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect, cpVect), joint, a, b, groove_a, groove_b, anchorB)
end

function cpGrooveJointNew(a, b, groove_a, groove_b, anchorB)
    ccall((:cpGrooveJointNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect, cpVect), a, b, groove_a, groove_b, anchorB)
end

function cpGrooveJointGetGrooveA(constraint)
    ccall((:cpGrooveJointGetGrooveA, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpGrooveJointSetGrooveA(constraint, grooveA)
    ccall((:cpGrooveJointSetGrooveA, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, grooveA)
end

function cpGrooveJointGetGrooveB(constraint)
    ccall((:cpGrooveJointGetGrooveB, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpGrooveJointSetGrooveB(constraint, grooveB)
    ccall((:cpGrooveJointSetGrooveB, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, grooveB)
end

function cpGrooveJointGetAnchorB(constraint)
    ccall((:cpGrooveJointGetAnchorB, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpGrooveJointSetAnchorB(constraint, anchorB)
    ccall((:cpGrooveJointSetAnchorB, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorB)
end

function cpConstraintIsDampedSpring(constraint)
    ccall((:cpConstraintIsDampedSpring, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpDampedSpringAlloc()
    ccall((:cpDampedSpringAlloc, libchipmunk_name), Ptr{cpDampedSpring}, ())
end

function cpDampedSpringInit(joint, a, b, anchorA, anchorB, restLength, stiffness, damping)
    ccall((:cpDampedSpringInit, libchipmunk_name), Ptr{cpDampedSpring}, (Ptr{cpDampedSpring}, Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect, cpFloat, cpFloat, cpFloat), joint, a, b, anchorA, anchorB, restLength, stiffness, damping)
end

function cpDampedSpringNew(a, b, anchorA, anchorB, restLength, stiffness, damping)
    ccall((:cpDampedSpringNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpVect, cpVect, cpFloat, cpFloat, cpFloat), a, b, anchorA, anchorB, restLength, stiffness, damping)
end

function cpDampedSpringGetAnchorA(constraint)
    ccall((:cpDampedSpringGetAnchorA, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpDampedSpringSetAnchorA(constraint, anchorA)
    ccall((:cpDampedSpringSetAnchorA, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorA)
end

function cpDampedSpringGetAnchorB(constraint)
    ccall((:cpDampedSpringGetAnchorB, libchipmunk_name), cpVect, (Ptr{cpConstraint},), constraint)
end

function cpDampedSpringSetAnchorB(constraint, anchorB)
    ccall((:cpDampedSpringSetAnchorB, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpVect), constraint, anchorB)
end

function cpDampedSpringGetRestLength(constraint)
    ccall((:cpDampedSpringGetRestLength, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpDampedSpringSetRestLength(constraint, restLength)
    ccall((:cpDampedSpringSetRestLength, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, restLength)
end

function cpDampedSpringGetStiffness(constraint)
    ccall((:cpDampedSpringGetStiffness, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpDampedSpringSetStiffness(constraint, stiffness)
    ccall((:cpDampedSpringSetStiffness, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, stiffness)
end

function cpDampedSpringGetDamping(constraint)
    ccall((:cpDampedSpringGetDamping, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpDampedSpringSetDamping(constraint, damping)
    ccall((:cpDampedSpringSetDamping, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, damping)
end

function cpDampedSpringGetSpringForceFunc(constraint)
    ccall((:cpDampedSpringGetSpringForceFunc, libchipmunk_name), cpDampedSpringForceFunc, (Ptr{cpConstraint},), constraint)
end

function cpDampedSpringSetSpringForceFunc(constraint, springForceFunc)
    ccall((:cpDampedSpringSetSpringForceFunc, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpDampedSpringForceFunc), constraint, springForceFunc)
end

function cpConstraintIsDampedRotarySpring(constraint)
    ccall((:cpConstraintIsDampedRotarySpring, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpDampedRotarySpringAlloc()
    ccall((:cpDampedRotarySpringAlloc, libchipmunk_name), Ptr{cpDampedRotarySpring}, ())
end

function cpDampedRotarySpringInit(joint, a, b, restAngle, stiffness, damping)
    ccall((:cpDampedRotarySpringInit, libchipmunk_name), Ptr{cpDampedRotarySpring}, (Ptr{cpDampedRotarySpring}, Ptr{cpBody}, Ptr{cpBody}, cpFloat, cpFloat, cpFloat), joint, a, b, restAngle, stiffness, damping)
end

function cpDampedRotarySpringNew(a, b, restAngle, stiffness, damping)
    ccall((:cpDampedRotarySpringNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpFloat, cpFloat, cpFloat), a, b, restAngle, stiffness, damping)
end

function cpDampedRotarySpringGetRestAngle(constraint)
    ccall((:cpDampedRotarySpringGetRestAngle, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpDampedRotarySpringSetRestAngle(constraint, restAngle)
    ccall((:cpDampedRotarySpringSetRestAngle, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, restAngle)
end

function cpDampedRotarySpringGetStiffness(constraint)
    ccall((:cpDampedRotarySpringGetStiffness, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpDampedRotarySpringSetStiffness(constraint, stiffness)
    ccall((:cpDampedRotarySpringSetStiffness, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, stiffness)
end

function cpDampedRotarySpringGetDamping(constraint)
    ccall((:cpDampedRotarySpringGetDamping, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpDampedRotarySpringSetDamping(constraint, damping)
    ccall((:cpDampedRotarySpringSetDamping, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, damping)
end

function cpDampedRotarySpringGetSpringTorqueFunc(constraint)
    ccall((:cpDampedRotarySpringGetSpringTorqueFunc, libchipmunk_name), cpDampedRotarySpringTorqueFunc, (Ptr{cpConstraint},), constraint)
end

function cpDampedRotarySpringSetSpringTorqueFunc(constraint, springTorqueFunc)
    ccall((:cpDampedRotarySpringSetSpringTorqueFunc, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpDampedRotarySpringTorqueFunc), constraint, springTorqueFunc)
end

function cpConstraintIsRotaryLimitJoint(constraint)
    ccall((:cpConstraintIsRotaryLimitJoint, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpRotaryLimitJointAlloc()
    ccall((:cpRotaryLimitJointAlloc, libchipmunk_name), Ptr{cpRotaryLimitJoint}, ())
end

function cpRotaryLimitJointInit(joint, a, b, min, max)
    ccall((:cpRotaryLimitJointInit, libchipmunk_name), Ptr{cpRotaryLimitJoint}, (Ptr{cpRotaryLimitJoint}, Ptr{cpBody}, Ptr{cpBody}, cpFloat, cpFloat), joint, a, b, min, max)
end

function cpRotaryLimitJointNew(a, b, min, max)
    ccall((:cpRotaryLimitJointNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpFloat, cpFloat), a, b, min, max)
end

function cpRotaryLimitJointGetMin(constraint)
    ccall((:cpRotaryLimitJointGetMin, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpRotaryLimitJointSetMin(constraint, min)
    ccall((:cpRotaryLimitJointSetMin, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, min)
end

function cpRotaryLimitJointGetMax(constraint)
    ccall((:cpRotaryLimitJointGetMax, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpRotaryLimitJointSetMax(constraint, max)
    ccall((:cpRotaryLimitJointSetMax, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, max)
end

function cpConstraintIsRatchetJoint(constraint)
    ccall((:cpConstraintIsRatchetJoint, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpRatchetJointAlloc()
    ccall((:cpRatchetJointAlloc, libchipmunk_name), Ptr{cpRatchetJoint}, ())
end

function cpRatchetJointInit(joint, a, b, phase, ratchet)
    ccall((:cpRatchetJointInit, libchipmunk_name), Ptr{cpRatchetJoint}, (Ptr{cpRatchetJoint}, Ptr{cpBody}, Ptr{cpBody}, cpFloat, cpFloat), joint, a, b, phase, ratchet)
end

function cpRatchetJointNew(a, b, phase, ratchet)
    ccall((:cpRatchetJointNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpFloat, cpFloat), a, b, phase, ratchet)
end

function cpRatchetJointGetAngle(constraint)
    ccall((:cpRatchetJointGetAngle, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpRatchetJointSetAngle(constraint, angle)
    ccall((:cpRatchetJointSetAngle, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, angle)
end

function cpRatchetJointGetPhase(constraint)
    ccall((:cpRatchetJointGetPhase, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpRatchetJointSetPhase(constraint, phase)
    ccall((:cpRatchetJointSetPhase, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, phase)
end

function cpRatchetJointGetRatchet(constraint)
    ccall((:cpRatchetJointGetRatchet, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpRatchetJointSetRatchet(constraint, ratchet)
    ccall((:cpRatchetJointSetRatchet, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, ratchet)
end

function cpConstraintIsGearJoint(constraint)
    ccall((:cpConstraintIsGearJoint, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpGearJointAlloc()
    ccall((:cpGearJointAlloc, libchipmunk_name), Ptr{cpGearJoint}, ())
end

function cpGearJointInit(joint, a, b, phase, ratio)
    ccall((:cpGearJointInit, libchipmunk_name), Ptr{cpGearJoint}, (Ptr{cpGearJoint}, Ptr{cpBody}, Ptr{cpBody}, cpFloat, cpFloat), joint, a, b, phase, ratio)
end

function cpGearJointNew(a, b, phase, ratio)
    ccall((:cpGearJointNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpFloat, cpFloat), a, b, phase, ratio)
end

function cpGearJointGetPhase(constraint)
    ccall((:cpGearJointGetPhase, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpGearJointSetPhase(constraint, phase)
    ccall((:cpGearJointSetPhase, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, phase)
end

function cpGearJointGetRatio(constraint)
    ccall((:cpGearJointGetRatio, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpGearJointSetRatio(constraint, ratio)
    ccall((:cpGearJointSetRatio, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, ratio)
end

function cpConstraintIsSimpleMotor(constraint)
    ccall((:cpConstraintIsSimpleMotor, libchipmunk_name), cpBool, (Ptr{cpConstraint},), constraint)
end

function cpSimpleMotorAlloc()
    ccall((:cpSimpleMotorAlloc, libchipmunk_name), Ptr{cpSimpleMotor}, ())
end

function cpSimpleMotorInit(joint, a, b, rate)
    ccall((:cpSimpleMotorInit, libchipmunk_name), Ptr{cpSimpleMotor}, (Ptr{cpSimpleMotor}, Ptr{cpBody}, Ptr{cpBody}, cpFloat), joint, a, b, rate)
end

function cpSimpleMotorNew(a, b, rate)
    ccall((:cpSimpleMotorNew, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpBody}, Ptr{cpBody}, cpFloat), a, b, rate)
end

function cpSimpleMotorGetRate(constraint)
    ccall((:cpSimpleMotorGetRate, libchipmunk_name), cpFloat, (Ptr{cpConstraint},), constraint)
end

function cpSimpleMotorSetRate(constraint, rate)
    ccall((:cpSimpleMotorSetRate, libchipmunk_name), Cvoid, (Ptr{cpConstraint}, cpFloat), constraint, rate)
end

function cpSpaceAlloc()
    ccall((:cpSpaceAlloc, libchipmunk_name), Ptr{cpSpace}, ())
end

function cpSpaceInit(space)
    ccall((:cpSpaceInit, libchipmunk_name), Ptr{cpSpace}, (Ptr{cpSpace},), space)
end

function cpSpaceNew()
    ccall((:cpSpaceNew, libchipmunk_name), Ptr{cpSpace}, ())
end

function cpSpaceDestroy(space)
    ccall((:cpSpaceDestroy, libchipmunk_name), Cvoid, (Ptr{cpSpace},), space)
end

function cpSpaceFree(space)
    ccall((:cpSpaceFree, libchipmunk_name), Cvoid, (Ptr{cpSpace},), space)
end

function cpSpaceGetIterations(space)
    ccall((:cpSpaceGetIterations, libchipmunk_name), Cint, (Ptr{cpSpace},), space)
end

function cpSpaceSetIterations(space, iterations)
    ccall((:cpSpaceSetIterations, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Cint), space, iterations)
end

function cpSpaceGetGravity(space)
    ccall((:cpSpaceGetGravity, libchipmunk_name), cpVect, (Ptr{cpSpace},), space)
end

function cpSpaceSetGravity(space, gravity)
    ccall((:cpSpaceSetGravity, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpVect), space, gravity)
end

function cpSpaceGetDamping(space)
    ccall((:cpSpaceGetDamping, libchipmunk_name), cpFloat, (Ptr{cpSpace},), space)
end

function cpSpaceSetDamping(space, damping)
    ccall((:cpSpaceSetDamping, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpFloat), space, damping)
end

function cpSpaceGetIdleSpeedThreshold(space)
    ccall((:cpSpaceGetIdleSpeedThreshold, libchipmunk_name), cpFloat, (Ptr{cpSpace},), space)
end

function cpSpaceSetIdleSpeedThreshold(space, idleSpeedThreshold)
    ccall((:cpSpaceSetIdleSpeedThreshold, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpFloat), space, idleSpeedThreshold)
end

function cpSpaceGetSleepTimeThreshold(space)
    ccall((:cpSpaceGetSleepTimeThreshold, libchipmunk_name), cpFloat, (Ptr{cpSpace},), space)
end

function cpSpaceSetSleepTimeThreshold(space, sleepTimeThreshold)
    ccall((:cpSpaceSetSleepTimeThreshold, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpFloat), space, sleepTimeThreshold)
end

function cpSpaceGetCollisionSlop(space)
    ccall((:cpSpaceGetCollisionSlop, libchipmunk_name), cpFloat, (Ptr{cpSpace},), space)
end

function cpSpaceSetCollisionSlop(space, collisionSlop)
    ccall((:cpSpaceSetCollisionSlop, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpFloat), space, collisionSlop)
end

function cpSpaceGetCollisionBias(space)
    ccall((:cpSpaceGetCollisionBias, libchipmunk_name), cpFloat, (Ptr{cpSpace},), space)
end

function cpSpaceSetCollisionBias(space, collisionBias)
    ccall((:cpSpaceSetCollisionBias, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpFloat), space, collisionBias)
end

function cpSpaceGetCollisionPersistence(space)
    ccall((:cpSpaceGetCollisionPersistence, libchipmunk_name), cpTimestamp, (Ptr{cpSpace},), space)
end

function cpSpaceSetCollisionPersistence(space, collisionPersistence)
    ccall((:cpSpaceSetCollisionPersistence, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpTimestamp), space, collisionPersistence)
end

function cpSpaceGetUserData(space)
    ccall((:cpSpaceGetUserData, libchipmunk_name), cpDataPointer, (Ptr{cpSpace},), space)
end

function cpSpaceSetUserData(space, userData)
    ccall((:cpSpaceSetUserData, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpDataPointer), space, userData)
end

function cpSpaceGetStaticBody(space)
    ccall((:cpSpaceGetStaticBody, libchipmunk_name), Ptr{cpBody}, (Ptr{cpSpace},), space)
end

function cpSpaceGetCurrentTimeStep(space)
    ccall((:cpSpaceGetCurrentTimeStep, libchipmunk_name), cpFloat, (Ptr{cpSpace},), space)
end

function cpSpaceIsLocked(space)
    ccall((:cpSpaceIsLocked, libchipmunk_name), cpBool, (Ptr{cpSpace},), space)
end

function cpSpaceAddDefaultCollisionHandler(space)
    ccall((:cpSpaceAddDefaultCollisionHandler, libchipmunk_name), Ptr{cpCollisionHandler}, (Ptr{cpSpace},), space)
end

function cpSpaceAddCollisionHandler(space, a, b)
    ccall((:cpSpaceAddCollisionHandler, libchipmunk_name), Ptr{cpCollisionHandler}, (Ptr{cpSpace}, cpCollisionType, cpCollisionType), space, a, b)
end

function cpSpaceAddWildcardHandler(space, type)
    ccall((:cpSpaceAddWildcardHandler, libchipmunk_name), Ptr{cpCollisionHandler}, (Ptr{cpSpace}, cpCollisionType), space, type)
end

function cpSpaceAddShape(space, shape)
    ccall((:cpSpaceAddShape, libchipmunk_name), Ptr{cpShape}, (Ptr{cpSpace}, Ptr{cpShape}), space, shape)
end

function cpSpaceAddBody(space, body)
    ccall((:cpSpaceAddBody, libchipmunk_name), Ptr{cpBody}, (Ptr{cpSpace}, Ptr{cpBody}), space, body)
end

function cpSpaceAddConstraint(space, constraint)
    ccall((:cpSpaceAddConstraint, libchipmunk_name), Ptr{cpConstraint}, (Ptr{cpSpace}, Ptr{cpConstraint}), space, constraint)
end

function cpSpaceRemoveShape(space, shape)
    ccall((:cpSpaceRemoveShape, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Ptr{cpShape}), space, shape)
end

function cpSpaceRemoveBody(space, body)
    ccall((:cpSpaceRemoveBody, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Ptr{cpBody}), space, body)
end

function cpSpaceRemoveConstraint(space, constraint)
    ccall((:cpSpaceRemoveConstraint, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Ptr{cpConstraint}), space, constraint)
end

function cpSpaceContainsShape(space, shape)
    ccall((:cpSpaceContainsShape, libchipmunk_name), cpBool, (Ptr{cpSpace}, Ptr{cpShape}), space, shape)
end

function cpSpaceContainsBody(space, body)
    ccall((:cpSpaceContainsBody, libchipmunk_name), cpBool, (Ptr{cpSpace}, Ptr{cpBody}), space, body)
end

function cpSpaceContainsConstraint(space, constraint)
    ccall((:cpSpaceContainsConstraint, libchipmunk_name), cpBool, (Ptr{cpSpace}, Ptr{cpConstraint}), space, constraint)
end

function cpSpaceAddPostStepCallback(space, func, key, data)
    ccall((:cpSpaceAddPostStepCallback, libchipmunk_name), cpBool, (Ptr{cpSpace}, cpPostStepFunc, Ptr{Cvoid}, Ptr{Cvoid}), space, func, key, data)
end

function cpSpacePointQuery(space, point, maxDistance, filter, func, data)
    ccall((:cpSpacePointQuery, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpVect, cpFloat, cpShapeFilter, cpSpacePointQueryFunc, Ptr{Cvoid}), space, point, maxDistance, filter, func, data)
end

function cpSpacePointQueryNearest(space, point, maxDistance, filter, out)
    ccall((:cpSpacePointQueryNearest, libchipmunk_name), Ptr{cpShape}, (Ptr{cpSpace}, cpVect, cpFloat, cpShapeFilter, Ptr{cpPointQueryInfo}), space, point, maxDistance, filter, out)
end

function cpSpaceSegmentQuery(space, start, _end, radius, filter, func, data)
    ccall((:cpSpaceSegmentQuery, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpVect, cpVect, cpFloat, cpShapeFilter, cpSpaceSegmentQueryFunc, Ptr{Cvoid}), space, start, _end, radius, filter, func, data)
end

function cpSpaceSegmentQueryFirst(space, start, _end, radius, filter, out)
    ccall((:cpSpaceSegmentQueryFirst, libchipmunk_name), Ptr{cpShape}, (Ptr{cpSpace}, cpVect, cpVect, cpFloat, cpShapeFilter, Ptr{cpSegmentQueryInfo}), space, start, _end, radius, filter, out)
end

function cpSpaceBBQuery(space, bb, filter, func, data)
    ccall((:cpSpaceBBQuery, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpBB, cpShapeFilter, cpSpaceBBQueryFunc, Ptr{Cvoid}), space, bb, filter, func, data)
end

function cpSpaceShapeQuery(space, shape, func, data)
    ccall((:cpSpaceShapeQuery, libchipmunk_name), cpBool, (Ptr{cpSpace}, Ptr{cpShape}, cpSpaceShapeQueryFunc, Ptr{Cvoid}), space, shape, func, data)
end

function cpSpaceEachBody(space, func, data)
    ccall((:cpSpaceEachBody, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpSpaceBodyIteratorFunc, Ptr{Cvoid}), space, func, data)
end

function cpSpaceEachShape(space, func, data)
    ccall((:cpSpaceEachShape, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpSpaceShapeIteratorFunc, Ptr{Cvoid}), space, func, data)
end

function cpSpaceEachConstraint(space, func, data)
    ccall((:cpSpaceEachConstraint, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpSpaceConstraintIteratorFunc, Ptr{Cvoid}), space, func, data)
end

function cpSpaceReindexStatic(space)
    ccall((:cpSpaceReindexStatic, libchipmunk_name), Cvoid, (Ptr{cpSpace},), space)
end

function cpSpaceReindexShape(space, shape)
    ccall((:cpSpaceReindexShape, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Ptr{cpShape}), space, shape)
end

function cpSpaceReindexShapesForBody(space, body)
    ccall((:cpSpaceReindexShapesForBody, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Ptr{cpBody}), space, body)
end

function cpSpaceUseSpatialHash(space, dim, count)
    ccall((:cpSpaceUseSpatialHash, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpFloat, Cint), space, dim, count)
end

function cpSpaceStep(space, dt)
    ccall((:cpSpaceStep, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpFloat), space, dt)
end

function cpSpaceDebugDraw(space, options)
    ccall((:cpSpaceDebugDraw, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Ptr{cpSpaceDebugDrawOptions}), space, options)
end

function cpMomentForCircle(m, r1, r2, offset)
    ccall((:cpMomentForCircle, libchipmunk_name), cpFloat, (cpFloat, cpFloat, cpFloat, cpVect), m, r1, r2, offset)
end

function cpAreaForCircle(r1, r2)
    ccall((:cpAreaForCircle, libchipmunk_name), cpFloat, (cpFloat, cpFloat), r1, r2)
end

function cpMomentForSegment(m, a, b, radius)
    ccall((:cpMomentForSegment, libchipmunk_name), cpFloat, (cpFloat, cpVect, cpVect, cpFloat), m, a, b, radius)
end

function cpAreaForSegment(a, b, radius)
    ccall((:cpAreaForSegment, libchipmunk_name), cpFloat, (cpVect, cpVect, cpFloat), a, b, radius)
end

function cpMomentForPoly(m, count, verts, offset, radius)
    ccall((:cpMomentForPoly, libchipmunk_name), cpFloat, (cpFloat, Cint, Ptr{cpVect}, cpVect, cpFloat), m, count, verts, offset, radius)
end

function cpAreaForPoly(count, verts, radius)
    ccall((:cpAreaForPoly, libchipmunk_name), cpFloat, (Cint, Ptr{cpVect}, cpFloat), count, verts, radius)
end

function cpCentroidForPoly(count, verts)
    ccall((:cpCentroidForPoly, libchipmunk_name), cpVect, (Cint, Ptr{cpVect}), count, verts)
end

function cpMomentForBox(m, width, height)
    ccall((:cpMomentForBox, libchipmunk_name), cpFloat, (cpFloat, cpFloat, cpFloat), m, width, height)
end

function cpMomentForBox2(m, box)
    ccall((:cpMomentForBox2, libchipmunk_name), cpFloat, (cpFloat, cpBB), m, box)
end

function cpConvexHull(count, verts, result, first, tol)
    ccall((:cpConvexHull, libchipmunk_name), Cint, (Cint, Ptr{cpVect}, Ptr{cpVect}, Ptr{Cint}, cpFloat), count, verts, result, first, tol)
end

function cpClosetPointOnSegment(p, a, b)
    ccall((:cpClosetPointOnSegment, libchipmunk_name), cpVect, (cpVect, cpVect, cpVect), p, a, b)
end

function cpSpaceEachBody_b(space, block)
    ccall((:cpSpaceEachBody_b, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Cvoid), space, block)
end

function cpSpaceEachShape_b(space, block)
    ccall((:cpSpaceEachShape_b, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Cvoid), space, block)
end

function cpSpaceEachConstraint_b(space, block)
    ccall((:cpSpaceEachConstraint_b, libchipmunk_name), Cvoid, (Ptr{cpSpace}, Cvoid), space, block)
end

function cpBodyEachShape_b(body, block)
    ccall((:cpBodyEachShape_b, libchipmunk_name), Cvoid, (Ptr{cpBody}, Cvoid), body, block)
end

function cpBodyEachConstraint_b(body, block)
    ccall((:cpBodyEachConstraint_b, libchipmunk_name), Cvoid, (Ptr{cpBody}, Cvoid), body, block)
end

function cpBodyEachArbiter_b(body, block)
    ccall((:cpBodyEachArbiter_b, libchipmunk_name), Cvoid, (Ptr{cpBody}, Cvoid), body, block)
end

function cpSpacePointQuery_b(space, point, maxDistance, filter, block)
    ccall((:cpSpacePointQuery_b, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpVect, cpFloat, cpShapeFilter, cpSpacePointQueryBlock), space, point, maxDistance, filter, block)
end

function cpSpaceSegmentQuery_b(space, start, _end, radius, filter, block)
    ccall((:cpSpaceSegmentQuery_b, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpVect, cpVect, cpFloat, cpShapeFilter, cpSpaceSegmentQueryBlock), space, start, _end, radius, filter, block)
end

function cpSpaceBBQuery_b(space, bb, filter, block)
    ccall((:cpSpaceBBQuery_b, libchipmunk_name), Cvoid, (Ptr{cpSpace}, cpBB, cpShapeFilter, cpSpaceBBQueryBlock), space, bb, filter, block)
end

function cpSpaceShapeQuery_b(space, shape, block)
    ccall((:cpSpaceShapeQuery_b, libchipmunk_name), cpBool, (Ptr{cpSpace}, Ptr{cpShape}, cpSpaceShapeQueryBlock), space, shape, block)
end
# Julia wrapper for header: chipmunk_types.h
# Automatically generated using Clang.jl


function cpfmax(a, b)
    ccall((:cpfmax, libchipmunk_name), cpFloat, (cpFloat, cpFloat), a, b)
end

function cpfmin(a, b)
    ccall((:cpfmin, libchipmunk_name), cpFloat, (cpFloat, cpFloat), a, b)
end

function cpfabs(f)
    ccall((:cpfabs, libchipmunk_name), cpFloat, (cpFloat,), f)
end

function cpfclamp(f, min, max)
    ccall((:cpfclamp, libchipmunk_name), cpFloat, (cpFloat, cpFloat, cpFloat), f, min, max)
end

function cpfclamp01(f)
    ccall((:cpfclamp01, libchipmunk_name), cpFloat, (cpFloat,), f)
end

function cpflerp(f1, f2, t)
    ccall((:cpflerp, libchipmunk_name), cpFloat, (cpFloat, cpFloat, cpFloat), f1, f2, t)
end

function cpflerpconst(f1, f2, d)
    ccall((:cpflerpconst, libchipmunk_name), cpFloat, (cpFloat, cpFloat, cpFloat), f1, f2, d)
end
