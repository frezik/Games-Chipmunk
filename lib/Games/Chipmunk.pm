package Games::Chipmunk;

use 5.008000;
use strict;
use warnings;
use Carp;

require Exporter;
use AutoLoader;

our @ISA = qw(Exporter);

our @EXPORT =  qw(
	CP_ALLOW_PRIVATE_ACCESS
	CP_BUFFER_BYTES
	CP_VERSION_MAJOR
	CP_VERSION_MINOR
	CP_VERSION_RELEASE
	cpcalloc
	cpfree
	cprealloc
	cpArbiterGetContactPointSet
	cpArbiterGetCount
	cpArbiterGetDepth
	cpArbiterGetNormal
	cpArbiterGetPoint
	cpArbiterGetSurfaceVelocity
	cpArbiterIgnore
	cpArbiterIsFirstContact
	cpArbiterSetContactPointSet
	cpArbiterSetSurfaceVelocity
	cpArbiterTotalImpulse
	cpArbiterTotalImpulseWithFriction
	cpArbiterTotalKE
	cpAreaForCircle
	cpAreaForPoly
	cpAreaForSegment
	cpBBTreeAlloc
	cpBBTreeInit
	cpBBTreeNew
	cpBBTreeOptimize
	cpBBTreeSetVelocityFunc
	cpBBWrapVect
	cpBodyActivate
	cpBodyActivateStatic
	cpBodyAlloc
	cpBodyApplyForce
	cpBodyApplyImpulse
	cpBodyDestroy
	cpBodyEachArbiter
	cpBodyEachConstraint
	cpBodyEachShape
	cpBodyFree
	cpBodyGetVelAtLocalPoint
	cpBodyGetVelAtWorldPoint
	cpBodyInit
	cpBodyInitStatic
	cpBodyNew
	cpBodyNewStatic
	cpBodyResetForces
	cpBodySanityCheck
	cpBodySetAngle
	cpBodySetMass
	cpBodySetMoment
	cpBodySetPos
	cpBodySleep
	cpBodySleepWithGroup
	cpBodyUpdatePosition
	cpBodyUpdateVelocity
	cpBoxShapeInit
	cpBoxShapeInit2
	cpBoxShapeNew
	cpBoxShapeNew2
	cpCentroidForPoly
	cpCircleShapeAlloc
	cpCircleShapeGetOffset
	cpCircleShapeGetRadius
	cpCircleShapeInit
	cpCircleShapeNew
	cpConstraintDestroy
	cpConstraintFree
	cpConvexHull
	cpDampedRotarySpringAlloc
	cpDampedRotarySpringGetClass
	cpDampedRotarySpringInit
	cpDampedRotarySpringNew
	cpDampedSpringAlloc
	cpDampedSpringGetClass
	cpDampedSpringInit
	cpDampedSpringNew
	cpEnableSegmentToSegmentCollisions
	cpGearJointAlloc
	cpGearJointGetClass
	cpGearJointInit
	cpGearJointNew
	cpGearJointSetRatio
	cpGrooveJointAlloc
	cpGrooveJointGetClass
	cpGrooveJointInit
	cpGrooveJointNew
	cpGrooveJointSetGrooveA
	cpGrooveJointSetGrooveB
	cpInitChipmunk
	cpMessage
	cpMomentForBox
	cpMomentForBox2
	cpMomentForCircle
	cpMomentForPoly
	cpMomentForSegment
	cpPinJointAlloc
	cpPinJointGetClass
	cpPinJointInit
	cpPinJointNew
	cpPivotJointAlloc
	cpPivotJointGetClass
	cpPivotJointInit
	cpPivotJointNew
	cpPivotJointNew2
	cpPolyShapeAlloc
	cpPolyShapeGetNumVerts
	cpPolyShapeGetVert
	cpPolyShapeInit
	cpPolyShapeNew
	cpPolyValidate
	cpRatchetJointAlloc
	cpRatchetJointGetClass
	cpRatchetJointInit
	cpRatchetJointNew
	cpRecenterPoly
	cpResetShapeIdCounter
	cpRotaryLimitJointAlloc
	cpRotaryLimitJointGetClass
	cpRotaryLimitJointInit
	cpRotaryLimitJointNew
	cpSegmentShapeAlloc
	cpSegmentShapeGetA
	cpSegmentShapeGetB
	cpSegmentShapeGetNormal
	cpSegmentShapeGetRadius
	cpSegmentShapeInit
	cpSegmentShapeNew
	cpSegmentShapeSetNeighbors
	cpShapeCacheBB
	cpShapeDestroy
	cpShapeFree
	cpShapeNearestPointQuery
	cpShapePointQuery
	cpShapeSegmentQuery
	cpShapeSetBody
	cpShapeUpdate
	cpSimpleMotorAlloc
	cpSimpleMotorGetClass
	cpSimpleMotorInit
	cpSimpleMotorNew
	cpSlideJointAlloc
	cpSlideJointGetClass
	cpSlideJointInit
	cpSlideJointNew
	cpSpaceActivateShapesTouchingShape
	cpSpaceAddBody
	cpSpaceAddCollisionHandler
	cpSpaceAddConstraint
	cpSpaceAddPostStepCallback
	cpSpaceAddShape
	cpSpaceAddStaticShape
	cpSpaceAlloc
	cpSpaceBBQuery
	cpSpaceContainsBody
	cpSpaceContainsConstraint
	cpSpaceContainsShape
	cpSpaceConvertBodyToDynamic
	cpSpaceConvertBodyToStatic
	cpSpaceDestroy
	cpSpaceEachBody
	cpSpaceEachConstraint
	cpSpaceEachShape
	cpSpaceFree
	cpSpaceHashAlloc
	cpSpaceHashInit
	cpSpaceHashNew
	cpSpaceHashResize
	cpSpaceInit
	cpSpaceNearestPointQuery
	cpSpaceNearestPointQueryNearest
	cpSpaceNew
	cpSpacePointQuery
	cpSpacePointQueryFirst
	cpSpaceReindexShape
	cpSpaceReindexShapesForBody
	cpSpaceReindexStatic
	cpSpaceRemoveBody
	cpSpaceRemoveCollisionHandler
	cpSpaceRemoveConstraint
	cpSpaceRemoveShape
	cpSpaceRemoveStaticShape
	cpSpaceSegmentQuery
	cpSpaceSegmentQueryFirst
	cpSpaceSetDefaultCollisionHandler
	cpSpaceShapeQuery
	cpSpaceStep
	cpSpaceUseSpatialHash
	cpSpatialIndexCollideStatic
	cpSpatialIndexFree
	cpSweep1DAlloc
	cpSweep1DInit
	cpSweep1DNew
	cpvslerp
	cpvslerpconst
	cpvstr
    $CPV_ZERO

    cpv
    cpveql
    cpvadd
    cpvneg
    cpvsub
    cpvmult
    cpvdot
    cpvcross
    cpvperp
    cpvrperp
    cpvproject
    cpvforangle
    cpvtoangle
    cpvrotate
    cpvunrotate
    cpvlengthsq
    cpvlength
    cpvlerp
    cpvnormalize
    cpvnormalize_safe
    cpvclamp
    cpvlerpconst
    cpvdist
    cpvdistsq
    cpvnear
    cpfmax
    cpfmin
    cpfabs
    cpfclamp
    cpflerp
    cpflerpconst
    cpBBNew
    cpBBNewForCircle
    cpBBIntersects
    cpBBContainsBB
    cpBBContainsVect
    cpBBMerge
    cpBBExpand
    cpBBArea
    cpBBMergedArea
    cpBBSegmentQuery
    cpBBIntersectsSegment
    cpBBClampVect
    cpBodyGetMass
    cpBodyGetMoment
    cpBodyGetPos
    cpBodyGetAngle
    cpBodyGetRot
    cpBodyIsSleeping
    cpBodyIsStatic
    cpBodyIsRogue
    cpBodyLocal2World
    cpBodyWorld2Local
    cpBodyKineticEnergy
    cpShapeGetBB
    cpArbiterGetShapes
    cpArbiterGetBodies
    cpArbiterIsFirstContact
    cpArbiterGetCount
    cpConstraintGetA
    cpConstraintGetB
    cpConstraintGetImpulse
    cpGearJointGetRatio
    cpGrooveJointGetGrooveA
    cpGrooveJointGetGrooveB
    cpSegmentQueryHitPoint
    cpSegmentQueryHitDist
    cpSpaceSetGravity
    cpSpatialIndexDestroy
    cpSpatialIndexCount
    cpSpatialIndexEach
    cpSpatialIndexContains
    cpSpatialIndexInsert
    cpSpatialIndexRemove
    cpSpatialIndexReindex
    cpSpatialIndexReindexObject
    cpSpatialIndexSegmentQuery
    cpSpatialIndexQuery
    cpSpatialIndexReindexQuery
    cpSpaceGetStaticBody
    cpSpaceGetCurrentTimeStep
    cpSpaceIsLocked
    cpShapeSetFriction
    cpBodyGetPos
    cpBodyGetVel
);
our @EXPORT_OK = @EXPORT;


our $VERSION = '0.01';

sub AUTOLOAD {
    # This AUTOLOAD is used to 'autoload' constants from the constant()
    # XS function.

    my $constname;
    our $AUTOLOAD;
    ($constname = $AUTOLOAD) =~ s/.*:://;
    croak "&Games::Chipmunk::constant not defined" if $constname eq 'constant';
    my ($error, $val) = constant($constname);
    if ($error) { croak $error; }
    {
	no strict 'refs';
	# Fixed between 5.005_53 and 5.005_61
#XXX	if ($] >= 5.00561) {
#XXX	    *$AUTOLOAD = sub () { $val };
#XXX	}
#XXX	else {
	    *$AUTOLOAD = sub { $val };
#XXX	}
    }
    goto &$AUTOLOAD;
}

require XSLoader;
XSLoader::load('Games::Chipmunk', $VERSION);

our $CPV_ZERO = _CPVZERO();



1;
__END__
# Below is stub documentation for your module. You'd better edit it!

=head1 NAME

Games::Chipmunk - Perl extension for blah blah blah

=head1 SYNOPSIS

  use Games::Chipmunk;
  blah blah blah

=head1 DESCRIPTION

Stub documentation for Games::Chipmunk, created by h2xs. It looks like the
author of the extension was negligent enough to leave the stub
unedited.

Blah blah blah.

=head2 EXPORT

None by default.

=head2 Exportable constants

  CP_ALLOW_PRIVATE_ACCESS
  CP_BUFFER_BYTES
  CP_VERSION_MAJOR
  CP_VERSION_MINOR
  CP_VERSION_RELEASE
  cpcalloc
  cpfree
  cprealloc

=head2 Exportable functions

  cpFloat cpArbiterGetDepth(const cpArbiter *arb, int i)
  cpVect cpArbiterGetNormal(const cpArbiter *arb, int i)
  cpVect cpArbiterGetPoint(const cpArbiter *arb, int i)
  cpVect cpArbiterGetSurfaceVelocity(cpArbiter *arb)
  void cpArbiterIgnore(cpArbiter *arb)
  cpBool cpArbiterIsFirstContact(const cpArbiter *arb)
  void cpArbiterSetContactPointSet(cpArbiter *arb, cpContactPointSet *set)
  void cpArbiterSetSurfaceVelocity(cpArbiter *arb, cpVect vr)
  cpVect cpArbiterTotalImpulse(const cpArbiter *arb)
  cpVect cpArbiterTotalImpulseWithFriction(const cpArbiter *arb)
  cpFloat cpArbiterTotalKE(const cpArbiter *arb)
  cpFloat cpAreaForCircle(cpFloat r1, cpFloat r2)
  cpFloat cpAreaForPoly(const int numVerts, const cpVect *verts)
  cpFloat cpAreaForSegment(cpVect a, cpVect b, cpFloat r)
  cpBBTree* cpBBTreeAlloc(void)
  cpSpatialIndex* cpBBTreeInit(cpBBTree *tree, cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex)
  cpSpatialIndex* cpBBTreeNew(cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex)
  void cpBBTreeOptimize(cpSpatialIndex *index)
  void cpBBTreeSetVelocityFunc(cpSpatialIndex *index, cpBBTreeVelocityFunc func)
  cpVect cpBBWrapVect(const cpBB bb, const cpVect v)
  void cpBodyActivate(cpBody *body)
  void cpBodyActivateStatic(cpBody *body, cpShape *filter)
  cpBody* cpBodyAlloc(void)
  void cpBodyApplyForce(cpBody *body, const cpVect f, const cpVect r)
  void cpBodyApplyImpulse(cpBody *body, const cpVect j, const cpVect r)
  void cpBodyDestroy(cpBody *body)
  void cpBodyEachArbiter(cpBody *body, cpBodyArbiterIteratorFunc func, void *data)
  void cpBodyEachConstraint(cpBody *body, cpBodyConstraintIteratorFunc func, void *data)
  void cpBodyEachShape(cpBody *body, cpBodyShapeIteratorFunc func, void *data)
  void cpBodyFree(cpBody *body)
  cpVect cpBodyGetVelAtLocalPoint(cpBody *body, cpVect point)
  cpVect cpBodyGetVelAtWorldPoint(cpBody *body, cpVect point)
  cpBody* cpBodyInit(cpBody *body, cpFloat m, cpFloat i)
  cpBody* cpBodyInitStatic(cpBody *body)
  cpBody* cpBodyNew(cpFloat m, cpFloat i)
  cpBody* cpBodyNewStatic(void)
  void cpBodyResetForces(cpBody *body)
  void cpBodySanityCheck(cpBody *body)
  void cpBodySetAngle(cpBody *body, cpFloat a)
  void cpBodySetMass(cpBody *body, cpFloat m)
  void cpBodySetMoment(cpBody *body, cpFloat i)
  void cpBodySetPos(cpBody *body, cpVect pos)
  void cpBodySleep(cpBody *body)
  void cpBodySleepWithGroup(cpBody *body, cpBody *group)
  void cpBodyUpdatePosition(cpBody *body, cpFloat dt)
  void cpBodyUpdateVelocity(cpBody *body, cpVect gravity, cpFloat damping, cpFloat dt)
  cpPolyShape* cpBoxShapeInit(cpPolyShape *poly, cpBody *body, cpFloat width, cpFloat height)
  cpPolyShape* cpBoxShapeInit2(cpPolyShape *poly, cpBody *body, cpBB box)
  cpShape* cpBoxShapeNew(cpBody *body, cpFloat width, cpFloat height)
  cpShape* cpBoxShapeNew2(cpBody *body, cpBB box)
  cpVect cpCentroidForPoly(const int numVerts, const cpVect *verts)
  cpCircleShape* cpCircleShapeAlloc(void)
  cpVect cpCircleShapeGetOffset(const cpShape *shape)
  cpFloat cpCircleShapeGetRadius(const cpShape *shape)
  cpCircleShape* cpCircleShapeInit(cpCircleShape *circle, cpBody *body, cpFloat radius, cpVect offset)
  cpShape* cpCircleShapeNew(cpBody *body, cpFloat radius, cpVect offset)
  void cpConstraintDestroy(cpConstraint *constraint)
  void cpConstraintFree(cpConstraint *constraint)
  int cpConvexHull(int count, cpVect *verts, cpVect *result, int *first, cpFloat tol)
  cpDampedRotarySpring* cpDampedRotarySpringAlloc(void)
  const cpConstraintClass *cpDampedRotarySpringGetClass(void)
  cpDampedRotarySpring* cpDampedRotarySpringInit(cpDampedRotarySpring *joint, cpBody *a, cpBody *b, cpFloat restAngle, cpFloat stiffness, cpFloat damping)
  cpConstraint* cpDampedRotarySpringNew(cpBody *a, cpBody *b, cpFloat restAngle, cpFloat stiffness, cpFloat damping)
  cpDampedSpring* cpDampedSpringAlloc(void)
  const cpConstraintClass *cpDampedSpringGetClass(void)
  cpDampedSpring* cpDampedSpringInit(cpDampedSpring *joint, cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2, cpFloat restLength, cpFloat stiffness, cpFloat damping)
  cpConstraint* cpDampedSpringNew(cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2, cpFloat restLength, cpFloat stiffness, cpFloat damping)
  void cpEnableSegmentToSegmentCollisions(void)
  cpGearJoint* cpGearJointAlloc(void)
  const cpConstraintClass *cpGearJointGetClass(void)
  cpGearJoint* cpGearJointInit(cpGearJoint *joint, cpBody *a, cpBody *b, cpFloat phase, cpFloat ratio)
  cpConstraint* cpGearJointNew(cpBody *a, cpBody *b, cpFloat phase, cpFloat ratio)
  void cpGearJointSetRatio(cpConstraint *constraint, cpFloat value)
  cpGrooveJoint* cpGrooveJointAlloc(void)
  const cpConstraintClass *cpGrooveJointGetClass(void)
  cpGrooveJoint* cpGrooveJointInit(cpGrooveJoint *joint, cpBody *a, cpBody *b, cpVect groove_a, cpVect groove_b, cpVect anchr2)
  cpConstraint* cpGrooveJointNew(cpBody *a, cpBody *b, cpVect groove_a, cpVect groove_b, cpVect anchr2)
  void cpGrooveJointSetGrooveA(cpConstraint *constraint, cpVect value)
  void cpGrooveJointSetGrooveB(cpConstraint *constraint, cpVect value)
  void cpInitChipmunk(void)
  void cpMessage(const char *condition, const char *file, int line, int isError, int isHardError, const char *message, ...)
  cpFloat cpMomentForBox(cpFloat m, cpFloat width, cpFloat height)
  cpFloat cpMomentForBox2(cpFloat m, cpBB box)
  cpFloat cpMomentForCircle(cpFloat m, cpFloat r1, cpFloat r2, cpVect offset)
  cpFloat cpMomentForPoly(cpFloat m, int numVerts, const cpVect *verts, cpVect offset)
  cpFloat cpMomentForSegment(cpFloat m, cpVect a, cpVect b)
  cpPinJoint* cpPinJointAlloc(void)
  const cpConstraintClass *cpPinJointGetClass(void)
  cpPinJoint* cpPinJointInit(cpPinJoint *joint, cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2)
  cpConstraint* cpPinJointNew(cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2)
  cpPivotJoint* cpPivotJointAlloc(void)
  const cpConstraintClass *cpPivotJointGetClass(void)
  cpPivotJoint* cpPivotJointInit(cpPivotJoint *joint, cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2)
  cpConstraint* cpPivotJointNew(cpBody *a, cpBody *b, cpVect pivot)
  cpConstraint* cpPivotJointNew2(cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2)
  cpPolyShape* cpPolyShapeAlloc(void)
  int cpPolyShapeGetNumVerts(cpShape *shape)
  cpVect cpPolyShapeGetVert(cpShape *shape, int idx)
  cpPolyShape* cpPolyShapeInit(cpPolyShape *poly, cpBody *body, int numVerts, const cpVect *verts, cpVect offset)
  cpShape* cpPolyShapeNew(cpBody *body, int numVerts, cpVect *verts, cpVect offset)
  cpBool cpPolyValidate(const cpVect *verts, const int numVerts)
  cpRatchetJoint* cpRatchetJointAlloc(void)
  const cpConstraintClass *cpRatchetJointGetClass(void)
  cpRatchetJoint* cpRatchetJointInit(cpRatchetJoint *joint, cpBody *a, cpBody *b, cpFloat phase, cpFloat ratchet)
  cpConstraint* cpRatchetJointNew(cpBody *a, cpBody *b, cpFloat phase, cpFloat ratchet)
  void cpRecenterPoly(const int numVerts, cpVect *verts)
  void cpResetShapeIdCounter(void)
  cpRotaryLimitJoint* cpRotaryLimitJointAlloc(void)
  const cpConstraintClass *cpRotaryLimitJointGetClass(void)
  cpRotaryLimitJoint* cpRotaryLimitJointInit(cpRotaryLimitJoint *joint, cpBody *a, cpBody *b, cpFloat min, cpFloat max)
  cpConstraint* cpRotaryLimitJointNew(cpBody *a, cpBody *b, cpFloat min, cpFloat max)
  cpSegmentShape* cpSegmentShapeAlloc(void)
  cpVect cpSegmentShapeGetA(const cpShape *shape)
  cpVect cpSegmentShapeGetB(const cpShape *shape)
  cpVect cpSegmentShapeGetNormal(const cpShape *shape)
  cpFloat cpSegmentShapeGetRadius(const cpShape *shape)
  cpSegmentShape* cpSegmentShapeInit(cpSegmentShape *seg, cpBody *body, cpVect a, cpVect b, cpFloat radius)
  cpShape* cpSegmentShapeNew(cpBody *body, cpVect a, cpVect b, cpFloat radius)
  void cpSegmentShapeSetNeighbors(cpShape *shape, cpVect prev, cpVect next)
  cpBB cpShapeCacheBB(cpShape *shape)
  void cpShapeDestroy(cpShape *shape)
  void cpShapeFree(cpShape *shape)
  cpFloat cpShapeNearestPointQuery(cpShape *shape, cpVect p, cpNearestPointQueryInfo *out)
  cpBool cpShapePointQuery(cpShape *shape, cpVect p)
  cpBool cpShapeSegmentQuery(cpShape *shape, cpVect a, cpVect b, cpSegmentQueryInfo *info)
  void cpShapeSetBody(cpShape *shape, cpBody *body)
  cpBB cpShapeUpdate(cpShape *shape, cpVect pos, cpVect rot)
  cpSimpleMotor* cpSimpleMotorAlloc(void)
  const cpConstraintClass *cpSimpleMotorGetClass(void)
  cpSimpleMotor* cpSimpleMotorInit(cpSimpleMotor *joint, cpBody *a, cpBody *b, cpFloat rate)
  cpConstraint* cpSimpleMotorNew(cpBody *a, cpBody *b, cpFloat rate)
  cpSlideJoint* cpSlideJointAlloc(void)
  const cpConstraintClass *cpSlideJointGetClass(void)
  cpSlideJoint* cpSlideJointInit(cpSlideJoint *joint, cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2, cpFloat min, cpFloat max)
  cpConstraint* cpSlideJointNew(cpBody *a, cpBody *b, cpVect anchr1, cpVect anchr2, cpFloat min, cpFloat max)
  void cpSpaceActivateShapesTouchingShape(cpSpace *space, cpShape *shape)
  cpBody* cpSpaceAddBody(cpSpace *space, cpBody *body)
  void cpSpaceAddCollisionHandler(
 cpSpace *space,
 cpCollisionType a, cpCollisionType b,
 cpCollisionBeginFunc begin,
 cpCollisionPreSolveFunc preSolve,
 cpCollisionPostSolveFunc postSolve,
 cpCollisionSeparateFunc separate,
 void *data
)
  cpConstraint* cpSpaceAddConstraint(cpSpace *space, cpConstraint *constraint)
  cpBool cpSpaceAddPostStepCallback(cpSpace *space, cpPostStepFunc func, void *key, void *data)
  cpShape* cpSpaceAddShape(cpSpace *space, cpShape *shape)
  cpShape* cpSpaceAddStaticShape(cpSpace *space, cpShape *shape)
  cpSpace* cpSpaceAlloc(void)
  void cpSpaceBBQuery(cpSpace *space, cpBB bb, cpLayers layers, cpGroup group, cpSpaceBBQueryFunc func, void *data)
  cpBool cpSpaceContainsBody(cpSpace *space, cpBody *body)
  cpBool cpSpaceContainsConstraint(cpSpace *space, cpConstraint *constraint)
  cpBool cpSpaceContainsShape(cpSpace *space, cpShape *shape)
  void cpSpaceConvertBodyToDynamic(cpSpace *space, cpBody *body, cpFloat mass, cpFloat moment)
  void cpSpaceConvertBodyToStatic(cpSpace *space, cpBody *body)
  void cpSpaceDestroy(cpSpace *space)
  void cpSpaceEachBody(cpSpace *space, cpSpaceBodyIteratorFunc func, void *data)
  void cpSpaceEachConstraint(cpSpace *space, cpSpaceConstraintIteratorFunc func, void *data)
  void cpSpaceEachShape(cpSpace *space, cpSpaceShapeIteratorFunc func, void *data)
  void cpSpaceFree(cpSpace *space)
  cpSpaceHash* cpSpaceHashAlloc(void)
  cpSpatialIndex* cpSpaceHashInit(cpSpaceHash *hash, cpFloat celldim, int numcells, cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex)
  cpSpatialIndex* cpSpaceHashNew(cpFloat celldim, int cells, cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex)
  void cpSpaceHashResize(cpSpaceHash *hash, cpFloat celldim, int numcells)
  cpSpace* cpSpaceInit(cpSpace *space)
  void cpSpaceNearestPointQuery(cpSpace *space, cpVect point, cpFloat maxDistance, cpLayers layers, cpGroup group, cpSpaceNearestPointQueryFunc func, void *data)
  cpShape *cpSpaceNearestPointQueryNearest(cpSpace *space, cpVect point, cpFloat maxDistance, cpLayers layers, cpGroup group, cpNearestPointQueryInfo *out)
  cpSpace* cpSpaceNew(void)
  void cpSpacePointQuery(cpSpace *space, cpVect point, cpLayers layers, cpGroup group, cpSpacePointQueryFunc func, void *data)
  cpShape *cpSpacePointQueryFirst(cpSpace *space, cpVect point, cpLayers layers, cpGroup group)
  void cpSpaceReindexShape(cpSpace *space, cpShape *shape)
  void cpSpaceReindexShapesForBody(cpSpace *space, cpBody *body)
  void cpSpaceReindexStatic(cpSpace *space)
  void cpSpaceRemoveBody(cpSpace *space, cpBody *body)
  void cpSpaceRemoveCollisionHandler(cpSpace *space, cpCollisionType a, cpCollisionType b)
  void cpSpaceRemoveConstraint(cpSpace *space, cpConstraint *constraint)
  void cpSpaceRemoveShape(cpSpace *space, cpShape *shape)
  void cpSpaceRemoveStaticShape(cpSpace *space, cpShape *shape)
  void cpSpaceSegmentQuery(cpSpace *space, cpVect start, cpVect end, cpLayers layers, cpGroup group, cpSpaceSegmentQueryFunc func, void *data)
  cpShape *cpSpaceSegmentQueryFirst(cpSpace *space, cpVect start, cpVect end, cpLayers layers, cpGroup group, cpSegmentQueryInfo *out)
  void cpSpaceSetDefaultCollisionHandler(
 cpSpace *space,
 cpCollisionBeginFunc begin,
 cpCollisionPreSolveFunc preSolve,
 cpCollisionPostSolveFunc postSolve,
 cpCollisionSeparateFunc separate,
 void *data
)
  cpBool cpSpaceShapeQuery(cpSpace *space, cpShape *shape, cpSpaceShapeQueryFunc func, void *data)
  void cpSpaceStep(cpSpace *space, cpFloat dt)
  void cpSpaceUseSpatialHash(cpSpace *space, cpFloat dim, int count)
  void cpSpatialIndexCollideStatic(cpSpatialIndex *dynamicIndex, cpSpatialIndex *staticIndex, cpSpatialIndexQueryFunc func, void *data)
  void cpSpatialIndexFree(cpSpatialIndex *index)
  cpSweep1D* cpSweep1DAlloc(void)
  cpSpatialIndex* cpSweep1DInit(cpSweep1D *sweep, cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex)
  cpSpatialIndex* cpSweep1DNew(cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex)
  cpVect cpvslerp(const cpVect v1, const cpVect v2, const cpFloat t)
  cpVect cpvslerpconst(const cpVect v1, const cpVect v2, const cpFloat a)
  char* cpvstr(const cpVect v)


=head1 SEE ALSO

Mention other useful documentation such as the documentation of
related modules or operating system documentation (such as man pages
in UNIX), or any relevant external documentation such as RFCs or
standards.

If you have a mailing list set up for your module, mention it here.

If you have a web site set up for your module, mention it here.

=head1 AUTHOR

tmurray <tmurray@wumpus-cave.net>

=head1 COPYRIGHT AND LICENSE

Copyright (c) 2015,  Timm Murray
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are 
permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of 
      conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of 
      conditions and the following disclaimer in the documentation and/or other materials 
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=cut
