#define PERL_NO_GET_CONTEXT
#include "EXTERN.h"
#include "perl.h"
#include "XSUB.h"

#include "ppport.h"

#include <chipmunk/chipmunk.h>

#include "const-c.inc"

MODULE = Games::Chipmunk		PACKAGE = Games::Chipmunk		

INCLUDE: const-xs.inc

INCLUDE: inc/cpArbiter.xsh

cpVect
_CPVZERO()
    CODE:
        RETVAL = cpvzero;
    OUTPUT:
        RETVAL

cpFloat
cpAreaForCircle(r1, r2)
	cpFloat	r1
	cpFloat	r2

cpFloat
cpAreaForPoly(numVerts, verts, radius)
	int	numVerts
	const cpVect *	verts
    cpFloat radius

cpFloat
cpAreaForSegment(a, b, r)
	cpVect	a
	cpVect	b
	cpFloat	r

cpBBTree *
cpBBTreeAlloc()

cpSpatialIndex *
cpBBTreeInit(tree, bbfunc, staticIndex)
	cpBBTree *	tree
	cpSpatialIndexBBFunc	bbfunc
	cpSpatialIndex *	staticIndex

cpSpatialIndex *
cpBBTreeNew(bbfunc, staticIndex)
	cpSpatialIndexBBFunc	bbfunc
	cpSpatialIndex *	staticIndex

void
cpBBTreeOptimize(index)
	cpSpatialIndex *	index

void
cpBBTreeSetVelocityFunc(index, func)
	cpSpatialIndex *	index
	cpBBTreeVelocityFunc	func

cpVect
cpBBWrapVect(bb, v)
	cpBB	bb
	cpVect	v

void
cpBodyActivate(body)
	cpBody *	body

void
cpBodyActivateStatic(body, filter)
	cpBody *	body
	cpShape *	filter

cpBody *
cpBodyAlloc()

void
cpBodyDestroy(body)
	cpBody *	body

void
cpBodyEachArbiter(body, func, data)
	cpBody *	body
	cpBodyArbiterIteratorFunc	func
	void *	data

void
cpBodyEachConstraint(body, func, data)
	cpBody *	body
	cpBodyConstraintIteratorFunc	func
	void *	data

void
cpBodyEachShape(body, func, data)
	cpBody *	body
	cpBodyShapeIteratorFunc	func
	void *	data

void
cpBodyFree(body)
	cpBody *	body

cpBody *
cpBodyInit(body, m, i)
	cpBody *	body
	cpFloat	m
	cpFloat	i

cpBody *
cpBodyNew(m, i)
	cpFloat	m
	cpFloat	i

cpBody *
cpBodyNewStatic()

void
cpBodySetAngle(body, a)
	cpBody *	body
	cpFloat	a

void
cpBodySetMass(body, m)
	cpBody *	body
	cpFloat	m

void
cpBodySetMoment(body, i)
	cpBody *	body
	cpFloat	i

void
cpBodySleep(body)
	cpBody *	body

void
cpBodySleepWithGroup(body, group)
	cpBody *	body
	cpBody *	group

void
cpBodyUpdatePosition(body, dt)
	cpBody *	body
	cpFloat	dt

void
cpBodyUpdateVelocity(body, gravity, damping, dt)
	cpBody *	body
	cpVect	gravity
	cpFloat	damping
	cpFloat	dt

cpPolyShape *
cpBoxShapeInit(poly, body, width, height, radius)
	cpPolyShape *	poly
	cpBody *	body
	cpFloat	width
	cpFloat	height
    cpFloat radius

cpPolyShape *
cpBoxShapeInit2(poly, body, box, radius)
	cpPolyShape *	poly
	cpBody *	body
	cpBB	box
    cpFloat radius

cpShape *
cpBoxShapeNew(body, width, height, radius)
	cpBody *	body
	cpFloat	width
	cpFloat	height
    cpFloat radius

cpShape *
cpBoxShapeNew2(body, box, radius)
	cpBody *	body
	cpBB	box
    cpFloat radius

cpVect
cpCentroidForPoly(numVerts, verts)
	int	numVerts
	const cpVect *	verts

cpCircleShape *
cpCircleShapeAlloc()

cpVect
cpCircleShapeGetOffset(shape)
	const cpShape *	shape

cpFloat
cpCircleShapeGetRadius(shape)
	const cpShape *	shape

cpCircleShape *
cpCircleShapeInit(circle, body, radius, offset)
	cpCircleShape *	circle
	cpBody *	body
	cpFloat	radius
	cpVect	offset

cpShape *
cpCircleShapeNew(body, radius, offset)
	cpBody *	body
	cpFloat	radius
	cpVect	offset

void
cpConstraintDestroy(constraint)
	cpConstraint *	constraint

void
cpConstraintFree(constraint)
	cpConstraint *	constraint

int
cpConvexHull(count, verts, result, first, tol)
	int	count
	cpVect *	verts
	cpVect *	result
	int *	first
	cpFloat	tol

cpDampedRotarySpring *
cpDampedRotarySpringAlloc()

cpDampedRotarySpring *
cpDampedRotarySpringInit(joint, a, b, restAngle, stiffness, damping)
	cpDampedRotarySpring *	joint
	cpBody *	a
	cpBody *	b
	cpFloat	restAngle
	cpFloat	stiffness
	cpFloat	damping

cpConstraint *
cpDampedRotarySpringNew(a, b, restAngle, stiffness, damping)
	cpBody *	a
	cpBody *	b
	cpFloat	restAngle
	cpFloat	stiffness
	cpFloat	damping

cpDampedSpring *
cpDampedSpringAlloc()

cpDampedSpring *
cpDampedSpringInit(joint, a, b, anchr1, anchr2, restLength, stiffness, damping)
	cpDampedSpring *	joint
	cpBody *	a
	cpBody *	b
	cpVect	anchr1
	cpVect	anchr2
	cpFloat	restLength
	cpFloat	stiffness
	cpFloat	damping

cpConstraint *
cpDampedSpringNew(a, b, anchr1, anchr2, restLength, stiffness, damping)
	cpBody *	a
	cpBody *	b
	cpVect	anchr1
	cpVect	anchr2
	cpFloat	restLength
	cpFloat	stiffness
	cpFloat	damping

cpGearJoint *
cpGearJointAlloc()

cpGearJoint *
cpGearJointInit(joint, a, b, phase, ratio)
	cpGearJoint *	joint
	cpBody *	a
	cpBody *	b
	cpFloat	phase
	cpFloat	ratio

cpConstraint *
cpGearJointNew(a, b, phase, ratio)
	cpBody *	a
	cpBody *	b
	cpFloat	phase
	cpFloat	ratio

void
cpGearJointSetRatio(constraint, value)
	cpConstraint *	constraint
	cpFloat	value

cpGrooveJoint *
cpGrooveJointAlloc()

cpGrooveJoint *
cpGrooveJointInit(joint, a, b, groove_a, groove_b, anchr2)
	cpGrooveJoint *	joint
	cpBody *	a
	cpBody *	b
	cpVect	groove_a
	cpVect	groove_b
	cpVect	anchr2

cpConstraint *
cpGrooveJointNew(a, b, groove_a, groove_b, anchr2)
	cpBody *	a
	cpBody *	b
	cpVect	groove_a
	cpVect	groove_b
	cpVect	anchr2

void
cpGrooveJointSetGrooveA(constraint, value)
	cpConstraint *	constraint
	cpVect	value

void
cpGrooveJointSetGrooveB(constraint, value)
	cpConstraint *	constraint
	cpVect	value

void
cpMessage(condition, file, line, isError, isHardError, message, ...)
	const char *	condition
	const char *	file
	int	line
	int	isError
	int	isHardError
	const char *	message

cpFloat
cpMomentForBox(m, width, height)
	cpFloat	m
	cpFloat	width
	cpFloat	height

cpFloat
cpMomentForBox2(m, box)
	cpFloat	m
	cpBB	box

cpFloat
cpMomentForCircle(m, r1, r2, offset)
	cpFloat	m
	cpFloat	r1
	cpFloat	r2
	cpVect	offset

cpFloat
cpMomentForPoly(m, count, verts, offset, radius)
	cpFloat	m
    int count
    cpVect * verts
    cpVect offset
    cpFloat radius

cpFloat
cpMomentForSegment(m, a, b, radius)
	cpFloat	m
    cpVect a
    cpVect b
    cpFloat radius

cpPinJoint *
cpPinJointAlloc()

cpPinJoint *
cpPinJointInit(joint, a, b, anchr1, anchr2)
	cpPinJoint *	joint
	cpBody *	a
	cpBody *	b
	cpVect	anchr1
	cpVect	anchr2

cpConstraint *
cpPinJointNew(a, b, anchr1, anchr2)
	cpBody *	a
	cpBody *	b
	cpVect	anchr1
	cpVect	anchr2

cpPivotJoint *
cpPivotJointAlloc()

cpPivotJoint *
cpPivotJointInit(joint, a, b, anchr1, anchr2)
	cpPivotJoint *	joint
	cpBody *	a
	cpBody *	b
	cpVect	anchr1
	cpVect	anchr2

cpConstraint *
cpPivotJointNew(a, b, pivot)
	cpBody *	a
	cpBody *	b
	cpVect	pivot

cpConstraint *
cpPivotJointNew2(a, b, anchr1, anchr2)
	cpBody *	a
	cpBody *	b
	cpVect	anchr1
	cpVect	anchr2

cpPolyShape *
cpPolyShapeAlloc()

cpVect
cpPolyShapeGetVert(shape, idx)
	cpShape *	shape
	int	idx

cpPolyShape *
cpPolyShapeInit(poly, body, count, verts, transform, radius)
	cpPolyShape *	poly
	cpBody *	body
	int	count
	const cpVect *	verts
    cpTransform transform
    cpFloat radius

cpShape *
cpPolyShapeNew(body, count, verts, transform, radius)
	cpBody *	body
	int	count
	cpVect *	verts
    cpTransform transform
    cpFloat radius

cpRatchetJoint *
cpRatchetJointAlloc()

cpRatchetJoint *
cpRatchetJointInit(joint, a, b, phase, ratchet)
	cpRatchetJoint *	joint
	cpBody *	a
	cpBody *	b
	cpFloat	phase
	cpFloat	ratchet

cpConstraint *
cpRatchetJointNew(a, b, phase, ratchet)
	cpBody *	a
	cpBody *	b
	cpFloat	phase
	cpFloat	ratchet

cpRotaryLimitJoint *
cpRotaryLimitJointAlloc()

cpRotaryLimitJoint *
cpRotaryLimitJointInit(joint, a, b, min, max)
	cpRotaryLimitJoint *	joint
	cpBody *	a
	cpBody *	b
	cpFloat	min
	cpFloat	max

cpConstraint *
cpRotaryLimitJointNew(a, b, min, max)
	cpBody *	a
	cpBody *	b
	cpFloat	min
	cpFloat	max

cpSegmentShape *
cpSegmentShapeAlloc()

cpVect
cpSegmentShapeGetA(shape)
	const cpShape *	shape

cpVect
cpSegmentShapeGetB(shape)
	const cpShape *	shape

cpVect
cpSegmentShapeGetNormal(shape)
	const cpShape *	shape

cpFloat
cpSegmentShapeGetRadius(shape)
	const cpShape *	shape

cpSegmentShape *
cpSegmentShapeInit(seg, body, a, b, radius)
	cpSegmentShape *	seg
	cpBody *	body
	cpVect	a
	cpVect	b
	cpFloat	radius

cpShape *
cpSegmentShapeNew(body, a, b, radius)
	cpBody *	body
	cpVect	a
	cpVect	b
	cpFloat	radius

void
cpSegmentShapeSetNeighbors(shape, prev, next)
	cpShape *	shape
	cpVect	prev
	cpVect	next

cpBB
cpShapeCacheBB(shape)
	cpShape *	shape

void
cpShapeDestroy(shape)
	cpShape *	shape

void
cpShapeFree(shape)
	cpShape *	shape

cpFloat
cpShapePointQuery(shape, p, out)
	cpShape *	shape
    cpVect      p
    cpPointQueryInfo * out

cpBool
cpShapeSegmentQuery(shape, a, b, radius, info)
	cpShape *	shape
	cpVect	a
	cpVect	b
    cpFloat radius
	cpSegmentQueryInfo *	info

void
cpShapeSetBody(shape, body)
	cpShape *	shape
	cpBody *	body

cpBB
cpShapeUpdate(shape, transform)
	cpShape *	shape
    cpTransform transform

cpSimpleMotor *
cpSimpleMotorAlloc()

cpSimpleMotor *
cpSimpleMotorInit(joint, a, b, rate)
	cpSimpleMotor *	joint
	cpBody *	a
	cpBody *	b
	cpFloat	rate

cpConstraint *
cpSimpleMotorNew(a, b, rate)
	cpBody *	a
	cpBody *	b
	cpFloat	rate

cpSlideJoint *
cpSlideJointAlloc()

cpSlideJoint *
cpSlideJointInit(joint, a, b, anchr1, anchr2, min, max)
	cpSlideJoint *	joint
	cpBody *	a
	cpBody *	b
	cpVect	anchr1
	cpVect	anchr2
	cpFloat	min
	cpFloat	max

cpConstraint *
cpSlideJointNew(a, b, anchr1, anchr2, min, max)
	cpBody *	a
	cpBody *	b
	cpVect	anchr1
	cpVect	anchr2
	cpFloat	min
	cpFloat	max

cpBody *
cpSpaceAddBody(space, body)
	cpSpace *	space
	cpBody *	body

cpCollisionHandler*
cpSpaceAddCollisionHandler(space, a, b)
	cpSpace *	space
	cpCollisionType	a
	cpCollisionType	b

cpConstraint *
cpSpaceAddConstraint(space, constraint)
	cpSpace *	space
	cpConstraint *	constraint

cpBool
cpSpaceAddPostStepCallback(space, func, key, data)
	cpSpace *	space
	cpPostStepFunc	func
	void *	key
	void *	data

cpShape *
cpSpaceAddShape(space, shape)
	cpSpace *	space
	cpShape *	shape

cpSpace *
cpSpaceAlloc()

void
cpSpaceBBQuery(space, bb, filter, func, data)
	cpSpace *	space
	cpBB	bb
    cpShapeFilter filter
	cpSpaceBBQueryFunc	func
	void *	data

cpBool
cpSpaceContainsBody(space, body)
	cpSpace *	space
	cpBody *	body

cpBool
cpSpaceContainsConstraint(space, constraint)
	cpSpace *	space
	cpConstraint *	constraint

cpBool
cpSpaceContainsShape(space, shape)
	cpSpace *	space
	cpShape *	shape

void
cpSpaceDestroy(space)
	cpSpace *	space

void
cpSpaceEachBody(space, func, data)
	cpSpace *	space
	cpSpaceBodyIteratorFunc	func
	void *	data

void
cpSpaceEachConstraint(space, func, data)
	cpSpace *	space
	cpSpaceConstraintIteratorFunc	func
	void *	data

void
cpSpaceEachShape(space, func, data)
	cpSpace *	space
	cpSpaceShapeIteratorFunc	func
	void *	data

void
cpSpaceFree(space)
	cpSpace *	space

cpSpaceHash *
cpSpaceHashAlloc()

cpSpatialIndex *
cpSpaceHashInit(hash, celldim, numcells, bbfunc, staticIndex)
	cpSpaceHash *	hash
	cpFloat	celldim
	int	numcells
	cpSpatialIndexBBFunc	bbfunc
	cpSpatialIndex *	staticIndex

cpSpatialIndex *
cpSpaceHashNew(celldim, cells, bbfunc, staticIndex)
	cpFloat	celldim
	int	cells
	cpSpatialIndexBBFunc	bbfunc
	cpSpatialIndex *	staticIndex

void
cpSpaceHashResize(hash, celldim, numcells)
	cpSpaceHash *	hash
	cpFloat	celldim
	int	numcells

cpSpace *
cpSpaceInit(space)
	cpSpace *	space

cpSpace *
cpSpaceNew()

void
cpSpacePointQuery(space, point, maxDistance, filter, func, data)
	cpSpace *	space
	cpVect	point
    cpFloat maxDistance
    cpShapeFilter filter
	cpSpacePointQueryFunc	func
	void *	data

void
cpSpaceReindexShape(space, shape)
	cpSpace *	space
	cpShape *	shape

void
cpSpaceReindexShapesForBody(space, body)
	cpSpace *	space
	cpBody *	body

void
cpSpaceReindexStatic(space)
	cpSpace *	space

void
cpSpaceRemoveBody(space, body)
	cpSpace *	space
	cpBody *	body

void
cpSpaceRemoveConstraint(space, constraint)
	cpSpace *	space
	cpConstraint *	constraint

void
cpSpaceRemoveShape(space, shape)
	cpSpace *	space
	cpShape *	shape

void
cpSpaceSegmentQuery(space, start, end, radius, filter, func, data)
	cpSpace *	space
	cpVect	start
	cpVect	end
    cpFloat radius
    cpShapeFilter filter
	cpSpaceSegmentQueryFunc	func
	void *	data

cpShape *
cpSpaceSegmentQueryFirst(space, start, end, radius, filter, out)
	cpSpace *	space
	cpVect	start
	cpVect	end
    cpFloat radius
    cpShapeFilter filter
    cpSegmentQueryInfo * out

cpBool
cpSpaceShapeQuery(space, shape, func, data)
	cpSpace *	space
	cpShape *	shape
	cpSpaceShapeQueryFunc	func
	void *	data

void
cpSpaceStep(space, dt)
	cpSpace *	space
	cpFloat	dt

void
cpSpaceUseSpatialHash(space, dim, count)
	cpSpace *	space
	cpFloat	dim
	int	count

void
cpSpatialIndexFree(index)
	cpSpatialIndex *	index

cpSweep1D *
cpSweep1DAlloc()

cpSpatialIndex *
cpSweep1DInit(sweep, bbfunc, staticIndex)
	cpSweep1D *	sweep
	cpSpatialIndexBBFunc	bbfunc
	cpSpatialIndex *	staticIndex

cpSpatialIndex *
cpSweep1DNew(bbfunc, staticIndex)
	cpSpatialIndexBBFunc	bbfunc
	cpSpatialIndex *	staticIndex

cpVect
cpvslerp(v1, v2, t)
	cpVect	v1
	cpVect	v2
	cpFloat	t

cpVect
cpvslerpconst(v1, v2, a)
	cpVect	v1
	cpVect	v2
	cpFloat	a

cpVect
cpv( x, y )
    const cpFloat x
    const cpFloat y

cpBool
cpveql(v1, v2)
    cpVect v1
    cpVect v2

cpVect
cpvadd(v1, v2)
    cpVect v1
    cpVect v2

void
cpSpaceSetGravity( space_ptr, gravity )
        cpSpace * space_ptr
        cpVect gravity

cpBody *
cpSpaceGetStaticBody(space)
    cpSpace * space

void
cpShapeSetFriction(shape, value)
    cpShape * shape
    cpFloat value

cpVect
cpBodyGetPosition( body )
    cpBody * body

void
cpBodySetPosition( body, pos )
    cpBody * body
    cpVect pos

cpVect
cpBodyGetVelocity( body )
    cpBody * body

cpVect
cpvneg( v )
    cpVect v

cpVect
cpvsub( v1, v2 )
    cpVect v1
    cpVect v2

cpVect
cpvmult( v, s )
    cpVect v
    cpFloat s

cpFloat
cpvdot( v1, v2 )
    cpVect v1
    cpVect v2

cpFloat
cpvcross( v1, v2 )
    cpVect v1
    cpVect v2

cpVect
cpvperp( v )
    cpVect v

cpVect
cpvrperp( v )
    cpVect v

cpVect
cpvproject( v1, v2 )
    cpVect v1
    cpVect v2

cpVect
cpvforangle( a )
    cpFloat a

cpFloat
cpvtoangle( v )
    cpVect v

cpVect
cpvrotate( v1, v2 )
    cpVect v1
    cpVect v2

cpVect
cpvunrotate( v1, v2 )
    cpVect v1
    cpVect v2

cpFloat
cpvlengthsq( v )
    cpVect v

cpFloat
cpvlength( v )
    cpVect v

cpVect
cpvlerp( v1, v2, t )
    cpVect v1
    cpVect v2
    cpFloat t

cpVect
cpvnormalize( v )
    cpVect v

cpVect
cpvclamp( v, len )
    cpVect v
    cpFloat len

cpVect
cpvlerpconst( v1, v2, d )
    cpVect v1
    cpVect v2
    cpFloat d

cpFloat
cpvdist( v1, v2 )
    cpVect v1
    cpVect v2

cpFloat
cpvdistsq( v1, v2 )
    cpVect v1
    cpVect v2

cpBool
cpvnear( v1, v2, dist )
    cpVect v1
    cpVect v2
    cpFloat dist

cpFloat
cpfmax( a, b )
    cpFloat a
    cpFloat b

cpFloat
cpfmin( a, b )
    cpFloat a
    cpFloat b

cpFloat
cpfabs( f )
    cpFloat f

cpFloat
cpfclamp( f, min, max )
    cpFloat f
    cpFloat min
    cpFloat max

cpFloat
cpflerp( f1, f2, t )
    cpFloat f1
    cpFloat f2
    cpFloat t

cpFloat
cpflerpconst( f1, f2, d )
    cpFloat f1
    cpFloat f2
    cpFloat d

cpBB
cpBBNew( l, b, r, t )
    cpFloat l
    cpFloat b
    cpFloat r
    cpFloat t

cpBB
cpBBNewForCircle( p, r )
    cpVect p
    cpFloat r

cpBool
cpBBIntersects( a, b )
    cpBB a
    cpBB b

cpBool
cpBBContainsBB( bb, other )
    cpBB bb
    cpBB other 

cpBool
cpBBContainsVect( bb, v )
    cpBB bb
    cpVect v

cpBB
cpBBMerge( a, b )
    cpBB a
    cpBB b

cpBB
cpBBExpand( bb, v )
    cpBB bb
    cpVect v

cpFloat
cpBBArea( bb )
    cpBB bb

cpFloat
cpBBMergedArea( a, b )
    cpBB a
    cpBB b

cpFloat
cpBBSegmentQuery( bb, a, b )
    cpBB bb
    cpVect a
    cpVect b

cpBool
cpBBIntersectsSegment( bb, a, b )
    cpBB bb
    cpVect a
    cpVect b

cpVect
cpBBClampVect( bb, v )
    cpBB bb
    cpVect v

cpFloat
cpBodyGetMass( body )
    cpBody * body

cpFloat
cpBodyGetMoment( body )
    cpBody * body

cpFloat
cpBodyGetAngle( body )
    cpBody * body

cpVect
cpBodyGetRotation( body )
    cpBody * body

cpBool
cpBodyIsSleeping( body )
    cpBody * body

cpFloat
cpBodyKineticEnergy( body )
    cpBody * body

cpBB
cpShapeGetBB( shape )
    cpShape * shape

void
cpArbiterGetShapes( arb, a, b )
    cpArbiter * arb
    cpShape ** a
    cpShape ** b

void
cpArbiterGetBodies( arb, a, b )
    cpArbiter * arb
    cpBody **a
    cpBody **b

cpBody *
cpConstraintGetBodyA( constraint )
    cpConstraint * constraint

cpBody *
cpConstraintGetBodyB( constraint )
    cpConstraint * constraint

cpFloat
cpConstraintGetImpulse( constraint )
    cpConstraint * constraint

cpFloat
cpGearJointGetRatio( constraint )
    cpConstraint * constraint

void
cpSpatialIndexDestroy( index )
    cpSpatialIndex * index 

int
cpSpatialIndexCount( index )
    cpSpatialIndex * index

cpBool
cpSpatialIndexContains( index, obj, hashid )
    cpSpatialIndex * index
    void * obj
    cpHashValue hashid

void
cpSpatialIndexInsert( index, obj, hashid )
    cpSpatialIndex * index
    void * obj
    cpHashValue hashid

void
cpSpatialIndexRemove( index, obj, hashid )
    cpSpatialIndex * index
    void * obj
    cpHashValue hashid

void
cpSpatialIndexReindex( index )
    cpSpatialIndex * index

void
cpSpatialIndexReindexObject( index, obj, hashid )
    cpSpatialIndex * index
    void * obj
    cpHashValue hashid

cpFloat
cpSpaceGetCurrentTimeStep( space )
    cpSpace * space

cpBool
cpSpaceIsLocked( space )
    cpSpace * space


MODULE = Games::Chipmunk		PACKAGE = Games::Chipmunk::cpVect

cpFloat
x( vect )
        cpVect vect
    CODE:
        RETVAL = vect.x;
    OUTPUT:
        RETVAL

cpFloat
y( vect )
        cpVect vect
    CODE:
        RETVAL = vect.y;
    OUTPUT:
        RETVAL
