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
cpSpaceSetGravity( space_ptr, gravity )
        cpSpace * space_ptr
        cpVect gravity

cpBody *
cpSpaceGetStaticBody(space)
    cpSpace * space

cpFloat
cpSpaceGetCurrentTimeStep( space )
    cpSpace * space

cpBool
cpSpaceIsLocked( space )
    cpSpace * space
