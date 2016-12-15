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
