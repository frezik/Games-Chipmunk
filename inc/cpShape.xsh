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
