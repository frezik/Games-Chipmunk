cpDampedSpring *
cpDampedSpringAlloc()

cpConstraint *
cpDampedSpringNew(a, b, anchr1, anchr2, restLength, stiffness, damping)
	cpBody *	a
	cpBody *	b
	cpVect	anchr1
	cpVect	anchr2
	cpFloat	restLength
	cpFloat	stiffness
	cpFloat	damping

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
