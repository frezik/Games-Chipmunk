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
