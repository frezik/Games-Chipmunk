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

cpFloat
cpGearJointGetRatio( constraint )
    cpConstraint * constraint
