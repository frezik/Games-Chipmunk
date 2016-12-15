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
