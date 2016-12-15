cpContactPointSet
cpArbiterGetContactPointSet(arb)
	const cpArbiter *	arb

int
cpArbiterGetCount(arb)
	const cpArbiter *	arb

cpFloat
cpArbiterGetDepth(arb, i)
	const cpArbiter *	arb
	int	i

cpVect
cpArbiterGetNormal(arb)
	const cpArbiter *	arb

cpVect
cpArbiterGetSurfaceVelocity(arb)
	cpArbiter *	arb

cpBool
cpArbiterIgnore(arb)
	cpArbiter *	arb

cpBool
cpArbiterIsFirstContact(arb)
	const cpArbiter *	arb

void
cpArbiterSetContactPointSet(arb, set)
	cpArbiter *	arb
	cpContactPointSet *	set

void
cpArbiterSetSurfaceVelocity(arb, vr)
	cpArbiter *	arb
	cpVect	vr

cpVect
cpArbiterTotalImpulse(arb)
	const cpArbiter *	arb

cpFloat
cpArbiterTotalKE(arb)
	const cpArbiter *	arb

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
