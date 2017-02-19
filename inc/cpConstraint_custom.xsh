void
cpConstraintSetPreSolveFunc( constraint, preSolveFunc )
    cpConstraint *constraint
    cpConstraintPreSolveFunc preSolveFunc
  PREINIT:
    dMY_CXT;
  CODE:
    hv_store(
        MY_CXT.constraintPreSolveFuncs,
        (char*)&constraint,
        sizeof(constraint),
        preSolveFunc,
        0
    );

    cpConstraintSetPreSolveFunc(
        constraint,
        (cpConstraintPreSolveFunc) __perlCpConstraintPreSolveFunc
    );


void
cpConstraintSetPostSolveFunc( constraint, postSolveFunc )
    cpConstraint *constraint
    cpConstraintPostSolveFunc postSolveFunc
  PREINIT:
    dMY_CXT;
  CODE:
    hv_store(
        MY_CXT.constraintPostSolveFuncs,
        (char*)&constraint,
        sizeof(constraint),
        postSolveFunc,
        0
    );

    cpConstraintSetPostSolveFunc(
        constraint,
        (cpConstraintPostSolveFunc) __perlCpConstraintPostSolveFunc
    );
