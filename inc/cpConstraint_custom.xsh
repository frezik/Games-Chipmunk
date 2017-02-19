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


