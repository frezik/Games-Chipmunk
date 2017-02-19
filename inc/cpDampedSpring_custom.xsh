void
cpDampedSpringSetSpringForceFunc( constraint, springForceFunc )
    cpConstraint *constraint
    cpDampedSpringForceFunc springForceFunc
  PREINIT:
    dMY_CXT;
  CODE:
    hv_store(
        MY_CXT.dampedSpringForceFuncs,
        (char*)&constraint,
        sizeof(constraint),
        springForceFunc,
        0
    );

    cpDampedSpringSetSpringForceFunc(
        constraint,
        (cpDampedSpringForceFunc) __perlCpDampedSpringSetSpringForceFunc
    );

