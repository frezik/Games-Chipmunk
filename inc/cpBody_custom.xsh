void
cpBodySetVelocityUpdateFunc( body, velocityFunc )
    cpBody *body
    SV* velocityFunc
  PREINIT:
    dMY_CXT;
  CODE:
    hv_store(
        MY_CXT.bodyVelocityFuncs,
        (char*)&body,
        sizeof(body),
        velocityFunc,
        0
    );

    cpBodySetVelocityUpdateFunc(
        body,
        (cpBodyVelocityFunc) __perlCpBodyVelocityFunc
    );

void
cpBodySetPositionUpdateFunc( body, func )
    cpBody *body
    SV* func
  PREINIT:
    dMY_CXT;
  CODE:
    hv_store(
        MY_CXT.bodyPositionFuncs,
        (char*)&body,
        sizeof(body),
        func,
        0
    );

    cpBodySetPositionUpdateFunc(
        body,
        (cpBodyPositionFunc) __perlCpBodyPositionFunc
    );
