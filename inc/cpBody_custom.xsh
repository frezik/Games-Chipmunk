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
