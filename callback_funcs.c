static void __perlCpBodyVelocityFunc(
    cpBody* body,
    cpVect gravity,
    cpFloat damping,
    cpFloat dt
);
static void
__perlCpBodyVelocityFunc(
    cpBody* body,
    cpVect gravity,
    cpFloat damping,
    cpFloat dt
) {
    dTHX;
    dSP;
    dMY_CXT;
    SV ** perl_func = hv_fetch(
        MY_CXT.bodyVelocityFuncs,
        (char*)&body,
        sizeof(body),
        FALSE
    );
    if( perl_func == (SV**) NULL ) {
        croak( "No cpBodyVelocityFunc found" );
    }

    PUSHMARK(SP);
    EXTEND( SP, 4 );
    PUSHs( sv_2mortal( sv_setref_pv( newSV(0), "cpBodyPtr", body ) ) );
    PUSHs( sv_2mortal( sv_setref_pv( newSV(0), "cpGravityPtr", &gravity ) ) );
    PUSHs( sv_2mortal( newSVnv( damping ) ) );
    PUSHs( sv_2mortal( newSVnv( dt ) ) );
    PUTBACK;

    call_sv( *perl_func, G_VOID );
}


static void __perlCpBodyPositionFunc(
    cpBody* body,
    cpFloat dt
);
static void
__perlCpBodyPositionFunc(
    cpBody* body,
    cpFloat dt
) {
    dTHX;
    dSP;
    dMY_CXT;
    SV ** perl_func = hv_fetch(
        MY_CXT.bodyPositionFuncs,
        (char*)&body,
        sizeof(body),
        FALSE
    );
    if( perl_func == (SV**) NULL ) {
        croak( "No cpBodyPositionFunc found" );
    }

    PUSHMARK(SP);
    EXTEND( SP, 2 );
    PUSHs( sv_2mortal( sv_setref_pv( newSV(0), "cpBodyPtr", body ) ) );
    PUSHs( sv_2mortal( newSVnv( dt ) ) );
    PUTBACK;

    call_sv( *perl_func, G_VOID );
}


static void __perlCpBodyShapeIteratorFunc(
    cpBody* body,
    cpShape* shape,
    void *data
);
static void
__perlCpBodyShapeIteratorFunc(
    cpBody* body,
    cpShape* shape,
    void *data
) {
    dTHX;
    dSP;
    dMY_CXT;
    SV ** perl_func = hv_fetch(
        MY_CXT.bodyEachShapeFuncs,
        (char*)&body,
        sizeof(body),
        FALSE
    );
    if( perl_func == (SV**) NULL ) {
        croak( "No cpBodyShapeIteratorFunc found" );
    }

    SV * sv_data = (SV*) data;

    PUSHMARK(SP);
    EXTEND( SP, 3 );
    PUSHs( sv_2mortal( sv_setref_pv( newSV(0), "cpBodyPtr", body ) ) );
    PUSHs( sv_2mortal( sv_setref_pv( newSV(0), "cpShapePtr", shape ) ) );
    PUSHs( sv_2mortal( sv_data ) );
    PUTBACK;

    call_sv( *perl_func, G_VOID );
}


static void __perlCpBodyConstraintIteratorFunc(
    cpBody* body,
    cpConstraint* constraint,
    void *data
);
static void
__perlCpBodyConstraintIteratorFunc(
    cpBody* body,
    cpConstraint* constraint,
    void *data
) {
    dTHX;
    dSP;
    dMY_CXT;
    SV ** perl_func = hv_fetch(
        MY_CXT.bodyEachConstraintFuncs,
        (char*)&body,
        sizeof(body),
        FALSE
    );
    if( perl_func == (SV**) NULL ) {
        croak( "No cpBodyConstraintIteratorFunc found" );
    }

    SV * sv_data = (SV*) data;

    PUSHMARK(SP);
    EXTEND( SP, 3 );
    PUSHs( sv_2mortal( sv_setref_pv( newSV(0), "cpBodyPtr", body ) ) );
    PUSHs( sv_2mortal( sv_setref_pv( newSV(0), "cpConstraintPtr",
        constraint ) ) );
    PUSHs( sv_2mortal( sv_data ) );
    PUTBACK;

    call_sv( *perl_func, G_VOID );
}


static void __perlCpBodyArbiterIteratorFunc(
    cpBody* body,
    cpArbiter* arbiter,
    void *data
);
static void
__perlCpBodyArbiterIteratorFunc(
    cpBody* body,
    cpArbiter* arbiter,
    void *data
) {
    dTHX;
    dSP;
    dMY_CXT;
    SV ** perl_func = hv_fetch(
        MY_CXT.bodyArbiterIteratorFuncs,
        (char*)&body,
        sizeof(body),
        FALSE
    );
    if( perl_func == (SV**) NULL ) {
        croak( "No cpBodyArbiterIteratorFunc found" );
    }

    SV * sv_data = (SV*) data;

    PUSHMARK(SP);
    EXTEND( SP, 3 );
    PUSHs( sv_2mortal( sv_setref_pv( newSV(0), "cpBodyPtr", body ) ) );
    PUSHs( sv_2mortal( sv_setref_pv( newSV(0), "cpArbiterPtr",
        arbiter ) ) );
    PUSHs( sv_2mortal( sv_data ) );
    PUTBACK;

    call_sv( *perl_func, G_VOID );
}
