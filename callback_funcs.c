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
