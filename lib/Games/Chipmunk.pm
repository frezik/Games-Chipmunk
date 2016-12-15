package Games::Chipmunk;

use 5.008000;
use strict;
use warnings;
use Carp;

require Exporter;
use AutoLoader;

our @ISA = qw(Exporter);

our @EXPORT =  qw(
	CP_ALLOW_PRIVATE_ACCESS
	CP_BUFFER_BYTES
	CP_VERSION_MAJOR
	CP_VERSION_MINOR
	CP_VERSION_RELEASE
	cpcalloc
	cpfree
	cprealloc
	cpArbiterGetContactPointSet
	cpArbiterGetCount
	cpArbiterGetDepth
	cpArbiterGetNormal
	cpArbiterGetPoint
	cpArbiterGetSurfaceVelocity
	cpArbiterIgnore
	cpArbiterIsFirstContact
	cpArbiterSetContactPointSet
	cpArbiterSetSurfaceVelocity
	cpArbiterTotalImpulse
	cpArbiterTotalImpulseWithFriction
	cpArbiterTotalKE
	cpAreaForCircle
	cpAreaForPoly
	cpAreaForSegment
	cpBBTreeAlloc
	cpBBTreeInit
	cpBBTreeNew
	cpBBTreeOptimize
	cpBBTreeSetVelocityFunc
	cpBBWrapVect
	cpBodyActivate
	cpBodyActivateStatic
	cpBodyAlloc
	cpBodyApplyForce
	cpBodyApplyImpulse
	cpBodyDestroy
	cpBodyEachArbiter
	cpBodyEachConstraint
	cpBodyEachShape
	cpBodyFree
    cpBodySetPosition
	cpBodyGetVelAtLocalPoint
	cpBodyGetVelAtWorldPoint
	cpBodyInit
	cpBodyInitStatic
	cpBodyNew
	cpBodyNewStatic
	cpBodyResetForces
	cpBodySanityCheck
	cpBodySetAngle
	cpBodySetMass
	cpBodySetMoment
	cpBodySleep
	cpBodySleepWithGroup
	cpBodyUpdatePosition
	cpBodyUpdateVelocity
	cpBoxShapeInit
	cpBoxShapeInit2
	cpBoxShapeNew
	cpBoxShapeNew2
	cpCentroidForPoly
	cpCircleShapeAlloc
	cpCircleShapeGetOffset
	cpCircleShapeGetRadius
	cpCircleShapeInit
	cpCircleShapeNew
	cpConstraintDestroy
	cpConstraintFree
	cpConvexHull
	cpDampedRotarySpringAlloc
	cpDampedRotarySpringGetClass
	cpDampedRotarySpringInit
	cpDampedRotarySpringNew
	cpDampedSpringAlloc
	cpDampedSpringGetClass
	cpDampedSpringInit
	cpDampedSpringNew
	cpEnableSegmentToSegmentCollisions
	cpGearJointAlloc
	cpGearJointGetClass
	cpGearJointInit
	cpGearJointNew
	cpGearJointSetRatio
	cpGrooveJointAlloc
	cpGrooveJointGetClass
	cpGrooveJointInit
	cpGrooveJointNew
	cpGrooveJointSetGrooveA
	cpGrooveJointSetGrooveB
	cpInitChipmunk
	cpMessage
	cpMomentForBox
	cpMomentForBox2
	cpMomentForCircle
	cpMomentForPoly
	cpMomentForSegment
	cpPinJointAlloc
	cpPinJointGetClass
	cpPinJointInit
	cpPinJointNew
	cpPivotJointAlloc
	cpPivotJointGetClass
	cpPivotJointInit
	cpPivotJointNew
	cpPivotJointNew2
	cpPolyShapeAlloc
	cpPolyShapeGetNumVerts
	cpPolyShapeGetVert
	cpPolyShapeInit
	cpPolyShapeNew
	cpPolyValidate
	cpRatchetJointAlloc
	cpRatchetJointGetClass
	cpRatchetJointInit
	cpRatchetJointNew
	cpRecenterPoly
	cpResetShapeIdCounter
	cpRotaryLimitJointAlloc
	cpRotaryLimitJointGetClass
	cpRotaryLimitJointInit
	cpRotaryLimitJointNew
	cpSegmentShapeAlloc
	cpSegmentShapeGetA
	cpSegmentShapeGetB
	cpSegmentShapeGetNormal
	cpSegmentShapeGetRadius
	cpSegmentShapeInit
	cpSegmentShapeNew
	cpSegmentShapeSetNeighbors
	cpShapeCacheBB
	cpShapeDestroy
	cpShapeFree
	cpShapeNearestPointQuery
	cpShapePointQuery
	cpShapeSegmentQuery
	cpShapeSetBody
	cpShapeUpdate
	cpSimpleMotorAlloc
	cpSimpleMotorGetClass
	cpSimpleMotorInit
	cpSimpleMotorNew
	cpSlideJointAlloc
	cpSlideJointGetClass
	cpSlideJointInit
	cpSlideJointNew
	cpSpaceActivateShapesTouchingShape
	cpSpaceAddBody
	cpSpaceAddCollisionHandler
	cpSpaceAddConstraint
	cpSpaceAddPostStepCallback
	cpSpaceAddShape
	cpSpaceAddStaticShape
	cpSpaceAlloc
	cpSpaceBBQuery
	cpSpaceContainsBody
	cpSpaceContainsConstraint
	cpSpaceContainsShape
	cpSpaceConvertBodyToDynamic
	cpSpaceConvertBodyToStatic
	cpSpaceDestroy
	cpSpaceEachBody
	cpSpaceEachConstraint
	cpSpaceEachShape
	cpSpaceFree
	cpSpaceHashAlloc
	cpSpaceHashInit
	cpSpaceHashNew
	cpSpaceHashResize
	cpSpaceInit
	cpSpaceNearestPointQuery
	cpSpaceNearestPointQueryNearest
	cpSpaceNew
	cpSpacePointQuery
	cpSpacePointQueryFirst
	cpSpaceReindexShape
	cpSpaceReindexShapesForBody
	cpSpaceReindexStatic
	cpSpaceRemoveBody
	cpSpaceRemoveCollisionHandler
	cpSpaceRemoveConstraint
	cpSpaceRemoveShape
	cpSpaceRemoveStaticShape
	cpSpaceSegmentQuery
	cpSpaceSegmentQueryFirst
	cpSpaceSetDefaultCollisionHandler
	cpSpaceShapeQuery
	cpSpaceStep
	cpSpaceUseSpatialHash
	cpSpatialIndexCollideStatic
	cpSpatialIndexFree
	cpSweep1DAlloc
	cpSweep1DInit
	cpSweep1DNew
	cpvslerp
	cpvslerpconst
	cpvstr
    $CPV_ZERO

    cpv
    cpveql
    cpvadd
    cpvneg
    cpvsub
    cpvmult
    cpvdot
    cpvcross
    cpvperp
    cpvrperp
    cpvproject
    cpvforangle
    cpvtoangle
    cpvrotate
    cpvunrotate
    cpvlengthsq
    cpvlength
    cpvlerp
    cpvnormalize
    cpvnormalize_safe
    cpvclamp
    cpvlerpconst
    cpvdist
    cpvdistsq
    cpvnear
    cpfmax
    cpfmin
    cpfabs
    cpfclamp
    cpflerp
    cpflerpconst
    cpBBNew
    cpBBNewForCircle
    cpBBIntersects
    cpBBContainsBB
    cpBBContainsVect
    cpBBMerge
    cpBBExpand
    cpBBArea
    cpBBMergedArea
    cpBBSegmentQuery
    cpBBIntersectsSegment
    cpBBClampVect
    cpBodyGetMass
    cpBodyGetMoment
    cpBodyGetPosition
    cpBodyGetAngle
    cpBodyGetRot
    cpBodyIsSleeping
    cpBodyIsStatic
    cpBodyIsRogue
    cpBodyLocal2World
    cpBodyWorld2Local
    cpBodyKineticEnergy
    cpShapeGetBB
    cpArbiterGetShapes
    cpArbiterGetBodies
    cpArbiterIsFirstContact
    cpArbiterGetCount
    cpConstraintGetA
    cpConstraintGetB
    cpConstraintGetImpulse
    cpGearJointGetRatio
    cpGrooveJointGetGrooveA
    cpGrooveJointGetGrooveB
    cpSegmentQueryHitPoint
    cpSegmentQueryHitDist
    cpSpaceSetGravity
    cpSpatialIndexDestroy
    cpSpatialIndexCount
    cpSpatialIndexEach
    cpSpatialIndexContains
    cpSpatialIndexInsert
    cpSpatialIndexRemove
    cpSpatialIndexReindex
    cpSpatialIndexReindexObject
    cpSpatialIndexSegmentQuery
    cpSpatialIndexQuery
    cpSpatialIndexReindexQuery
    cpSpaceGetStaticBody
    cpSpaceGetCurrentTimeStep
    cpSpaceIsLocked
    cpShapeSetFriction
    cpBodyGetPos
    cpBodyGetVelocity
);
our @EXPORT_OK = @EXPORT;


our $VERSION = '0.01';

sub AUTOLOAD {
    # This AUTOLOAD is used to 'autoload' constants from the constant()
    # XS function.

    my $constname;
    our $AUTOLOAD;
    ($constname = $AUTOLOAD) =~ s/.*:://;
    croak "&Games::Chipmunk::constant not defined" if $constname eq 'constant';
    my ($error, $val) = constant($constname);
    if ($error) { croak $error; }
    {
	no strict 'refs';
	# Fixed between 5.005_53 and 5.005_61
#XXX	if ($] >= 5.00561) {
#XXX	    *$AUTOLOAD = sub () { $val };
#XXX	}
#XXX	else {
	    *$AUTOLOAD = sub { $val };
#XXX	}
    }
    goto &$AUTOLOAD;
}

require XSLoader;
XSLoader::load('Games::Chipmunk', $VERSION);

our $CPV_ZERO = _CPVZERO();



1;
__END__
# Below is stub documentation for your module. You'd better edit it!

=head1 NAME

Games::Chipmunk - Perl API for the Chipmunk 2D physics library

=head1 SYNOPSIS

    use Games::Chipmunk;
    use strict;
    use warnings;

    # cpVect is a 2D vector and cpv() is a shortcut for initializing them.
    my $gravity = cpv(0, -100);

    # Create an empty space.
    my $space = cpSpaceNew();
    cpSpaceSetGravity($space, $gravity);

    # Add a static line segment shape for the ground.
    # We'll make it slightly tilted so the ball will roll off.
    # We attach it to space->staticBody to tell Chipmunk it shouldn't be movable.
    my $ground = cpSegmentShapeNew(
        cpSpaceGetStaticBody( $space ), cpv(-20, 5), cpv(20, -5), 0 );
    cpShapeSetFriction($ground, 1);
    cpSpaceAddShape($space, $ground);

    # Now let's make a ball that falls onto the line and rolls off.
    # First we need to make a cpBody to hold the physical properties of the object.
    # These include the mass, position, velocity, angle, etc. of the object.
    # Then we attach collision shapes to the cpBody to give it a size and shape.

    my $radius = 5;
    my $mass = 1;

    # The moment of inertia is like mass for rotation
    # Use the cpMomentFor*() functions to help you approximate it.
    my $moment = cpMomentForCircle($mass, 0, $radius, $CPV_ZERO);

    # The cpSpaceAdd*() functions return the thing that you are adding.
    # It's convenient to create and add an object in one line.
    my $ballBody = cpSpaceAddBody($space, cpBodyNew($mass, $moment));
    cpBodySetPos($ballBody, cpv(0, 15));

    # Now we create the collision shape for the ball.
    # You can create multiple collision shapes that point to the same body.
    # They will all be attached to the body and move around to follow it.
    my $ballShape = cpSpaceAddShape($space, cpCircleShapeNew($ballBody, $radius, $CPV_ZERO));
    cpShapeSetFriction($ballShape, 0.7);

    # Now that it's all set up, we simulate all the objects in the space by
    # stepping forward through time in small increments called steps.
    # It is *highly* recommended to use a fixed size time step.
    my $timeStep = 1.0/60.0;

    # For our tests, we want to check that there was some kind of movement.
    # Problem is, there might not be enough acceleration at the start to actually 
    # move anything.  We'll just do a few runs to prime the system.
    cpSpaceStep($space, $timeStep) for 1 .. 5;
    for(my $time = $timeStep * 5; $time < 2; $time += $timeStep){
        my $pos = cpBodyGetPos($ballBody);
        my $vel = cpBodyGetVel($ballBody);
        printf(
            'Time is %5.2f. ballBody is at (%5.2f, %5.2f). Its velocity is (%5.2f, %5.2f)' . "\n",
            $time, $pos->x, $pos->y, $vel->x, $vel->y
        );

        cpSpaceStep($space, $timeStep);
    }

    # Clean up our objects and exit!
    cpShapeFree($ballShape);
    cpBodyFree($ballBody);
    cpShapeFree($ground);
    cpSpaceFree($space);

=head1 DESCRIPTION

Chipmunk 2D is a physics library that supports fast, lightweight rigid body 
physics for games or other simulations.

This Perl module is a pretty straight implementation of the C library, so 
the Chipmunk API docs should give you most of what you need. The complete API 
is exported when you C<use> the module.

A few cavets:

=over 4

=item * The cpvzero global is accessible as C<$CPV_ZERO>

=item * Anything that requires a callback function is not yet implemented

=item * The API is based on Chipmunk 6, because that's what Ubuntu currently installs from its package manager. There are incompatible changes in Chipmunk 7.

=back

=head1 TODO

Write the callback function mapping

Convert to Dist::Zilla

=head1 SEE ALSO

Chipmunk 2D Website: L<http://chipmunk-physics.net>

L<Alien::Chipmunk>

=head1 AUTHOR

Timm Murray <tmurray@wumpus-cave.net>

=head1 LICENSE

Copyright (c) 2016,  Timm Murray
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are 
permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of 
      conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of 
      conditions and the following disclaimer in the documentation and/or other materials 
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=cut
