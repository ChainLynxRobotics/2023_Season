GenericPID by Miles Caprio 2023

This folder is standalone besides for java depencies, and can be applied in other contexts.

These packages offer general basic PID functions, a 2d path class, pid tuning functions, and test
classes, which include custom control effect adapters, path following compensations, motor catch
up, tuning heuristics, automatic max acceleration connection, and many other things, so they can be
extended for more functionality. This is great for use on any motors which need PID Control.

----------------------------------------------------------------

Package class structure and descriptions:
[n] is dependency number, (x,y) are dependencies, (-w, -z) are main function testing dependencies








--GenericPID/
|
|--PIDConfig.java [0]
|--GenericPID.java [1] (0, 10, 11, 12, 13, -20, -21)
|--PathSegmentBase.java [2] (12)
|--LinearSegment.java [3] (2)
|--PolynomialSegment.java [4] (2)
|--GenericPath.java [5] (2, 12, -21)
|--Approximations/
||
||--ApproximateDerivative.java [10]
||>
||>A class which keeps track of a derivative of a function using a nonzero dt which is the distance
||>between two known points on that function. A smaller dt value creates more precision but may be
||>unnecessary and cause an excess of computation time.
||>The derivative can be calculated with two (x,y) points or with a DoubleFunction abstraction and
||>two x points. It can also be done with the static derivative method for single-times or by 
||>constructing the ApproximateDerivative with the correct information and calling the derivative 
||>method. It can be reset with the reset() function if necessary.
||>
||--ApproximateIntegral.java [11]
||>
||>A class which keeps track of an integral of a function using a nonzero dt which represents a left
||>riemann sum step. Same concept of dt values as ApproximateDerivative, except integrals may be 
||>more significant because the entire thing has to be done for needed information.
||>Again, an ApproximateIntegral can be tracked with a DoubleFunction abstraction (which is more
||>convenient) or individual fx values. There isn't a static method because an integral is 
||>accumulated.
||>
||--DoubleFunction.java [12]
||>
|>>An interface used in many classes in this folder which represents an algebraic function. It 
|>>contains one function eval(doublex) which must be implemented by each subclass, and then the
|>>subclass instances can be use anywhere a DoubleFunction is called for.
||>
||--Zero.java [13]
||>
||>An example subclass of DoubleFunction which always returns zero, for default fields.
||>
|--Testing/
|
|>
|>These classes are used for testing and viewing results of the others, and are less generic.
|>
|
||--ArtificialMotor.java [20]
||--Graph.java [21]
|
|--Tuning/
|
||--TunerBase.java [30]
||--IterativeIncreaseAndDampenTuner.java [31]
||--TwiddlerTuner.java [32]

----------------------------------------------------------------

Typical Use Case:

The structure of this library is both intuitive and overwhelming. It is written with flexibility, 
but it's resultingly very abstract.

Anyway, let's rundown how one could make use of this library to control a motor, following a path,
in real time...




Let's start in context of a class that is handling controls. There would likely be an init function
for the motor, and then one for the beginning of a movement, and then a repeated polling function
for this single motor movement. Let's start at the first init function, where it'd be likely we'd 
set up PID and motor "policies".

This library does not provide an direct extensions or places for your own motor abstraction to sit,
but rather outputs you can plug right into it. There are ArtificialMotor and Graph classes to test,
but we can test with the auto-tuner later. The first thing that would be important to set up would
be a PID configuration, the main purpose of this library's controller. Let's use a guess and tune
it later.

import GenericPID.PIDConfig;

PIDConfig pidConfig = new PIDConfig();
pidConfig.kP = 1.0;
pidConfig.kI = 0.01;
pidConfig.KD = 0.1;

This configuration is more like a constant, and will be passed to a real PID object later for real-
time control effects. 

Next, we should consider setting up some functions that we will use later,
because this init function can save them as final variables. Just to be clear, you don't have to 
remember this all at the beginning, and you can add more of these functions as you need them later
in the engineering and design process. But anyway, here we're going to put a catch-up function for
how to handle the motor getting behind, a control effect profile for how to handle the difference 
between the PID Control Effect and the Motor Output (which needs a control strategy function), and 
then a path-tracking PID modification function. However, we can use library implementations for
these if they aren't important; most of the constructors for the controllers use these default
implementations, so many of these aren't strictly necessary.


import GenericPID.ControlEffectProfile;
import GenericPID.ControlEffectProfile.ControlStrategy;
import GenericPID.PIDMotorFollow.CatchUpFunction;






































----------------------------------------------------------------

To Do:
Write this documentation
Read this documentation
Finish debugging insert segment and the little connectors
Finish wrappers
Make good
Auto Tuning