GenericPID by Miles Caprio 2023

This folder is standalone besides for java depencies, and can be applied in other contexts.

These packages offer general basic PID functions, a 2d path class, pid tuning functions, and test
classes, most of which are customizable using polymorphic methods, and can also be extended for 
more functionality. They are good for use on any normal motors which need PID Control.

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



To Do:
Write this documentation
Read this documentation
Finish debugging insert segment and the little connectors
Finish wrappers
Make good
Auto Tuning