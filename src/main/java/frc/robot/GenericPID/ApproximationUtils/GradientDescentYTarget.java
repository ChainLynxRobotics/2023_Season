package frc.robot.GenericPID.ApproximationUtils;

import frc.robot.GenericPID.Extensible.DoubleFunction;
import frc.robot.GenericPID.Implementations.Zero;

public class GradientDescentYTarget {
    public Double startIters = null;
    public Double checkIters = null;
    public Double cutOffIters = null; 
    public Double alpha = null;
    public Double acceptableError = null;
    public Double maxdx = null;
    public EndMethod endMethod = null;
    public ConvergeMethod convergeMethod = null;
    public DoubleFunction errorFunc = new Zero();
    public DerivativeMethod derivativeMethod = null;
    private double lastx;
    public static GradientDescentYTarget AlphaConverge(            DoubleFunction error, DoubleFunction derrordt, double cutOffIters,                   double acceptable,double alpha) {
        var ret = new GradientDescentYTarget();
        ret.cutOffIters = cutOffIters;
        ret.alpha = alpha;
        ret.errorFunc = error;
        ret.convergeMethod = ConvergeMethod.ALPHA;
        ret.endMethod = EndMethod.CUTOFF;
        ret.derivativeMethod = DerivativeMethod.ABSOLUTE;
        return ret;
    }
    public static GradientDescentYTarget ProjectionConverge(       DoubleFunction error, DoubleFunction derrordt, double cutOffIters, double maxdx,     double acceptable) {
        var ret = new GradientDescentYTarget();
        ret.cutOffIters = cutOffIters;
        ret.acceptableError = acceptable;
        ret.errorFunc = error;
        ret.convergeMethod = ConvergeMethod.PROJECT;
        ret.endMethod = EndMethod.CUTOFF;
        ret.derivativeMethod = DerivativeMethod.ABSOLUTE;
        return ret;
    }
    public static GradientDescentYTarget ProjectionAlphaConverge(  DoubleFunction error, DoubleFunction derrordt, double cutOffIters, double maxdx,     double acceptable,double alpha) {
        var ret = new GradientDescentYTarget();
        ret.cutOffIters = cutOffIters;
        ret.acceptableError = acceptable;
        ret.errorFunc = error;
        ret.alpha = alpha;
        ret.convergeMethod = ConvergeMethod.PROJECTALPHA;
        ret.endMethod = EndMethod.CUTOFF;
        ret.derivativeMethod = DerivativeMethod.ABSOLUTE;
        return ret;
    }
    public static GradientDescentYTarget AlphaIterations(          DoubleFunction error, DoubleFunction derrordt, double startIters, double checkIters,                   double alpha) {
        var ret = new GradientDescentYTarget();
        ret.startIters = startIters;
        ret.checkIters = checkIters;
        ret.alpha = alpha;
        ret.errorFunc = error;
        ret.convergeMethod = ConvergeMethod.ALPHA;
        ret.endMethod = EndMethod.DEADLINE;
        ret.derivativeMethod = DerivativeMethod.ABSOLUTE;
        return ret;
    }
    public static GradientDescentYTarget ProjectionIterations(     DoubleFunction error, DoubleFunction derrordt, double startIters, double checkIters) {
        var ret = new GradientDescentYTarget();
        ret.startIters = startIters;
        ret.checkIters = checkIters;
        ret.errorFunc = error;
        ret.convergeMethod = ConvergeMethod.PROJECT;
        ret.endMethod = EndMethod.DEADLINE;
        ret.derivativeMethod = DerivativeMethod.ABSOLUTE;
        return ret;
    } 
    public static GradientDescentYTarget ProjectionAlphaIterations(DoubleFunction error, DoubleFunction derrordt, double startIters, double checkIters,                 double alpha) {
        var ret = new GradientDescentYTarget();
        ret.startIters = startIters;
        ret.checkIters = checkIters;
        ret.errorFunc = error;
        ret.alpha = alpha;
        ret.convergeMethod = ConvergeMethod.PROJECTALPHA;
        ret.endMethod = EndMethod.DEADLINE;
        ret.derivativeMethod = DerivativeMethod.ABSOLUTE;
        return ret;
    }
    public Double descend() {
        //todo
        DoubleFunction next;
        if (convergeMethod == ConvergeMethod.PROJECT) {
            next = new DoubleFunction() {
                public double eval(double x) {
                    //todo
                    return 0;
                }
            };
        } else {
            next = new DoubleFunction () {
                public double eval(double x) {
                    //todo
                    return 0;
                }
            };
        }
        if (endMethod == EndMethod.DEADLINE) {
            for (int i = 0; i < startIters; i++) {
                //todo
            }
            for (int i = 0; i < checkIters; i++) {

            }
        }
        return 0;
    }
    public enum EndMethod {
        DEADLINE,
        CUTOFF,
    }
    public enum ConvergeMethod {
        ALPHA,
        PROJECT,
        PROJECTALPHA,
    }
    public enum DerivativeMethod {
        ABSOLUTE,
        ESTIMATE
    }
}