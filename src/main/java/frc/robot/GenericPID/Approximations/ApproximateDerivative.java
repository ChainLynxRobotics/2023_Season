package frc.robot.GenericPID.Approximations;

import frc.robot.GenericPID.Debug;

public class ApproximateDerivative {
    private static final boolean debug = Debug.debug_Approximations;
    //a sometimes static class which takes the derivative of a certain point of a function
    //just use the derivative method, and a double function
    //or track ongoing derivative by using nextDerivative()
    //use either DoubleFunction unit or manual evaluation, either f or ylast
    private double xlast;
    private double ylast;
    private DoubleFunction f;
    //static instant version
    public static double derivative(DoubleFunction f, double dx, double x) {
        return (f.eval(x) - f.eval(x - dx)) / dx;
    }
    //static instant non fancy version
    public static double derivative(double x1, double y1, double x, double y) {
        if (debug)System.out.printf("derivative %f-%f/%f-%f\n", y, y1, x, x1);
        return (y - y1) / (x - x1); //hehe
    }
    public static double derivative(double y1, double y2, double dx) {
        if(debug)System.out.printf("derivative %f-%f/%f\n", y2, y1, dx);
        return (y2 - y1) / dx;
    }
    public void reset(DoubleFunction f, double x) {
        this.f = f;
        this.xlast = x;
    }
    public void reset(double x, double y) {
        this.xlast = x;
        this.ylast = y;
    }
    public double nextDerivative(double dx) {
        xlast += dx;
        return derivative(f, dx, xlast);
    }
    public double nextDerivative(double x, double fx) {
        double dx = x - xlast;
        double ret = derivative(ylast, fx, dx);
        xlast = x;
        ylast = fx;
        return ret;
    }
    public ApproximateDerivative(DoubleFunction f, double x) {
        this.reset(f, x);
        ylast = 0;
    }
    public ApproximateDerivative(double x, double y) {
        this.xlast = x;
        this.ylast = y;
    }
    public ApproximateDerivative() {
        this.xlast = 0;
        ylast = 0;
    }
}