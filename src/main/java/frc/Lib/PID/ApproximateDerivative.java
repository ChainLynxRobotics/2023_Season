package frc.Lib.PID;

public class ApproximateDerivative { 
    private double xlast;
    private double ylast;
    private DoubleFunction f;
    

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

    //static instant version
    public static double derivative(DoubleFunction f, double dx, double x) {
        return (f.eval(x) - f.eval(x - dx)) / dx;
    }
    //static instant non fancy version
    public static double derivative(double x1, double y1, double x, double y) {
        return (y - y1) / (x - x1); //hehe
    }
    public static double derivative(double y1, double y2, double dx) {
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
}