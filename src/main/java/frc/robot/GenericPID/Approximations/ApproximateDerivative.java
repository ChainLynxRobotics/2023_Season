public class ApproximateDerivative {
    //a sometimes static class which takes the derivative of a certain point of a function
    //just use the derivative method, and a double function
    //or track ongoing derivative by using nextDerivative()
    private double xlast;
    private double ylast;
    private DoubleFunction f;
    //static instant version
    public static double derivative(DoubleFunction f, double dx, double x) {
        return (f.eval(x) - f.eval(x - dx)) / dx;
    }
    //static instant non fancy version
    public static double derivative(x1, y1, x, y) {
        return (y - y1) / (x - x1);
    }
    public static double derivative(y1, y2, dx) {
        return (y2 - y1) / dx;
    }
    public void reset(DoubleFunction f, double x) {
        this.f = f;
        this.xlast = x;
    }
    //this one doesnt seem too good tbh
    public double nextDerivative(double dx) {
        xlast += dx;
        return derivative(f, dx, xlast);
    }
    public double nextDerivative(double x, double fx) {
        double dx = x - xlast;
        double ret = derivative(ylast, fx, dx)
        xlast += dx;
        ylast = fx;
        return ret;
    }
    public ApproximateDerivative (DoubleFunction f, double x) {
        this.reset(f, x);
        ylast = 0;
    }
}