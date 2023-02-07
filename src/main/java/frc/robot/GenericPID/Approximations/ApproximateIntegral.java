import DoubleFunction;
public class ApproximateIntegral {
    private double x1;
    private double x;
    private double y1;
    private double Y;
    private DoubleFunction f;
    public double x() {
        return x;
    }
    public double x1() {
        return x1;
    }
    public double y1() {
        return y1;
    }
    public double val() {
        return Y;
    }
    //not requiring a function for this so there can be an alternative of the next method
    //unfortunately integrals can't exactly be staic like derivatives so this is how it looks
    public void reset(double x, double y) {
        this.x = x;
        this.y = y;
        this.x1 = x;
        this.y1 = y;
    }
    public void reset(DoubleFunction f, double x, double y) {
        this.x = x;
        this.y = y;
        this.x1 = x;
        this.y1 = y;
        this.f = f;
    }
    public void next(double fx, double dx) {
        x += dx;
        Y += fx * dx;
    }
    public void next(double dx) {
        x += dx;
        Y += f.eval(x1) * dx;
    }
    public void integrate(DoubleFunction f, double dx, double x2) {
        while (x < x2) {
            next(f, dx);
        }
    }

    public ApproximateIntegral(double x, double y) {
        this.reset(x, y);
        f = new Zero();
    }
    
    public ApproximateIntegral(DoubleFunction f, double x, double y) {
        this.reset(f, x, y);
    }
}