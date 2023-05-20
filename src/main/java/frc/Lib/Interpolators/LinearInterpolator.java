package frc.Lib.Interpolators;

import edu.wpi.first.math.interpolation.Interpolatable;

public class LinearInterpolator implements Interpolatable<LinearInterpolator>, InverseInterpolator<LinearInterpolator> {
    private double value;

    public LinearInterpolator(double value) {
        this.value = value;
    }

    @Override
    public LinearInterpolator interpolate(LinearInterpolator point, double x) {
        return new LinearInterpolator((this.value-point.value)*x+point.value);
    }

    //shouldn't be necessary
    public double value(double[][] points, double goal) {
        return points[1][0] + (points[1][1]-points[1][0])*(goal-points[0][0])/(points[0][1]-points[0][0]);
    }

    @Override
    public void inverseInterpolate(LinearInterpolator interpolationType, double key) {
        throw new UnsupportedOperationException("Unimplemented method 'inverseInterpolate'");
    }

}
