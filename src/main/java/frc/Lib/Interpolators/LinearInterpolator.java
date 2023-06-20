package frc.Lib.Interpolators;

import edu.wpi.first.math.interpolation.Interpolatable;


/**Implement using TreeMapInterpolator*/
public class LinearInterpolator implements Interpolatable<LinearInterpolator>, InverseInterpolator<LinearInterpolator> {
    private double value;

    public LinearInterpolator(double value) {
        this.value = value;
    }

    @Override
    public LinearInterpolator interpolate(LinearInterpolator point, double x) {
        return new LinearInterpolator((this.value-point.value)*x+point.value);
    }

    @Override
    public double inverseInterpolate(LinearInterpolator output, LinearInterpolator key) {
        if (output.value - value <= 0 || key.value - value <= 0) {
            return 0;
        }
        return (output.value - value)/(key.value - value);
    }

    /** For direct implementations of LinearInterpolator (without TreeMapInterpolator)*/
    public double value(double[][] points) {
        return points[1][0] + (points[1][1]-points[1][0])*(value-points[0][0])/(points[0][1]-points[0][0]);
    }
}
