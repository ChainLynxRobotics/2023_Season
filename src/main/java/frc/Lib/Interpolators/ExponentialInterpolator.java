package frc.Lib.Interpolators;

import edu.wpi.first.math.interpolation.Interpolatable;


/**Implement using TreeMapInterpolator*/
public class ExponentialInterpolator implements Interpolatable<ExponentialInterpolator>, InverseInterpolator<ExponentialInterpolator> {
    double[] value;

    public ExponentialInterpolator(double[] value) {
        this.value = value;
    }

    @Override
    public ExponentialInterpolator interpolate(ExponentialInterpolator point, double x) {
        //(a, c), (b, d) -> y = c * (d/c)^((x-a) / (b-a))
        return new ExponentialInterpolator(new double[]{x,
            value[1]*Math.pow(point.value[1]/value[1], (x-value[0])/(point.value[0]-value[0]))});
    }

    @Override
    public double inverseInterpolate(ExponentialInterpolator output, ExponentialInterpolator key) {
        //x = (b-a)log(y/c)/log(d/c) + a
        return (key.value[0]-value[0])*Math.log10(output.value[1]/value[1])/Math.log10(key.value[1]/value[1])+value[0];
    }
    
}
