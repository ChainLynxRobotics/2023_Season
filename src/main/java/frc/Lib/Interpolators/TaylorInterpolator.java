package frc.Lib.Interpolators;

import edu.wpi.first.math.interpolation.Interpolatable;

/**Implement using TreeMapInterpolator*/
public class TaylorInterpolator implements Interpolatable<TaylorInterpolator>, InverseInterpolator<TaylorInterpolator> {
    double value;
    double numTerms;

    public TaylorInterpolator(double value, double numTerms) {
        this.value = value;
        this.numTerms = numTerms;
    }

    @Override
    public double inverseInterpolate(TaylorInterpolator interpolationType, TaylorInterpolator key) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'inverseInterpolate'");
    }

    @Override
    public TaylorInterpolator interpolate(TaylorInterpolator endValue, double t) {
        // TODO Auto-generated method stub (integral)
        throw new UnsupportedOperationException("Unimplemented method 'interpolate'");
    }
    
}
