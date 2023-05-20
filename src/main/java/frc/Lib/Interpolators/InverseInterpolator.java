package frc.Lib.Interpolators;

/*returns key from value and interpolation curve */
public interface InverseInterpolator<T> {
    
    public void inverseInterpolate(T interpolationType, double key);
}
