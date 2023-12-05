package frc.Lib.Interpolators;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.interpolation.Interpolatable;

/**Get output of undefined input by extrapolating from nearby inputs, useful for finding a value for a key by making an esimate from given key-value pairs 
 * E.g., given points on a 2D trajectory, this is a way to estimate more for increased accuracy.
*/
public class TreeMapInterpolator <K extends InverseInterpolator<K> & Comparable<K>, V extends Interpolatable<V>> extends TreeMap<K,V> {
    int maxSize;

    public TreeMapInterpolator(int maxSize) {
        this.maxSize = maxSize;
    }

    @Override
    public V put(K key, V value) {
        if (size() >= maxSize) {
            remove(firstKey());
        }
        super.put(key, value);
        return value;
    }

    @Override
    public void putAll(Map<? extends K, ? extends V> map) {
        if (map.size() + size() >= maxSize) {
            while (map.size() + size() < maxSize) {
                super.put(((TreeMap<K, V>) map).lastKey(), map.get(lastKey()));
                map.remove(((TreeMap<K, V>) map).lastKey());
            }
        }
    }

    /**Get value if already exists in map or interpolate for it, if not possible return null*/
    public V getInterpolated(K key) {
        V value = get(key);
        if (value == null) {
            //get the keys with the closest greater and less than values than current key's value
            K upperBound = ceilingKey(key);
            K lowerBound = floorKey(key);

            if (upperBound == null && lowerBound == null) {
                return null;
            } else if (upperBound == null) {
                return get(lowerBound);
            } else if (lowerBound == null) {
                return get(upperBound);
            }

            V upperValue = get(upperBound);
            V lowerValue = get(lowerBound);

            /*get output from getting value between lower and upper bound through inverse interpolation
            then interpolating for the output along the curve in the range*/
            V output = lowerValue.interpolate(upperValue, lowerBound.inverseInterpolate(upperBound, key));
            super.put(key, output);
            return output;
        } else {
            return value;
        }
    }
}
