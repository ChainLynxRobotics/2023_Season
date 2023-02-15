package frc.robot.GenericPID;

import java.util.ArrayList;

import java.awt.Color;
import frc.robot.GenericPID.Approximations.DoubleFunction;
import frc.robot.GenericPID.PathSegmentBase;
import frc.robot.GenericPID.Testing.Graph;

class GenericPath {
    private ArrayList<PathSegmentBase> path; //is private, and should contain back to back segments, non overlapping or gapping, in order.
    //Then can search for segments in O(lg n) time.

    private Double cachea;
    private double cacheb;
    private int cachei;

    private final double gapepsilon = 0.00001;

    public static void main(String[] args) {
        test();
    }

    private class f implements DoubleFunction {
        public double eval(double x) {
            return y(x);
        }
    }

    public static void test() {
        Graph g = new Graph(new Graph.GraphConfig());
        g.init(1000, 1000, "Path");
        g.addPlot(Color.BLUE);
        GenericPath p = new GenericPath();
        f ff = p.new f();
        p.insertSegment(new LinearSegment(5,6,10,3));
        g.plot(5,10, ff, 0.01, 0);
    }

    private boolean epEquals(double a, double b) {
        return Math.abs(a - b) < gapepsilon;
    }

    public int findJunction(double x) {
        //takes the x value and returns where it would lie in the path; the index of the segment that it would be in. 
        //it will always round up the segment.
        //O(lg n)
        int lastPivot = path.size() / 2;
        int dpivot = ((path.size() + 1) / 2 + 1) / 2;
        if (cachea != null && cachea < x && cacheb > x) {
            return cachei;
        }
        if (x >= path.get(path.size() - 1).x2) {
            return path.size();
        }
        if (x < path.get(0).x2) {
            return -1;
        }
        while (true) {
            if (dpivot == 0) {
                cachei = lastPivot;
                cachea = path.get(lastPivot).x1;
                cacheb = path.get(lastPivot).x2;
                return lastPivot;
            } else if (path.get(lastPivot).x2 < x) {
                lastPivot = (lastPivot + dpivot) / 2;
                dpivot = (dpivot + 1) / 2;
            } else if (path.get(lastPivot).x1 >= x) {
                lastPivot = (lastPivot - dpivot) / 2;
                dpivot = (dpivot + 1) / 2;
            } else {
                return lastPivot;
            }
        }
    }

    public boolean insertSegment(PathSegmentBase segment) { //index not needed, they are held within the bounds of the segment
        //returns false if not possible to add without gap

        if (path.size() == 0) {
            path.add(segment);
            return true;
        }

        //find spot
        int i = findJunction(segment.x1);
        
        //if it's before the beginning
        if (i == -1) {
            if (segment.x2 < path.get(0).x1) { //gap check
                return false;
            }
            //insert
            path.add(0, segment);
            int j;
            for (j = 1; j < path.size() && segment.x2 > path.get(j).x2;) {
                //remove all segments that are completely contained within the new segment
                path.set(j, null);
                j++;
            }

            //chop the next one if it's partially contained, won't ever be non-contained because the gap is 
            //always checked, and then it would be the last one that would be chopped.
            path.set(j, path.get(j).chopBeginning(segment.x2));
        }
        if (i == path.size()) {
            if (segment.x1 > path.get(path.size() - 1).x2) { //gap check
                return false;
            }
            //insert
            path.add(segment);
        }

        //if it's right on
        if (epEquals(path.get(i).x1, segment.x1)) {
            segment.x1 = path.get(i).x1; //epequal
            if (path.get(i).x2 < segment.x2) {
                return false; //gap check
            }
            if (epEquals(path.get(i).x2, segment.x2)) {
                segment.x2 = path.get(i).x2; //epequal
                path.set(i, segment); //swap!
            } else {
                path.set(i, segment);
                int j;
                if (i+1 < path.size()) {
                    for (j = i + 1; j < path.size() && segment.x2 > path.get(j).x2;) {
                        //remove all segments that are completely contained within the new segment
                        path.set(j, null);
                        j++;
                    }
                    path.set(i + 1, path.get(i + 1).chopBeginning(segment.x2));
                }
            }
        }
        

        cachea = null;

        //clear nulls, nonNullIndex iterates at the same rate over only existing elements, then they're all copied, and the list is resized.
        int nonNullIndex = 0;
        int index;
        for (index = 0; index < path.size() && nonNullIndex < path.size();index++) {
            while (path.get(nonNullIndex) == null) {
                nonNullIndex++;
            }
            if (index != nonNullIndex) {
                path.set(index, path.get(nonNullIndex));
            }
            nonNullIndex++;
        }
        while (path.size() > index) {
            path.remove(path.size() - 1);
        }
        return true;
    }

    public Double y(double x) {
        int i = findJunction(x);
        if (i == -1) {
            return null;
        }
        if (i == path.size()) {
            return null;
        }
        return path.get(i).y(x);
    }

    public Double derivative(double x) {
        int i = findJunction(x);
        if (i == -1) {
            return null;
        }
        if (i == path.size()) {
            return null;
        }
        return path.get(i).derivative(x);
    }

    public void insertSegmentSmooth(PathSegmentBase segment, double maxdydx) {
        //difference is that it will auto generate a new segment (a quadratic) to smooth out the transition, with the maxdydx.
        cachea = null;
    }
}