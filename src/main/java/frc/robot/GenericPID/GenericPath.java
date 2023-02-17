package frc.robot.GenericPID;

import java.util.ArrayList;

import java.awt.Color;
import frc.robot.GenericPID.Approximations.DoubleFunction;
import frc.robot.GenericPID.PathSegmentBase;
import frc.robot.GenericPID.Testing.Graph;

class GenericPath {
    private final static boolean debug = Debug.debug_GenericPath;
    private ArrayList<PathSegmentBase> path = new ArrayList<PathSegmentBase>(); //is private, and should contain back to back segments, non overlapping or gapping, in order.
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
            Double ret = y(x);
            if (ret == null) {
                return 0;
            } else {
                return ret;
            }
        }
    }

    public static void test() {
        Graph.GraphConfig gc = new Graph.GraphConfig();
        gc.x1 = 0;
        gc.x2 = 30;
        Graph g = new Graph(gc);
        g.addPlot(Color.BLUE);
        GenericPath p = new GenericPath();
        f F = p.new f();
        if (!p.insertSegment(new LinearSegment(5,6,10,3))) {
            throw new RuntimeException("failed to insert segment 1");
        };
        if(debug) p.debugPath();
        if (!p.insertSegment(new LinearSegment(9,3,15,3))) {
            throw new RuntimeException("failed to insert segment 2");
        };
        if(debug) p.debugPath();

        // assert(p.insertSegment(new LinearSegment(11,6,15,3)));
        // p.insertSegment(new LinearSegment(5,7,10,3));
        // p.insertSegment(new LinearSegment(9,5,14,9));
        // p.insertSegment(new LinearSegment(10,4,15,6));
        // p.insertSegment(new LinearSegment(5,6,10,3));
        // p.insertSegment(new LinearSegment(6,3,15,3));
        // p.insertSegment(new LinearSegment(15,3,20,10));
        // p.insertSegment(new LinearSegment(20,10,25,10));
        // p.insertSegment(new LinearSegment(25,10,30,3));
        g.plot(0, 30, F, 0.01, 0);
        g.init(1000, 1000, "Path");
    }

    private boolean epEquals(double a, double b) {
        return Math.abs(a - b) < gapepsilon;
    }

    public int findJunction(double x) {
        //takes the x value and returns where it would lie in the path; the index of the segment that it would be in. 
        //it will always round up the segment.

        //O(n) cause i don't feel like debugging
        if (x < path.get(0).x1) {
            return -1;
        }
        if (x > path.get(path.size() - 1).x2) {
            return path.size();
        }
        for (int i = 0; i < path.size(); i++) {
            if (x >= path.get(i).x1) {
                return i;
            }
        }
        return -1;

        // //O(lg n)
        // int rangeBeginning = 0;
        // int rangeEnd = path.size() - 1; //these should converge to the same value
        // int dpivot = (rangeEnd - rangeBeginning + 1) / 2;
        // if (cachea != null && cachea < x && cacheb > x) {
        //     return cachei;
        // }
        // if (x >= path.get(path.size() - 1).x2) {
        //     if(debug) System.out.println("x is greater than the last segment");
        //     return path.size();
        // }
        // if (x < path.get(0).x1) {
        //     if(debug) System.out.println("x is less than the first segment");
        //     return -1;
        // }
        // while (true) {
        //     if (debug) System.out.printf("pivot %d %d, we are not exiting\n", rangeBeginning, rangeEnd);
        //     if (dpivot == 0) {
        //         cachei = rangeBeginning;
        //         cachea = path.get(rangeBeginning).x1;
        //         cacheb = path.get(rangeBeginning).x2;
        //         return rangeBeginning;
        //     } else if (path.get(rangeBeginning + dpivot).x2 < x) {
        //         rangeBeginning += dpivot;
        //         dpivot = (rangeEnd - rangeBeginning + 1) / 2;
        //         if (debug) System.out.printf("pivot up   +%d to %d %d", dpivot, rangeBeginning, rangeEnd);
        //     } else if (path.get(rangeEnd - dpivot).x1 >= x) {
        //         rangeEnd -= dpivot;
        //         dpivot = (rangeEnd - rangeBeginning + 1) / 2;
        //         if (debug) System.out.printf("pivot down +%d to %d %d", dpivot, rangeBeginning, rangeEnd);
        //     } else {
        //         return rangeBeginning;
        //     }
        // }
    }

    public boolean insertSegment(PathSegmentBase segment) { //index not needed, they are held within the bounds of the segment
        //returns false if not possible to add without gap

        if (path.size() == 0) {
            path.add(segment);
            return true;
        }

        //find spot
        int i = findJunction(segment.x1);
        if (debug) System.out.printf("Insert Junction is located at %d\n" , i);
        
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
            return true;
        }
        if (i == path.size()) {
            if(debug) System.out.println("at end!");
            if (segment.x1 > path.get(path.size() - 1).x2) { //gap check
                return false;
            }
            //insert
            path.add(segment);
            return true;
        }

        //if it's right on
        if (epEquals(path.get(i).x1, segment.x1)) {
            if (debug) System.out.printf("slot in equals!!\n");
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
                if (debug) System.out.printf("chopping next: %b\n", i + 1 < path.size());
                if (i + 1 < path.size()) {
                    for (j = i + 1; j < path.size() && segment.x2 > path.get(j).x2;) {
                        //remove all segments that are completely contained within the new segment
                        path.set(j, null);
                        j++;
                    }
                    path.set(i + 1, path.get(i + 1).chopBeginning(segment.x2));
                }
            }
            path.add(segment);
        } else {
            if (debug) System.out.printf("in normal bucket %d\n", i);
            path.set(i, path.get(i).chopEnd(segment.x2));
            if (i + 1 < path.size()) {
                int j;
                for (j = i + 1; j < path.size() && segment.x2 > path.get(j).x2;) {
                    //remove all segments that are completely contained within the new segment
                    path.set(j, null);
                    j++;
                }
                path.set(i + 1, path.get(i + 1).chopBeginning(segment.x2));
            }
            path.add(i, segment);
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
        //if(debug) System.out.printf("bucket: %d\n", i);
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

    public void debugPath() {
        for (int i = 0; i < path.size(); i++) {
            System.out.printf("segment %d: %f %f, function %s\n", i, path.get(i).x1, path.get(i).x2, path.get(i).getClass());
        }
    }
}