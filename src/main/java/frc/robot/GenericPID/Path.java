package frc.robot.GenericPID;

import java.util.ArrayList;

import java.awt.Color;
import frc.robot.GenericPID.Approximations.DoubleFunction;
import frc.robot.GenericPID.Implementations.LinearSegment;
import frc.robot.GenericPID.Implementations.PathSegmentBase;
import frc.robot.GenericPID.Testing.Graph;

/**
 * A 2d Composite Path, from a dynamic array of SegmentBase slices, each of which have their own extensible implementation.
 * Very easy to use, just use insert the segment with insert() and it will automatically find where it fits.Then access the value anywhere with y().
 */
class Path {
    private final static boolean debug = Debug.debug_GenericPath;
    private final static boolean debug2 = Debug.debug_GenericPath2;
    private ArrayList<PathSegmentBase> path = new ArrayList<PathSegmentBase>(); //is private, and should contain back to back segments, non overlapping or gapping, in order.
    //Then can search for segments in O(lg n) time.

    private Double cachea;
    private double cacheb;
    private int cachei;

    private final double gapepsilon = 0.00001;

    public static void main(String[] args) {
        test();
    }

    
    public static void test() {
        Graph.GraphConfig gc = new Graph.GraphConfig();
        gc.x1 = 0;
        gc.x2 = 30;
        Graph g = new Graph(gc);
        g.addPlot(Color.BLUE);
        Path p = new Path();
        pathf F = p.new pathf();
        if (!p.insertSegment(new LinearSegment(5,6,9,3))) {
            throw new RuntimeException("failed to insert segment 1");
        };
        if(debug)p.debugPath();
        if (!p.insertSegment(new LinearSegment(9,3,15,7))) {
            throw new RuntimeException("failed to insert segment 2");
        };
        if(debug)p.debugPath();
        if (!p.insertSegment(new LinearSegment(7,4,9.5,-2))) {
            throw new RuntimeException("failed to insert segment 3");
        };
        if(debug)p.debugPath();
        if (!p.insertSegment(new LinearSegment(0,4,5,-2))) {
            throw new RuntimeException("failed to insert segment 3");
        };
        if(debug)p.debugPath();
        if (!p.insertSegment(new LinearSegment(1,4,14,-2))) {
            throw new RuntimeException("failed to insert segment 3");
        };
        if(debug) p.debugPath();
        g.plot(0, 30, F, 0.01, 0);
        g.init(1000, 1000, "Path");
    }

    private class pathf implements DoubleFunction {
        public double eval(double x) {
            Double ret = y(x);
            if (ret == null) {
                return 0;
            } else {
                return ret;
            }
        }
    }
    
    private boolean epEquals(double a, double b) {
        return Math.abs(a - b) < gapepsilon;
    }

    public int findJunction(double x) {
        //takes the x value and returns where it would lie in the path; the index of the segment that it would be in. 
        //it will always round up the segment.

        //O(n), someone could change this to O(lg n) by using divide and conquer
        if (x < path.get(0).x1) {
            if(debug2) System.out.printf("underjunction on %f < %f, so\n", x, path.get(0).x1);
            return -1;
        }
        if (x >= path.get(path.size() - 1).x2) {
            if(debug2) System.out.printf("overjunction on %f > %f, so\n", x, path.get(path.size() -1 ).x2);
            return path.size();
        }
        for (int i = 0; i < path.size(); i++) {
            if (x < path.get(i).x2) {
                if(debug2) System.out.printf("normal junction %d on %f, so\n", i, x);
                return i;
            }
        }
        if(debug) System.out.println("No junction?!");
        return -1;
    }

    public boolean insertSegment(PathSegmentBase segment) { 
        //This is probably the sketchiest function in the entire library, 
        //but it seems to work; may need to deploy more unit tests though

        //index not needed, they are held within the bounds of the segment
        //returns false if not possible to add without gap
        
        if (path.size() == 0) {
            if (debug) System.out.println("Segment initialized");
            path.add(segment);
            return true;
        }

        //we can assume the path is at least one big now

        //find spot
        int i = findJunction(segment.x1);
        if (debug) System.out.printf("Insert Junction is located at %d\n" , i);
        
        //if it's before the beginning
        if (debug) System.out.println("Time to add segment");
        if (i == -1) {
            if (segment.x2 < path.get(0).x1) { //gap check
                return false;
            }
            //insert
            path.add(0, segment);
            int j; //view last clause for explanation of this
            for (j = 1; j < path.size() && segment.x2 > path.get(j).x2;) {
                path.set(j, null);
                if (debug) System.out.println("eaten segment : beginning");
                j++;
            }

            //chop the next one if it's partially contained, won't ever be non-contained because the gap is 
            //always checked, and then it would be the last one that would be chopped.
            //the only weird case is that there was definitely at least one there, but if j is now at the end, we dont chop next
            if (j < path.size()) {
                path.set(j, path.get(j).chopBeginning(segment.x2));
            }
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
                if (i + 1 < path.size()) { //view } else { for the explanation of this
                    for (j = i + 1; j < path.size() && segment.x2 > path.get(j).x2;) {
                        path.set(j, null);
                        if (debug) System.out.println("eaten segment : right on");
                        j++;
                    }
                    path.set(j, path.get(j).chopBeginning(segment.x2));
                }
            }
            path.add(segment);
        } else { //normal segments, x1 lays in i of an existing segment
            if (debug) System.out.printf("in normal bucket %d\n", i);
            path.set(i, path.get(i).chopEnd(segment.x1)); //chop existing
            if (i + 1 < path.size()) {
                int j;
                for (j = i + 1; j < path.size() && segment.x2 > path.get(j).x2;) { //first checked "eaten" segment is the one directly after one it's inside; no insertion yet
                    //remove all segments that are completely contained within the new segment
                    path.set(j, null); //set all the eatens to null; if the current segment has a higher x2, it's def eaten, due to smart placement of i of x1
                    if (debug) System.out.println("eaten segment : normal");
                    j++;
                }
                path.set(j, path.get(j).chopBeginning(segment.x2)); //chop last not replaced
            }
            path.add(i + 1, segment); //put it in right after the one it chopped back, before the nulls, but it doesn't matter
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
            if (debug) System.out.println("eaten segment removed");
        }
        if (debug) System.out.println("done \n");
        return true;
    }
    

    public Double y(double x) {
        int i = findJunction(x);
        if (debug2) System.out.printf("bucket: %d\n", i);
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