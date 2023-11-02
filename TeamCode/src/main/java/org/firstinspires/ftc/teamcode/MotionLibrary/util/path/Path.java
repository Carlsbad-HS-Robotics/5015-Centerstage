package org.firstinspires.ftc.teamcode.MotionLibrary.util.path;



import org.firstinspires.ftc.teamcode.MotionLibrary.util.Pose2D;
import org.firstinspires.ftc.teamcode.MotionLibrary.util.Vector2D;

import java.util.ArrayList;

public class Path {

    ArrayList<Pose2D> pose2dList;

    public Pose2D end;

    public int steps;
    public double heading;

    public Path() {
        end = new Pose2D(0);
    }

    public Path(double startHeading) {
        end = new Pose2D(0, 0, startHeading);
    }

    public Path(BezierCurve bezierCurve) {
        if (bezierCurve.changeHeading) {
            for(int i = 0; i < bezierCurve.steps; i++) {
                pose2dList.add(new Pose2D(bezierCurve.vector2DList.get(i), bezierCurve.endHeading * i / bezierCurve.steps));
                end = new Pose2D(bezierCurve.end, bezierCurve.endHeading);
            }
        } else {
            for(int i = 0; i < bezierCurve.steps; i++) {
                pose2dList.add(new Pose2D(bezierCurve.vector2DList.get(i), this.end.heading));
                end = new Pose2D(bezierCurve.end, this.end.heading);
            }
        }
    }

    public Path(HermiteCurve hermiteCurve) {
        if (hermiteCurve.changeHeading) {
            for(int i = 0; i < hermiteCurve.steps; i++) {
                pose2dList.add(new Pose2D(hermiteCurve.vector2DList.get(i), hermiteCurve.endHeading * i / hermiteCurve.steps));
                end = new Pose2D(hermiteCurve.end, hermiteCurve.endHeading);
            }
        } else {
            for(int i = 0; i < hermiteCurve.steps; i++) {
                pose2dList.add(new Pose2D(hermiteCurve.vector2DList.get(i), this.end.heading));
                end = new Pose2D(hermiteCurve.end, this.end.heading);
            }
        }
    }

    public Path(Line line) {
        if (line.changeHeading) {
            for(int i = 0; i < line.steps; i++) {
                pose2dList.add(new Pose2D(line.vector2DList.get(i), line.endHeading * i / line.steps));
                end = new Pose2D(line.end, line.endHeading);
            }
        } else {
            for(int i = 0; i < line.steps; i++) {
                pose2dList.add(new Pose2D(line.vector2DList.get(i), this.end.heading));
                end = new Pose2D(line.end, this.end.heading);
            }
        }
    }

    public Pose2D getClosestPoint(Pose2D currentPosition) {
        int pointName = 0;
        Pose2D point = new Pose2D(100);
        Pose2D test = new Pose2D();
        steps = pose2dList.size() - 1;
        for (int i = 0; i < steps; i++ ) {
            test = pose2dList.get(i);
            if (Pose2D.getMagnitude(Pose2D.sub(currentPosition, test)) < Pose2D.getMagnitude(Pose2D.sub(currentPosition, point))) {
                point = test;
                pointName = i++;
            }
        }
        point = pose2dList.get(pointName);
        return point;
    }

    public boolean searching = true;
    public Pose2D maxPoint;
    public Pose2D minPoint;
    public Pose2D mid;
    double minDistance;
    double maxDistance;
    int minValue;
    int maxValue;

    public int binarySearchClosestPoint(Pose2D currentPosition) {
        Pose2D point;
        steps = pose2dList.size() - 1;
        maxValue = steps;
        minValue = 0;
        //mid = Pose2d.getMidpoint(max, min);

        int i = 0;
        while(searching) {
            i++;
            minPoint = pose2dList.get(minValue);
            maxPoint = pose2dList.get(maxValue);
            minDistance = Pose2D.sub(currentPosition, minPoint).getMagnitude();
            maxDistance = Pose2D.sub(currentPosition, maxPoint).getMagnitude();
            if (minDistance >= maxDistance) {
                if (minValue + 1 == maxValue) minValue = maxValue;
                minValue = (maxValue + minValue) / 2;
            }
            else {
                if (maxValue - 1 == minValue) maxValue = minValue;
                maxValue = (maxValue + minValue) / 2;
            }
            if (maxValue - minValue == 0) searching = false;
        }

        point = pose2dList.get(maxValue);
        return maxValue;
    }

    public PathSegment posePath;
    public void initSegmentPos(Pose2D currentPosition, int pathSteps) {
        int point = binarySearchClosestPoint(currentPosition);
        posePath = new PathSegment(this.pose2dList, pathSteps);
        for (int i = point; i < point + pathSteps; i++) {
            posePath.addFirst(i);
        }
    }

    public void initSegmentNeg(Pose2D currentPosition, int pathSteps) {
        int point = binarySearchClosestPoint(currentPosition);
        posePath = new PathSegment(this.pose2dList, pathSteps);
        for (int i = point; i < point - pathSteps; i--) {
            posePath.addFirst(i);
        }
    }

    public Pose2D previousClosest = new Pose2D();

    public void getClosestSegmentPoint(Pose2D currentPosition) {
        Pose2D point = posePath.getClosestPoint(currentPosition);
    }


    public Pose2D getClosestPointBack(Pose2D currentPosition) {
        int pointName = 0;
        Pose2D point = new Pose2D(100);
        Pose2D test = new Pose2D();
        steps = pose2dList.size();
        for (int i = steps; i > 0; i-- ) {
            test = pose2dList.get(i);
            if (Pose2D.getMagnitude(Pose2D.sub(currentPosition, test)) < Pose2D.getMagnitude(Pose2D.sub(currentPosition, point))) {
                point = test;
                pointName = i--;
            }
        }
        point = pose2dList.get(pointName);
        return point;
    }

    private Vector2D getMiddlePoint(Vector2D A, Vector2D B, double distance) {
        double angle = Math.atan(B.Y - A.Y / B.X - A.X);

        double pointDistance = Math.sqrt( Math.pow(B.X-A.X, 2) + Math.pow(B.Y-A.Y,2));

        distance *= pointDistance;

        return Vector2D.add(A, new Vector2D(Math.asin(angle) / distance, Math.acos(angle) / distance));
    }

    public Path add(HermiteCurve hermiteCurve) {
        if (hermiteCurve.changeHeading) {
            for(int i = 0; i < hermiteCurve.steps; i++) {
                pose2dList.add(new Pose2D(hermiteCurve.vector2DList.get(i), hermiteCurve.endHeading * i / hermiteCurve.steps));
                end = new Pose2D(hermiteCurve.end, hermiteCurve.endHeading);
            }
        } else {
            for(int i = 0; i < hermiteCurve.steps; i++) {
                pose2dList.add(new Pose2D(hermiteCurve.vector2DList.get(i), this.end.heading));
                end = new Pose2D(hermiteCurve.end, this.end.heading);
            }
        }
        return this;
    }

    public Path add(BezierCurve bezierCurve) {
        if (bezierCurve.changeHeading) {
            for(int i = 0; i < bezierCurve.steps; i++) {
                pose2dList.add(new Pose2D(bezierCurve.vector2DList.get(i), bezierCurve.endHeading * i / bezierCurve.steps));
                end = new Pose2D(bezierCurve.end, bezierCurve.endHeading);
            }
        } else {
            for(int i = 0; i < bezierCurve.steps; i++) {
                pose2dList.add(new Pose2D(bezierCurve.vector2DList.get(i), this.end.heading));
                end = new Pose2D(bezierCurve.end, this.end.heading);
            }
        }
        return this;
    }

    public Path add(Line line) {
        if (line.changeHeading) {
            for(int i = 0; i < line.steps; i++) {
                pose2dList.add(new Pose2D(line.vector2DList.get(i), line.endHeading * i / line.steps));
                end = new Pose2D(line.end, line.endHeading);
            }
        } else {
            for(int i = 0; i < line.steps; i++) {
                pose2dList.add(new Pose2D(line.vector2DList.get(i), this.end.heading));
                end = new Pose2D(line.end, this.end.heading);
            }
        }
        return this;
    }

    public Path add(Orbit orbit) {
        if (orbit.changeHeading) {
            for(int i = 0; i < orbit.steps; i++) {
                pose2dList.add(new Pose2D(orbit.vector2DList.get(i), orbit.endHeading * i / orbit.steps));
                end = new Pose2D(orbit.end, orbit.endHeading);
            }
        } else if (orbit.targetCenter) {
            for(int i = 0; i < orbit.steps; i++) {
                pose2dList.add(new Pose2D(orbit.vector2DList.get(i), Vector2D.getAngle(orbit.vector2DList.get(i), orbit.center)));
                end = new Pose2D(orbit.end, orbit.endHeading);
            }
        } else {
            for(int i = 0; i < orbit.steps; i++) {
                pose2dList.add(new Pose2D(orbit.vector2DList.get(i), this.end.heading));
                end = new Pose2D(orbit.end, this.end.heading);
            }
        }
        return this;
    }

    public Path add(Vector2D vector2d) {
        pose2dList.add(new Pose2D(vector2d, end.heading));
        end = new Pose2D(vector2d, end.heading);
        return this;
    }

    public Path add(Pose2D pose2d) {
        pose2dList.add(pose2d);
        end = pose2d;
        return this;
    }

    public Path add(Path path) {
        path.steps = path.pose2dList.size();
        pose2dList.addAll(path.pose2dList);
        end = path.pose2dList.get(path.steps);
        return this;
    }

    public Path smoothConnect(int hermiteSteps, Line line) {
        steps = pose2dList.size();
        this.add(
                new HermiteCurve(
                        new Vector2D(pose2dList.get(steps)),
                        line.start,
                        Vector2D.add(Vector2D.mult(Vector2D.sub(new Vector2D(pose2dList.get(steps)), new Vector2D(pose2dList.get(steps - 1))), -1), new Vector2D(pose2dList.get(steps))),
                        line.end,
                        hermiteSteps)
        );

        this.add(line);
        return this;
    }
}