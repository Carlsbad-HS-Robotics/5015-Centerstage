package org.firstinspires.ftc.teamcode.MotionLibrary.util.path;

import org.firstinspires.ftc.teamcode.MotionLibrary.util.Pose2D;
import org.firstinspires.ftc.teamcode.MotionLibrary.util.Vector2D;

import java.util.ArrayList;
import java.util.List;

/**
 * Points going in a straight line from one end point to another
 */
public class Line {

    public int steps;

    public double endHeading;
    public boolean changeHeading;

    List<Vector2D> vector2DList = new ArrayList<>();

    Vector2D start, end;


    public Line(Vector2D start, Vector2D stop, double heading) {
        vector2DList.add(start);
        vector2DList.add(stop);
        end = stop;
        this.start = start;
        changeHeading = true;
        endHeading = heading;
        steps = 2;
    }

    public Line(Vector2D start, Pose2D stop) {
        vector2DList.add(start);
        vector2DList.add(new Vector2D(stop));
        end = new Vector2D(stop);
        this.start = start;
        changeHeading = true;
        endHeading = stop.heading;
        steps = 2;
    }

    public Line(Vector2D start, Vector2D stop) {
        vector2DList.add(start);
        vector2DList.add(stop);
        end = stop;
        this.start = start;
        changeHeading = false;
        steps = 2;
    }

    public Line(Vector2D start, Vector2D stop, double heading, int steps) {
        for(int i = 0; i < steps; i++) {
            vector2DList.add(getDistanceAlongLine(start, stop, i/steps));
        }
        end = stop;
        this.start = start;
        changeHeading = true;
        endHeading = heading;
        this.steps = steps;
    }

    public Line(Vector2D start, Pose2D stop, int steps) {
        for(int i = 0; i < steps; i++) {
            vector2DList.add(getDistanceAlongLine(start, new Vector2D(stop), i/steps));
        }
        end = new Vector2D(stop);
        this.start = start;
        changeHeading = true;
        endHeading = stop.heading;
        this.steps = steps;
    }

    public Line(Vector2D start, Vector2D stop, int steps) {
        for(int i = 0; i < steps; i++) {
            vector2DList.add(getDistanceAlongLine(start, stop, i/steps));
        }
        end = stop;
        this.start = start;
        changeHeading = false;
        this.steps = steps;
    }

    /**
     * Returns the point that is in between the two given points.
     * @param start
     * @param stop
     * @param distance determines where in between the two points that the return point is EX: 1 will return start, 0 will return stop
     * @return
     */
    public static Vector2D getDistanceAlongLine(Vector2D start, Vector2D stop, double distance) {
        double lineDistance = start.getMagnitude() - stop.getMagnitude();
        lineDistance *= distance;
        Vector2D point = Vector2D.sub(start, stop);
        point.normalize();
        point.mult(lineDistance);
        return point;
    }
}