package org.firstinspires.ftc.teamcode.MotionLibrary.util.path;

import org.firstinspires.ftc.teamcode.MotionLibrary.util.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class BezierCurve {

    List<Vector2D> vector2DList = new ArrayList<>();

    public Vector2D A, B, end = new Vector2D();

    public int steps;
    public double endHeading;
    public boolean changeHeading;

    public BezierCurve(Vector2D a, Vector2D b, Vector2D End, double Steps) {
        steps = (int)Steps;
        for (int i = 0; i < Steps; i++ ) {
            Vector2D point = getMiddlePoint(getMiddlePoint(a, b, i / Steps), getMiddlePoint(b, End, i/Steps), i/Steps);

            vector2DList.add(i, point);
        }
        changeHeading = false;
        A = a;
        B = b;
        end = End;
    }

    public BezierCurve(Vector2D a, Vector2D b, Vector2D End, double Steps, double heading) {
        steps = (int)Steps;
        for (int i = 0; i < Steps; i++ ) {
            Vector2D point = getMiddlePoint(getMiddlePoint(a, b, i / Steps), getMiddlePoint(b, End, i/Steps), i/Steps);

            vector2DList.add(i, point);
        }
        A = a;
        B = b;
        end = End;
        changeHeading = true;
        this.endHeading = heading;
    }

    private Vector2D getMiddlePoint(Vector2D A, Vector2D B, double distance) {
        double angle = Math.atan(B.Y - A.Y / B.X - A.X);

        double pointDistance = Math.sqrt( Math.pow(B.X-A.X, 2) + Math.pow(B.Y-A.Y,2));

        distance *= pointDistance;

        return Vector2D.add(A, new Vector2D(Math.asin(angle) / distance, Math.acos(angle) / distance));
    }


}
