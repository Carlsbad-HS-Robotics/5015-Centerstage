package org.firstinspires.ftc.teamcode.MotionLibrary.util.path;

import org.firstinspires.ftc.teamcode.MotionLibrary.util.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class HermiteCurve {

    public int steps;
    public double endHeading;
    public boolean changeHeading;

    List<Vector2D> vector2DList = new ArrayList<>();

    Vector2D p1, p2, t1, t2, end;


    public HermiteCurve(Vector2D P1, Vector2D P2, Vector2D T1, Vector2D T2, int Steps) {
        steps = Steps;

        p1 = P1;
        p2 = P2;
        t1 = T1;
        t2 = T2;
        end = P2;

        for(int i = 0; i < steps; i++) {

            double s = i / steps;

            double h1 = (2 * s * s * s) - (3 * s * s) + 1; // 2s^3 - 3s^2 + 1
            double h2 = -(2 * s * s * s) + (3 * s * s);    //-2s^3 + 3s^2
            double h3 = (s * s * s) - (2 * s * s) + s;     // s^3 - 2s^2 + s
            double h4 = (s * s * s) - (s * s);             // s^3 -  s^2

            Vector2D p = Vector2D.add(Vector2D.mult(P1, h1), Vector2D.mult(P2, h2), Vector2D.mult(T1, h3), Vector2D.mult(T2, h4));

            changeHeading = false;
            vector2DList.add(i, p);
        }
    }

    public HermiteCurve(Vector2D P1, Vector2D P2, Vector2D T1, Vector2D T2, int Steps, double heading) {
        steps = Steps;

        p1 = P1;
        p2 = P2;
        t1 = T1;
        t2 = T2;
        end = P2;

        for(int i = 0; i < steps; i++) {

            double s = i / steps;

            double h1 = (2 * s * s * s) - (3 * s * s) + 1; // 2s^3 - 3s^2 + 1
            double h2 = -(2 * s * s * s) + (3 * s * s);    //-2s^3 + 3s^2
            double h3 = (s * s * s) - (2 * s * s) + s;     // s^3 - 2s^2 + s
            double h4 = (s * s * s) - (s * s);             // s^3 -  s^2

            Vector2D p = Vector2D.add(Vector2D.mult(P1, h1), Vector2D.mult(P2, h2), Vector2D.mult(T1, h3), Vector2D.mult(T2, h4));

            changeHeading = true;
            this.endHeading = heading;

            vector2DList.add(i, p);
        }
    }
}
