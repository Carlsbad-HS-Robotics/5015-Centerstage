package org.firstinspires.ftc.teamcode.MotionLibrary.util.path;

import org.firstinspires.ftc.teamcode.MotionLibrary.util.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class Orbit {

    public int steps;
    public double endHeading;
    public boolean changeHeading;
    public boolean targetCenter;

    Vector2D end, center;

    List<Vector2D> vector2DList = new ArrayList<>();

    public Orbit(Vector2D center, Vector2D start, double radians, int Steps) {

        steps = Steps;
        Vector2D startRelativeToCenter = Vector2D.sub(center, start);

        for(int i = 0; i < steps; i++) {

           Vector2D p = startRelativeToCenter;
           p.rotateRadians(radians * i/ steps);
           p.add(center);

            vector2DList.add(i, p);
        }
        end = vector2DList.get(steps);
        changeHeading = false;
        this.center = center;
    }

    public Orbit(Vector2D center, Vector2D start, double radians, int Steps, double endHeading) {

        steps = Steps;
        Vector2D startRelativeToCenter = Vector2D.sub(center, start);

        for(int i = 0; i < steps; i++) {

            Vector2D p = startRelativeToCenter;
            p.rotateRadians(radians * i/steps);
            p.add(center);

            vector2DList.add(i, p);
        }

        end = vector2DList.get(steps);
        this.endHeading = endHeading;
        changeHeading = true;
    }

    public Orbit(Vector2D center, Vector2D start, double radians, int Steps, double endHeading, boolean targetCenter) {

        steps = Steps;
        Vector2D startRelativeToCenter = Vector2D.sub(center, start);

        for(int i = 0; i < steps; i++) {

            Vector2D p = startRelativeToCenter;
            p.rotateRadians(radians * i/ steps);
            p.add(center);

            vector2DList.add(i, p);
        }

        end = vector2DList.get(steps);
        this.targetCenter = targetCenter;
    }
}
