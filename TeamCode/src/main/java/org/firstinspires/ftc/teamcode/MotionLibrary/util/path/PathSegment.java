package org.firstinspires.ftc.teamcode.MotionLibrary.util.path;

import org.firstinspires.ftc.teamcode.MotionLibrary.util.Pose2D;

import java.util.ArrayList;

public class PathSegment {

    int steps;

    public int lastPoint;
    public int firstPoint;

    ArrayList<Pose2D> segmentList = new ArrayList<>();
    ArrayList<Pose2D> pose2dList = new ArrayList<>();

    public PathSegment(ArrayList<Pose2D> pose2dList, int steps) {
        this.steps = steps;
        this.pose2dList = pose2dList;
    }

    public void addLast(int pose2d) {
        segmentList.add(pose2dList.get(pose2d));
        steps = segmentList.size();
        lastPoint = pose2d;
    }

    public void addFirst(int pose2d) {
        steps = segmentList.size();
        ArrayList<Pose2D> newList = segmentList;
        segmentList.clear();
        segmentList.add(pose2dList.get(pose2d));
        for (int i = 0; i < steps; i++){
            segmentList.add(newList.get(i));
        }
        steps = segmentList.size();
        firstPoint = pose2d;
    }

    public void removeFirst() {
        steps = segmentList.size();
        ArrayList<Pose2D> newList = segmentList;
        segmentList.clear();
        for (int i = 1; i < steps; i++){
            segmentList.add(newList.get(i));
        }
        steps = segmentList.size();
        firstPoint += 1;
    }

    public void removeLast() {
        steps = segmentList.size();
        ArrayList<Pose2D> newList = segmentList;
        segmentList.clear();
        for (int i = 0; i < steps - 1; i++){
            segmentList.add(newList.get(i));
        }
        steps = segmentList.size();
        lastPoint -= 1;
    }

    public Pose2D getClosestPoint(Pose2D currentPosition) {
        int pointName = 0;
        Pose2D point = new Pose2D(100);
        Pose2D test;
        steps = segmentList.size() - 1;
        for (int i = 0; i < steps; i++ ) {
            test = segmentList.get(i);
            if (Pose2D.getMagnitude(Pose2D.sub(currentPosition, test)) < Pose2D.getMagnitude(Pose2D.sub(currentPosition, point))) {
                point = test;
                pointName = i;
            }
        }
        point = segmentList.get(pointName);
        if(point.X != segmentList.get(0).X && point.Y != segmentList.get(0).Y) {
            removeFirst();
            addLast(lastPoint++);
        }
        point = segmentList.get(pointName);
        return point;
    }

    public Pose2D getClosestPointBack(Pose2D currentPosition) {
        int pointName = 0;
        Pose2D point = new Pose2D(100);
        Pose2D test;
        steps = segmentList.size();
        for (int i = steps; i > 0; i-- ) {
            test = segmentList.get(i);
            if (Pose2D.getMagnitude(Pose2D.sub(currentPosition, test)) < Pose2D.getMagnitude(Pose2D.sub(currentPosition, point))) {
                point = test;
                pointName = i;
            }
        }
        point = segmentList.get(pointName);
        if(point.X != segmentList.get(0).X && point.Y != segmentList.get(0).Y) {
            removeFirst();
            addLast(lastPoint++);
        }
        point = segmentList.get(pointName);
        return point;
    }
}