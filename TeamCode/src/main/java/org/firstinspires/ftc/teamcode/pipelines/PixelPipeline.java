package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PixelPipeline extends OpenCvPipeline {
    public static final Scalar
    lowerRange = new Scalar(200,200,200,200),
    upperRange = new Scalar(255,255,255,255);
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.blur(input, input, new Size(5,5));

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(input, input, Imgproc.MORPH_CLOSE, kernel);
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Core.inRange(input,lowerRange,upperRange,input);
        Mat hierarchy = new Mat();
        Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_FLOODFILL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, contours, -1, new Scalar(255,0,0));


        return input;
    }
}
