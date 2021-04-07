package org.firstinspires.ftc.teamcode.subsystem.detector;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.subsystem.detector.RingConstants.HORIZON;

public class RingPipeline extends OpenCvPipeline {

    private Mat mat = new Mat();
    private Mat ret = new Mat();

    public enum Rings {
        ZERO,
        ONE,
        FOUR
    }

    private Rings rings = Rings.ZERO;

    private Scalar lowerOrange = new Scalar(0, 141, 0);
    private Scalar upperOrange = new Scalar(255.0, 240, 100);

    @Override
    public Mat processFrame(Mat input) {
        ret.release();

        ret = new Mat();
        try {

            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

            Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1);

            Core.inRange(mat, lowerOrange, upperOrange, mask);
            Core.bitwise_and(input, input, ret, mask);

            Imgproc.GaussianBlur(mask, mask, new Size(5.0, 15.0), 0.00);

            Mat hierarchy = new Mat();
            List<MatOfPoint> contours = new ArrayList<>();

            Imgproc.findContours(mask,contours,hierarchy,Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
            Imgproc.drawContours(ret, contours, -1, new Scalar(0.0, 255.0, 0.0), 3);

            int maxWidth = 0;
            Rect bestRect = new Rect();
            for(MatOfPoint contour : contours){
                Rect rect = Imgproc.boundingRect(contour);

                int width = rect.width;
                if(width > maxWidth && width < 100 && rect.y + rect.height > HORIZON){
                    maxWidth = width;
                    bestRect = rect;
                }
                contour.release();
            }

            Imgproc.rectangle(ret, bestRect, new Scalar(0.0, 0.0, 255.0), 2);

            if(maxWidth >= 40){
                double height = (double)bestRect.height;
                if(height/maxWidth >= 0.8){
                    rings = Rings.FOUR;
                } else {
                    rings = Rings.ONE;
                }
            } else {
                rings = Rings.ZERO;
            }

            mat.release();
            mask.release();
            hierarchy.release();

        } catch (Exception e){
            //hope for the best!
        }
        return ret;
    }

    public Rings getRings(){
        return rings;
    }
}