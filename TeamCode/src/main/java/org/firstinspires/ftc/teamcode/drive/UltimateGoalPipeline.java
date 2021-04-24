package org.firstinspires.ftc.teamcode.drive;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class UltimateGoalPipeline extends OpenCvPipeline {

    // some constants

    public enum Rings { FOUR, ONE, ZERO } // number of rings
    // color constants and other constants
    private static final Scalar BLUE = new Scalar(0,0,255);
    private static final Scalar GREEN = new Scalar(0,255,0);
    private static final int REGION_WIDTH = 70;
    private static final int REGION_HEIGHT = 55;
    private final int FOUR_RING_THRESHOLD = 135; // This can really be changed (based on the time of day)
    private final int ONE_RING_THRESHOLD = 120; // As long as the ONE_RING_THRESHOLD <= FOUR_RING_THRESHOLD - 10

    // camera positioning location
    private static final Point ANCHOR_POINT = new Point(95, 105); // anchor point for box to be drawn
    private Point REGION_TOP_LEFT = new Point(ANCHOR_POINT.x, ANCHOR_POINT.y);
    private Point REGION_BOTTOM_RIGHT = new Point(ANCHOR_POINT.x + REGION_WIDTH, ANCHOR_POINT.y + REGION_HEIGHT);

    // Detection variables
    Mat regionCB, YCbCr = new Mat(), Cb = new Mat();
    int average = 0;

    public volatile Rings pos = Rings.FOUR;

    void getCb(Mat in) {
        Imgproc.cvtColor(in, YCbCr, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCbCr, Cb, 1);
    }

    @Override
    public void init(Mat frame) {
        getCb(frame);
        regionCB = Cb.submat(new Rect(REGION_TOP_LEFT, REGION_BOTTOM_RIGHT));
    }

    @Override
    public Mat processFrame(Mat in) {
        getCb(in);
        average = (int)Core.mean(regionCB).val[0];

        Imgproc.rectangle(in, REGION_TOP_LEFT, REGION_BOTTOM_RIGHT, BLUE, 2);

        pos = Rings.FOUR;
        if(average > FOUR_RING_THRESHOLD)
            pos = Rings.FOUR;
        else if(average > ONE_RING_THRESHOLD)
            pos = Rings.ONE;
        else
            pos = Rings.ZERO;

        // Imgproc.rectangle(in, REGION_TOP_LEFT, REGION_BOTTOM_RIGHT, GREEN, -1);

        return in;
    }

    public int getOutput() { return average; }
}
