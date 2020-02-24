package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


public class SkystoneDetector {
    private AutoBase.SkystoneRelativePos SkystonePosition = AutoBase.SkystoneRelativePos.RIGHT;

    private OpenCvCamera webcam;
    private Telemetry telemetry;

    public SkystoneDetector(HardwareMap hardwareMap) {
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        webcam.openCameraDevice();
    }

    public AutoBase.SkystoneRelativePos getPosition(Telemetry newTelmetry, boolean blueSide) {
        telemetry = newTelmetry;

        if (blueSide) {
            webcam.setPipeline(new Pipeline(new double[]{0.35, 0.7}));
        } else {
            webcam.setPipeline(new Pipeline(new double[]{0.4, 0.75}));
        }

        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        pause(500);

        webcam.closeCameraDevice();

        return SkystonePosition;
    }

    public static void pause(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            System.err.format("IOException: %s%n", e);
        }
    }

    class Pipeline extends OpenCvPipeline {

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        double[] fractionalDistances;

        Pipeline(double[] distanceValues) {
            fractionalDistances = distanceValues;
        }

        @Override
        public Mat processFrame(Mat input) {
            int cutoff = input.height() / 4;
            input = input.submat(new Rect(0, cutoff, input.width(), input.height() - cutoff));
            Mat hsv = new Mat();

            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            Scalar lower_yellow = new Scalar(10, 100, 20);
            Scalar upper_yellow = new Scalar(35, 255, 255);

            Scalar lower_black = new Scalar(0, 0, 0);
            Scalar upper_black = new Scalar(180, 255, 30);

            Mat mask = new Mat();

            Core.inRange(hsv, lower_yellow, upper_yellow, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (int i = 0; i < contours.size(); i++) {

                Rect bounder = Imgproc.boundingRect(contours.get(i));
                double contourArea = Imgproc.contourArea(contours.get(i));
                //Average is 60K
                if (contourArea > 40000) {

                    Imgproc.rectangle(input, bounder, new Scalar(255, 0, 0), 10);
                    Mat newMask = new Mat();
                    Core.inRange(hsv.submat(bounder), lower_black, upper_black, newMask);

                    List<MatOfPoint> contours2 = new ArrayList<>();
                    Mat hierarchy2 = new Mat();
                    Mat originalCrop = input.submat(bounder);

                    Imgproc.findContours(newMask, contours2, hierarchy2, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                    for (int j = 0; j < contours2.size(); j++) {
                        //Average is 15K
                        if (Imgproc.contourArea(contours2.get(j)) > 8500) {

                            Imgproc.drawContours(originalCrop, contours2, j, new Scalar(0, 255, 0), 10);

                            int cX = (int) (bounder.tl().x + Imgproc.moments(contours2.get(j)).get_m10() / Imgproc.moments(contours2.get(j)).get_m00());
                            int cY = (int) (bounder.tl().y + Imgproc.moments(contours2.get(j)).get_m01() / Imgproc.moments(contours2.get(j)).get_m00());

                            Imgproc.circle(input, new Point(cX, cY), 10, new Scalar(0, 0, 255), 10);

                            if (Imgproc.moments(contours2.get(j)).get_m10() / Imgproc.moments(contours2.get(j)).get_m00() < fractionalDistances[0] * bounder.width) {
                                SkystonePosition = AutoBase.SkystoneRelativePos.LEFT;
                            } else if (Imgproc.moments(contours2.get(j)).get_m10() / Imgproc.moments(contours2.get(j)).get_m00() < fractionalDistances[1] * bounder.width) {
                                SkystonePosition = AutoBase.SkystoneRelativePos.MIDDLE;
                            } else {
                                SkystonePosition = AutoBase.SkystoneRelativePos.RIGHT;
                            }
                        }

                    }
                }

            }
            telemetry.addData("Position", SkystonePosition.name());
            telemetry.update();
            return input;
        }
    }
}