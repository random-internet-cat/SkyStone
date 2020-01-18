package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.auto.AutoBase;
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
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


public class SkystoneDetector {
    private AutoBase.SkystoneRelativePos SkystonePosition = AutoBase.SkystoneRelativePos.RIGHT;

    private OpenCvCamera phoneCam;

    private Telemetry telemetry;

    public AutoBase.SkystoneRelativePos getPosition(HardwareMap hardwareMap, Telemetry newTelmetry, boolean nearBlueTape) {
        telemetry = newTelmetry;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        if (nearBlueTape) {
            phoneCam.setPipeline(new Pipeline(new double[]{3.0 / 7, 11.0 / 14}, true));
        } else {
            phoneCam.setPipeline(new Pipeline(new double[]{0.4, 0.6}, false));
        }

        phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

        pause(500);

        phoneCam.closeCameraDevice();

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
        boolean isBlueSide;

        Pipeline(double[] distanceValues, boolean blueSide) {
            fractionalDistances = distanceValues;
            isBlueSide = blueSide;
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

                if (contourArea > 20000) {

                    Imgproc.rectangle(input, bounder, new Scalar(255, 0, 0), 10);
                    Mat newMask = new Mat();
                    Core.inRange(hsv.submat(bounder), lower_black, upper_black, newMask);

                    List<MatOfPoint> contours2 = new ArrayList<>();
                    Mat hierarchy2 = new Mat();
                    Mat originalCrop = input.submat(bounder);

                    Imgproc.findContours(newMask, contours2, hierarchy2, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                    for (int j = 0; j < contours2.size(); j++) {

                        if (Imgproc.contourArea(contours2.get(j)) > 10000) {

                            Imgproc.drawContours(originalCrop, contours2, j, new Scalar(0, 255, 0), 10);

                            int cX = (int) (bounder.tl().x + Imgproc.moments(contours2.get(j)).get_m10() / Imgproc.moments(contours2.get(j)).get_m00());
                            int cY = (int) (bounder.tl().y + Imgproc.moments(contours2.get(j)).get_m01() / Imgproc.moments(contours2.get(j)).get_m00());

                            Imgproc.circle(input, new Point(cX, cY), 10, new Scalar(0, 0, 255), 10);

                            if (Imgproc.moments(contours2.get(j)).get_m10() / Imgproc.moments(contours2.get(j)).get_m00() < fractionalDistances[0] * bounder.width) {
                                if (isBlueSide) {
                                    SkystonePosition = AutoBase.SkystoneRelativePos.LEFT;
                                } else {
                                    SkystonePosition = AutoBase.SkystoneRelativePos.RIGHT;
                                }
                            } else if (Imgproc.moments(contours2.get(j)).get_m10() / Imgproc.moments(contours2.get(j)).get_m00() < fractionalDistances[1] * bounder.width) {
                                if (isBlueSide) {
                                    SkystonePosition = AutoBase.SkystoneRelativePos.MIDDLE;
                                } else {
                                    SkystonePosition = AutoBase.SkystoneRelativePos.LEFT;
                                }
                            } else {
                                if (isBlueSide) {
                                    SkystonePosition = AutoBase.SkystoneRelativePos.RIGHT;
                                } else {
                                    SkystonePosition = AutoBase.SkystoneRelativePos.MIDDLE;
                                }
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
