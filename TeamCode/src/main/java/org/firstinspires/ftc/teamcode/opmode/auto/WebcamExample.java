package org.firstinspires.ftc.teamcode.opmode.auto;

/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class WebcamExample extends LinearOpMode
{
    OpenCvCamera webcam;

    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Open the connection to the camera device
         */
        webcam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        //BLUE SIDE: 0.35, 0.7
        //RED SIDE: 0.4, 0.75

        webcam.setPipeline(new SamplePipeline(new double[]{0.4, 0.75}));

        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        /*
         * Tell the webcam to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
         * supports streaming from the webcam in the uncompressed YUV image format. This means
         * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
         * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
         *
         * Also, we specify the rotation that the webcam is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        //    sleep(1000);

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
//            telemetry.addData("Frame Count", webcam.getFrameCount());
//            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
//            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
//            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
//            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
//            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
//            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * The "if" statements below will pause the viewport if the "X" button on gamepad1 is pressed,
             * and resume the viewport if the "Y" button on gamepad1 is pressed.
             */
            else if(gamepad1.x)
            {
                webcam.pauseViewport();
            }
            else if(gamepad1.y)
            {
                webcam.resumeViewport();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        double[] fractionalDistances;
        //        boolean isBlueSide;
        AutoBase.SkystoneRelativePos SkystonePosition = AutoBase.SkystoneRelativePos.RIGHT;

        SamplePipeline(double[] distanceValues) {
            fractionalDistances = distanceValues;
//            isBlueSide = blueSide;
        }

        @Override
        public Mat processFrame(Mat input)
        {
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

                    telemetry.addData("Big bounder size", contourArea);

                    Imgproc.rectangle(input, bounder, new Scalar(255, 0, 0), 10);
                    Mat newMask = new Mat();
                    Core.inRange(hsv.submat(bounder), lower_black, upper_black, newMask);

                    List<MatOfPoint> contours2 = new ArrayList<>();
                    Mat hierarchy2 = new Mat();
                    Mat originalCrop = input.submat(bounder);

                    Imgproc.findContours(newMask, contours2, hierarchy2, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                    for (int j = 0; j < contours2.size(); j++) {
                        //Average is 15Kish
                        if (Imgproc.contourArea(contours2.get(j)) > 8500) {

                            telemetry.addData("Contour" + i + "Area",  Imgproc.contourArea(contours2.get(j)));

                            Imgproc.drawContours(originalCrop, contours2, j, new Scalar(0, 255, 0), 10);

                            int cX = (int) (bounder.tl().x + Imgproc.moments(contours2.get(j)).get_m10() / Imgproc.moments(contours2.get(j)).get_m00());
                            int cY = (int) (bounder.tl().y + Imgproc.moments(contours2.get(j)).get_m01() / Imgproc.moments(contours2.get(j)).get_m00());

                            Imgproc.circle(input, new Point(cX, cY), 10, new Scalar(0, 0, 255), 10);

                            Imgproc.line (
                                    input,                    //Matrix obj of the image
                                    new Point(bounder.tl().x + fractionalDistances[0] * bounder.width, cY - 100),        //p1
                                    new Point(bounder.tl().x + fractionalDistances[0] * bounder.width, cY + 100),       //p2
                                    new Scalar(0, 0, 255),     //Scalar object for color
                                    5                          //Thickness of the line
                            );

                            Imgproc.line (
                                    input,                    //Matrix obj of the image
                                    new Point(bounder.tl().x + fractionalDistances[1] * bounder.width, cY - 100),        //p1
                                    new Point(bounder.tl().x + fractionalDistances[1] * bounder.width, cY + 100),       //p2
                                    new Scalar(0, 0, 255),     //Scalar object for color
                                    5                          //Thickness of the line
                            );

                            if (Imgproc.moments(contours2.get(j)).get_m10() / Imgproc.moments(contours2.get(j)).get_m00() < fractionalDistances[0] * bounder.width) {
//                                if (isBlueSide) {
                                SkystonePosition = AutoBase.SkystoneRelativePos.LEFT;
//                                }
//                                else {
//                                    SkystonePosition = AutoBase.SkystoneRelativePos.RIGHT;
//                                }
                            } else if (Imgproc.moments(contours2.get(j)).get_m10() / Imgproc.moments(contours2.get(j)).get_m00() < fractionalDistances[1] * bounder.width) {
//                                if (isBlueSide) {
                                SkystonePosition = AutoBase.SkystoneRelativePos.MIDDLE;
//                                }
//                                else {
//                                    SkystonePosition = AutoBase.SkystoneRelativePos.LEFT;
//                                }
                            } else {
//                                if (isBlueSide) {
                                SkystonePosition = AutoBase.SkystoneRelativePos.RIGHT;
//                                }
//                                else {
//                                    SkystonePosition = AutoBase.SkystoneRelativePos.MIDDLE;
//                                }
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