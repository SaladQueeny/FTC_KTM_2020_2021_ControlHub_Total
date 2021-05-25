/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2021.
   @author Kolpakov Egor
*/

package org.firstinspires.ftc.teamcode.AutoOPs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVisionL;
@TeleOp
public class webcam_correct_test extends LinearOpMode
{
    OpenCvCamera webcam;

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVisionL pipeline = new org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVisionL();
        webcam.setPipeline(pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();
        {

            while (opModeIsActive()) {
                /*
                 * Send some stats to the telemetry
                 */
                telemetry.addData("Frame Count", webcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
                telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
                telemetry.addData("pipeline working", pipeline.position);
                telemetry.update();

                /*
                 * NOTE: stopping the stream from the camera early (before the end of the OpMode
                 * when it will be automatically stopped for you) *IS* supported. The "if" statement
                 * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
                 */
                if (gamepad1.a) {
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
                 * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
                 * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
                 * anyway). Of course in a real OpMode you will likely not want to do this.
                 */
                webcam.stopStreaming();
                sleep(100);
            }
        }
    }

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}