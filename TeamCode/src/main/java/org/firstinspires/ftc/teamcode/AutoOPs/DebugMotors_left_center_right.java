/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2021.
   @author Kolpakov Egor
*/
package org.firstinspires.ftc.teamcode.AutoOPs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name= "DebugMotors_left_center_right", group="AutoOP")
public class DebugMotors_left_center_right extends Robot {
    OpenCvCamera webcam;
    private ElapsedTime runtime = new ElapsedTime();
    private int time = 3000; //One way rotation time (ms)

    @Override
    public void runOpMode() {
        initHW(hardwareMap);
        BNO055IMU imu;
        Orientation angles;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision pipeline = new org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision();
        //org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision pipeline1 = new org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision();

        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//        });

        telemetry.addLine("Webcam ready for start");
        telemetry.update();
        waitForStart();
        {
            telemetry.clear();
            runtime.reset();

            //Voltage regulation depending on the battery charge level
            double voltage = BatteryVoltage();
            double koeff = 13.0 / voltage;
            koeff = Math.pow(koeff, 1.25);
            imu.initialize(parameters);
            double time1;
            double time2;
            time1=getRuntime();
            time2=getRuntime();
            while(opModeIsActive()&&!isStopRequested()) {
                //m6Intake.setPower(-0.86);
                time2=getRuntime();
                telemetry.addData("count of rings", pipeline.position);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //telemetry.addData("Right sensor", DistanceSensor_right.getDistance(DistanceUnit.CM));
                //telemetry.addData("Left sensor", DistanceSensor_left.getDistance(DistanceUnit.CM));
                //telemetry.addData("Forward sensor", DistanceSensor_forward.getDistance(DistanceUnit.CM));
                telemetry.addData("angle1", angles.firstAngle);
                telemetry.addData("angle2", angles.secondAngle);
                telemetry.addData("angle3", angles.thirdAngle);
                telemetry.addData("avg1", pipeline.getAnalysis() );
                telemetry.update();
            }
        }
    }
}
