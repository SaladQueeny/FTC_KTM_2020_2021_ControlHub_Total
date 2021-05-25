/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2021.
   @author Kolpakov Egor
*/

package org.firstinspires.ftc.teamcode.AutoOPs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision;
@Autonomous(name = "LeftCenter", group = "AutoOP")
public class      LeftCenter extends Robot
{
    OpenCvCamera webcam;

    org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVisionL pipeline;
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 307.699557;

    String rfName = "m4 drive", rbName = "m1 drive", lfName = "m2 drive", lbName = "m3 drive";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    @Override
    public void runOpMode()
    {
        initHW(hardwareMap);
        BNO055IMU imu;
        Orientation angles;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision pipeline = new org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        sleep(100);


        telemetry.addLine("Webcam ready for start");
        telemetry.update();
        s1TopClaw.setPosition(0);
        s4Kicker.setPosition(1);
        double voltage = BatteryVoltage();
        double koeff = 13.0 / voltage;
        koeff = Math.pow(koeff, 1.25);
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 50);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
        globalPositionUpdate.reverseLeftEncoder();
        waitForStart();
        {
            ElapsedTime timer = new ElapsedTime();
            webcam.stopStreaming();
            imu.initialize(parameters);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.clear();
            telemetry.addData("Number of rings ", pipeline.position);
            telemetry.update();
            int countOfRings=12;
            if((pipeline.position == org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision.RingPosition.FOUR)){
                countOfRings = 4;
            }
            if((pipeline.position == org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision.RingPosition.ONE)){
                countOfRings = 1;
            }
            if((pipeline.position == org.firstinspires.ftc.teamcode.Vision.EasyOpenCVVision.RingPosition.NONE)){
                countOfRings = 0;
            }
            //Voltage regulation depending on the battery charge level
            telemetry.addData("count_of_rings ", countOfRings);
            telemetry.update();
            boolean check= true;

            //shooting targets
/*

            Shooting(koeff,0.68);
            goToPosition(-10* COUNTS_PER_INCH, -170*COUNTS_PER_INCH,0.5*koeff,0,7*COUNTS_PER_INCH);
            goToPosition(-10* COUNTS_PER_INCH, -227*COUNTS_PER_INCH,0.3*koeff,0,2*COUNTS_PER_INCH);
            Shoot();

            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            while(!isStopRequested()&&angles.firstAngle>-5.5){
                setMotorsPower(0.15*koeff,0.15*koeff,0.15*koeff,0.15*koeff);
                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            setMotorsPower(0,0,0,0);
            Shooting(koeff,0.7);
            Shoot();

            while(!isStopRequested()&&angles.firstAngle>-11){
                setMotorsPower(0.15*koeff,0.15*koeff,0.15*koeff,0.15*koeff);
                angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
            setMotorsPower(0,0,0,0);
            Shoot();

            endShooting();
            s3Rotation.setPosition(1);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                while (!isStopRequested() && angles.firstAngle < -5) {
                    setMotorsPower(-0.3 * koeff, -0.3 * koeff, -0.3 * koeff, -0.3 * koeff);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                setMotorsPower(0, 0, 0, 0);
                s3Rotation.setPosition(1);
            s4Kicker.setPosition(0);

*/





            if(countOfRings==4&&check) {
                check=false;

                //shooting high gates

                Shooting(koeff,0.76);
                goToPosition(29* COUNTS_PER_INCH, -160*COUNTS_PER_INCH,0.5*koeff,0,12*COUNTS_PER_INCH);
                goToPosition(29* COUNTS_PER_INCH, -207*COUNTS_PER_INCH,0.2*koeff,8,2*COUNTS_PER_INCH);

                sleep(500);

                Shooting(koeff, 0.78);
                Shoot();
                sleep(50);
                Shoot();
                sleep(50);
                Shoot();
                s4Kicker.setPosition(0);
                endShooting();

                s3Rotation.setPosition(1);


                //go to rings

                goToPosition(5 * COUNTS_PER_INCH, -130 * COUNTS_PER_INCH, 0.6 * koeff, 0, 13 * COUNTS_PER_INCH);
                goToPosition(-45 * COUNTS_PER_INCH, -103 * COUNTS_PER_INCH, 0.25 * koeff, 0, 2 * COUNTS_PER_INCH);


                //collecting rings

                setMotorsPower(-0.7,0.7,0.7,-0.7);
                m5Lift.setPower(-1);
                sleep(200);
                setMotorsPower(0,0,0,0);
                sleep(200);

                setMotorsPower(-0.2,0.2,0.2,-0.2);
                sleep(550);
                setMotorsPower(0,0,0,0);
                sleep(200);

                setMotorsPower(-0.2,0.2,0.2,-0.2);
                sleep(550);
                setMotorsPower(0,0,0,0);
                sleep(1100);

                s4Kicker.setPosition(1);
                sleep(500);
                m5Lift.setPower(0);


                //shooting high gates

                Shooting(koeff, 0.76);
                sleep(500);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                while (!isStopRequested() && angles.firstAngle > -2) {
                    setMotorsPower(0.12 * koeff, 0.18 * koeff, 0.12 * koeff, 0.18 * koeff);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                setMotorsPower(0, 0, 0, 0);

                sleep(500);
                Shooting(koeff, 0.78);
                Shoot();
                sleep(50);
                Shoot();
                sleep(50);
                Shoot();


                //collecting rings2

                s4Kicker.setPosition(0);
                m5Lift.setPower(-1);
                sleep(600);

                setMotorsPower(-0.25,0.25,0.25,-0.25);
                sleep(1000);
                setMotorsPower(0,0,0,0);
                sleep(1200);

                s4Kicker.setPosition(1);
                sleep(800);
                m5Lift.setPower(0);

                Shooting(koeff, 0.77);
                Shoot();
                sleep(50);
                Shoot();
                sleep(50);
                s4Kicker.setPosition(0);
                endShooting();


                //go to target zone

                goToPosition(-45* COUNTS_PER_INCH,-390*COUNTS_PER_INCH,0.6*koeff,0,15*COUNTS_PER_INCH);
                s3Rotation.setPosition(0);
                goToPosition(-129* COUNTS_PER_INCH,-455*COUNTS_PER_INCH,0.3*koeff,0,3*COUNTS_PER_INCH);

                otpustivobl(koeff);

                goToPosition(-38 * COUNTS_PER_INCH, -430 * COUNTS_PER_INCH, 0.5*koeff, 0, 15* COUNTS_PER_INCH);
                goToPosition(30* COUNTS_PER_INCH, -305 * COUNTS_PER_INCH, 0.4*koeff, 0, 4* COUNTS_PER_INCH);

            }



            if(countOfRings==1&&check){//добавить координату
                check=false;


                //shooting high gates

                Shooting(koeff,0.75);
                goToPosition(29* COUNTS_PER_INCH, -160*COUNTS_PER_INCH,0.5*koeff,0,12*COUNTS_PER_INCH);
                goToPosition(29* COUNTS_PER_INCH, -207*COUNTS_PER_INCH,0.2*koeff,8,2*COUNTS_PER_INCH);

                sleep(500);

                Shooting(koeff, 0.76);
                Shoot();
                sleep(50);
                Shoot();
                sleep(50);
                Shoot();
                s4Kicker.setPosition(0);
                endShooting();

                s3Rotation.setPosition(1);


                /*targets*/
                /*
                Shooting(koeff,0.65);
                goToPosition(32* COUNTS_PER_INCH, -190*COUNTS_PER_INCH,0.5*koeff,0,12*COUNTS_PER_INCH);
                goToPosition(32* COUNTS_PER_INCH, -255*COUNTS_PER_INCH,0.15*koeff,8,1.5*COUNTS_PER_INCH);

                while(!isStopRequested()&&globalPositionUpdate.returnOrientation()>-1.1){
                    m1Drive.setPower(0.15);
                    m2Drive.setPower(0.15);
                    m3Drive.setPower(0.15);
                    m4Drive.setPower(0.15);

                }
                m1Drive.setPower(0);
                m2Drive.setPower(0);
                m3Drive.setPower(0);
                m4Drive.setPower(0);

                sleep(400);
                s5Shovel.setPosition(0);
                sleep(400);
                s5Shovel.setPosition(0.19);
                sleep(200);


                while(!isStopRequested()&&globalPositionUpdate.returnOrientation()>-6.4){
                    m1Drive.setPower(0.15);
                    m2Drive.setPower(0.15);
                    m3Drive.setPower(0.15);
                    m4Drive.setPower(0.15);

                }
                m1Drive.setPower(0);
                m2Drive.setPower(0);
                m3Drive.setPower(0);
                m4Drive.setPower(0);

                sleep(400);
                s5Shovel.setPosition(0);
                sleep(400);
                s5Shovel.setPosition(0.19);
                sleep(200);


                while(!isStopRequested()&&globalPositionUpdate.returnOrientation()>-12){
                    m1Drive.setPower(0.15);
                    m2Drive.setPower(0.15);
                    m3Drive.setPower(0.15);
                    m4Drive.setPower(0.15);

                }
                m1Drive.setPower(0);
                m2Drive.setPower(0);
                m3Drive.setPower(0);
                m4Drive.setPower(0);

                sleep(400);
                s5Shovel.setPosition(0);
                sleep(400);
                s5Shovel.setPosition(0.19);
                sleep(200);
                endShooting();


                s4Kicker.setPosition(0);
*/
                //go to rings

                m5Lift.setPower(-1);

                goToPosition(5 * COUNTS_PER_INCH, -130 * COUNTS_PER_INCH, 0.6 * koeff, 0, 13 * COUNTS_PER_INCH);
                goToPosition(-48 * COUNTS_PER_INCH, -103 * COUNTS_PER_INCH, 0.2 * koeff, 0, 2 * COUNTS_PER_INCH);
                goToPosition(-44 * COUNTS_PER_INCH, -180 * COUNTS_PER_INCH, 0.15 * koeff, 0, 2 * COUNTS_PER_INCH);

                s4Kicker.setPosition(1);
                Shooting(koeff, 0.74);
                sleep(200);
                m5Lift.setPower(0);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                while (!isStopRequested() && angles.firstAngle > -2) {
                    setMotorsPower(0.12 * koeff, 0.18 * koeff, 0.12 * koeff, 0.18 * koeff);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                setMotorsPower(0, 0, 0, 0);

                sleep(1200);
                Shooting(koeff, 0.76);
                Shoot();
                sleep(50);
                Shoot();
                endShooting();
                s4Kicker.setPosition(0);

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                while (!isStopRequested() && angles.firstAngle < -5) {
                    setMotorsPower(-0.3 * koeff, -0.3 * koeff, -0.3 * koeff, -0.3 * koeff);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                setMotorsPower(0, 0, 0, 0);

                goToPosition(-23* COUNTS_PER_INCH,-350*COUNTS_PER_INCH,0.3*koeff,0,2*COUNTS_PER_INCH);
                s3Rotation.setPosition(0);
                otpustivobl(koeff);

                goToPosition(30 * COUNTS_PER_INCH, -300 * COUNTS_PER_INCH, 0.3*koeff, 0, 3 * COUNTS_PER_INCH);

            }




            if(countOfRings==0&&check){//добавить координату
                check=false;


                //shooting targets

                Shooting(koeff,0.68);
                goToPosition(32* COUNTS_PER_INCH, -190*COUNTS_PER_INCH,0.5*koeff,0,12*COUNTS_PER_INCH);
                goToPosition(32* COUNTS_PER_INCH, -255*COUNTS_PER_INCH,0.15*koeff,8,1.5*COUNTS_PER_INCH);

                while(!isStopRequested()&&globalPositionUpdate.returnOrientation()>-1.3){
                    m1Drive.setPower(0.15);
                    m2Drive.setPower(0.15);
                    m3Drive.setPower(0.15);
                    m4Drive.setPower(0.15);

                }
                m1Drive.setPower(0);
                m2Drive.setPower(0);
                m3Drive.setPower(0);
                m4Drive.setPower(0);

                sleep(400);
                s5Shovel.setPosition(0);
                sleep(400);
                s5Shovel.setPosition(0.19);
                sleep(200);


                while(!isStopRequested()&&globalPositionUpdate.returnOrientation()>-6.8){
                    m1Drive.setPower(0.15);
                    m2Drive.setPower(0.15);
                    m3Drive.setPower(0.15);
                    m4Drive.setPower(0.15);

                }
                m1Drive.setPower(0);
                m2Drive.setPower(0);
                m3Drive.setPower(0);
                m4Drive.setPower(0);

                sleep(400);
                s5Shovel.setPosition(0);
                sleep(400);
                s5Shovel.setPosition(0.19);
                sleep(200);


                while(!isStopRequested()&&globalPositionUpdate.returnOrientation()>-12.2){
                    m1Drive.setPower(0.15);
                    m2Drive.setPower(0.15);
                    m3Drive.setPower(0.15);
                    m4Drive.setPower(0.15);

                }
                m1Drive.setPower(0);
                m2Drive.setPower(0);
                m3Drive.setPower(0);
                m4Drive.setPower(0);

                sleep(400);
                s5Shovel.setPosition(0);
                sleep(400);
                s5Shovel.setPosition(0.19);
                sleep(200);
                endShooting();

                s3Rotation.setPosition(1);

                /* wobble */
                goToPosition(-63* COUNTS_PER_INCH,-275*COUNTS_PER_INCH,0.5*koeff,0,7*COUNTS_PER_INCH);
                goToPosition(-117* COUNTS_PER_INCH,-275*COUNTS_PER_INCH,0.3*koeff,0,2*COUNTS_PER_INCH);

                otpustivobl(koeff);


                //go for dropped rings

                s4Kicker.setPosition(0);
                m5Lift.setPower(-1);

                goToPosition(-63* COUNTS_PER_INCH,-260*COUNTS_PER_INCH,0.45*koeff,0,15*COUNTS_PER_INCH);
                goToPosition(-100* COUNTS_PER_INCH,-450*COUNTS_PER_INCH,0.5*koeff,0,15*COUNTS_PER_INCH);
                goToPosition(-100* COUNTS_PER_INCH,-515*COUNTS_PER_INCH,0.25*koeff,0,5*COUNTS_PER_INCH);

                while(!isStopRequested()&&globalPositionUpdate.returnOrientation()>-69){
                    m1Drive.setPower(0.25);
                    m2Drive.setPower(0.25);
                    m3Drive.setPower(0.25);
                    m4Drive.setPower(0.25);

                }

                sleep(150);
                setMotorsPower(-0.3,0.3,0.3,-0.3);
                sleep(2300);
                setMotorsPower(0,0,-0.5,0.5);
                sleep(1500);

                s4Kicker.setPosition(1);
                m5Lift.setPower(0);
                s3Rotation.setPosition(0);

                Shooting(koeff,0.75);
                goToPosition(29* COUNTS_PER_INCH, -240*COUNTS_PER_INCH,0.5*koeff,0,12*COUNTS_PER_INCH);
                goToPosition(29* COUNTS_PER_INCH, -207*COUNTS_PER_INCH,0.2*koeff,8,2*COUNTS_PER_INCH);

                sleep(500);

                Shooting(koeff, 0.76);
                Shoot();
                sleep(50);
                Shoot();
                sleep(50);
                Shoot();
                s4Kicker.setPosition(0);
                endShooting();

                /*

                goToPosition(-63* COUNTS_PER_INCH,-275*COUNTS_PER_INCH,0.5*koeff,0,7*COUNTS_PER_INCH);
                goToPosition(-117* COUNTS_PER_INCH,-275*COUNTS_PER_INCH,0.3*koeff,0,2*COUNTS_PER_INCH);

                s3Rotation.setPosition(0);
                otpustivobl(koeff);
                */

                /* parking */
                goToPosition(30 * COUNTS_PER_INCH, -300 * COUNTS_PER_INCH, 0.3*koeff, 0, 3 * COUNTS_PER_INCH);

            }











            globalPositionUpdate.stop();

        }

    }
    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation,double allowableDistanceError){
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget,distanceToYTarget);

        while(!isStopRequested()&&distance>allowableDistanceError){

            distance = Math.hypot(distanceToXTarget,distanceToYTarget);
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
            double d1 = -pivotCorrection/50+robot_movement_y_component-robot_movement_x_component;
            double d2 = -pivotCorrection/50-robot_movement_y_component+robot_movement_x_component;
            double d3 = -pivotCorrection/50-robot_movement_y_component-robot_movement_x_component;
            double d4 = -pivotCorrection/50+robot_movement_y_component+robot_movement_x_component;
            double koeff = 0.5;
            setMotorsPowerOdom(d1,d2,d3,d4);
//            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
//            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
//            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
//            telemetry.addData("robot_movement_x_component", robot_movement_x_component);
//            telemetry.addData("robot_movement_y_component", robot_movement_y_component);
//            telemetry.addData("pivot", pivotCorrection);
//            telemetry.addData("motor1_power", d1);
//            telemetry.addData("motor2_power", d2);
//            telemetry.addData("motor3_power", d3);
//            telemetry.addData("motor4_power", d4);
//            telemetry.update();
        }
        stopMovement();
    }
    protected void setMotorsPowerOdom(double D1_power, double D2_power, double D3_power, double D4_power) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        right_back.setPower(D1_power);
        left_front.setPower(D2_power);
        left_back.setPower(D3_power);
        right_front.setPower(D4_power);
    }
    protected void stopMovement(){
        right_back.setPower(0);
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //left_front.setDirection(DcMotor.Direction.FORWARD);
        //right_front.setDirection(DcMotor.Direction.REVERSE);
        //right_back.setDirection(DcMotor.Direction.REVERSE);
        //left_back.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
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