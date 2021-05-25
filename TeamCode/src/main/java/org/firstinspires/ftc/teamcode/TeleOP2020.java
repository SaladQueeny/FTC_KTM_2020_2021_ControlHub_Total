/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2021.
   @author Kolpakov Egor
*/


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.odometry.OdometryGlobalCoordinatePosition;



@TeleOp(name = "KTM TeleOp 2020", group = "Linear Opmode")

//@Disabled
public class TeleOP2020 extends LinearOpMode {
    private static final int LED_CHANNEL = 5;
    //    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //Cha
    private DcMotor m6Intake = null;
    private DcMotor m5Lift = null;
    private Servo s5Shovel = null;

    //-------
    double magic(double input) {
        if(Math.abs(input)<0.02){

            double nol=0;
            return nol;
        } else{
            return Math.signum(input) * (0.9* Math.pow(Math.abs(input), 2)+0.1);
        }
    }


    protected double BatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }


    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation,double allowableDistanceError, DcMotor right_back, DcMotor left_front, DcMotor left_back, DcMotor right_front){
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget,distanceToYTarget);
        double time1=getRuntime();
        double time2 = getRuntime();
        while(!isStopRequested()&&distance>allowableDistanceError&& Math.abs(time1-time2)<5){
            time2 = getRuntime();
            distance = Math.hypot(distanceToXTarget,distanceToYTarget);
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();


            double d1 = -pivotCorrection/17+robot_movement_y_component-robot_movement_x_component;
            double d2 = -pivotCorrection/30-robot_movement_y_component+robot_movement_x_component;
            double d3 = -pivotCorrection/30-robot_movement_y_component-robot_movement_x_component;
            double d4 = -pivotCorrection/17+robot_movement_y_component+robot_movement_x_component;
            double koeff = 0.5;
            setMotorsPowerOdom(d1,d2,d3,d4, right_back, left_front, left_back, right_front);
        }
        stopMovement(right_back, left_front, left_back, right_front);
    }






    protected void setMotorsPowerOdom(double D1_power, double D2_power, double D3_power, double D4_power, DcMotor right_back, DcMotor left_front, DcMotor left_back, DcMotor right_front) { //Warning: Р­С‚Р° С„СѓРЅРєС†РёСЏ РІРєР»СЋС‡РёС‚ РјРѕС‚РѕСЂС‹ РЅРѕ, РІС‹РєР»СЋС‡РёС‚СЊ РёС… РЅР°РґРѕ Р±СѓРґРµС‚ РїРѕСЃР»Рµ РІС‹РїРѕР»РЅРµРЅРёСЏ РєР°РєРѕРіРѕ Р»РёР±Рѕ СѓСЃР»РѕРІРёСЏ
        // Send power to wheels
        right_back.setPower(D1_power);
        left_front.setPower(D2_power);
        left_back.setPower(D3_power);
        right_front.setPower(D4_power);
    }
    protected void stopMovement(DcMotor right_back, DcMotor left_front, DcMotor left_back, DcMotor right_front){
        right_back.setPower(0);
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
    }
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

    /**
     * End of functions declaration
     */
    OdometryGlobalCoordinatePosition globalPositionUpdate;
    final double COUNTS_PER_INCH = 307.699557;
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Chassis
        DcMotor m1Drive = hardwareMap.get(DcMotor.class, "m1 drive");
        DcMotor m2Drive = hardwareMap.get(DcMotor.class, "m2 drive");
        DcMotor m3Drive = hardwareMap.get(DcMotor.class, "m3 drive");
        DcMotor m4Drive = hardwareMap.get(DcMotor.class, "m4 drive");
        DcMotor m5Lift = hardwareMap.get(DcMotor.class, "m5 lift");
        DcMotor m6Intake = hardwareMap.get(DcMotor.class, "m6 intake");
        DcMotor m7ruletka = hardwareMap.get(DcMotor.class, "m7 rul");
        Servo s1RelicExtRet = hardwareMap.get(Servo.class, "s1 top claw");
        //s2_bottom_Claw = hardwareMap.get(CRServo.class, "s2 bottom claw");
        Servo s3Rotation = hardwareMap.get(Servo.class, "s3 rotation");
        Servo s4Kicker = hardwareMap.get(Servo.class, "s4 kick");
        Servo s5Shovel = hardwareMap.get(Servo.class, "s5 shovel");
        Servo s6RelicClaw = hardwareMap.get(Servo.class, "s6 relic claw");
        Servo s7RelicArm = hardwareMap.get(Servo.class, "s7 relic arm");
        BNO055IMU imu;
        Orientation angles;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        m1Drive.setDirection(DcMotor.Direction.FORWARD);
        m2Drive.setDirection(DcMotor.Direction.FORWARD);
        m3Drive.setDirection(DcMotor.Direction.FORWARD);
        m4Drive.setDirection(DcMotor.Direction.FORWARD);
        m5Lift.setDirection(DcMotor.Direction.FORWARD);
        m7ruletka.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m6Intake.setDirection(DcMotor.Direction.REVERSE);
        m7ruletka.setDirection(DcMotorSimple.Direction.FORWARD);
        m1Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m5Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m6Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m7ruletka.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        runtime.reset();

        double m1DrivePower;
        double m2DrivePower;
        double m3DrivePower;
        double m4DrivePower;
        double m5LiftPower;
        double m6IntakePower=0;
        double m1m1m1=0;
        double m2m2m2=0;
        double m3m3m3=0;
        double m4m4m4=0;
        double m1m1=0;
        double m2m2=0;
        double m3m3=0;
        double m4m4=0;
        double m1left=0;
        double m1right=0;
        double m2left=0;
        double m2right=0;
        double m3left=0;
        double m3right=0;
        double m4left=0;
        double m4right=0;
        double m1DrivePowerforrotation=0;
        double m2DrivePowerforrotation=0;
        double m3DrivePowerforrotation=0;
        double m4DrivePowerforrotation=0;
        double m1DrivePowerfordrivetofoundation=0;
        double m2DrivePowerfordrivetofoundation=0;
        double m3DrivePowerfordrivetofoundation=0;
        double m4DrivePowerfordrivetofoundation=0;
        double m1DrivePowerfordrivetofoundation2=0;
        double m2DrivePowerfordrivetofoundation2=0;
        double m3DrivePowerfordrivetofoundation2=0;
        double m4DrivePowerfordrivetofoundation2=0;
        double m1DrivePowerfordrivetofoundation1=0;
        double m2DrivePowerfordrivetofoundation1=0;
        double m3DrivePowerfordrivetofoundation1=0;
        double m4DrivePowerfordrivetofoundation1=0;
        double m1DrivePowerfordrivetofoundation11=0;
        double m2DrivePowerfordrivetofoundation11=0;
        double m3DrivePowerfordrivetofoundation11=0;
        double m4DrivePowerfordrivetofoundation11=0;
        double a=0;
        double b=0;
        double prevangel=0;
        boolean shoot=false;
        String positionServo="not ready";

        s5Shovel.setPosition(0.19);
        s3Rotation.setPosition(0);
        double voltage = BatteryVoltage();
        double koeff = 13.0 / voltage;
        koeff = Math.pow(koeff, 1.25);
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(m1Drive, m2Drive, m4Drive, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
        globalPositionUpdate.reverseLeftEncoder();

        while (opModeIsActive()) {
            // POV Mode uses right stick to go forward and right to slide.
            // - This uses basic math to combine motions and is easier to drive straight.
            double driveL = -gamepad1.left_stick_y;
            double driveR = -gamepad1.right_stick_y;
            float relic = gamepad2.left_stick_x;
            boolean ruletka_tuda = gamepad2.dpad_up;
            boolean ruletka_suda = gamepad2.dpad_down;
            double zagrebalo = -0.7*gamepad2.left_stick_y;
            double podiem = 1*gamepad2.right_stick_y;
            double slideR = -0.7*gamepad1.left_trigger;
            double slideL = 0.7*gamepad1.right_trigger;
            double vpernazad=gamepad1.left_stick_y;
            double vleovpravo= -gamepad1.left_stick_x;
            double povorot= gamepad1.right_stick_x;

            //Slide Related
            slideL=magic(slideL);
            slideR=magic(slideR);
            povorot=magic(povorot);
            vpernazad=magic(vpernazad);
            //vleovpravo=magic(vleovpravo);
                /*m2DrivePower = povorot-vpernazad-vleovpravo;
                m4DrivePower = povorot+vpernazad-vleovpravo;
                m1DrivePower = povorot+vpernazad+vleovpravo;
                m3DrivePower = povorot-vpernazad+vleovpravo;*/
            m2DrivePower = (m2left+m2right+m1m1+m1m1m1+m1DrivePowerfordrivetofoundation+m1DrivePowerfordrivetofoundation2+m1DrivePowerfordrivetofoundation1+m1DrivePowerfordrivetofoundation11) + povorot-vpernazad+(slideL+slideR)-(vleovpravo);
            m4DrivePower = (m4left+m4right+m2m2+m2m2m2+m2DrivePowerfordrivetofoundation+m2DrivePowerfordrivetofoundation2+m2DrivePowerfordrivetofoundation1+m2DrivePowerfordrivetofoundation11)+ povorot+vpernazad+(slideL+slideR)-(vleovpravo);
            m1DrivePower = (m1left+m1right+m3m3+m3m3m3+m3DrivePowerfordrivetofoundation+m3DrivePowerfordrivetofoundation2+m3DrivePowerfordrivetofoundation1+m3DrivePowerfordrivetofoundation11)+povorot+vpernazad+(slideL+slideR)+(vleovpravo);
            m3DrivePower = (m3left+m3right+m4m4+m4m4m4+m4DrivePowerfordrivetofoundation+m4DrivePowerfordrivetofoundation2+m4DrivePowerfordrivetofoundation1+m4DrivePowerfordrivetofoundation11)+ povorot-vpernazad+(slideL+slideR)+(vleovpravo);
            double mochs=1;
            double max = Math.max(Math.max(m1DrivePower, m2DrivePower), Math.max(m3DrivePower, m4DrivePower));
            // Send calculated power to wheelsв
            if (max >= 1) {
                m1Drive.setPower(mochs*m1DrivePower *1/ max);
                m2Drive.setPower(mochs*m2DrivePower *1/ max);
                m3Drive.setPower(mochs*m3DrivePower *1/ max);
                m4Drive.setPower(mochs*m4DrivePower *1/ max);
            } else {
                m1Drive.setPower(mochs*m1DrivePower*1);
                m2Drive.setPower(mochs*m2DrivePower*1);
                m3Drive.setPower(mochs*m3DrivePower*1);
                m4Drive.setPower(mochs*m4DrivePower*1);
            }


            /*
             * End of chassis related code.
             */


            if(podiem!=0&&a==1){
                if(podiem>0) {
                    m5Lift.setPower(podiem);
                }else{
                    m5Lift.setPower(podiem);
                }
            }else{m5Lift.setPower(0);}

                if (zagrebalo != 0) {
                    if (zagrebalo > 0) {
                        m7ruletka.setPower(zagrebalo);
                    } else {
                        m7ruletka.setPower(zagrebalo);
                    }
                } else {
                    m7ruletka.setPower(0);
                }
           // }

            if(gamepad2.right_bumper){
                s1RelicExtRet.setPosition(1);
            }
            if(gamepad2.left_bumper){
                s1RelicExtRet.setPosition(0);
            }

            //подъём магазина для катапульты
            if (gamepad2.y) { s4Kicker.setPosition(1);
            a=0;

            }
            if (gamepad2.a) { s4Kicker.setPosition(0);
            a=1;

            }

            //стрельба катапульты
            if(gamepad2.b){

                m1Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m2Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m4Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m1Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m2Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m4Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                globalPositionUpdate.stop();
                globalPositionUpdate = new OdometryGlobalCoordinatePosition(m1Drive, m2Drive, m4Drive, COUNTS_PER_INCH, 60);
                positionThread = new Thread(globalPositionUpdate);
                positionThread.start();
                globalPositionUpdate.reverseLeftEncoder();

                m1Drive.setPower(0);
                m2Drive.setPower(0);
                m3Drive.setPower(0);
                m4Drive.setPower(0);

                s5Shovel.setPosition(0);
                sleep(400);
                s5Shovel.setPosition(0.19);
                sleep(300);
            }



//----------------------------------------

            if(gamepad2.dpad_left){
                m6Intake.setPower(koeff*0.79);
            }else if(gamepad2.dpad_right||shoot){
                m6Intake.setPower(koeff*0.68);
            }else{
                m6Intake.setPower(0);
            }

            if(gamepad1.a){
                //imu.initialize(parameters);


                m1Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m2Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m4Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m1Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m2Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m4Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                globalPositionUpdate.stop();
                globalPositionUpdate = new OdometryGlobalCoordinatePosition(m1Drive, m2Drive, m4Drive, COUNTS_PER_INCH, 60);
                positionThread = new Thread(globalPositionUpdate);
                positionThread.start();
                globalPositionUpdate.reverseLeftEncoder();

            }
            if(gamepad1.b){

                //reset odometry

                m1Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m2Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m4Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m1Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m2Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m4Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                globalPositionUpdate.stop();
                globalPositionUpdate = new OdometryGlobalCoordinatePosition(m1Drive, m2Drive, m4Drive, COUNTS_PER_INCH, 60);
                positionThread = new Thread(globalPositionUpdate);
                positionThread.start();
                globalPositionUpdate.reverseLeftEncoder();

                m6Intake.setPower(koeff*0.69);
                s4Kicker.setPosition(1);

                globalPositionUpdate.returnOrientation();
                goToPosition(-120* COUNTS_PER_INCH, 0*COUNTS_PER_INCH,0.7*koeff,0,8*COUNTS_PER_INCH, m1Drive,m2Drive,m3Drive,m4Drive);
                goToPosition(-157* COUNTS_PER_INCH, 0*COUNTS_PER_INCH,0.18*koeff,-1.5,1*COUNTS_PER_INCH, m1Drive,m2Drive,m3Drive,m4Drive);
                sleep(200);

               while(!isStopRequested()&&globalPositionUpdate.returnOrientation()<1.5){
                  m1Drive.setPower(-0.15);
                  m2Drive.setPower(-0.15);
                  m3Drive.setPower(-0.15);
                  m4Drive.setPower(-0.15);

                }
               m1Drive.setPower(0);
               m2Drive.setPower(0);
               m3Drive.setPower(0);
               m4Drive.setPower(0);

                sleep(300);
                s5Shovel.setPosition(0);
                sleep(400);
                s5Shovel.setPosition(0.19);

               while(!isStopRequested()&&globalPositionUpdate.returnOrientation()<7){
                   m1Drive.setPower(-0.15);
                   m2Drive.setPower(-0.15);
                   m3Drive.setPower(-0.15);
                   m4Drive.setPower(-0.15);

               }
               m1Drive.setPower(0);
               m2Drive.setPower(0);
               m3Drive.setPower(0);
               m4Drive.setPower(0);

                sleep(300);
                s5Shovel.setPosition(0);
                sleep(400);
                s5Shovel.setPosition(0.19);

               while(!isStopRequested()&&globalPositionUpdate.returnOrientation()<12){
                   m1Drive.setPower(-0.15);
                   m2Drive.setPower(-0.15);
                   m3Drive.setPower(-0.15);
                   m4Drive.setPower(-0.15);
                   //angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
               m1Drive.setPower(0);
               m2Drive.setPower(0);
               m3Drive.setPower(0);
               m4Drive.setPower(0);

                sleep(300);
                s5Shovel.setPosition(0);
                sleep(400);
                s5Shovel.setPosition(0.19);

                shoot=false;

            }
            if(gamepad1.x){

                m1Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m2Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m4Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m1Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m2Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m4Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                globalPositionUpdate.stop();
                globalPositionUpdate = new OdometryGlobalCoordinatePosition(m1Drive, m2Drive, m4Drive, COUNTS_PER_INCH, 60);
                positionThread = new Thread(globalPositionUpdate);
                positionThread.start();
                globalPositionUpdate.reverseLeftEncoder();

                m6Intake.setPower(koeff*0.7);
                s4Kicker.setPosition(1);

                goToPosition(160* COUNTS_PER_INCH, 0*COUNTS_PER_INCH,0.7*koeff,0,8*COUNTS_PER_INCH, m1Drive,m2Drive,m3Drive,m4Drive);
                goToPosition(193* COUNTS_PER_INCH, 0*COUNTS_PER_INCH,0.18*koeff,3,1*COUNTS_PER_INCH, m1Drive,m2Drive,m3Drive,m4Drive);
                sleep(400);


                while(!isStopRequested()&&globalPositionUpdate.returnOrientation()>-1.5){
                    m1Drive.setPower(0.15);
                    m2Drive.setPower(0.15);
                    m3Drive.setPower(0.15);
                    m4Drive.setPower(0.15);

                }
                m1Drive.setPower(0);
                m2Drive.setPower(0);
                m3Drive.setPower(0);
                m4Drive.setPower(0);

                s5Shovel.setPosition(0);
                sleep(400);
                s5Shovel.setPosition(0.19);
                sleep(200);


                while(!isStopRequested()&&globalPositionUpdate.returnOrientation()>-7){
                    m1Drive.setPower(0.15);
                    m2Drive.setPower(0.15);
                    m3Drive.setPower(0.15);
                    m4Drive.setPower(0.15);

                }
                m1Drive.setPower(0);
                m2Drive.setPower(0);
                m3Drive.setPower(0);
                m4Drive.setPower(0);

                s5Shovel.setPosition(0);
                sleep(400);
                s5Shovel.setPosition(0.19);
                sleep(200);


                while(!isStopRequested()&&globalPositionUpdate.returnOrientation()>-13){
                    m1Drive.setPower(0.15);
                    m2Drive.setPower(0.15);
                    m3Drive.setPower(0.15);
                    m4Drive.setPower(0.15);

                }
                m1Drive.setPower(0);
                m2Drive.setPower(0);
                m3Drive.setPower(0);
                m4Drive.setPower(0);

                s5Shovel.setPosition(0);
                sleep(400);
                s5Shovel.setPosition(0.19);
                sleep(200);
                shoot=false;

            }
            if(gamepad2.x){

                m6Intake.setPower(koeff*0.75);
                s4Kicker.setPosition(1);
                s3Rotation.setPosition(0);
                goToPosition(0* COUNTS_PER_INCH, 0*COUNTS_PER_INCH,0.8*koeff,0,10*COUNTS_PER_INCH, m1Drive,m2Drive,m3Drive,m4Drive);

            }

            if(gamepad1.right_bumper){
                shoot=false;
            }


//----------------------------------------
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
            if(gamepad1.dpad_up){
                m1DrivePowerfordrivetofoundation11=0.3;
                m2DrivePowerfordrivetofoundation11=-0.3;//мотор енкодера с минусом
                m3DrivePowerfordrivetofoundation11=-0.3;//мотор енкодера с минусом
                m4DrivePowerfordrivetofoundation11=0.3;
            }else{
                m1DrivePowerfordrivetofoundation11=0;
                m2DrivePowerfordrivetofoundation11=0;
                m3DrivePowerfordrivetofoundation11=0;
                m4DrivePowerfordrivetofoundation11=0;
            }
            if(gamepad1.dpad_down){
                m1DrivePowerfordrivetofoundation1=-0.3;
                m2DrivePowerfordrivetofoundation1=0.3;
                m3DrivePowerfordrivetofoundation1=0.3;
                m4DrivePowerfordrivetofoundation1=-0.3;
            }else{
                m1DrivePowerfordrivetofoundation1=0;
                m2DrivePowerfordrivetofoundation1=0;
                m3DrivePowerfordrivetofoundation1=0;
                m4DrivePowerfordrivetofoundation1=0;
            }
            if(gamepad1.dpad_left){
                m1DrivePowerfordrivetofoundation=-0.33;
                m2DrivePowerfordrivetofoundation=-0.33;
                m3DrivePowerfordrivetofoundation=0.33;
                m4DrivePowerfordrivetofoundation=0.33;
            }else{
                m1DrivePowerfordrivetofoundation=0;
                m2DrivePowerfordrivetofoundation=0;
                m3DrivePowerfordrivetofoundation=0;
                m4DrivePowerfordrivetofoundation=0;
            }
            if(gamepad1.dpad_right){
                m1DrivePowerfordrivetofoundation2=0.33;
                m2DrivePowerfordrivetofoundation2=0.33;
                m3DrivePowerfordrivetofoundation2=-0.33;
                m4DrivePowerfordrivetofoundation2=-0.33;
            }else{
                m1DrivePowerfordrivetofoundation2=0;
                m2DrivePowerfordrivetofoundation2=0;
                m3DrivePowerfordrivetofoundation2=0;
                m4DrivePowerfordrivetofoundation2=0;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("SErvo status", positionServo);
            telemetry.addData("SErvo position", s5Shovel.getPosition());
            telemetry.addData("Podiem position ", m7ruletka.getCurrentPosition());

            telemetry.addData("angleofrotate", angles.firstAngle);

            //telemetry.addData("Distance left: ", DistanceSensor_left.getDistance(DistanceUnit.CM));
            //telemetry.addData("Distance right: ", DistanceSensor_right.getDistance(DistanceUnit.CM));
            telemetry.addData("Motors", "m1Drive (%.2f), m2Drive (%.1f), m3Drive (%.2f), m4Drive (%.2f)", m1DrivePower, m2DrivePower, m3DrivePower, m4DrivePower);
            telemetry.addData("Motors power for rotation", "m1Drive (%.2f), m2Drive (%.1f), m3Drive (%.2f), m4Drive (%.2f)", m1DrivePowerforrotation, m2DrivePowerforrotation, m3DrivePowerforrotation, m4DrivePowerforrotation);

            telemetry.addData("current x position", globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH);
            telemetry.addData("current y position", globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH);
            telemetry.addData("current odometry angle", globalPositionUpdate.returnOrientation());
            telemetry.addData("Vertical left encoder position", m1Drive.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", m2Drive.getCurrentPosition());
            telemetry.addData("horizontal encoder position", m4Drive.getCurrentPosition());
            telemetry.update();


            if(gamepad2.dpad_up) { s3Rotation.setPosition(0);}
            if(gamepad2.dpad_down) { s3Rotation.setPosition(1);}

        }


        globalPositionUpdate.stop();

    }


}
