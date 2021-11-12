package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Config
@TeleOp
public class Teleop_2021 extends LinearOpMode {

    Hardware robot = new Hardware();

    boolean intakeOut                       = false;
    boolean intakeToggle                    = true;
    boolean freightInIntake                 = false;
    boolean freightInClaw                   = false;
    boolean clawToggle                      = true;
    boolean clawClosed                      = false;

    public int liftPosition                  = 0;
    public int level1                       = -300;
    public int level2                       = 1350;
    public int level3                       = 950;
    public int currentPosition              = 0;

    public static double liftSpeed          = 0.8;
    public static double liftP              = 3.5;
    public static int liftTolerance         = 200;

    public double clawDistance              = 0;
    public double intakeDistance            = 0;
    public double forwardSpeed              = 0;
    public double turnSpeed                 = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for(LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        ElapsedTime clawTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        clawTimer.reset();

        ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        intakeTimer.reset();

        //Motor lift        = new Motor(hardwareMap, "lift", Motor.GoBILDA.RPM_117);
        Motor intake      = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "lift");

        Motor rightFront  = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_1150);
        Motor rightMiddle = new Motor(hardwareMap, "rightMiddle", Motor.GoBILDA.RPM_1150);
        Motor rightBack   = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_1150);
        Motor leftFront   = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_1150);
        Motor leftMiddle  = new Motor(hardwareMap, "leftMiddle", Motor.GoBILDA.RPM_1150);
        Motor leftBack    = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_1150);

        rightMiddle.setInverted(true);
        leftMiddle.setInverted(true);


        MotorGroup rightMotors = new MotorGroup(rightFront, rightMiddle, rightBack);
        MotorGroup leftMotors = new MotorGroup(leftFront, leftMiddle, leftBack);

        DifferentialDrive DT = new DifferentialDrive(leftMotors, rightMotors);


        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPositionPIDFCoefficients(liftP);
        lift.setTargetPositionTolerance(liftTolerance);

        Servo intakeRight       = hardwareMap.get(Servo.class, "intakeRight");
        Servo intakeLeft        = hardwareMap.get(Servo.class, "intakeLeft");
        Servo claw              = hardwareMap.get(Servo.class, "claw");
        Servo clawAngleRight    = hardwareMap.get(Servo.class, "clawAngleRight");
        Servo clawAngleLeft     = hardwareMap.get(Servo.class, "clawAngleLeft");

        CRServo duckRight       = hardwareMap.get(CRServo.class, "duckRight");
        CRServo duckLeft       = hardwareMap.get(CRServo.class, "duckLeft");

        RevColorSensorV3 intakeSensor = hardwareMap.get(RevColorSensorV3.class, "intakeSensor");
        RevColorSensorV3 clawSensor = hardwareMap.get(RevColorSensorV3.class, "clawSensor");

        intakeSensor.initialize();
        clawSensor.initialize();

        waitForStart();
        lift.setTargetPosition(0);

        while(opModeIsActive()) {  
            liftPosition    = lift.getCurrentPosition();
            intakeDistance  = intakeSensor.getDistance(DistanceUnit.MM);

            //dt control
            if(gamepad1.dpad_right){
                turnSpeed = 0.25;
            }
            else if(gamepad1.dpad_left){
                turnSpeed = 0.25;
            }
            else{
                turnSpeed = 0.66*gamepad1.left_stick_x;
            }

            if(gamepad1.dpad_down){
                forwardSpeed = -0.25;
            }
            else if(gamepad1.dpad_up){
                forwardSpeed = 0.25;
            }
            else{
                forwardSpeed = -gamepad1.right_stick_y;
            }

            DT.arcadeDrive(forwardSpeed, turnSpeed);




            //intake control
            if(gamepad1.right_bumper){
                intake.set(1);
            }
            else if(gamepad1.left_bumper){
                intake.set(-1);
            }
            else{
                intake.set(0);
            }



            //intake angle control
            if(intakeDistance<10){
                freightInIntake = true;
                intakeOut = false;
            }

            if(gamepad1.right_bumper && gamepad1.left_bumper && intakeToggle){
                intakeToggle = false;
                intakeOut = !intakeOut;
            }
            else if((!gamepad1.right_bumper || !gamepad1.left_bumper) && !intakeToggle){
                intakeToggle = true;
            }

            if(!intakeOut){
                intakeLeft.setPosition(0);
                intakeRight.setPosition(1);
            }
            else{
                intakeLeft.setPosition(0.7);
                intakeRight.setPosition(0.3);
            }


            //arm control
            if(gamepad2.dpad_left){
                lift.setTargetPosition(level1);
                currentPosition = 1;

            }
            else if(gamepad2.dpad_up){
                lift.setTargetPosition(level2);
                currentPosition = 2;
            }
            else if(gamepad2.dpad_right){
                lift.setTargetPosition(level3);
                currentPosition = 3;
            }
            else if(gamepad2.dpad_down){
                lift.setTargetPosition(0);
                currentPosition = 0;
            }

            lift.setPower(liftSpeed);



            //claw angle control
            if(gamepad2.x){
                clawDistance = 0;
            }
            else{
                clawDistance = clawSensor.getDistance(DistanceUnit.MM);
            }

            if(clawDistance>50 && !freightInClaw){
                clawTimer.reset();
            }

            if(liftPosition > lift.getTargetPosition()-liftTolerance & liftPosition< lift.getTargetPosition()+liftTolerance){
                if(currentPosition == 0 && !freightInClaw){
                    clawAngleLeft.setPosition(1);
                    clawAngleRight.setPosition(0);
                }
                else if(currentPosition == 0 && freightInClaw && clawTimer.time()>750){
                    clawAngleLeft.setPosition(0.6);
                    clawAngleRight.setPosition(0.4);
                }
                else if(currentPosition == 1){
                    clawAngleLeft.setPosition(0.3);
                    clawAngleRight.setPosition(0.7);
                }
                else if(currentPosition == 2){
                    clawAngleLeft.setPosition(0.3);
                    clawAngleRight.setPosition(0.7);
                }
                else if(currentPosition == 3){
                    clawAngleLeft.setPosition(0.2);
                    clawAngleRight.setPosition(0.8);
                }
            }
            else{
                clawAngleLeft.setPosition(0.6);
                clawAngleRight.setPosition(0.4);
            }


            //claw control
            if(clawDistance<10){
                freightInClaw = true;
                clawClosed = true;
            }
            else if(!freightInClaw && currentPosition==0 && liftPosition > lift.getTargetPosition()-20 & liftPosition< lift.getTargetPosition()+20){
                clawClosed=false;
            }
            else if(gamepad2.circle && clawDistance>10 && liftPosition > lift.getTargetPosition()-20 & liftPosition< lift.getTargetPosition()+20){
                freightInClaw = false;
                clawClosed = false;
            }
            else if(currentPosition == 0 && liftPosition > lift.getTargetPosition()-20 & liftPosition< lift.getTargetPosition()+20){
                clawClosed = true;
            }
            else{
                clawClosed = true;
            }


            if(clawClosed){
                claw.setPosition(0.43);
            }
            else{
                claw.setPosition(0.55);
            }






            //duck control
            if(gamepad2.right_bumper){
                duckRight.setPower(1);
                duckLeft.setPower(1);
            }
            else if(gamepad2.left_bumper){
                duckRight.setPower(-1);
                duckLeft.setPower(-1);
            }
            else{
                duckRight.setPower(0);
                duckLeft.setPower(0);
            }


            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            telemetry.addData("Left Claw Angle", clawAngleLeft.getPosition());
            telemetry.addData("Right Claw Angle", clawAngleRight.getPosition());
            telemetry.addData("Arm Position", currentPosition);
            telemetry.addData("Lift Motor", liftPosition);
            telemetry.addData("Claw Timer", clawTimer.time());
            telemetry.addData("Freight In Claw", freightInClaw);
            telemetry.update();
        }
    }
}
