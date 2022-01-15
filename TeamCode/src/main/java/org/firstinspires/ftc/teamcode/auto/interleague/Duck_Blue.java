package org.firstinspires.ftc.teamcode.auto.interleague;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

import static org.firstinspires.ftc.teamcode.Teleop_2021.level3;
import static org.firstinspires.ftc.teamcode.Teleop_2021.liftP;
import static org.firstinspires.ftc.teamcode.Teleop_2021.liftSpeed;
import static org.firstinspires.ftc.teamcode.Teleop_2021.liftTolerance;

@Autonomous(name = "Blue Duck")
public class Duck_Blue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo intakeRight       = hardwareMap.get(Servo.class, "intakeRight");
        Servo intakeLeft        = hardwareMap.get(Servo.class, "intakeLeft");
        Servo claw              = hardwareMap.get(Servo.class, "claw");
        Servo clawAngleRight    = hardwareMap.get(Servo.class, "clawAngleRight");
        Servo clawAngleLeft     = hardwareMap.get(Servo.class, "clawAngleLeft");
        Servo TSEClaw           = hardwareMap.get(Servo.class, "TSE Claw");
        Servo TSEArm            = hardwareMap.get(Servo.class, "TSE Arm");

        CRServo duckRight       = hardwareMap.get(CRServo.class, "duckRight");

        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "lift");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPositionPIDFCoefficients(liftP);
        lift.setTargetPositionTolerance(liftTolerance);

        Motor duck        = new Motor(hardwareMap, "duck");

        clawAngleLeft.setPosition(0.6);
        clawAngleRight.setPosition(0.4);
        claw.setPosition(0.43);
        intakeLeft.setPosition(0.03);
        intakeRight.setPosition(0.97);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36,63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory Score = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-22, 36, Math.toRadians(135)))
                .build();
        Trajectory Duck = drive.trajectoryBuilder(Score.end())
                .lineTo(new Vector2d(-55, 62))
                //.splineToConstantHeading(new Vector2d(-54, 50), Math.toRadians(-160))
                .build();
        Trajectory Park = drive.trajectoryBuilder(Duck.end())
                .lineToLinearHeading(new Pose2d(-60, 36, Math.toRadians(0)))
                .build();

        while(!isStarted()){
            lift.setPower(1);
        }

        waitForStart();
        TSEClaw.setPosition(1);
        TSEArm.setPosition(0.05);

        drive.followTrajectory(Score);
        score();
        drive.followTrajectory(Duck);
        duckRight.setPower(1);
        duck.set(0.7);
        sleep(2000);
        duckRight.setPower(0);
        duck.set(0);
        drive.followTrajectory(Park);

    }
    public void score() throws InterruptedException {
        Servo clawAngleRight    = hardwareMap.get(Servo.class, "clawAngleRight");
        Servo clawAngleLeft     = hardwareMap.get(Servo.class, "clawAngleLeft");
        Servo claw              = hardwareMap.get(Servo.class, "claw");

        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "lift");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPositionPIDFCoefficients(liftP);
        lift.setTargetPositionTolerance(liftTolerance);

        lift.setTargetPosition(level3);
        while(lift.isBusy()){
            lift.setPower(liftSpeed);
        }
        clawAngleLeft.setPosition(0.2);
        clawAngleRight.setPosition(0.8);
        sleep(200);
        claw.setPosition(0.55);
        sleep(200);
        claw.setPosition(0.43);
        sleep(200);
        clawAngleLeft.setPosition(0.6);
        clawAngleRight.setPosition(0.4);
        lift.setTargetPosition(0);
        while(lift.getCurrentPosition() > lift.getTargetPosition()-10 & lift.getCurrentPosition()< lift.getTargetPosition()+10){
            lift.setPower(liftSpeed);
        }
        clawAngleLeft.setPosition(1);
        clawAngleRight.setPosition(0);
    }
}