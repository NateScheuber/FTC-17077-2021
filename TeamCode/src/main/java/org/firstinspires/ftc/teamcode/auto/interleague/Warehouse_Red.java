package org.firstinspires.ftc.teamcode.auto.interleague;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

import static org.firstinspires.ftc.teamcode.Teleop_2021.level3;
import static org.firstinspires.ftc.teamcode.Teleop_2021.liftP;
import static org.firstinspires.ftc.teamcode.Teleop_2021.liftSpeed;
import static org.firstinspires.ftc.teamcode.Teleop_2021.liftTolerance;

@Autonomous
public class Warehouse_Red extends LinearOpMode {




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

        clawAngleLeft.setPosition(0.6);
        clawAngleRight.setPosition(0.4);
        claw.setPosition(0.43);
        intakeLeft.setPosition(0.72);
        intakeRight.setPosition(0.28);

        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-31,-63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory GrabTSE = drive.trajectoryBuilder(startPose)
                .forward(6)
                .build();
        Trajectory Score = drive.trajectoryBuilder(GrabTSE.end().plus(new Pose2d(0,0, Math.toRadians(135))))
                .back(36)
                .build();
        Trajectory Reposition = drive.trajectoryBuilder(Score.end())
                .forward(36)
                .build();
        Trajectory Park = drive.trajectoryBuilder(Reposition.end().plus(new Pose2d(0, 0, Math.toRadians(-45))))
                .forward(48)
                .build();

        waitForStart();
        TSEArm.setPosition(0.7);
        TSEClaw.setPosition(0);
        drive.followTrajectory(GrabTSE);
        TSEClaw.setPosition(1);
        sleep(750);
        TSEArm.setPosition(0.05);
        sleep(500);
        drive.turn(Math.toDegrees(135));
        drive.followTrajectory(Score);
        score();
        drive.followTrajectory(Reposition);
        drive.turn(-45);
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
        while(lift.getCurrentPosition() > lift.getTargetPosition()-10 & lift.getCurrentPosition()< lift.getTargetPosition()+10){
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
    }
}
