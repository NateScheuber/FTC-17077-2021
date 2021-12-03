package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

import static org.firstinspires.ftc.teamcode.Teleop_2021.liftP;
import static org.firstinspires.ftc.teamcode.Teleop_2021.liftTolerance;
import static org.firstinspires.ftc.teamcode.Teleop_2021.level3;

@Autonomous(name = "Red")
public class Meet_2_Blue extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo intakeRight       = hardwareMap.get(Servo.class, "intakeRight");
        Servo intakeLeft        = hardwareMap.get(Servo.class, "intakeLeft");
        Servo claw              = hardwareMap.get(Servo.class, "claw");
        Servo clawAngleRight    = hardwareMap.get(Servo.class, "clawAngleRight");
        Servo clawAngleLeft     = hardwareMap.get(Servo.class, "clawAngleLeft");

        CRServo duckRight       = hardwareMap.get(CRServo.class, "duckRight");

        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "lift");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPositionPIDFCoefficients(liftP);
        lift.setTargetPositionTolerance(liftTolerance);

        clawAngleLeft.setPosition(0.6);
        clawAngleRight.setPosition(0.4);
        claw.setPosition(0.43);

        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-31,63, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        Trajectory Carousel = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-50, 60), Math.toRadians(180))
                .build();

        Trajectory Place_Element = drive.trajectoryBuilder(Carousel.end(), true)
                .addDisplacementMarker(() ->{
                    lift.setTargetPosition(level3);
                })
                .splineTo(new Vector2d(-20, 50), Math.toRadians(-220))
                .build();
        Trajectory Park = drive.trajectoryBuilder(Place_Element.end())
                .addDisplacementMarker(() ->{
                    lift.setTargetPosition(0);
                })
                .splineTo(new Vector2d(-60, 36), Math.toRadians(-90))
                .build();


        waitForStart();
        drive.followTrajectory(Carousel);
        duckRight.setPower(1);
        sleep(2000);
        duckRight.setPower(0);

        drive.followTrajectory(Place_Element);
        clawAngleLeft.setPosition(0.2);
        clawAngleRight.setPosition(0.8);
        sleep(500);
        claw.setPosition(0.55);
        sleep(500);
        clawAngleLeft.setPosition(0.6);
        clawAngleRight.setPosition(0.4);
        claw.setPosition(0.43);

        drive.followTrajectory(Park);
        clawAngleLeft.setPosition(1);
        clawAngleRight.setPosition(0);
        sleep(200);
    }
}
