package org.firstinspires.ftc.teamcode.auto.interleague;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

import static org.firstinspires.ftc.teamcode.Teleop_2021.level3;
import static org.firstinspires.ftc.teamcode.Teleop_2021.liftP;
import static org.firstinspires.ftc.teamcode.Teleop_2021.liftSpeed;
import static org.firstinspires.ftc.teamcode.Teleop_2021.liftTolerance;

@Autonomous
public class Warehouse_Blue extends LinearOpMode {

    enum State{
        Preload,
        AutoCycle,
        Park,
        IDLE
    }

    State currentState = State.IDLE;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Servo intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        Servo intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo clawAngleRight = hardwareMap.get(Servo.class, "clawAngleRight");
        Servo clawAngleLeft = hardwareMap.get(Servo.class, "clawAngleLeft");
        Servo TSEClaw = hardwareMap.get(Servo.class, "TSE Claw");
        Servo TSEArm = hardwareMap.get(Servo.class, "TSE Arm");

        CRServo duckRight = hardwareMap.get(CRServo.class, "duckRight");

        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "lift");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPositionPIDFCoefficients(liftP);
        lift.setTargetPositionTolerance(liftTolerance);

        RevColorSensorV3 intakeSensor = hardwareMap.get(RevColorSensorV3.class, "intakeSensor");
        intakeSensor.initialize();

        ElapsedTime AutoTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        AutoTimer.reset();

        clawAngleLeft.setPosition(0.6);
        clawAngleRight.setPosition(0.4);
        claw.setPosition(0.43);
        intakeLeft.setPosition(0.72);
        intakeRight.setPosition(0.28);

        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-31, 63, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        Trajectory Preload = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0, 40, Math.toRadians(-135)))
                .build();

        Trajectory AutoCycle = drive.trajectoryBuilder(Preload.end())
                .splineTo(new Vector2d(8, 65), Math.toRadians(0))

                .splineTo(new Vector2d(0, 40), Math.toRadians(-135))
                .build();

        Trajectory Park = drive.trajectoryBuilder(AutoCycle.end())
                .forward(48)
                .build();

        waitForStart();
        AutoTimer.reset();


        while (opModeIsActive() & !isStopRequested()){
            switch (currentState) {
                case Preload:
                    if (!drive.isBusy()) {
                        currentState = State.AutoCycle;
                        drive.followTrajectoryAsync(AutoCycle);
                    }
                    break;
                case AutoCycle:
                    if (!drive.isBusy() && AutoTimer.time() < 20) {
                        currentState = State.AutoCycle;
                        drive.followTrajectoryAsync(AutoCycle);
                    } else if (!drive.isBusy()) {
                        currentState = State.Park;
                        drive.followTrajectoryAsync(Park);
                    }
                    break;
                case IDLE:

                    break;
            }
        drive.update();

            if()){
                lift.setTargetPosition(0);
            }

            if(intakeSensor.getDistance(DistanceUnit.MM)<10 && intakeToggle){
                intakeToggle = false;
                if(!intakeOut){
                    drive.
                }

            }

        }
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
