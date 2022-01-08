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

    boolean intakeToggle = true;
    boolean intakeOut = false;
    boolean deposit = false;
    boolean clawClosed = true;




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
        Motor intake      = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);

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

        ElapsedTime intakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        intakeTimer.reset();

        ElapsedTime transferTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        transferTimer.reset();

        clawAngleLeft.setPosition(0.6);
        clawAngleRight.setPosition(0.4);
        claw.setPosition(0.43);
        intakeLeft.setPosition(0.72);
        intakeRight.setPosition(0.28);

        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, 63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory Preload = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-2, 44, Math.toRadians(65)))
                .build();

        Trajectory AutoCycle = drive.trajectoryBuilder(Preload.end())
                .addDisplacementMarker(5, () -> {
                    deposit = false;
                })
                .addDisplacementMarker(36,() -> {
                    deposit = true;
                })
                .splineTo(new Vector2d(8, 64), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    intakeLeft.setPosition(0.03);
                    intakeRight.setPosition(0.97);
                    intake.set(1);
                })
                .splineTo(new Vector2d(40, 64), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    intakeLeft.setPosition(0.72);
                    intakeRight.setPosition(0.28);
                    intake.set(0);
                })
                .splineToSplineHeading(new Pose2d(16, 64, Math.toRadians(0)), Math.toRadians(180))
                .splineTo(new Vector2d(-2, 44), Math.toRadians(-108))
                .addDisplacementMarker(() -> {
                    clawClosed = false;
                })
                .build();

        Trajectory Park = drive.trajectoryBuilder(AutoCycle.end())
                .splineTo(new Vector2d(8, 64), Math.toRadians(0))
                .splineTo(new Vector2d(40, 64), Math.toRadians(0))
                .build();

        waitForStart();
        AutoTimer.reset();
        intakeTimer.reset();
        transferTimer.reset();


        while (opModeIsActive() & !isStopRequested()){

            switch (currentState) {
                case Preload:
                    if (!drive.isBusy()) {
                        currentState = State.AutoCycle;
                        drive.followTrajectoryAsync(AutoCycle);
                    }
                    break;
                case AutoCycle:
                    if (!drive.isBusy() && AutoTimer.time() < 20 && !lift.isBusy()) {
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


            if(intakeSensor.getDistance(DistanceUnit.MM)<10 && intakeToggle){
                intakeToggle = false;
                intakeOut = false;
                intakeLeft.setPosition(0.72);
                intakeRight.setPosition(0.28);
                transferTimer.reset();
            }
            else{
                intakeToggle=true;
                intakeTimer.reset();
            }

            if(!intakeOut && (intakeTimer.time()>500 || transferTimer.time()<1200)){
                intake.set(-0.7);
            }


            //lift control
            if(deposit && clawClosed){
                lift.setTargetPosition(level3);
            }
            else if(!deposit){
                lift.setTargetPosition(0);
            }


            //claw angle control
            if(!lift.isBusy()){
                if(!deposit && !clawClosed){
                    clawAngleLeft.setPosition(1);
                    clawAngleRight.setPosition(0);
                }
                else if(!deposit){
                    clawAngleLeft.setPosition(0.6);
                    clawAngleRight.setPosition(0.4);
                }
                else{
                    clawAngleLeft.setPosition(0.2);
                    clawAngleRight.setPosition(0.8);
                }
            }
            else{
                clawAngleLeft.setPosition(0.6);
                clawAngleRight.setPosition(0.4);
            }


            //claw control
            if(transferTimer.time()>1200 && transferTimer.time()<1250){
                clawClosed = true;
            }

            if(clawClosed){
                claw.setPosition(0.43);
            }
            else{
                claw.setPosition(0.55);
            }
        }
    }
}
