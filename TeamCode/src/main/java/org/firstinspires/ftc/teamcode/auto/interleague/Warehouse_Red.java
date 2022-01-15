package org.firstinspires.ftc.teamcode.auto.interleague;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

import static org.firstinspires.ftc.teamcode.Teleop_2021.level3;
import static org.firstinspires.ftc.teamcode.Teleop_2021.liftP;
import static org.firstinspires.ftc.teamcode.Teleop_2021.liftSpeed;
import static org.firstinspires.ftc.teamcode.Teleop_2021.liftTolerance;

@Autonomous(name = "Red Autocylce")
public class Warehouse_Red extends LinearOpMode {
    boolean intakeOut = false;
    boolean deposit = false;
    boolean clawClosed = true;




    enum Path{
        Preload,
        AutoCycle1,
        AutoCycle2,
        Park,
        IDLE
    }
    Path currentState = Path.IDLE;

    public enum clawAngle{
        Score,
        Safe,
        Collect
    }
    clawAngle currentPosition = clawAngle.Safe;





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
        Motor intake      = new Motor(hardwareMap, "intake1", Motor.GoBILDA.RPM_1150);

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
        intakeLeft.setPosition(0.03);
        intakeRight.setPosition(0.97);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(6.5, -63, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        Trajectory Preload = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-4, -44, Math.toRadians(-65)))
                .addDisplacementMarker(() ->{
                    clawClosed = false;
                })
                .build();

        Trajectory AutoCycle1 = drive.trajectoryBuilder(Preload.end())
                .addDisplacementMarker(2, () -> {
                    deposit = false;
                    clawClosed = true;
                })
                .addDisplacementMarker(23, () -> {
                    clawClosed = false;
                })
                .splineTo(new Vector2d(4, -56), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(16, -64.5), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    intakeOut = true;
                    //intake.set(-1);
                })
                .splineTo(new Vector2d(56, -64.5), Math.toRadians(0))
                .build();

        Trajectory AutoCycle2 = drive.trajectoryBuilder(AutoCycle1.end(), true)
                .addDisplacementMarker(22, () -> {
                    clawClosed = true;
                })
                .addDisplacementMarker(40,() -> {
                    deposit = true;
                })
                .addDisplacementMarker(() -> {
                    intakeOut = false;
                    //intake.set(0);
                    transferTimer.reset();
                })
                .splineToSplineHeading(new Pose2d(20, -64.5, Math.toRadians(0)), Math.toRadians(-180))
                .splineTo(new Vector2d(-3, -46), Math.toRadians(118))
                .addDisplacementMarker(() -> {
                    clawClosed = false;
                })
                .build();

        Trajectory Park = drive.trajectoryBuilder(AutoCycle2.end())
                .addDisplacementMarker(() -> {
                    clawAngleLeft.setPosition(0.6);
                    clawAngleRight.setPosition(0.4);
                    clawClosed = false;
                    deposit = false;
                })
                .splineTo(new Vector2d(20, -64.5), Math.toRadians(0))
                .splineTo(new Vector2d(56, -64.5), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    clawAngleLeft.setPosition(1);
                    clawAngleRight.setPosition(0);
                })
                .build();

        while(!isStarted()){
            lift.setPower(1);
        }

        waitForStart();
        TSEClaw.setPosition(1);
        TSEArm.setPosition(0.05);
        AutoTimer.reset();
        intakeTimer.reset();
        transferTimer.reset();


        currentState = Path.Preload;
        drive.followTrajectoryAsync(Preload);
        deposit = true;


        while (opModeIsActive() & !isStopRequested()){

            switch (currentState) {
                case Preload:
                    if (!drive.isBusy()) {
                        currentState = Path.AutoCycle1;
                        drive.followTrajectoryAsync(AutoCycle1);
                    }
                    break;
                case AutoCycle1:
                    if (!drive.isBusy()) {
                        currentState = Path.AutoCycle2;
                        drive.followTrajectoryAsync(AutoCycle2);
                    }
                    break;
                case AutoCycle2:
                    if (!drive.isBusy() && AutoTimer.time() < 21 && !lift.isBusy()) {
                        currentState = Path.AutoCycle1;
                        drive.followTrajectoryAsync(AutoCycle1);
                    }
                    else if (!drive.isBusy()) {
                        currentState = Path.Park;
                        drive.followTrajectoryAsync(Park);
                    }

                    break;
                case IDLE:

                    break;
            }
            drive.update();

            switch (currentPosition){
                case Score:
                    clawAngleLeft.setPosition(0.2);
                    clawAngleRight.setPosition(0.8);
                    break;

                case Safe:
                    clawAngleLeft.setPosition(0.6);
                    clawAngleRight.setPosition(0.4);
                    break;

                case Collect:
                    clawAngleLeft.setPosition(1);
                    clawAngleRight.setPosition(0);
                    break;


            }

            if(!intakeOut && transferTimer.time()<500){
                intake.set(0);
            }
            else if(!intakeOut && transferTimer.time()<2000){
                intake.set(0.7);
            }
            else if(intakeOut){
                intake.set(-1);
            }
            else{
                intake.set(0);
            }


            if(intakeOut){
                intakeLeft.setPosition(0.72);
                intakeRight.setPosition(0.28);
                //intake.set(1);
            }
            else{
                intakeLeft.setPosition(0.03);
                intakeRight.setPosition(0.97);
                /*
                if(intakeTimer.time()>500){
                    intake.set(0.7);
                }

                 */
            }


            //lift control
            if(deposit){
                lift.setTargetPosition(level3);
            }
            else{
                lift.setTargetPosition(10);
            }
            lift.setPower(1);


            //claw angle control
            if(lift.isBusy()){
                currentPosition = clawAngle.Safe;
            }
            else if(deposit){
                currentPosition = clawAngle.Score;
            }
            else if(!clawClosed){
                currentPosition = clawAngle.Collect;
            }
            else if(transferTimer.time()>1200){
                currentPosition = clawAngle.Safe;
            }


            if(clawClosed){
                claw.setPosition(0.43);
            }
            else{
                claw.setPosition(0.55);
            }

            telemetry.addData("Transfer Timer", transferTimer.time());
            telemetry.addData("Intake Timer", intakeTimer.time());
            telemetry.addData("Claw", clawClosed);
            telemetry.addData("Lift", lift.isBusy());
            telemetry.update();
        }
    }
}
