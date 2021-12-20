package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Teleop_2021.liftP;
import static org.firstinspires.ftc.teamcode.Teleop_2021.liftTolerance;

@Autonomous
public class Warehouse extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        Motor rightFront  = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_1150);
        Motor rightMiddle = new Motor(hardwareMap, "rightMiddle", Motor.GoBILDA.RPM_1150);
        Motor rightBack   = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_1150);
        Motor leftFront   = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_1150);
        Motor leftMiddle  = new Motor(hardwareMap, "leftMiddle", Motor.GoBILDA.RPM_1150);
        Motor leftBack    = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_1150);

        Servo TSEArm            = hardwareMap.get(Servo.class, "TSE Arm");

        rightMiddle.setInverted(true);
        leftMiddle.setInverted(true);


        MotorGroup rightMotors = new MotorGroup(rightFront, rightMiddle, rightBack);
        MotorGroup leftMotors = new MotorGroup(leftFront, leftMiddle, leftBack);

        DifferentialDrive DT = new DifferentialDrive(leftMotors, rightMotors);

        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPositionPIDFCoefficients(liftP);
        lift.setTargetPositionTolerance(liftTolerance);

        waitForStart();
        TSEArm.setPosition(0.05);

        lift.setTargetPosition(0);
        lift.setPower(1);

        DT.tankDrive(1,1);
        sleep(500);
        DT.tankDrive(0,0);
    }
}
