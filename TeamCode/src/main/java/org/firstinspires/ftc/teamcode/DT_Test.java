package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drivetrain Test")
public class DT_Test extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {

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

        waitForStart();

        while(opModeIsActive()){
            DT.arcadeDrive(-gamepad1.right_stick_y, 0.66*gamepad1.left_stick_x);
        }
    }
}
