package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Teleop_2021 extends LinearOpMode {

    Hardware robot = new Hardware();



    @Override
    public void runOpMode() throws InterruptedException {
        robot.initHardware(hardwareMap);

        while(opModeIsActive()) {
            DifferentialDrive DT = new DifferentialDrive(robot.leftMotors, robot.rightMotors);



            waitForStart();
            DT.arcadeDrive(-gamepad1.right_stick_y, gamepad1.left_stick_x);



            if(gamepad1.right_bumper){
                robot.intake.set(1);
            }
            else if(gamepad1.left_bumper){
                robot.intake.set(-1);
            }
            else{
                robot.intake.set(0);
            }



            if(gamepad2.dpad_left){
                robot.lift.setTargetPosition(robot.level1);
            }
            else if(gamepad2.dpad_up){
                robot.lift.setTargetPosition(robot.level2);
            }
            else if(gamepad2.dpad_right){
                robot.lift.setTargetPosition(robot.level3);
            }
            else if(gamepad2.dpad_down){
                robot.lift.setTargetPosition(0);
            }

            if(!robot.lift.atTargetPosition()){
                robot.lift.set(0.75);
            }
            else{
                robot.lift.set(0);
            }

        }
    }
}
