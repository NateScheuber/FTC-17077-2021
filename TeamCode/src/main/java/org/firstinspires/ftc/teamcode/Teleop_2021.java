package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class Teleop_2021 extends LinearOpMode {

    Hardware robot = new Hardware();



    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        robot.initHardware(hardwareMap);

        while(opModeIsActive()) {
            DifferentialDrive DT = new DifferentialDrive(robot.leftMotors, robot.rightMotors);



            waitForStart();
            //dt control
            DT.arcadeDrive(-gamepad1.right_stick_y, gamepad1.left_stick_x);


            //intake control
            if(gamepad1.right_bumper){
                robot.intake.set(1);
            }
            else if(gamepad1.left_bumper){
                robot.intake.set(-1);
            }
            else{
                robot.intake.set(0);
            }



            //intake angle control
            if(gamepad1.right_bumper && gamepad1.left_bumper && robot.intakeToggle){
                robot.intakeToggle = false;
                if(!robot.intakeOut){
                    robot.intakeLeft.setPosition(0);
                    robot.intakeRight.setPosition(1);
                    robot.intakeOut = true;
                }
                else{
                    robot.intakeLeft.setPosition(0.6);
                    robot.intakeRight.setPosition(0.4);
                    robot.intakeOut = false;
                }
            }
            else if(!gamepad1.right_bumper && !gamepad1.left_bumper && !robot.intakeToggle){
                robot.intakeToggle = true;
            }


            //arm control
            if(gamepad2.dpad_left){
                robot.lift.setTargetPosition(robot.level1);
                robot.currentPosition = 1;

            }
            else if(gamepad2.dpad_up){
                robot.lift.setTargetPosition(robot.level2);
                robot.currentPosition = 2;
            }
            else if(gamepad2.dpad_right){
                robot.lift.setTargetPosition(robot.level3);
                robot.currentPosition = 3;
            }
            else if(gamepad2.dpad_down){
                robot.lift.setTargetPosition(0);
                robot.currentPosition = 0;
            }

            if(!robot.lift.atTargetPosition()){
                robot.lift.set(0.75);
            }
            else{
                robot.lift.set(0);
            }


            //claw angle control
            if(robot.lift.atTargetPosition()){
                if(robot.currentPosition == 0){
                    robot.clawAngleLeft.setPosition(0);
                    robot.clawAngleRight.setPosition(1);
                }
                else if(robot.currentPosition == 1){
                    robot.clawAngleLeft.setPosition(0.8);
                    robot.clawAngleRight.setPosition(0.2);
                }
                else if(robot.currentPosition == 2){
                    robot.clawAngleLeft.setPosition(0.5);
                    robot.clawAngleRight.setPosition(0.5);
                }
                else if(robot.currentPosition == 3){
                    robot.clawAngleLeft.setPosition(0.75);
                    robot.clawAngleRight.setPosition(0.25);
                }
            }
            else{
                robot.clawAngleLeft.setPosition(0.25);
                robot.clawAngleRight.setPosition(0.75);
            }


            //claw control
            if(robot.currentPosition == 0 && robot.lift.atTargetPosition() && robot.clawSensor.getDistance(DistanceUnit.MM)<100){

            }



            //duck control
            if(gamepad2.right_bumper){
                robot.duckRight.set(1);
                robot.duckLeft.set(1);
            }
            else if(gamepad2.left_bumper){
                robot.duckRight.set(-1);
                robot.duckLeft.set(-1);
            }
            else{
                robot.duckRight.set(0);
                robot.duckLeft.set(0);
            }
        }
    }
}
