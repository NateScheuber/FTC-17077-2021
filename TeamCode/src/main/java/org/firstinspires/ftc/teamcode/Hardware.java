package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

    boolean intakeOut                       = false;
    boolean intakeToggle                    = true;
    boolean freightInIntake                 = false;
    boolean freightInClaw                   = false;
    boolean clawToggle                      = true;
    boolean clawClosed                      = false;

    public int level1                       = -300;
    public int level2                       = 750;
    public int level3                       = 500;
    public int currentPosition              = 0;

    public Motor rightFront                 = null;
    public Motor rightMiddle                = null;
    public Motor rightBack                  = null;
    public Motor leftFront                  = null;
    public Motor leftMiddle                 = null;
    public Motor leftBack                   = null;
    public Motor lift                       = null;
    public Motor intake                     = null;

    public MotorGroup rightMotors           = null;
    public MotorGroup leftMotors            = null;

    public Servo intakeRight                = null;
    public Servo intakeLeft                 = null;
    public Servo claw                       = null;
    public Servo clawAngleRight             = null;
    public Servo clawAngleLeft              = null;
    /*
    public CRServo duckRight                = null;
    public CRServo duckLeft                 = null;

     */

    public RevColorSensorV3 intakeSensor    = null;
    public RevColorSensorV3 clawSensor      = null;

    HardwareMap hardwareMap                 = null;


    public Hardware(){

    }


    public void initHardware(HardwareMap ahwMap){

        hardwareMap = ahwMap;

        rightFront  = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_1150);
        rightMiddle = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_1150);
        rightBack   = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_1150);
        leftFront   = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_1150);
        leftMiddle  = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_1150);
        leftBack    = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_1150);
        lift        = new Motor(hardwareMap, "lift", Motor.GoBILDA.RPM_117);
        intake      = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);

        rightFront.set(0);
        rightMiddle.set(0);
        rightBack.set(0);
        leftFront.set(0);
        leftMiddle.set(0);
        leftBack.set(0);
        lift.set(0);
        intake.set(0);


        lift.setRunMode(Motor.RunMode.PositionControl);
        lift.resetEncoder();
        lift.setPositionCoefficient(0.05);
        lift.setPositionTolerance(56);

        rightMiddle.setInverted(true);
        leftFront.setInverted(true);
        leftBack.setInverted(true);

        MotorGroup rightMotors = new MotorGroup(rightFront, rightMiddle, rightBack);
        MotorGroup leftMotors = new MotorGroup(leftFront, leftMiddle, leftBack);

        rightMotors.set(0);
        leftMotors.set(0);

        Servo intakeRight       = ahwMap.get(Servo.class, "intakeRight");
        Servo intakeLeft        = ahwMap.get(Servo.class, "intakeLeft");
        Servo claw              = ahwMap.get(Servo.class, "claw");
        Servo clawAngleRight    = ahwMap.get(Servo.class, "clawAngleRight");
        Servo clawAngleLeft     = ahwMap.get(Servo.class, "clawAngleLeft");

        intakeRight.setPosition(0);
        intakeLeft.setPosition(0);
        claw.setPosition(0);
        clawAngleRight.setPosition(0);
        clawAngleLeft.setPosition(0);

    /*
        CRServo duckRight       = hardwareMap.get(CRServo.class, "duck Right");
        CRServo duckLeft        = hardwareMap.get(CRServo.class, "duck Left");

     */


    }
}
