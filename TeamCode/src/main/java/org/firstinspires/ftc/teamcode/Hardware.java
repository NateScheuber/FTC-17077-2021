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
    public CRServo duckRight                = null;
    public CRServo duckLeft                 = null;

    public RevColorSensorV3 intakeSensor    = null;
    public RevColorSensorV3 clawSensor      = null;

    HardwareMap hardwareMap                 = null;


    public Hardware(){

    }


    public void initHardware(HardwareMap hardwareMap){

    rightFront  = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_1150);
    rightMiddle = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_1150);
    rightBack   = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_1150);
    leftFront   = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_1150);
    leftMiddle  = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_1150);
    leftBack    = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_1150);
    lift        = new Motor(hardwareMap, "lift", Motor.GoBILDA.RPM_117);
    intake      = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);

    lift.setRunMode(Motor.RunMode.PositionControl);
    lift.resetEncoder();
    lift.setPositionCoefficient(0.05);
    lift.setPositionTolerance(56);

    rightMiddle.setInverted(true);
    leftFront.setInverted(true);
    leftBack.setInverted(true);

    MotorGroup rightMotors = new MotorGroup(rightFront, rightMiddle, rightBack);
    MotorGroup leftMotors = new MotorGroup(leftFront, leftMiddle, leftBack);

    Servo intakeRight       = hardwareMap.get(Servo.class, "rightIntake");
    Servo intakeLeft        = hardwareMap.get(Servo.class, "rightIntake");
    Servo claw              = hardwareMap.get(Servo.class, "rightIntake");
    Servo clawAngleRight    = hardwareMap.get(Servo.class, "rightIntake");
    Servo clawAngleLeft     = hardwareMap.get(Servo.class, "rightIntake");
    CRServo duckRight       = hardwareMap.get(CRServo.class, "duckRight");
    CRServo duckLeft        = hardwareMap.get(CRServo.class, "duckLeft");


    RevColorSensorV3 intakeSensor = hardwareMap.get(RevColorSensorV3.class, "intakeSensor");
    RevColorSensorV3 clawSensor = hardwareMap.get(RevColorSensorV3.class, "clawSensor");
    }
}
