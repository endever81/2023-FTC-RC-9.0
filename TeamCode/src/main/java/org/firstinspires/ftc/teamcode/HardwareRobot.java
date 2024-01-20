package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;


public class HardwareRobot {

    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightRearDrive = null;
    public DcMotor extender = null;
    public DcMotor intake = null;
    public Servo servorelease = null;
    public RevBlinkinLedDriver blinkinLedDriver = null;

    //new code:
    public Servo lowArm = null;
    public Servo hopperRotate = null;
    public Servo upperHopperRotate = null;
    public Servo droneClamp = null;

    HardwareMap hwMap = null;
   
    private ElapsedTime period = new ElapsedTime();

    public HardwareRobot (){
    }
   
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap; //saves a reference Hardware Map

        leftFrontDrive = hwMap.get(DcMotor.class, "motor_front_left");
        rightFrontDrive = hwMap.get(DcMotor.class, "motor_front_right");
        leftRearDrive = hwMap.get(DcMotor.class, "motor_rear_left");
        rightRearDrive = hwMap.get(DcMotor.class, "motor_rear_right");
        extender = hwMap.get(DcMotor.class, "extender");
        intake = hwMap.get(DcMotor.class, "intake");

        lowArm = hwMap.get(Servo.class, "servo_low_arm");
        hopperRotate = hwMap.get(Servo.class, "servo_hopper_rotate");
        upperHopperRotate = hwMap.get(Servo.class, "servo_upper_hopper");

        droneClamp = hwMap.get(Servo.class, "drone_Clamp");

        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);



        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        extender.setDirection(DcMotor.Direction.REVERSE);

        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
        lowArm.setPosition(.5);
        hopperRotate.setPosition(.5);
        upperHopperRotate.setPosition(.5);
        droneClamp.setPosition(0.5);



    }

}

