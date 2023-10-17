package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.util.Hardware;
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
    public DcMotor liftleft = null;
    public DcMotor liftright = null;
    public DcMotor liftleft2 = null;
    public DcMotor liftright2 = null;
    public DcMotor intake = null;
    public CRServo leftintake = null;
    public CRServo rightintake = null;
    public Servo servorelease = null;
    public CRServo rightPickup = null;
    public CRServo leftPickup = null;
    public RevBlinkinLedDriver blinkinLedDriver = null;
public Servo Pincher = null;
public Servo Artic = null;
public Servo Rotate = null;

    //public Servo grabber = null;

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
        liftleft = hwMap.get(DcMotor.class, "lift_left");
        liftright = hwMap.get(DcMotor.class, "lift_right");
       // liftleft2 = hwMap.get(DcMotor.class, "lift_left_2");
       // liftright2 = hwMap.get(DcMotor.class, "lift_right_2");
       
       leftintake = hwMap.get(CRServo.class, "left_intake");
       rightintake = hwMap.get(CRServo.class, "right_intake");
        servorelease = hwMap.get(Servo.class, "servo_release");
        //grabber = hwMap.get(Servo.class, "servo_grabber");
        leftPickup = hwMap.get(CRServo.class, "servo_left_pickup");
        rightPickup = hwMap.get(CRServo.class, "servo_right_pickup");
Pincher = hwMap.get(Servo.class, "Pincher");
Artic = hwMap.get(Servo.class, "Artic");
Rotate = hwMap.get(Servo.class, "Rotate");

        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        //liftleft2.setDirection(DcMotor.Direction.REVERSE);
      //  liftright2.setDirection(DcMotor.Direction.REVERSE);
        liftleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // liftleft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // liftright2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
        leftintake.setPower(0);
        rightintake.setPower(0);
        servorelease.setPosition(.5);
        Pincher.setPosition(.5);
        Artic.setPosition(.5);
        Rotate.setPosition(.5);
    }

}

