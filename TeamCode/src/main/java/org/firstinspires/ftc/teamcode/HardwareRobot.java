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
    public DcMotor dronelauncher = null;
    public DcMotor liftleft = null;
    public DcMotor liftright = null;
    public DcMotor spinner = null;
    //public Servo servorelease = null;
    public RevBlinkinLedDriver blinkinLedDriver = null;

    //new code:
    public Servo leftGrab = null;
    public CRServo leftIntakeRear = null;
    public Servo rightGrab = null;
    public CRServo rightIntakeRear = null;
    public Servo leftRotate = null;
    public Servo rightRotate = null;
    public Servo droneClamp = null;
    public CRServo GrabWheelRight = null;
    public CRServo GrabWheelLeft = null;

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

        //servorelease = hwMap.get(Servo.class, "servo_release");

        leftGrab = hwMap.get(Servo.class, "servo_left_grab");
        //leftIntakeRear = hwMap.get(CRServo.class, "servo_left_rear");
        leftRotate = hwMap.get(Servo.class, "servo_left_rotate");
        rightGrab = hwMap.get(Servo.class, "servo_right_grab");
       // rightIntakeRear = hwMap.get(CRServo.class, "servo_right_rear");
        rightRotate = hwMap.get(Servo.class, "servo_right_rotate");

        dronelauncher = hwMap.get(DcMotor.class, "drone_launcher");
        droneClamp = hwMap.get(Servo.class, "drone_Clamp");

        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        GrabWheelRight = hwMap.get(CRServo.class, "Wheel_Right");
        GrabWheelLeft = hwMap.get(CRServo.class, "Wheel_Left");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        liftleft.setDirection(DcMotor.Direction.REVERSE);

        liftleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        dronelauncher.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
        rightGrab.setPosition(.5);
        leftGrab.setPosition(.5);
        //servorelease.setPosition(.5);
        droneClamp.setPosition(0.25);
        leftRotate.setPosition(0.8);
        rightRotate.setPosition(0);


    }

}

