package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "Driver Controlled", group = "Robot")

public class DriverControlled extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();
    
@Override
public void runOpMode() {
    robot.init(hardwareMap);
    RevBlinkinLedDriver.BlinkinPattern pattern;
    pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;

    RevBlinkinLedDriver.BlinkinPattern patternPrime;
    patternPrime = RevBlinkinLedDriver.BlinkinPattern.GREEN;

      BNO055IMU imu;

    Orientation angles;

    robot.init(hardwareMap);


    //IMU Initialization
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.loggingEnabled = false;

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            telemetry.addData("Mode", "calibrating...");
            telemetry.update();

            // make sure the imu gyro is calibrated before continuing.
            while (!isStopRequested() && !imu.isGyroCalibrated()) {
                sleep(50);
                idle();
            }

/*
     telemetry.addData("Mode", "waiting for start");
             telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
             angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
             telemetry.addData("Heading1", angles.firstAngle);
             telemetry.addData("Heading2", angles.secondAngle);
             telemetry.addData("Heading3", angles.thirdAngle);
*/



    telemetry.addData("Say", "Waiting for Start");
    telemetry.update();

    robot.liftleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.liftright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    waitForStart();


    while (opModeIsActive()){

/*
        telemetry.addData("Pole",robot.pole.getDeviceName() );
        telemetry.addData("range", String.format("%.01f in", robot.pole.getDistance(DistanceUnit.INCH)));

        telemetry.addData("Heading1", angles.firstAngle);
        telemetry.addData("Heading2", angles.secondAngle);
        telemetry.addData("Heading3", angles.thirdAngle);

        telemetry.update();
*/

    double Turn = gamepad1.left_stick_x;  //speed
    double Speed = -gamepad1.left_stick_y; //turn
    double Strafe = -gamepad1.right_stick_x;
    double MAX_SPEED = 1.0;

    Turn = Turn*.9;

    double front_left = Speed + Turn - Strafe;
    double front_right = Speed - Turn + Strafe;
    double rear_left = Speed + Turn + Strafe;
    double rear_right = Speed - Turn - Strafe;
    
    front_left = Range.clip(front_left, -1, 1);
    front_right = Range.clip(front_right, -1, 1);
    rear_left = Range.clip(rear_left, -1, 1);
    rear_right = Range.clip(rear_right, -1, 1);
    
    
    front_left = (float)scaleInput(front_left);
    front_right = (float)scaleInput(front_right);
    rear_left = (float)scaleInput(rear_left);
    rear_right = (float)scaleInput(rear_right);
    
    front_left /=2;
    front_right /=2;
    rear_left /=2;
    rear_right /=2;
    
    if (gamepad1.right_bumper){
    front_left *=2;
    front_right *=2;
    rear_left *=2;
    rear_right *=2;
    }    

    if (gamepad1.left_bumper){
    front_left /=2.5;
    front_right /=2.5;
    rear_left /=2.5;
    rear_right /=2.5;
    }
   
   
   double servoLeftPower = 0;
   double servoRightPower = 0;
   
   if (gamepad2.a){
    servoRightPower = -1;
    servoLeftPower = 1;
    }
    
    if (gamepad2.x){
    servoRightPower = 1;
    servoLeftPower = -1;
    }


        double quickrelease = .5;
        if (gamepad2.b){
            quickrelease = .35;
        }

    
    
       double PickUpLeftPosition = 1;
        double PickupRightPosition =0;
   
 if (gamepad1.a){
            PickUpLeftPosition = .80;
            PickupRightPosition = .20;
        }

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (angles.thirdAngle >= 170){
            PickUpLeftPosition = .75;
            PickupRightPosition = .25;
        }
    

   double liftleftPower = -gamepad2.left_stick_y;
    double liftrightPower = gamepad2.left_stick_y;

    if (liftleftPower == 0) {

        robot.liftleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int newLiftTargetLeft = robot.liftleft.getCurrentPosition();
        int newLiftTargetRight = robot.liftright.getCurrentPosition();

        robot.liftleft.setTargetPosition(newLiftTargetLeft);
        robot.liftright.setTargetPosition(newLiftTargetRight);
        robot.liftleft.setPower(1);
        robot.liftright.setPower(1);
    }
    else {    robot.liftleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);}

//*******************************************************************
        //Robot Coloration Conditions and Controls
        //***********************************************************
        if (gamepad2.dpad_right){
            patternPrime = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
        }

        if (gamepad2.dpad_left){
            patternPrime = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
        }
/*
        if (robot.pole.getDistance(DistanceUnit.INCH) > 9 && robot.pole.getDistance(DistanceUnit.INCH) <14) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
        }
        else {
            pattern = patternPrime;
        }
*/
//************************************************************************



        robot.blinkinLedDriver.setPattern(patternPrime);

    robot.leftFrontDrive.setPower(front_left);
    robot.rightFrontDrive.setPower(front_right);
    robot.leftRearDrive.setPower(rear_left);
    robot.rightRearDrive.setPower(rear_right);
    robot.liftleft.setPower(liftleftPower);
    robot.liftright.setPower(liftrightPower);
    //robot.spinner.setPower(motorspinner);
    robot.rightintake.setPower(servoRightPower);
    robot.leftintake.setPower(servoLeftPower);
    robot.servorelease.setPosition(quickrelease);
    robot.leftPickup.setPosition(PickUpLeftPosition);
    robot.rightPickup.setPosition(PickupRightPosition);

    }

}

double scaleInput(double dVal)  {
      double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
            0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
      
      // get the corresponding index for the scaleInput array.
      int index = (int) (dVal * 16.0);
      if (index < 0) {
         index = -index;
      } else if (index > 16) {
         index = 16;
      }
      
      double dScale = 0.0;
      if (dVal < 0) {
         dScale = -scaleArray[index];
      } else {
         dScale = scaleArray[index];
      }
      
      return dScale;
   }
    
}


