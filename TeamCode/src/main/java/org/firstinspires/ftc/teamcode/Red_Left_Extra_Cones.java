package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "RED: Left Side EXTRA", group = "Automonous")
public class Red_Left_Extra_Cones extends LinearOpMode{

    // private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String TFOD_MODEL_ASSET = "InitialModel22-23.tflite";


    private static final String[] LABELS = {
            "1 Bolt", // "1 Bolt"
            "2 Bulb", // "2 Bulb"
            "3 Panel" // "3 Panel"
    };

    private static final String VUFORIA_KEY =
            "AcSWQyT/////AAABmXXMracDmUH2jST1AK/jjlB9bLitWfl+EeHaTDyQwcEVZ0/pIKzSLLnKb++x6kKcTYJnrBSWXcbq43Pa/x7v0cEfSLljqPHAntPUwrcTa7Ag5MR/KnSvxThO52HlzZ1T9S5JJtViLz5JLvrm8siLeJIK9uPiqKkYG3IkLBtXnHMLjB/4kfn5zfjnDzpwjgl+2bNzztz/dM91B1u6kroe/QCHWSWBeEgG8vJnVG/ko1aVkiroqaR/al9iui+lPRzAMMcSMKgxxW5sV5DcVdKWVXJq309wm2lUDXKT/4V3C8w48/KkI1J/B7YdB5um6TPCo6Jt8eaczYV3cuX3HmStOvTH1S5ixph1K/9TmyPoKhas";


    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 384.5;    // Neverest 40:1
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.2;
    static final double STRAFE_SPEED = 0.2;
    static final double TURN_SPEED = 0.2;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.02;     // Larger is more responsive, but also less stable
    //int size = 0;



    // Calls up IMU (Inertial Measruement Unit within REV Hub)
    BNO055IMU imu;

    Orientation angles;

    HardwareRobot robot = new HardwareRobot();



    @Override
    public void runOpMode() {


        robot.init(hardwareMap);

        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);

        initVuforia();
        initTfod();


        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.5, 10.0 / 9.0);
        }


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


        telemetry.addData("Wheel Encoders", "Starting at %7d :%7d :%7d :%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.leftRearDrive.getCurrentPosition(),
                robot.rightRearDrive.getCurrentPosition()
        );


        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);


        telemetry.update();


        waitForStart();

        //Identify which signal is being displayed.

        int x = 0;
        if (opModeIsActive()) {
            //  while (opModeIsActive()) {
            if (tfod != null) {

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    x = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());


                        if (recognition.getLabel() == "1 Bolt") {
                            x = 1;
                        }
                        if (recognition.getLabel() == "2 Bulb") {
                            x = 2;
                        }
                        if (recognition.getLabel() == "3 Panel") {
                            x = 3;
                        }

                    }


                }
                telemetry.update();
            }

        }

        // ***|| LEFT POSITION ||***
        // ***|| LEFT POSITION ||***
        // ***|| LEFT POSITION ||***
        if (x == 1) {
            telemetry.addData(">", "Left");
            telemetry.update();
            sleep(500);
//||||||||    Take Cone to Medium Pole
            gyroStrafe(.5, 4, 0); //strafe away from low junction
            gyroDriveLift(.4,42,0,1,29);
            gyroTurn(.5, -47); //turn toward high junction
            gyroDrive(.4, 9.5, -47);  //approach high junction
// *****************************************************************************
//Sleeve and Drop the Cone
            lift(.1, -4); // lower lift and hold position
            robot.servorelease.setPosition(.35);
            sleep(500);

// *****************************************************************************
//Go To Cone Stack
            gyroDrive(.4, -10.5,-47); // back away from terminal
            robot.servorelease.setPosition(.5); //return intake to correct position
            gyroTurn(.5, 0);
            gyroStrafe(.5,-5,0);

            gyroDriveLift(.5,27, 0, 1,-18);
            gyroTurn(.5, 92); //turn toward stack
            robot.rightintake.setPower(1);
            robot.leftintake.setPower(-1);
            gyroDrive(.3, 32,92); // back into end position
            sleep(500);
            robot.rightintake.setPower(0);
            robot.leftintake.setPower(0);
            lift(.8, 10);
//*****************************************************************************
// Return to medium terminal with cone
            gyroDrive(.5, -32,92); // back into end position
            gyroTurn(.5, 240); //turn to 0 degress
            lift(1, 13);
            gyroDrive(.5, 14.5,240); // back into end position
// *****************************************************************************
//Sleeve and Drop the Cone
            lift(.1, -4); // lower lift and hold position
            robot.servorelease.setPosition(.35);
            sleep(500);
            gyroDrive(.4, -10.5,240); // back away from terminal
            robot.servorelease.setPosition(.5);

// *****************************************************************************
//Final Orientation and park in designated zone
            gyroTurn(.5, 0); //turn to 0 degress
            gyroStrafe(.4,-40,0);
            gyroDriveLift(.5,-15,0,1,-25);



        }

        // ***|| CENTER POSITION ||***
        // ***|| CENTER POSITION ||***
        // ***|| CENTER POSITION ||***

        if (x == 2) {
            telemetry.addData(">", "Center");
            telemetry.update();
            sleep(500);
//||||||||    Take Cone to Medium Pole
            gyroStrafe(.5, 4, 0); //strafe away from low junction
            gyroDriveLift(.4,42,0,1,29);
            gyroTurn(.5, -47); //turn toward high junction
            gyroDrive(.4, 10, -47);  //approach high junction
// *****************************************************************************
//Sleeve and Drop the Cone
            lift(.1, -4); // lower lift and hold position
            robot.servorelease.setPosition(.35);
            sleep(500);

// *****************************************************************************
//Go To Cone Stack
            gyroDrive(.4, -10.5,-47); // back away from terminal
            robot.servorelease.setPosition(.5); //return intake to correct position
            gyroTurn(.5, 0);
            gyroStrafe(.5,-5,0);

            gyroDriveLift(.5,28, 0, 1,-18);
            gyroTurn(.5, 92); //turn toward stack
            robot.rightintake.setPower(1);
            robot.leftintake.setPower(-1);
            gyroDrive(.3, 32,92); // back into end position
            sleep(500);
            robot.rightintake.setPower(0);
            robot.leftintake.setPower(0);
            lift(.8, 10);
//*****************************************************************************
// Return to medium terminal with cone
            gyroDrive(.5, -32,92); // back into end position
            gyroTurn(.5, 240); //turn to 0 degress
            lift(1, 13);
            gyroDrive(.5, 14,240); // back into end position
// *****************************************************************************
//Sleeve and Drop the Cone
            lift(.1, -4); // lower lift and hold position
            robot.servorelease.setPosition(.35);
            sleep(500);
            gyroDrive(.4, -10.5,240); // back away from terminal
            robot.servorelease.setPosition(.5);

// *****************************************************************************
//Final Orientation and park in designated zone
            gyroTurn(.5, 0); //turn to 0 degress




        }

        // ***|| RIGHT POSITION ||***
        // ***|| RIGHT POSITION ||***
        // ***|| RIGHT POSITION ||***

        if (x == 3) {
            telemetry.addData(">", "Right");
            telemetry.update();
            sleep(1000);
            //Position 3 - RIGHT
//||||||||    Take Cone to Medium Pole
            gyroStrafe(.5, 4, 0); //strafe away from low junction
            gyroDriveLift(.4,42,0,1,29);
            gyroTurn(.5, -47); //turn toward high junction
            gyroDrive(.4, 9.5, -47);  //approach high junction
// *****************************************************************************
//Sleeve and Drop the Cone
            lift(.1, -4); // lower lift and hold position
            robot.servorelease.setPosition(.35);
            sleep(500);

// *****************************************************************************
//Go To Cone Stack
            gyroDrive(.4, -10.5,-47); // back away from terminal
            robot.servorelease.setPosition(.5); //return intake to correct position
            gyroTurn(.5, 0);
            gyroStrafe(.5,-5,0);

            gyroDriveLift(.5,27, 0, 1,-18);
            gyroTurn(.5, 92); //turn toward stack
            robot.rightintake.setPower(1);
            robot.leftintake.setPower(-1);
            gyroDrive(.3, 31,92); // back into end position
            sleep(500);
            robot.rightintake.setPower(0);
            robot.leftintake.setPower(0);
            lift(.8, 10);
//*****************************************************************************
// Return to medium terminal with cone
            gyroDrive(.5, -32,92); // back into end position
            gyroTurn(.5, 241); //turn to 0 degress
            lift(1, 13);
            gyroDrive(.5, 14.5,241); // back into end position
// *****************************************************************************
//Sleeve and Drop the Cone
            lift(.1, -4); // lower lift and hold position
            robot.servorelease.setPosition(.35);
            sleep(500);
            gyroDrive(.4, -10.5,241); // back away from terminal
            robot.servorelease.setPosition(.5);

// *****************************************************************************
//Final Orientation and park in designated zone
            gyroTurn(.5, 0); //turn to 0 degress
            gyroStrafe(.4,45,0);
            gyroDriveLift(.5,-15,0,1,-25);

        }

    }




    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.

        robot.rightFrontDrive.setPower(rightSpeed);
        robot.leftFrontDrive.setPower(leftSpeed);
        robot.rightRearDrive.setPower(rightSpeed);
        robot.leftRearDrive.setPower(leftSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }


    public void gyroDrive ( double speed,  double distance,  double angle) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontRightTarget = robot.rightFrontDrive.getCurrentPosition() + moveCounts;
            newFrontLeftTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newRearRightTarget = robot.rightRearDrive.getCurrentPosition() + moveCounts;
            newRearLeftTarget = robot.leftRearDrive.getCurrentPosition() + moveCounts;

            robot.rightFrontDrive.setTargetPosition(newFrontRightTarget);
            robot.leftFrontDrive.setTargetPosition(newFrontLeftTarget);
            robot.rightRearDrive.setTargetPosition(newRearRightTarget);
            robot.leftRearDrive.setTargetPosition(newRearLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.rightFrontDrive.setPower(speed);
            robot.leftFrontDrive.setPower(speed);
            robot.rightRearDrive.setPower(speed);
            robot.leftRearDrive.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.rightRearDrive.isBusy()
                            && robot.leftRearDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;



                //Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }





                robot.rightFrontDrive.setPower(rightSpeed);
                robot.leftFrontDrive.setPower(leftSpeed);
                robot.rightRearDrive.setPower(rightSpeed);
                robot.leftRearDrive.setPower(leftSpeed);


                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.rightFrontDrive.getCurrentPosition(),
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightRearDrive.getCurrentPosition(),
                        robot.leftRearDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Heading", angles.firstAngle);
                //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
            robot.rightFrontDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            robot.leftRearDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


    }

    public void gyroDriveNoBrake ( double speed,  double distance,  double angle) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontRightTarget = robot.rightFrontDrive.getCurrentPosition() + moveCounts;
            newFrontLeftTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newRearRightTarget = robot.rightRearDrive.getCurrentPosition() + moveCounts;
            newRearLeftTarget = robot.leftRearDrive.getCurrentPosition() + moveCounts;

            robot.rightFrontDrive.setTargetPosition(newFrontRightTarget);
            robot.leftFrontDrive.setTargetPosition(newFrontLeftTarget);
            robot.rightRearDrive.setTargetPosition(newRearRightTarget);
            robot.leftRearDrive.setTargetPosition(newRearLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.rightFrontDrive.setPower(speed);
            robot.leftFrontDrive.setPower(speed);
            robot.rightRearDrive.setPower(speed);
            robot.leftRearDrive.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.rightRearDrive.isBusy()
                            && robot.leftRearDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;



                //Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }





                robot.rightFrontDrive.setPower(rightSpeed);
                robot.leftFrontDrive.setPower(leftSpeed);
                robot.rightRearDrive.setPower(rightSpeed);
                robot.leftRearDrive.setPower(leftSpeed);


                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.rightFrontDrive.getCurrentPosition(),
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightRearDrive.getCurrentPosition(),
                        robot.leftRearDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Heading", angles.firstAngle);
                //telemetry.addData("Correction", correction);
                telemetry.update();
            }




            // Turn off RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


    }

    public void gyroStrafe ( double speed,  double distance,  double angle) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontRightTarget = robot.rightFrontDrive.getCurrentPosition() - moveCounts;
            newFrontLeftTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newRearRightTarget = robot.rightRearDrive.getCurrentPosition() + moveCounts;
            newRearLeftTarget = robot.leftRearDrive.getCurrentPosition() - moveCounts;


            robot.rightFrontDrive.setTargetPosition(newFrontRightTarget);
            robot.leftFrontDrive.setTargetPosition(newFrontLeftTarget);
            robot.rightRearDrive.setTargetPosition(newRearRightTarget);
            robot.leftRearDrive.setTargetPosition(newRearLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.rightFrontDrive.setPower(speed);
            robot.leftFrontDrive.setPower(speed);
            robot.rightRearDrive.setPower(speed);
            robot.leftRearDrive.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.rightRearDrive.isBusy()
                            && robot.leftRearDrive.isBusy()))  {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;



                //Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.rightFrontDrive.setPower(leftSpeed);
                robot.leftFrontDrive.setPower(speed);
                robot.rightRearDrive.setPower(speed);
                robot.leftRearDrive.setPower(rightSpeed);



                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.rightFrontDrive.getCurrentPosition(),
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightRearDrive.getCurrentPosition(),
                        robot.leftRearDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Heading", angles.firstAngle);
                //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
            robot.rightFrontDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            robot.leftRearDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        }

    }




    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.60f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

    }






    boolean onHeadingShoot(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }
        // Send desired speeds to motors.

        robot.rightFrontDrive.setPower(rightSpeed);
        robot.leftFrontDrive.setPower(leftSpeed);
        robot.rightRearDrive.setPower(rightSpeed);
        robot.leftRearDrive.setPower(leftSpeed);



//motorfeeder.setPower(1);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }



    public void lift(double power, double inches)
    {
        int newLiftTargetRight;
        int newLiftTargetLeft;


        if (opModeIsActive()) {


            newLiftTargetLeft = robot.liftleft.getCurrentPosition() + (int) (inches * (1140 / (3.5 * 3.1415)));
            newLiftTargetRight = robot.liftright.getCurrentPosition() - (int) (inches * (1140 / (3.5 * 3.1415)));

            robot.liftleft.setTargetPosition(newLiftTargetLeft);
            robot.liftright.setTargetPosition(newLiftTargetRight);

            robot.liftleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftright.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            robot.liftleft.setPower(Math.abs(power));
            robot.liftright.setPower(Math.abs(power));


            while (opModeIsActive() && robot.liftleft.isBusy() && robot.liftright.isBusy()) {
                telemetry.addData("Lift", "Running at %7d, 7%d",
                        robot.liftleft.getCurrentPosition(), robot.liftright.getCurrentPosition());
                telemetry.update();

            }
            //robot.lift.setPower(0);            //removed to hold motor position
            //robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);   //removed to hold motor position
        }

    }
    public void gyroDriveLift ( double speed,  double distance,  double angle, double liftSpeed, double liftDistance) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int newleftLiftTarget;
        int newrightLiftTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontRightTarget = robot.rightFrontDrive.getCurrentPosition() + moveCounts;
            newFrontLeftTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newRearRightTarget = robot.rightRearDrive.getCurrentPosition() + moveCounts;
            newRearLeftTarget = robot.leftRearDrive.getCurrentPosition() + moveCounts;
            newleftLiftTarget = robot.liftleft.getCurrentPosition() + (int) (liftDistance * (1140/(3.5 * 3.1415)));
            newrightLiftTarget = robot.liftright.getCurrentPosition() - (int) (liftDistance * (1140/(3.5 * 3.1415)));

            robot.rightFrontDrive.setTargetPosition(newFrontRightTarget);
            robot.leftFrontDrive.setTargetPosition(newFrontLeftTarget);
            robot.rightRearDrive.setTargetPosition(newRearRightTarget);
            robot.leftRearDrive.setTargetPosition(newRearLeftTarget);
            robot.liftleft.setTargetPosition(newleftLiftTarget);
            robot.liftright.setTargetPosition(newrightLiftTarget);

            // Turn On RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftright.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftFrontDrive.isBusy() && robot.liftright.isBusy() && robot.liftleft.isBusy() && robot.rightFrontDrive.isBusy() && robot.rightRearDrive.isBusy()
                            && robot.leftRearDrive.isBusy())) {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);


                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;



                //Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.rightFrontDrive.setPower(rightSpeed);
                robot.leftFrontDrive.setPower(leftSpeed);
                robot.rightRearDrive.setPower(rightSpeed);
                robot.leftRearDrive.setPower(leftSpeed);
                robot.liftleft.setPower(Math.abs(liftSpeed));
                robot.liftright.setPower(Math.abs(liftSpeed));

/*
                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",   robot.rightFrontDrive.getCurrentPosition(),
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightRearDrive.getCurrentPosition(),
                        robot.leftRearDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Heading", angles.firstAngle);
                //telemetry.addData("Correction", correction);
                telemetry.update();

 */
            }

            // Stop all motion;
            robot.rightFrontDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.liftleft.setPower(0);
            robot.liftright.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void gyroStrafeLift ( double speed,  double distance,  double angle, double liftSpeed, double liftDistance) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int newleftLiftTarget;
        int newrightLiftTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newFrontRightTarget = robot.rightFrontDrive.getCurrentPosition() - moveCounts;
            newFrontLeftTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newRearRightTarget = robot.rightRearDrive.getCurrentPosition() + moveCounts;
            newRearLeftTarget = robot.leftRearDrive.getCurrentPosition() - moveCounts;
            newleftLiftTarget = robot.liftleft.getCurrentPosition() + (int) (liftDistance * (1140/(3.5 * 3.1415)));
            newrightLiftTarget = robot.liftright.getCurrentPosition() - (int) (liftDistance * (1140/(3.5 * 3.1415)));

            robot.rightFrontDrive.setTargetPosition(newFrontRightTarget);
            robot.leftFrontDrive.setTargetPosition(newFrontLeftTarget);
            robot.rightRearDrive.setTargetPosition(newRearRightTarget);
            robot.leftRearDrive.setTargetPosition(newRearLeftTarget);
            robot.liftleft.setTargetPosition(newleftLiftTarget);
            robot.liftright.setTargetPosition(newrightLiftTarget);

            // Turn On RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftright.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.rightFrontDrive.setPower(speed);
            robot.leftFrontDrive.setPower(speed);
            robot.rightRearDrive.setPower(speed);
            robot.leftRearDrive.setPower(speed);
            robot.liftleft.setPower(-liftSpeed);
            robot.liftright.setPower(liftSpeed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftFrontDrive.isBusy() && robot.liftright.isBusy() && robot.liftleft.isBusy() && robot.rightFrontDrive.isBusy() && robot.rightRearDrive.isBusy()
                            && robot.leftRearDrive.isBusy())) {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;



                //Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }





                robot.rightFrontDrive.setPower(leftSpeed);
                robot.leftFrontDrive.setPower(speed);
                robot.rightRearDrive.setPower(speed);
                robot.leftRearDrive.setPower(rightSpeed);


                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontRightTarget, newFrontLeftTarget,
                        newRearRightTarget, newRearLeftTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      robot.rightFrontDrive.getCurrentPosition(),
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightRearDrive.getCurrentPosition(),
                        robot.leftRearDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("Heading", angles.firstAngle);
                //telemetry.addData("Correction", correction);
                telemetry.update();
            }

            // Stop all motion;
            robot.rightFrontDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightRearDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.liftleft.setPower(0);
            robot.liftright.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

}






