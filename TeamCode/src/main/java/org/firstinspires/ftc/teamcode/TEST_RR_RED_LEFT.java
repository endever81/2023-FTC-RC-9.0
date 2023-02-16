package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;
@Config
@Autonomous(name = "TEST_RR_RED_LEFT", group = "Automonous")

public class TEST_RR_RED_LEFT extends LinearOpMode {
    //-----------------------------------------------------------
    // Vuforia Assets Called Up

    public static double MG1X = 44; public static double MG1Y = -28; public static double MG1A = 45;

    public static double BAM1 = 3;

    public static double SAM1 = -9;

    public static double PL1X = 35; public static double PL1Y = -10; public static double PL1A = 177;

    public static double TS1X = 13.75; public static double TS1Y = -10;
    public static double OS1X = 13.75; public static double OS1Y = -9; public static double OS1A = 177;

    public static double BS1X = 35; public static double BS1Y = -10; public static double BS1A = 20;

    public static double MG2X = 43; public static double MG2Y = -24; public static double MG2A = 20;

    public static double BAM2 = 4;

    public static double PL2X = 35; public static double PL2Y = -10.5; public static double PL2A = 180;

    public static double TS2X = 13.75; public static double TS2Y = -10.5;
    public static double OS2X = 13.5; public static double OS2Y = -10; public static double OS2A = 180;

    public static double BS2X = 35; public static double BS2Y = -10; public static double BS2A = 20;

    public static double MG3X = 43; public static double MG3Y = -23; public static double MG3A = 20;

    public static double BAM3 = 7;

    public static double SAM3 = -15;

    public static double P1X = 16; public static double P1Y = -10; public static double P1A = 270;
    public static double P2X = 38; public static double P2Y = -10; public static double P2A = 270;
    public static double P3X = 60; public static double P3Y = -10; public static double P3A = 270;




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
    HardwareRobot robot = new HardwareRobot(); //initialize RR hardware and software

//________________________________________________________

    @Override
    public void runOpMode() {
        //-----------------------------------------------------------

        robot.init(hardwareMap);  //initialize our hardwaremap
        //-----------------------------------------------------------
        // Vuforia Engine Run Commands and zoom
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.5, 10.0 / 9.0);
        }

        //-----------------------------------------------------------
        //Map All Paths for RoadRunner during initialization
        //All wheel based motion is defined here but is executed below.
        //Make changes here to change where the robot moves to.

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //preloaded cone delivery

        Pose2d startPose = new Pose2d(36, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //strafe away from junction

        Trajectory strafeRight = drive.trajectoryBuilder(startPose)
                .strafeRight(1)
                .addDisplacementMarker(() -> {
                    lift(1, 28);
                })
                .build();


        //approach mid goal 1


        Trajectory forward = drive.trajectoryBuilder(strafeRight.end())//strafeRight.end()


                .forward(18)

                .build();



        //place cone onto mid goal 1

        Trajectory toMidGoal1 = drive.trajectoryBuilder(forward.end())

                .lineToLinearHeading(new Pose2d (43, -26, Math.toRadians(45)))
                .build();


        //retreat from mid goal 1

        Trajectory backUp = drive.trajectoryBuilder(toMidGoal1.end())
                .back(3)
                .build();



        //strafe away from junction 1

        Trajectory clearTerminal = drive.trajectoryBuilder(backUp.end())
                .strafeRight(-9)
                .build();



        //approach line 1

        Trajectory positiontoLine1 = drive.trajectoryBuilder(clearTerminal.end())
                .lineToLinearHeading(new Pose2d(35, -12, Math.toRadians(178)))
                .build();



        //get cone off stack 1

        Trajectory toStack1 = drive.trajectoryBuilder(positiontoLine1.end())
                .addSpatialMarker(new Vector2d(12, -12), () -> {
                    robot.servorelease.setPosition(.5);
                })
                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(178)))
                .build();



        //back away from stack 1

        Trajectory backStack1 = drive.trajectoryBuilder(toStack1.end())
                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(20)))
                .build();



        //score cone 1

        Trajectory toMidGoal2 = drive.trajectoryBuilder(backStack1.end())
                .lineToLinearHeading(new Pose2d (44, -24, Math.toRadians(20)))
                .build();



        //back away from mid goal 2

        Trajectory backUp2 = drive.trajectoryBuilder(toMidGoal2.end())
                .back(4)
                .build();


        //approach line 2

        Trajectory positiontoLine2 = drive.trajectoryBuilder(backUp2.end())
                .lineToLinearHeading(new Pose2d(35, -11, Math.toRadians(178)))
                .build();



        //get cone 2

        Trajectory toStack2 = drive.trajectoryBuilder(positiontoLine2.end())
                .addSpatialMarker(new Vector2d(12, -12), () -> {
                    robot.servorelease.setPosition(.5);
                })
                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(178)))
                .build();



        //retreat from stack 2

        Trajectory backStack2 = drive.trajectoryBuilder(toStack2.end())
                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(20)))
                .build();



        //score cone 2

        Trajectory toMidGoal3 = drive.trajectoryBuilder(backStack2.end())
                .lineToLinearHeading(new Pose2d (44, -24, Math.toRadians(20)))
                .build();




        // back off stack 3

        Trajectory backUp3 = drive.trajectoryBuilder(toMidGoal3.end())
                .back(7)
                .build();


        //strafe away from junctions


        Trajectory strafeAway = drive.trajectoryBuilder(backUp3.end())
                .strafeRight(-15)
                .addDisplacementMarker(() -> {
                    lift(.5, -5);
                })
                .build();



        /*
        //approach stack 3

        Trajectory toStack3 = drive.trajectoryBuilder(backUp3.end())
                .lineToLinearHeading(new Pose2d(62, -8, Math.toRadians(0)))
                .build();



        //score cone 3

        Trajectory toMidGoal4 = drive.trajectoryBuilder(toStack3.end())
                .lineToLinearHeading(new Pose2d (34, -20, Math.toRadians(225)))
                .build();




        //back off junction FINAL

        Trajectory backUp4 = drive.trajectoryBuilder(toMidGoal4.end())
                .back(7)
                .build();
        */




        //Trajectory parking1 = drive.trajectoryBuilder(backUp3.end())
        Trajectory parking1 = drive.trajectoryBuilder(strafeAway.end())
                .lineToLinearHeading(new Pose2d (16, -10, Math.toRadians(270)))
                .build();
        Trajectory parking2 = drive.trajectoryBuilder(strafeAway.end())
                .lineToLinearHeading(new Pose2d (38, -10, Math.toRadians(270)))
                .build();
        Trajectory parking3 = drive.trajectoryBuilder(strafeAway.end())
                .lineToLinearHeading(new Pose2d (60, -10, Math.toRadians(270)))
                .build();
        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);

        telemetry.addData("Initiliazation Complete", "waiting for start");
        telemetry.update();

        Trajectory sleeve = parking1;
        //---------------------------------------- START COMMAND----------------------------
        waitForStart();
        //---------------------------------------- START COMMAND----------------------------
        if (isStopRequested())  return;

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
                            sleeve = parking1;
                        }
                        if (recognition.getLabel() == "2 Bulb") {
                            x = 2;
                            sleeve = parking2;
                        }
                        if (recognition.getLabel() == "3 Panel") {
                            x = 3;
                            sleeve = parking3;
                        }
                    }
                }
                telemetry.update();
            }
        }
        //----------------------------------------Trajectory Execution---------------------------
        // this section executes RR paths that were built above.
        // here we can inject servo and lift commands.
        drive.followTrajectory(strafeRight);
        drive.followTrajectory(forward);
        drive.followTrajectory(toMidGoal1);
        lift(.5, -3);
        sleep(500);
        robot.servorelease.setPosition(.35); //release Cone
        drive.followTrajectory(backUp);
        robot.servorelease.setPosition(.5); //intake release returned
        lift(1, -19);
        robot.servorelease.setPosition(.35);
        drive.followTrajectory(clearTerminal);
        drive.followTrajectory(positiontoLine1);
        drive.followTrajectory(toStack1);
        robot.servorelease.setPosition(.5);

        lift(.5, 23);
        robot.rightintake.setPower(.1);
        robot.leftintake.setPower(-.1);
        sleep(500);
        robot.rightintake.setPower(0);
        robot.leftintake.setPower(0);
        drive.followTrajectory(backStack1);
        drive.followTrajectory(toMidGoal2);
        lift(.5, -4);
        sleep(500);
        robot.servorelease.setPosition(.35); //release Cone
        drive.followTrajectory(backUp2);
        robot.servorelease.setPosition(.5); //intake release returned
        lift(1, -21);
        robot.servorelease.setPosition(.35);
        drive.followTrajectory(positiontoLine2);
        drive.turn(Math.toRadians(0));
        drive.followTrajectory(toStack2);
        robot.servorelease.setPosition(.5);
        lift(.5, 25);
        robot.rightintake.setPower(.1);
        robot.leftintake.setPower(-.1);
        sleep(500);
        robot.rightintake.setPower(0);
        robot.leftintake.setPower(0);
        drive.followTrajectory(backStack2);
        drive.followTrajectory(toMidGoal3);
        lift(.5, -5);
        sleep(500);
        robot.servorelease.setPosition(.35); //release Cone
        drive.followTrajectory(backUp3);
        robot.servorelease.setPosition(.5); //intake release returned
        lift(1, -18);
        // robot.rightintake.setPower(1);
        // robot.leftintake.setPower(-1);
        robot.servorelease.setPosition(.35);
    /*drive.followTrajectory(toStack3);
        robot.rightintake.setPower(0);
        robot.leftintake.setPower(0);
        robot.servorelease.setPosition(.5);
        lift(.5, 22);
        sleep(500);
    drive.followTrajectory(toMidGoal4);
        lift(.1, -4);
        sleep(1500);
        robot.servorelease.setPosition(.35); //release Cone
    drive.followTrajectory(backUp4);
        robot.servorelease.setPosition(.5); //intake release returned
        */
        drive.followTrajectory(strafeAway);
        drive.followTrajectory(sleeve);

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


            robot.liftleft.setPower(Math.abs(power));
            robot.liftright.setPower(Math.abs(power));


        }

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
}

