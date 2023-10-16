package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "RR_BLUE_LEFT", group = "Automonous")

public class RR_Auton_Base extends LinearOpMode {
    //-----------------------------------------------------------
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera


    private static final String TFOD_MODEL_ASSET = "InitialModel22-23.tflite";
    private static final String[] LABELS = {
            "1 Bolt", // "1 Bolt"
            "2 Bulb", // "2 Bulb"
            "3 Panel" // "3 Panel"
    };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    HardwareRobot robot = new HardwareRobot(); //initialize RR hardware and software

//________________________________________________________

    @Override
    public void runOpMode() {
        //-----------------------------------------------------------

        robot.init(hardwareMap);  //initialize our hardwaremap
        //-----------------------------------------------------------
        initTfod();


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
                    lift(1, 20.5);
                })
                .build();


        //approach mid goal 1


        Trajectory forward = drive.trajectoryBuilder(strafeRight.end())//strafeRight.end()


                .forward(18)

                .build();



        //place cone onto mid goal 1

        Trajectory toMidGoal1 = drive.trajectoryBuilder(forward.end())

                .lineToLinearHeading(new Pose2d (44, -28, Math.toRadians(45)))
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
                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(177)))
                .build();



        //get cone off stack 1

        Trajectory toStack1 = drive.trajectoryBuilder(positiontoLine1.end())
                .addSpatialMarker(new Vector2d(13.75, -10), () -> {
                    robot.servorelease.setPosition(.5);
                })
                .lineToLinearHeading(new Pose2d(13.75, -9, Math.toRadians(177)))
                .build();



        //back away from stack 1

        Trajectory backStack1 = drive.trajectoryBuilder(toStack1.end())
                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(20)))
                .build();



        //score cone 1

        Trajectory toMidGoal2 = drive.trajectoryBuilder(backStack1.end())
                .lineToLinearHeading(new Pose2d (43, -24, Math.toRadians(20)))
                .build();



        //back away from mid goal 2

        Trajectory backUp2 = drive.trajectoryBuilder(toMidGoal2.end())
                .back(4)
                .build();


        //approach line 2

        Trajectory positiontoLine2 = drive.trajectoryBuilder(backUp2.end())
                .lineToLinearHeading(new Pose2d(35, -10.5, Math.toRadians(180)))
                .build();



        //get cone 2

        Trajectory toStack2 = drive.trajectoryBuilder(positiontoLine2.end())
                .addSpatialMarker(new Vector2d(13.5, -10.5), () -> {
                    robot.servorelease.setPosition(.5);
                })
                .lineToLinearHeading(new Pose2d(13.5, -10, Math.toRadians(180)))
                .build();



        //retreat from stack 2

        Trajectory backStack2 = drive.trajectoryBuilder(toStack2.end())
                .lineToLinearHeading(new Pose2d(35, -10, Math.toRadians(20)))
                .build();



        //score cone 2

        Trajectory toMidGoal3 = drive.trajectoryBuilder(backStack2.end())
                .lineToLinearHeading(new Pose2d (43, -23, Math.toRadians(20)))
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
        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);

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

                List<Recognition> updatedRecognitions = tfod.getRecognitions();
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
        lift(1, -14);
        robot.servorelease.setPosition(.35);
        drive.followTrajectory(clearTerminal);
        drive.followTrajectory(positiontoLine1);
        drive.followTrajectory(toStack1);
        robot.servorelease.setPosition(.5);

        lift(.5, 17);
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
        lift(1, -13.5);
        robot.servorelease.setPosition(.35);
        drive.followTrajectory(positiontoLine2);
        drive.turn(Math.toRadians(0));
        drive.followTrajectory(toStack2);
        robot.servorelease.setPosition(.5);
        lift(.5, 17.5);
        robot.rightintake.setPower(.1);
        robot.leftintake.setPower(-.1);
        sleep(500);
        robot.rightintake.setPower(0);
        robot.leftintake.setPower(0);
        drive.followTrajectory(backStack2);
        drive.followTrajectory(toMidGoal3);
        lift(.5, -7);
        sleep(500);
        robot.servorelease.setPosition(.35); //release Cone
        drive.followTrajectory(backUp3);
        robot.servorelease.setPosition(.5); //intake release returned
        lift(1, -2);
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
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
}


