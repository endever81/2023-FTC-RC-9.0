package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous(name = "Blue Right", group = "Automonous")

public class RR_Auton_CS_Blue_Right extends LinearOpMode {
    //-----------------------------------------------------------
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private static final String TFOD_MODEL_ASSET = "model_23-24.tflite";  //TeamCode\build\intermediates\assets\debug  in program view
    private static final String[] LABELS = {
            "TSE"
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

        //Red Right starting position - Same for all paths
        Pose2d startPose = new Pose2d(-4, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //Approach Spike Line
        Trajectory spikeRight = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d (-3, -42, Math.toRadians(50)))
                .build();
        Trajectory spikeCenter = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d (-1, -35, Math.toRadians(90)))
                .build();
        Trajectory spikeLeft = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d (-4, -35, Math.toRadians(145)))
                .build();

        //back away from dropped pixel and spike lines
        Trajectory backFromPixelRight = drive.trajectoryBuilder(spikeRight.end())
                .lineToLinearHeading(new Pose2d (-4, -46, Math.toRadians(50)))
                .build();
        Trajectory backFromPixelCenter = drive.trajectoryBuilder(spikeCenter.end())
                .lineToLinearHeading(new Pose2d (8, -38, Math.toRadians(95)))
                .build();
        Trajectory backFromPixelLeft = drive.trajectoryBuilder(spikeLeft.end())
                .lineToLinearHeading(new Pose2d (9, -41, Math.toRadians(135)))
                .build();

        //Swing Out to Center
        Trajectory swingOutToMiddleRight = drive.trajectoryBuilder(backFromPixelRight.end())
                .lineToLinearHeading(new Pose2d (-3, -12, Math.toRadians(180)))
                .build();
        Trajectory swingOutToMiddleCenter = drive.trajectoryBuilder(backFromPixelCenter.end())
                .lineToLinearHeading(new Pose2d (14, -12, Math.toRadians(180)))
                .build();
        Trajectory swingOutToMiddleLeft = drive.trajectoryBuilder(backFromPixelLeft.end())
                .lineToLinearHeading(new Pose2d (14, -12, Math.toRadians(180)))
                .build();

        //Run up Center
        Trajectory runUpMiddleRight = drive.trajectoryBuilder(swingOutToMiddleRight.end())
                .lineToLinearHeading(new Pose2d (-60, -12, Math.toRadians(180)))
                .build();
        Trajectory runUpMiddleCenter = drive.trajectoryBuilder(swingOutToMiddleCenter.end())
                .lineToLinearHeading(new Pose2d (-60, -12, Math.toRadians(180)))
                .build();
        Trajectory runUpMiddleLeft = drive.trajectoryBuilder(swingOutToMiddleLeft.end())
                .lineToLinearHeading(new Pose2d (-60, -12, Math.toRadians(180)))
                .build();

        //Approach Backdrop
        Trajectory backDropRight = drive.trajectoryBuilder(runUpMiddleRight.end())
                .lineToLinearHeading(new Pose2d (-88, -30, Math.toRadians(180)))
                .build();
        Trajectory backDropCenter = drive.trajectoryBuilder(runUpMiddleCenter.end())
                .lineToLinearHeading(new Pose2d (-86.5, -37.5, Math.toRadians(180)))
                .build();
        Trajectory backDropLeft = drive.trajectoryBuilder(runUpMiddleLeft.end())
                .lineToLinearHeading(new Pose2d (-86.5, -44.5, Math.toRadians(180)))
                .build();

        //Back from Backdrop
        Trajectory backUpRight = drive.trajectoryBuilder(backDropRight.end())
                .lineToLinearHeading(new Pose2d (-74, -34, Math.toRadians(180)))
                .build();
        Trajectory backUpCenter = drive.trajectoryBuilder(backDropCenter.end())
                .lineToLinearHeading(new Pose2d (-74, -39, Math.toRadians(180)))
                .build();
        Trajectory backUpLeft = drive.trajectoryBuilder(backDropLeft.end())
                .lineToLinearHeading(new Pose2d (-74, -43, Math.toRadians(180)))
                .build();

        //Move to corner and park
        Trajectory toCornerRight = drive.trajectoryBuilder(backUpRight.end())
                .lineToLinearHeading(new Pose2d (-94, -14, Math.toRadians(0))) //Alternate end point center field x-75 y-10
                .build();
        Trajectory toCornerCenter = drive.trajectoryBuilder(backUpCenter.end())
                .lineToLinearHeading(new Pose2d (-94, -15, Math.toRadians(0)))
                .build();
        Trajectory toCornerLeft = drive.trajectoryBuilder(backUpLeft.end())
                .lineToLinearHeading(new Pose2d (-94, -15, Math.toRadians(0)))
                .build();

                //.strafeRight(1)
                //.forward(18)
                //.back(4)
                //.addDisplacementMarker(() -> {
                //     lift(1, 20.5);
                //       })
                //.build();

       // Trajectory toStack1 = drive.trajectoryBuilder(positiontoLine1.end())
              //  .addSpatialMarker(new Vector2d(13.75, -10), () -> {
             //       robot.servorelease.setPosition(.5);
            //         })
              //  .lineToLinearHeading(new Pose2d(13.75, -9, Math.toRadians(177)))
               // .build();



        robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);

        telemetry.addData("Initiliazation Complete", "waiting for start");
        telemetry.update();

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
                    x = 0; //TSE on Right Spike Tape

                    for (Recognition recognition : updatedRecognitions) {
                        double xPos = recognition.getLeft();

                        if (xPos <= 200) {
                            x = 1; // TSE on Left Spike Tape
                        }
                        if (xPos > 200) {
                            x = 2; // TSE on Center Spike Tape
                        }

                    }
                }
                telemetry.update();
            }
        }
        //----------------------------------------Trajectory Execution---------------------------
        // this section executes RR paths that were built above.
        // here we can inject servo and lift commands.

        // Right Spike Tape Trajectory Path
        if (x == 0) {
            telemetry.addData("Right Spike", 10);
            telemetry.update();
            sleep(2000);
            robot.leftRotate.setPosition(0.5); //lower intake
            robot.rightRotate.setPosition(0.5);
            drive.followTrajectory(spikeRight);
            robot.rightGrab.setPosition(0.4);//release pixel
            sleep(2000);
            robot.leftRotate.setPosition(0.3); //raise intake
            robot.rightRotate.setPosition(0.67);
            drive.followTrajectory(backFromPixelRight);
            robot.rightGrab.setPosition(0.5);//relax grab right
            drive.followTrajectory(swingOutToMiddleRight);
            drive.followTrajectory(runUpMiddleRight);
            lift(1, 5);
            drive.followTrajectory(backDropRight);
            robot.leftGrab.setPosition(0.6);//release pixel
            sleep(2000);
            robot.leftRotate.setPosition(0.18); //tuck intake
            robot.rightRotate.setPosition(0.8);
            drive.followTrajectory(backUpRight);
            robot.leftGrab.setPosition(0.5);//relax grab left

            lift(1, -5);
            drive.followTrajectory(toCornerRight);

//            drive.turn(Math.toRadians(0));
        }

        //Left Spike Tape
        if (x == 1) {
            telemetry.addData("Left Spike", 10);
            telemetry.update();
            sleep(3000);
            robot.leftRotate.setPosition(0.5); //lower intake
            robot.rightRotate.setPosition(0.5);
            drive.followTrajectory(spikeLeft);
            robot.rightGrab.setPosition(0.4);//release pixel
            sleep(2000);
            robot.leftRotate.setPosition(0.3); //raise intake
            robot.rightRotate.setPosition(0.67);
            drive.followTrajectory(backFromPixelLeft);
            robot.rightGrab.setPosition(0.5);//relax grab right
            drive.followTrajectory(swingOutToMiddleLeft);
            drive.followTrajectory(runUpMiddleLeft);
            lift(.5, 5);
            drive.followTrajectory(backDropLeft);
            robot.leftGrab.setPosition(0.6);//release pixel
            sleep(2000);
            robot.leftRotate.setPosition(0.18); //tuck intake
            robot.rightRotate.setPosition(0.8);
            drive.followTrajectory(backUpLeft);
            robot.leftGrab.setPosition(0.5);//relax grab left
            lift(1, -5);
            drive.followTrajectory(toCornerLeft);

//            drive.turn(Math.toRadians(0));
        }

        // Center Spike Tape
        if (x == 2) {
            telemetry.addData("Center Spike", 10);
            telemetry.update();
            sleep(3000);
            robot.leftRotate.setPosition(0.5); //lower intake
            robot.rightRotate.setPosition(0.5);
            drive.followTrajectory(spikeCenter);
            robot.rightGrab.setPosition(0.4);//release pixel
            sleep(2000);
            robot.leftRotate.setPosition(0.3); //raise intake
            robot.rightRotate.setPosition(0.67);
            drive.followTrajectory(backFromPixelCenter);
            robot.rightGrab.setPosition(0.5);//relax grab right
            drive.followTrajectory(swingOutToMiddleCenter);
            drive.followTrajectory(runUpMiddleCenter);
            lift(.5, 5);
            drive.followTrajectory(backDropCenter);
            robot.leftGrab.setPosition(0.6);
            sleep(2000);
            robot.leftRotate.setPosition(0.18); //tuck intake
            robot.rightRotate.setPosition(0.8);
            drive.followTrajectory(backUpCenter);
            robot.leftGrab.setPosition(0.4);//relax grab left
            lift(1, -5);
            drive.followTrajectory(toCornerCenter);

//            drive.turn(Math.toRadians(0));
        }


    }

    public void lift(double power, double inches)
    {
        int newLiftTargetRight;
        int newLiftTargetLeft;


        if (opModeIsActive()) {


            newLiftTargetLeft = robot.liftleft.getCurrentPosition() + (int) (inches * (1140 / (3.5 * 3.1415)));
            newLiftTargetRight = robot.liftright.getCurrentPosition() + (int) (inches * (1140 / (3.5 * 3.1415)));

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
                .setModelAssetName(TFOD_MODEL_ASSET)
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

