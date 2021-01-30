package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// ^^ The required packages and classes are imported into the autonomous code ^^

@Autonomous(name = "GM Auto TFOD")
public class GMAuto extends LinearOpMode {

    // Tensorflow labels are given to the ring stack size assets
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    // Import the variables from the util class for the motors and servos
    SampleMecanumDrive drivetrain;
    // Reset the ring stack size variables for use in Tensorflow
    int zero = 0;
    int one= 0;
    int four = 0;
    List<Recognition> updatedRecognitions;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     */
    private static final String VUFORIA_KEY =
            "AZT5cbD/////AAABmfJnIeddQEINhStd867KmsE/U5DxLb3la9BlLgqlAj7pwpiD/JFpwc61NP8dil/k9TpMthRa3J0OFg2P1oaCjeRLb8s4ku8mWqY142NCUbQmrnMtzCDezbfhmeXOdgONV7+oW2Nu50zXzUwG/tkR8UgWiMkSU9M3ZEyZEhaG4sscMoY/tW23IWyq6PoMwIz8aQdtc+hm68hvEES4GYTPnxz0XvyPSNcGWmBPMGWYmKRy9i7ZGJG6/L29z3Y7AP2bnQ5gzHPuWWr6plIcxNP2jvaafqTPz8Nn2tjZ7yk3KPeHmf4MLjuseGCWV8wub/+sDKt2B5/vVntltf5/0gVAb25avPL9aNOGLDHEePembGAb";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that first
        drivetrain = new SampleMecanumDrive(hardwareMap);
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2.0, 1.78);
        }

        /** Wait for the game to begin */
        waitForStart();

        //Autonomous begins
        if (opModeIsActive())
        {
            // Begin TFOD loop once autunomous starts
            // Tensorflow will run parameters 10 times to determine the ring stack size
            for(int ti = 1; ti < 10; ti++) {
                if (tfod != null) {
                    updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 0) {
                            // Empty list, no objects recognized.
                            telemetry.addData("TFOD", "No items detected.");
                            telemetry.addData("Target Zone", "A");
                            telemetry.update();
                            zero++;
                            // No rings detected, robot may move to zone A
                        } else {
                            // If list is not empty:
                            // Step through the list of recognitions and display boundary info
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                // Check label to see which target zone to go after
                                if (recognition.getLabel().equals("Single")) {
                                    telemetry.addData("Target Zone", "B");
                                    telemetry.update();
                                    one++;
                                    // One ring was detected, robot may move to zone B
                                } else if (recognition.getLabel().equals("Quad")) {
                                    telemetry.addData("Target Zone", "C");
                                    telemetry.update();
                                    four++;
                                    // Four rings were detected, robot may move to zone C
                                } else {
                                    telemetry.addData("Target Zone", "UNKNOWN");
                                    telemetry.update();
                                    // If there was an error in detection, this caption will appear
                                }
                            }
                        }
                    }
                }
            }
            /*
             * After the TFOD program has looped 10 times, the occurrence of each ring position is stored
             * The ring position (stack size) that was detected the most times will be found
             * The other variables are then discarded and the chosen ring position is executed
             */
            if (zero > one && zero > four) {
                // If 0 rings were detected, these methods will be executed
                ZoneA();
                //Trajectory shootPosition = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                //        .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(0)))
                //        .build();
                //drivetrain.followTrajectory(shootPosition);
                //drivetrain.turn(Math.toRadians(-90));
            }
            else if (one > zero && one > four) {
                // If 1 ring was detected, these methods will be executed
                ZoneB();
            }
            else if (four > zero && four > one) {
                // If 4 rings were detected, these methods will be executed
                ZoneC();
            }
        }
        //Tensorflow will shut down after the OpMode is successfully executed
        if (tfod != null) {
            tfod.shutdown();
        }
        // End of Autonomous
    }
    // > Methods for larger lists of actions are written here <

    public void ZoneA()
    {
        Trajectory shootPosition = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(65, 0, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(shootPosition);
        startFlywheel(0.44);
        sleep(1000);
        ringIndex();
        stopFlywheel();
        drivetrain.turn(Math.toRadians(-90));
        Trajectory wobbleDelivery = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(18, 0, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(wobbleDelivery);
        wobbleDrop();
        drivetrain.turn(Math.toRadians(-90));
        Trajectory wobblePickup = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(34, 0, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(wobblePickup);
        wobblePick();
        //Move back to place second wobble goal
        Trajectory wobbleDeliveryAgain = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-70, -22, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(wobbleDeliveryAgain);
        wobbleDrop();
        drivetrain.turn(Math.toRadians(-45));
        Trajectory linePark = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(25, 0, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(linePark);
    }
    public void ZoneB()
    {
        Trajectory shootPosition = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(65, 0, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(shootPosition);
        startFlywheel(0.44);
        sleep(1000);
        ringIndex();
        stopFlywheel();
        Trajectory wobbleDelivery = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(21, 18, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(wobbleDelivery);
        wobbleDrop();
        //drivetrain.turn(Math.toRadians(-150));
        Trajectory wobblePickup = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-75, -18, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(wobblePickup);
        drivetrain.turn(Math.toRadians(-90));
        Trajectory wobblePickupForward = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(4, 0, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(wobblePickupForward);
        wobblePick();
        drivetrain.turn(Math.toRadians(90));
        Trajectory wobbleDeliveryAgain = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(74, 0, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(wobbleDeliveryAgain);
        wobbleDrop();
        Trajectory linePark = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(0, -30, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(linePark);
    }
    public void ZoneC()
    {
        Trajectory shootPosition = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(65, 0, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(shootPosition);
        startFlywheel(0.44);
        sleep(1000);
        ringIndex();
        stopFlywheel();
        drivetrain.turn(Math.toRadians(-45));
        Trajectory wobbleDelivery = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(48, 0, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(wobbleDelivery);
        wobbleDrop();
        drivetrain.turn(Math.toRadians(-135));
        Trajectory wobblePickup = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(72.5, 20, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(wobblePickup);
        wobblePick();
        drivetrain.turn(Math.toRadians(180));
        //Move back to place second wobble goal
        Trajectory wobbleDeliveryAgain = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(72.5, 30, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(wobbleDeliveryAgain);
        wobbleDrop();
        Trajectory linePark = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-15, 0, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(linePark);
    }
    // Determines positions for the ring index servo
    public void ringIndex()
    {
        // Push ring out with the servo arm
        drivetrain.index.setPosition(0.25);
        sleep(100);
        // Pull servo arm back in
        drivetrain.index.setPosition(0.05);
        // Reset flywheel to max power
        startFlywheel(0.46);
        sleep(500);

        drivetrain.index.setPosition(0.25);
        sleep(100);
        drivetrain.index.setPosition(0.05);
        startFlywheel(0.50);
        sleep(500);

        drivetrain.index.setPosition(0.25);
        sleep(100);
        drivetrain.index.setPosition(0.05);
        sleep(100);
    }
    public void wobbleDrop()
    {
        // The wobble arm is brought down, then stopped
        double turn = 386 / 4;
        int valueDown = drivetrain.arm.getTargetPosition() + (int) turn;
        drivetrain.arm.setTargetPosition(valueDown);
        drivetrain.arm.setPower(0.75);
        sleep(750);
        drivetrain.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivetrain.arm.setPower(0.00);

        // The servo grabber releases the wobble
        drivetrain.botGrab.setPosition(0.10);
        drivetrain.topGrab.setPosition(0.05);
        sleep(400);

        int valueUp = drivetrain.arm.getTargetPosition() - (int) turn;
        drivetrain.arm.setTargetPosition(valueUp);
        drivetrain.arm.setPower(0.625);
        sleep(750);
        drivetrain.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivetrain.arm.setPower(0.00);
    }
    public void wobblePick()
    {
        double turn = 386 / 4;
        int valueDown = drivetrain.arm.getTargetPosition() + (int) turn;
        drivetrain.arm.setTargetPosition(valueDown);
        drivetrain.arm.setPower(0.625);
        sleep(750);
        drivetrain.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivetrain.arm.setPower(0.00);

        Trajectory wobblePickupRight = drivetrain.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(0, -5, Math.toRadians(0)))
                .build();
        drivetrain.followTrajectory(wobblePickupRight);

        drivetrain.botGrab.setPosition(0.85);
        drivetrain.topGrab.setPosition(0.80);
        sleep(600);

    }
    // Runs the flywheel at any desired power
    public void startFlywheel(double power)
    {
        drivetrain.flywheel.setPower(power);
        // The program will double check the power and make sure its is correct
        if(power > 0) {
            double currentPower = drivetrain.flywheel.getPower();
            double error = (power - currentPower);
            drivetrain.flywheel.setPower(currentPower + error);
        }
    }
    // Stops the flywheel
    public void stopFlywheel() {
        drivetrain.flywheel.setPower(0.00);
    }
    // Initialize the Vuforia localization engine.
    private void initVuforia()
    {
        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    // Initialize the TensorFlow Object Detection engine.
    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
