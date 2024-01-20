package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

/**
 * This 2023-2024 OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Disabled
@Autonomous
public class groundsRed extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * {@link #tfod} is the variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private static final String TFOD_MODEL_ASSET = "red.tflite";

    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/red.tflite";

    private static final String[] LABELS = {
            "Red Box",
    };

    private DcMotorEx pivotPixel, pivotArm, lift, pixel;

    private TouchSensor touchSensor;

    private static double BACKVEL = 15;

    private static double BACKACCEL = 15;

    private static double SETVEL = 7.5;

    private static double SETACCEL = 7.5;

    private static double SETD = 0;

    @Override
    public void runOpMode() {

        pivotPixel = hardwareMap.get(DcMotorEx.class, "pivotPixel");
        pivotArm = hardwareMap.get(DcMotorEx.class, "pivotArm");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        pixel = hardwareMap.get(DcMotorEx.class, "pixel");

        pivotPixel.setDirection(DcMotor.Direction.FORWARD);
        pivotArm.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);
        pixel.setDirection(DcMotor.Direction.REVERSE);

        touchSensor = hardwareMap.get(TouchSensor .class, "touchSensor");

        pivotPixel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pixel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivotPixel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pixel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initTfod();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        TrajectorySequence traj0_0 = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(10)
                .forward(23,
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .back(4)
                .turn(Math.toRadians(90))
                .back(16,
                        SampleMecanumDrive.getVelocityConstraint(BACKVEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(BACKACCEL))
                .back(10,
                        SampleMecanumDrive.getVelocityConstraint(SETVEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(SETVEL))
                .forward(SETD+3.5)
                .strafeRight(4.5-6)
                .build();

        TrajectorySequence traj0_0ii = drive.trajectorySequenceBuilder(traj0_0.end())
                .back(SETD+1.5,
                        SampleMecanumDrive.getVelocityConstraint(BACKVEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(BACKACCEL))
                .build();

        TrajectorySequence traj0_0i = drive.trajectorySequenceBuilder(traj0_0ii.end())
                .forward(1)
                .build();

        TrajectorySequence traj0_1 = drive.trajectorySequenceBuilder(traj0_0i.end())
                .strafeRight(20+6)
                .back(10)
                .build();

        TrajectorySequence traj1_0 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(23)
                .back(4)
                .turn(Math.toRadians(90))
                .back(26,
                        SampleMecanumDrive.getVelocityConstraint(BACKVEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(BACKACCEL))
                .back(10,
                        SampleMecanumDrive.getVelocityConstraint(SETVEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(SETVEL))
                .forward(SETD+3.5)
                .strafeRight(4.5)
                .build();

        TrajectorySequence traj1_0ii = drive.trajectorySequenceBuilder(traj1_0.end())
                .back(SETD+1.5,
                        SampleMecanumDrive.getVelocityConstraint(BACKVEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(BACKACCEL))
                .build();

        TrajectorySequence traj1_0i = drive.trajectorySequenceBuilder(traj1_0ii.end())
                .forward(1)
                .build();

        TrajectorySequence traj1_1 = drive.trajectorySequenceBuilder(traj1_0i.end())
                .strafeRight(20)
                .back(10)
                .build();

        TrajectorySequence traj2_0 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(11.5)
                .turn(Math.toRadians(45))
                .forward(10.5)
                .back(10.5)
                .turn(Math.toRadians(-45))
                .forward(7.5)
                .turn(Math.toRadians(90))
                .back(26,
                        SampleMecanumDrive.getVelocityConstraint(BACKVEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(BACKACCEL))
                .back(10,
                        SampleMecanumDrive.getVelocityConstraint(SETVEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(SETVEL))
                .forward(SETD+3.5)
                .strafeRight(4.5+6)
                .build();

        TrajectorySequence traj2_0ii = drive.trajectorySequenceBuilder(traj2_0.end())
                .back(SETD+1.5,
                        SampleMecanumDrive.getVelocityConstraint(BACKVEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(BACKACCEL))
                .build();

        TrajectorySequence traj2_0i = drive.trajectorySequenceBuilder(traj2_0ii.end())
                .forward(1)
                .build();

        TrajectorySequence traj2_1 = drive.trajectorySequenceBuilder(traj2_0i.end())
                .strafeRight(20-6)
                .back(10)
                .build();

        int result = -1;

        while (result == -1) {

            result = telemetryTfod();
            // Push telemetry to the Driver Station.
            telemetry.update();
            // Share the CPU;
            sleep(20);
        }
        visionPortal.stopStreaming();
        pivotArm.setPower(1);
        pivotPixel.setPower(0.5);
        sleep(500);
        pivotArm.setPower(0);
        pixel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        waitForStart();

        if(isStopRequested()) return;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (result == 0) {
                    drive.followTrajectorySequence(traj0_0);
                    pivotArm.setPower(1);
                    sleep(2000);
                    drive.followTrajectorySequence(traj0_0ii);
                    pixel.setPower(-0.3);
                    sleep(2000);
                    pixel.setPower(0);
                    drive.followTrajectorySequence(traj0_0i);
                    pivotPixel.setPower(-1);
                    sleep(1000);
                    pivotArm.setPower(-1);
                    sleep(2000);
                    pivotArm.setPower(0);
                    pivotPixel.setPower(0);
                    drive.followTrajectorySequence(traj0_1);
                } else if (result == 1) {
                    drive.followTrajectorySequence(traj1_0);
                    pivotArm.setPower(1);
                    sleep(2000);
                    drive.followTrajectorySequence(traj1_0ii);
                    pixel.setPower(-0.3);
                    sleep(2000);
                    pixel.setPower(0);
                    drive.followTrajectorySequence(traj1_0i);
                    pivotPixel.setPower(-1);
                    sleep(1000);
                    pivotArm.setPower(-1);
                    sleep(2000);
                    pivotArm.setPower(0);
                    pivotPixel.setPower(0);
                    drive.followTrajectorySequence(traj1_1);
                } else if (result == 2) {
                    drive.followTrajectorySequence(traj2_0);
                    pivotArm.setPower(1);
                    sleep(2000);
                    drive.followTrajectorySequence(traj2_0ii);
                    pixel.setPower(-0.3);
                    sleep(2000);
                    pixel.setPower(0);
                    drive.followTrajectorySequence(traj2_0i);
                    pivotPixel.setPower(-1);
                    sleep(1000);
                    pivotArm.setPower(-1);
                    sleep(2000);
                    pivotArm.setPower(0);
                    pivotPixel.setPower(0);
                    drive.followTrajectorySequence(traj2_1);
                }

                sleep(999999);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName("blue.tflite")
                .setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
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
        builder.setCameraResolution(new Size(1280,720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

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
        tfod.setMinResultConfidence(0.80f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Function to add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private int telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        int res = -1;
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            if (x < 500) {
                res = 2;
                // zone 1
            } else if (x >= 500 && x < 780) {
                res = 1;
                // zone 2
            } else if (x >= 780) {
                res = 0;
                // zone 3
            }
            telemetry.addData("Res","%d",res);
        }   // end for() loop
        return res;
    }   // end method telemetryTfod()

}   // end class
