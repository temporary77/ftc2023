package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous
public class FINALREDSHORTCUT extends LinearOpMode {

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

    private static double BACKVEL = 25;

    private static double BACKACCEL = 25;

    private static double SETVEL = 7.5;

    private static double SETACCEL = 7.5;

    private static double SETD = 3.5;

    private ElapsedTime runtime = new ElapsedTime();

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
                .forward(11.5)
                .turn(Math.toRadians(-45))
                .forward(10.5)
                .back(10.5)
                .turn(Math.toRadians(+45+90))
                .strafeLeft(9.5)
                .build();

        // 8 dist 5.5 ini 17 ref 22.5 mid
        TrajectorySequence traj0_1 = drive.trajectorySequenceBuilder(traj0_0.end())
                .back(-17+60,
                        SampleMecanumDrive.getVelocityConstraint(BACKVEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(BACKACCEL))
                .strafeRight(2-1+6+20-6)
                .build();

        TrajectorySequence traj0_1ii = drive.trajectorySequenceBuilder(traj0_1.end())
                .back(17+17,
                        SampleMecanumDrive.getVelocityConstraint(SETVEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(SETACCEL))
                .build();

        TrajectorySequence traj0_1i = drive.trajectorySequenceBuilder(traj0_1ii.end())
                .forward(1)
                .build();

        TrajectorySequence traj0_2 = drive.trajectorySequenceBuilder(traj0_1i.end())
                .strafeRight(20+6)
                .back(10)
                .build();

        TrajectorySequence traj1_0 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(23)
                .back(4)
                .turn(Math.toRadians(90))
                .strafeLeft(17)
                .build();

        // 8 dist 5.5 ini 17 ref 22.5 mid
        TrajectorySequence traj1_1 = drive.trajectorySequenceBuilder(traj1_0.end())
                .back(-17+60,
                        SampleMecanumDrive.getVelocityConstraint(BACKVEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(BACKACCEL))
                .strafeRight(6+20)
                .build();

        TrajectorySequence traj1_1ii = drive.trajectorySequenceBuilder(traj1_1.end())
                .back(17+17,
                        SampleMecanumDrive.getVelocityConstraint(SETVEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(SETACCEL))
                .build();

        TrajectorySequence traj1_1i = drive.trajectorySequenceBuilder(traj1_1ii.end())
                .forward(1)
                .build();

        TrajectorySequence traj1_2 = drive.trajectorySequenceBuilder(traj1_1i.end())
                .strafeRight(20)
                .back(10)
                .build();

        TrajectorySequence traj2_0 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(22,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(15))
                .strafeLeft(12,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .back(5,
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .turn(Math.toRadians(90))
                .back(12)
                .strafeLeft(14)
                .build();

        // 8 dist 5.5 ini 17 ref 22.5 mid
        TrajectorySequence traj2_1 = drive.trajectorySequenceBuilder(traj2_0.end())
                .back(-17+60,
                        SampleMecanumDrive.getVelocityConstraint(BACKVEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(BACKACCEL))
                .strafeRight(5+20+6)
                .build();

        TrajectorySequence traj2_1ii = drive.trajectorySequenceBuilder(traj2_1.end())
                .back(17+17,
                        SampleMecanumDrive.getVelocityConstraint(SETVEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(SETACCEL))
                .build();

        TrajectorySequence traj2_1i = drive.trajectorySequenceBuilder(traj2_1ii.end())
                .forward(1)
                .build();

        TrajectorySequence traj2_2 = drive.trajectorySequenceBuilder(traj2_1i.end())
                .strafeRight(20-6)
                .back(10)
                .build();

        runtime.reset();

        int result = -1;

        while (opModeInInit()) {
            result = telemetryTfod();
            // Push telemetry to the Driver Station.
            telemetry.addData("Time Scanned",runtime.time());
            telemetry.addData("RESULT","%d",result);
            telemetry.update();
            // Share the CPU;
            sleep(20);
        }
        waitForStart();
        if(isStopRequested()) return;

        ElapsedTime runtime = new ElapsedTime();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                visionPortal.stopStreaming();
                pivotArm.setPower(1);
                pivotPixel.setPower(0.5);
                sleep(500);
                pivotArm.setPower(0);
                pixel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                if (result == 0) {
                    drive.followTrajectorySequence(traj0_0);
                    pivotPixel.setPower(-1);
                    sleep(750);
                    while(runtime.time() < 15) {
                        sleep(20);
                    }
                    drive.followTrajectorySequence(traj0_1);
                    pivotPixel.setPower(1);
                    sleep(600);
                    pivotArm.setPower(1);
                    sleep(1500);
                    drive.followTrajectorySequence(traj0_1ii);
                    pixel.setPower(-0.3);
                    sleep(2000);
                    pixel.setPower(0);
                    drive.followTrajectorySequence(traj0_1i);
                    pivotPixel.setPower(-1);
                    sleep(1000);
                    pivotArm.setPower(-1);
                    sleep(2000);
                    pivotArm.setPower(0);
                    pivotPixel.setPower(0);
//                    drive.followTrajectorySequence(traj0_2);
                } else if (result == 1) {
                    drive.followTrajectorySequence(traj1_0);
                    pivotPixel.setPower(-1);
                    sleep(750);
                    while(runtime.time() < 15) {
                        sleep(20);
                    }
                    drive.followTrajectorySequence(traj1_1);
                    pivotPixel.setPower(1);
                    sleep(600);
                    pivotArm.setPower(1);
                    sleep(1500);
                    drive.followTrajectorySequence(traj1_1ii);
                    pixel.setPower(-0.3);
                    sleep(2000);
                    pixel.setPower(0);
                    drive.followTrajectorySequence(traj1_1i);
                    pivotPixel.setPower(-1);
                    sleep(1000);
                    pivotArm.setPower(-1);
                    sleep(2000);
                    pivotArm.setPower(0);
                    pivotPixel.setPower(0);
//                    drive.followTrajectorySequence(traj1_2);
                } else if (result == 2) {
                    drive.followTrajectorySequence(traj2_0);
                    pivotPixel.setPower(-1);
                    sleep(750);
                    while(runtime.time() < 15) {
                        sleep(20);
                    }
                    drive.followTrajectorySequence(traj2_1);
                    pivotPixel.setPower(1);
                    sleep(600);
                    pivotArm.setPower(1);
                    sleep(1500);
                    drive.followTrajectorySequence(traj2_1ii);
                    pixel.setPower(-0.3);
                    sleep(2000);
                    pixel.setPower(0);
                    drive.followTrajectorySequence(traj2_1i);
                    pivotPixel.setPower(-1);
                    sleep(1000);
                    pivotArm.setPower(-1);
                    sleep(2000);
                    pivotArm.setPower(0);
                    pivotPixel.setPower(0);
//                    drive.followTrajectorySequence(traj2_2);
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
                //.setModelAs'[-pectRatio(16.0 / 9.0)

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
        int res = 0;
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            if (x < 640) {
                res = 2;
                // zone 1
            } else if (x >= 640) {
                res = 1;
                // zone 2
            }
            telemetry.addData("Res","%d",res);
        }   // end for() loop
        return res;
    }   // end method telemetryTfod()

}   // end class
