package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Disabled
@Autonomous
public class autonomous1 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    private DcMotorEx pivotPixel, pivotArm, lift, pixel;

    private TouchSensor touchSensor;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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
        pixel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        if(isStopRequested()) return;

        pivotPixel.setPower(1);
        sleep(750);
        pivotArm.setPower(1);
        sleep(2000);
        pixel.setPower(-0.5);
        sleep(1500);
        pixel.setPower(0);
        pivotArm.setPower(-1);
        sleep(2000);
        pivotArm.setPower(0);
        pivotPixel.setPower(-1);
        sleep(1000);
        pivotPixel.setPower(0);
        sleep(999999);
    }
}
