package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class teleop1 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    private DcMotorEx pivotPixel, pivotArm, lift, pixel;

    private TouchSensor touchSensor;

    private Servo drone;

    @Override
    public void init() {
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        pivotPixel = hardwareMap.get(DcMotorEx.class, "pivotPixel");
        pivotArm = hardwareMap.get(DcMotorEx.class, "pivotArm");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        pixel = hardwareMap.get(DcMotorEx.class, "pixel");

        pivotPixel.setDirection(DcMotor.Direction.FORWARD);
        pivotArm.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);
        pixel.setDirection(DcMotor.Direction.REVERSE);

        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        drone = hardwareMap.get(Servo.class, "drone");

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotPixel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pixel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drone.setPosition(0.735);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotPixel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pixel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    boolean fired = false;

    boolean pressed = false;

    boolean xdown = false;

    boolean bdown = false;

    double pixPower = 0;

    boolean reset = true;

    public enum DownState {
        DLIFT_START,
        DLIFT_DOWN,
        DLIFT_SLOW,

        DLIFT_SPACE,
        DPIXEL_DOWN,

        DLIFT_READY,
    }

    public enum UpState {
        ULIFT_START,
        ULIFT_ULTIMATE,

        ULIFT_SLOW,

        UPIXEL_SET,
    }

    DownState dState = DownState.DLIFT_START;

    UpState uState = UpState.ULIFT_START;

    ElapsedTime dTimer = new ElapsedTime();

    ElapsedTime uTimer = new ElapsedTime();

    double pArmPower = 0;

    double pivPixPower = 0;

    double liftPower = 0;

    boolean allowlevitate = false;
    boolean levitate = false;

    @Override
    public void loop() {

        double max;

        double axial = 0.7*-gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = 0.7*gamepad1.right_stick_x;
        double yaw = 0.7*gamepad1.left_stick_x;//-gamepad1.left_trigger+gamepad1.right_trigger;

        if (gamepad1.dpad_right)lateral += 0.3;
        if (gamepad1.dpad_left)lateral -= 0.3;
        if (gamepad1.dpad_up)axial += 0.3;
        if (gamepad1.dpad_down)axial -= 0.3;
        if (gamepad1.x)yaw -= 0.3;
        if (gamepad1.b)yaw += 0.3;

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftRearPower = axial - lateral + yaw;
        double rightRearPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightRearPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftRearPower /= max;
            rightRearPower /= max;
        }

        double pArmCurPos = pivotArm.getCurrentPosition();

        double pArmUp = gamepad2.right_trigger;
        double pArmDown = gamepad2.left_trigger;

        if ((pArmUp > 0) ^ (pArmDown > 0)) {
            if (pArmUp > 0) {
                levitate = true;
                if (pivotArm.getCurrentPosition() > 285-20) {
                    pArmPower = pArmUp*0.7;
                } else {
                    pArmPower = pArmUp;
                }
            } else {
                levitate = false;
                pArmPower = -pArmDown;
            }
        } else {
            if (dState == DownState.DLIFT_START && uState == UpState.ULIFT_START) {
                pArmPower = 0;
            }
//            if (pivotArm.getCurrentPosition() >= 190) {
//                pArmPower = 0;
//            } else {
//                pArmPower = Math.sin(3.1415926 / 2 * ((285 - pivotArm.getCurrentPosition()) / 285.0)) * 0.2;
//            }
//            if (pivotArm.getCurrentPosition() < TargetPos-10) {
//                pArmPower = 0.3;
//            } else if (pivotArm.getCurrentPosition() > TargetPos+10) {
//                pArmPower = -0.3;
//            } else {
//                pArmPower = 0;
//            }
        }

        if(touchSensor.isPressed()) {
            allowlevitate = true;
            if (reset) {
                pivotArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (pArmPower < 0)pArmPower = 0;
            reset = false;
        }

        boolean pivPixUp = gamepad2.y;
        boolean pivPixDown = gamepad2.a;

        if (pivPixUp^pivPixDown) {
            if (pivPixUp) {
                pivPixPower = 0.7;
            } else if (pivPixDown) {
                pivPixPower = -0.4;
            }
        } else {
            if (dState == DownState.DLIFT_START && uState == UpState.ULIFT_START) {
                pivPixPower = 0;
            }
        }

        if (gamepad2.x) {
            if (!xdown) {
                xdown = true;
                if (pixPower == 0 || pixPower == -0.7) {
                    pixPower = 0.7;
                } else {
                    pixPower = 0;
                }
            }
        } else {
            xdown = false;
        }

        if (gamepad2.b) {
            if (!bdown) {
                bdown = true;
                if (pixPower == 0 || pixPower == 0.7) {
                    pixPower = -0.7;
                } else {
                    pixPower = 0;
                }
            }
        } else {
            bdown = false;
        }

        if (dState != DownState.DPIXEL_DOWN) {
            liftPower = -gamepad2.left_stick_y;
        } 
//        if (liftExtendo^liftRetracto) {
//            if (liftRetracto && lift.getCurrentPosition() <= 0) {
//                liftPower = 0.5;
//            } else if (liftExtendo && lift.getCurrentPosition() >= -3800) {
//                liftPower = -0.5;
//            }
//        }

        if (gamepad1.y) {
            if (!pressed) {
                pressed = true;
                if (!fired) {
                    fired = true;
                    drone.setPosition(0.5);
                } else {
                    fired = false;
                    drone.setPosition(0.735);
                }
            }
        } else {
            pressed = false;
        }

        boolean ISTOP = gamepad2.dpad_up & gamepad2.dpad_down;

        switch (dState) {
            case DLIFT_START:
                telemetry.addLine("WAIT: DLIFT_START");
                if (pivPixUp || pivPixDown || pArmUp > 0 || pArmDown > 0 || gamepad2.dpad_up || ISTOP) {
                    dState = DownState.DLIFT_START;
                    break;
                }
                if (gamepad2.dpad_down) {
                    pivPixPower = 0;
                    pArmPower = -0.7;
                    dState = DownState.DLIFT_DOWN;
                }
                break;
            case DLIFT_DOWN:
                telemetry.addLine("WAIT: DLIFT_DOWN");
                if (pivPixUp || pivPixDown || pArmUp > 0 || pArmDown > 0 || gamepad2.dpad_up || ISTOP) {
                    dState = DownState.DLIFT_START;
                    break;
                }
                if (pivotArm.getCurrentPosition() < 160) {
                    pArmPower = -0.4;
                    dState = DownState.DLIFT_SLOW;
                }
                break;
            case DLIFT_SLOW:
                telemetry.addLine("WAIT: DLIFT_SLOW");
                if (pivPixUp || pivPixDown || pArmUp > 0 || pArmDown > 0 || gamepad2.dpad_up || ISTOP) {
                    dState = DownState.DLIFT_START;
                    break;
                }
                if (touchSensor.isPressed() || pivotArm.getCurrentPosition() < 15) {
                    pArmPower = 1;
                    dState = DownState.DLIFT_SPACE;
                }
                break;
            case DLIFT_SPACE:
                telemetry.addLine("WAIT: DLIFT_SPACE");
                if (pivPixUp || pivPixDown || pArmUp > 0 || pArmDown > 0 || gamepad2.dpad_up || ISTOP) {
                    dState = DownState.DLIFT_START;
                    break;
                }
                if (pivotArm.getCurrentPosition() > 50) {
                    pArmPower = 0;
                    liftPower = 0.3;
                    pivPixPower = -0.4;
                    dTimer.reset();
                    dState = DownState.DPIXEL_DOWN;
                }
                break;
            case DPIXEL_DOWN:
                telemetry.addLine("WAIT: DPIXEL_DOWN");
                if (pivPixUp || pivPixDown || pArmUp > 0 || pArmDown > 0 || gamepad2.dpad_up || ISTOP) {
                    dState = DownState.DLIFT_START;
                    break;
                }
                if (dTimer.time() > 0.5) {
                    pArmPower = -0.2;
                    liftPower = 0;
                    pivPixPower = 0;
                    dState = DownState.DLIFT_READY;
                }
                break;
            case DLIFT_READY:
                telemetry.addLine("WAIT: DLIFT_READY");
                if (pivPixUp || pivPixDown || pArmUp > 0 || pArmDown > 0 || gamepad2.dpad_up || ISTOP) {
                    dState = DownState.DLIFT_START;
                    break;
                }
                if (pivotArm.getCurrentPosition() <= 30) {
                    pArmPower = 0;
                    pivPixPower = 0;
                    levitate = true;
                    pixPower = 0.7;
                    dState = DownState.DLIFT_READY;
                }
                break;
            default:
                telemetry.addLine("ERROR DOWN");
                dState = DownState.DLIFT_START;
        }

        switch (uState) {
            case ULIFT_START:
                telemetry.addLine("WAIT: ULIFT_START");
                if (pivPixUp || pivPixDown || pArmUp > 0 || pArmDown > 0 || gamepad2.dpad_down || ISTOP) {
                    uState = UpState.ULIFT_START;
                    break;
                }
                if (gamepad2.dpad_up) {
                    pArmPower = 1;
                    uState = UpState.ULIFT_ULTIMATE;
                }
                break;
            case ULIFT_ULTIMATE:
                telemetry.addLine("WAIT: ULIFT_ULTIMATE");
                if (pivPixUp || pivPixDown || pArmUp > 0 || pArmDown > 0 || gamepad2.dpad_down || ISTOP) {
                    uState = UpState.ULIFT_START;
                    break;
                }
                if (pivotArm.getCurrentPosition() >= 120) {
                    pArmPower = 0.25;
                    uState = UpState.ULIFT_SLOW;
                }
                break;
            case ULIFT_SLOW:
                telemetry.addLine("WAIT: ULIFT_SLOW");
                if (pivPixUp || pivPixDown || pArmUp > 0 || pArmDown > 0 || gamepad2.dpad_down || ISTOP) {
                    uState = UpState.ULIFT_START;
                    break;
                }
                if (pivotArm.getCurrentPosition() >= 285-20) {
                    pArmPower = 0;
                    pivPixPower = 0.7;
                    uTimer.reset();
                    uState = UpState.UPIXEL_SET;
                }
                break;
            case UPIXEL_SET:
                telemetry.addLine("WAIT: UPIXEL_SET");
                if (pivPixUp || pivPixDown || pArmUp > 0 || pArmDown > 0 || gamepad2.dpad_down || ISTOP) {
                    uState = UpState.ULIFT_START;
                    break;
                }
                if (uTimer.time() > 1.5) {
                    pArmPower = 0;
                    pivPixPower = 0;
                    uState = UpState.ULIFT_START;
                }
                break;
            default:
                telemetry.addLine("ERROR UP");
                uState = UpState.ULIFT_START;
        }

        if (allowlevitate && levitate && (dState == DownState.DLIFT_START) && (uState == UpState.ULIFT_START)) {
            if (touchSensor.isPressed()) {
                pArmPower = 1;
            } else if (pivotArm.getCurrentPosition() <= 30) {
                pArmPower = 0.5;
            }
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);

        pivotArm.setPower(pArmPower);
        lift.setPower(liftPower);
        pivotPixel.setPower(pivPixPower);
        pixel.setPower(pixPower);

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftRearPower, rightRearPower);
//        telemetry.addData("Speed Multiplier", "%4.2f", speedMultiplier);
        telemetry.addData("armpos","%d",pivotArm.getCurrentPosition());
        telemetry.addData("armpow","%4.2f",pArmPower);
        telemetry.addData("readarmpow","%4.2f",pivotArm.getPower());
        telemetry.addData("pixpos","%d",pivotPixel.getCurrentPosition());
        telemetry.addData("pixpow","%4.2f",pivPixPower);
        telemetry.addData("readpixpow","%4.2f",pivotPixel.getPower());
        telemetry.addData("levitate","%b",levitate);
        telemetry.update();
    }
}
