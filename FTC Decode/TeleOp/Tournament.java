package org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Code;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "TOURNAMENT CONFIGURATION", group = "TeleOp")
public class Tournament extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor leftShooter, rightShooter, intakeMotor, beltMotor;
    private Servo servo;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private final int ALIGN_TARGET_ID = 20;

    // ===== AGGRESSIVE PD & LOCK LOGIC =====
    private final double kP = 0.08;           // Increased for faster snapping
    private final double kD = 0.002;          // Adjusted for high-speed dampening
    private final double angleTolerance = 0.3; // Tightened for precision
    private double lastError = 0;
    private double lastTime = 0;

    private boolean isLockedOn = false;
    private double lastKnownBearing = 0;
    private ElapsedTime tagLostTimer = new ElapsedTime();
    private final double LOCK_GRACE_PERIOD_SEC = 0.4;

    private final double DRIVE_SPEED_MULTIPLIER = 1.0; // Max speed for tournament
    private boolean shooterOn = false, intakeOn = false, beltMotorOn = false;
    private double desiredShooterPower = 0.655;

    private boolean wasCirclePressed = false, wasR1Pressed = false, wasR2Pressed = false, wasXPressed = false, wasL1Pressed = false;

    private final double SERVO_STOP_POSITION = 0.5;
    private final double SERVO_FORWARD_POWER = 1.0;
    private final double SERVO_REVERSE_POWER = 0.0;
    private ElapsedTime servoTimer = new ElapsedTime();
    private final double FORWARD_TIME_MS = 450.0; // Faster cycle
    private int servoState = 0;
    private boolean isExtended = false;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        beltMotor = hardwareMap.get(DcMotor.class, "beltMotor");
        servo = hardwareMap.get(Servo.class, "servo");

        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        // Decimation 1.0 = Maximum precision detection
        aprilTag.setDecimation(1.0f);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .build();

        // Faster init loop
        while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            idle();
        }
        setManualExposure(5, 260); // Lower exposure = faster shutter, less blur

        waitForStart();
        lastTime = getRuntime();

        while (opModeIsActive()) {
            // Cubic scaling for fine control, but max power for driving
            double forward = Math.pow(gamepad1.left_stick_y, 3);
            double strafe = Math.pow(gamepad1.left_stick_x * 1.1, 3);
            double rotate = Math.pow(-gamepad1.right_stick_x, 3);

            AprilTagDetection targetTag = null;
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.id == ALIGN_TARGET_ID) {
                    targetTag = detection;
                    break;
                }
            }

            if (targetTag != null) {
                lastKnownBearing = targetTag.ftcPose.bearing;
                tagLostTimer.reset();
                if (gamepad1.y) isLockedOn = true;
            }

            if (!gamepad1.y) isLockedOn = false;

            if (isLockedOn) {
                if (targetTag != null || tagLostTimer.seconds() < LOCK_GRACE_PERIOD_SEC) {
                    double currentError = (targetTag != null) ? targetTag.ftcPose.bearing : lastKnownBearing;

                    if (Math.abs(currentError) > angleTolerance) {
                        double curTime = getRuntime();
                        double dT = curTime - lastTime;
                        double pTerm = currentError * kP;
                        double dTerm = ((currentError - lastError) / (dT > 0 ? dT : 0.001)) * kD;

                        // Increased max rotation speed for the lock
                        rotate = Range.clip(pTerm + dTerm, -0.6, 0.6);
                        lastError = currentError;
                        lastTime = curTime;
                    } else {
                        rotate = 0;
                    }
                }
            } else {
                lastError = 0;
                lastTime = getRuntime();
            }

            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
            frontLeft.setPower((forward + strafe + rotate) / denominator * DRIVE_SPEED_MULTIPLIER);
            backLeft.setPower((-forward - strafe + rotate) / denominator * DRIVE_SPEED_MULTIPLIER);
            frontRight.setPower((forward - strafe - rotate) / denominator * DRIVE_SPEED_MULTIPLIER);
            backRight.setPower((-forward + strafe - rotate) / denominator * DRIVE_SPEED_MULTIPLIER);

            // Subsystems
            if (gamepad1.x && !wasXPressed) intakeOn = !intakeOn;
            intakeMotor.setPower(intakeOn ? -1.0 : 0.0);
            wasXPressed = gamepad1.x;

            if (gamepad1.b && !wasCirclePressed) shooterOn = !shooterOn;
            if (gamepad1.right_bumper && !wasR1Pressed) desiredShooterPower = 0.655;
            if (gamepad1.right_trigger > 0.5 && !wasR2Pressed) desiredShooterPower = 0.56;
            leftShooter.setPower(shooterOn ? desiredShooterPower : 0);
            rightShooter.setPower(shooterOn ? desiredShooterPower : 0);
            wasCirclePressed = gamepad1.b; wasR1Pressed = gamepad1.right_bumper; wasR2Pressed = gamepad1.right_trigger > 0.5;

            // Servo/Belt System
            if (gamepad1.left_bumper && !wasL1Pressed && servoState == 0) {
                if (!isExtended) {
                    servo.setPosition(SERVO_REVERSE_POWER);
                    servoState = 2;
                    // Note: Removed sleep here to keep loop speed high,
                    // using the state machine timer instead for the belt delay
                    beltMotorOn = true; isExtended = true;
                } else {
                    beltMotorOn = false;
                    servo.setPosition(SERVO_FORWARD_POWER);
                    servoState = 1;
                    isExtended = false;
                }
                servoTimer.reset();
            }
            wasL1Pressed = gamepad1.left_bumper;
            beltMotor.setPower(beltMotorOn ? -0.75 : 0.0);

            if (servoState != 0 && servoTimer.milliseconds() >= FORWARD_TIME_MS) {
                servo.setPosition(SERVO_STOP_POSITION);
                servoState = 0;
            }

            telemetry.addData("LOCK", isLockedOn ? "ACTIVE" : "OFF");
            telemetry.addData("Bearing", lastKnownBearing);
            telemetry.update();
        }
        visionPortal.close();
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl != null) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        }
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        if (gainControl != null) gainControl.setGain(gain);
    }
}
