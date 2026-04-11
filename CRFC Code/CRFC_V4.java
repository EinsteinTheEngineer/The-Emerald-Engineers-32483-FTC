package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "CRFC_Tring_Final", group = "TeleOp")
public class CRFC_Tring extends LinearOpMode {

    // --- Hardware Members ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor leftShooter, rightShooter, intakeMotor, beltMotor;
    private Servo servo, anchor;

    // --- State Variables (Must be outside the loop to persist) ---
    private boolean intakeOn = false;
    private boolean shooterOn = false;
    private boolean beltMotorOn = false;
    private boolean flywheelsActive = false; // For L2 Full Speed Toggle
    private boolean isAnchorDeployed = false; // For Y/Triangle Toggle
    private boolean combinedToggle = false;   // For A Toggle
    private boolean isExtended = false;

    // --- Edge Detection (To prevent rapid flickering) ---
    private boolean wasCirclePressed = false;
    private boolean wasR1Pressed = false;
    private boolean wasR2Pressed = false;
    private boolean wasSquarePressed = false;
    private boolean wasL1Pressed = false;
    private boolean wasAPressed = false;
    private boolean lastL2State = false;
    private boolean lastTriangleState = false;
    private boolean wasDpadUpPressed = false;

    // --- Settings ---
    private final double DRIVE_SPEED_MULTIPLIER = 1.0;
    private double desiredShooterPower = 0.655;
    private final double SERVO_STOP_POSITION = 0.5;
    private final double SERVO_FORWARD_POWER = 1.0;
    private final double SERVO_REVERSE_POWER = 0.0;
   
    private ElapsedTime servoTimer = new ElapsedTime();
    private ElapsedTime dpadNudgeTimer = new ElapsedTime();
   
    private final double FORWARD_TIME_MS = 450.0;
    private final double BELT_DELAY_MS = 500.0;
    private final double NUDGE_TIME_MS = 100.0;
   
    private int servoState = 0;

    @Override
    public void runOpMode() {
        // --- Hardware Mapping ---
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
        anchor = hardwareMap.get(Servo.class, "anchor");

        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- 1. Drivetrain Logic ---
            double forward = Math.pow(gamepad1.left_stick_y, 3);
            double strafe = Math.pow(gamepad1.right_stick_x, 3);
            double rotate = Math.pow(gamepad1.left_stick_x * 1.1, 3);
           
            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
            frontLeft.setPower((forward + strafe + rotate) / denominator * DRIVE_SPEED_MULTIPLIER);
            backLeft.setPower((-forward - strafe + rotate) / denominator * DRIVE_SPEED_MULTIPLIER);
            frontRight.setPower((forward - strafe - rotate) / denominator * DRIVE_SPEED_MULTIPLIER);
            backRight.setPower((-forward + strafe - rotate) / denominator * DRIVE_SPEED_MULTIPLIER);

            // --- 2. Intake & Shooter Toggles ---
            if (gamepad1.x && !wasSquarePressed) intakeOn = !intakeOn;
            wasSquarePressed = gamepad1.x;

            if (gamepad1.b && !wasCirclePressed) shooterOn = !shooterOn;
            wasCirclePressed = gamepad1.b;

            if (gamepad1.right_bumper && !wasR1Pressed) desiredShooterPower = 0.655;
            wasR1Pressed = gamepad1.right_bumper;

            if (gamepad1.right_trigger > 0.5 && !wasR2Pressed) desiredShooterPower = 0.56;
            wasR2Pressed = gamepad1.right_trigger > 0.5;

            // --- 3. Flywheel Full Speed Toggle (L2) ---
            boolean currentL2State = gamepad1.left_trigger > 0.5;
            if (currentL2State && !lastL2State) {
                flywheelsActive = !flywheelsActive;
            }
            lastL2State = currentL2State;

            // Apply Power (Full Speed Toggle takes priority)
            if (flywheelsActive) {
                leftShooter.setPower(1.0);
                rightShooter.setPower(1.0);
            } else if (shooterOn) {
                leftShooter.setPower(desiredShooterPower);
                rightShooter.setPower(desiredShooterPower);
            } else {
                leftShooter.setPower(0);
                rightShooter.setPower(0);
            }

            // --- 4. Anchor Servo Toggle (Y/Triangle) ---
            boolean currentTriangleState = gamepad1.y;
            if (currentTriangleState && !lastTriangleState) {
                isAnchorDeployed = !isAnchorDeployed;
                anchor.setPosition(isAnchorDeployed ? 0.5 : 0.0);
            }
            lastTriangleState = currentTriangleState;

            // --- 5. Shared Toggle (A Button) ---
            if (gamepad1.a && !wasAPressed) {
                combinedToggle = !combinedToggle;
            }
            wasAPressed = gamepad1.a;

            // --- 6. Servo/Belt Cycle (L1) ---
            if (gamepad1.left_bumper && !wasL1Pressed && servoState == 0) {
                if (!isExtended) {
                    servo.setPosition(SERVO_REVERSE_POWER);
                    servoState = 2;
                    isExtended = true;
                } else {
                    beltMotorOn = false;
                    servo.setPosition(SERVO_FORWARD_POWER);
                    servoState = 1;
                    isExtended = false;
                }
                servoTimer.reset();
            }
            wasL1Pressed = gamepad1.left_bumper;

            // --- 7. State Machine for Automatic Servo Movements ---
            if (servoState == 1) { // Retracting
                if (servoTimer.milliseconds() >= FORWARD_TIME_MS) {
                    servo.setPosition(SERVO_STOP_POSITION);
                    servoState = 0;
                }
            } else if (servoState == 2) { // Extending
                if (servoTimer.milliseconds() >= BELT_DELAY_MS) {
                    beltMotorOn = true;
                }
                if (servoTimer.milliseconds() >= FORWARD_TIME_MS) {
                    servo.setPosition(SERVO_STOP_POSITION);
                    if (servoTimer.milliseconds() >= BELT_DELAY_MS) {
                        servoState = 0;
                    }
                }
            }

            // --- 8. Final Motor Power Assignments ---
            if (combinedToggle) {
                intakeMotor.setPower(1.0);
                beltMotor.setPower(1.0);
            } else {
                intakeMotor.setPower(intakeOn ? -1.0 : 0.0);
                beltMotor.setPower(beltMotorOn ? -0.75 : 0.0);
            }

            // --- 9. Telemetry ---
            telemetry.addData("Full Speed (L2)", flywheelsActive ? "ACTIVE" : "OFF");
            telemetry.addData("Anchor (Y)", isAnchorDeployed ? "DEPLOYED" : "RETRACTED");
            telemetry.addData("Combined (A)", combinedToggle ? "RUNNING" : "OFF");
            telemetry.update();
        }
    }
}
