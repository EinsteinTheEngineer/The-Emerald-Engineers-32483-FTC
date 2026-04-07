package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "CRFC_TOURNAMENT", group = "TeleOp")
public class TeleOp_CRFC extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor leftShooter, rightShooter, intakeMotor, beltMotor;
    private Servo servo, anchor;
    private VoltageSensor BatteryPowerFinder;

    private VisionPortal visionPortal;
   
    private final double DRIVE_SPEED_MULTIPLIER = 1.0;
    private boolean shooterOn = false, intakeOn = false, beltMotorOn = false;
    private double desiredShooterPower = 0.655;

    private boolean wasCirclePressed = false, wasR1Pressed = false, wasR2Pressed = false, wasXPressed = false, wasL1Pressed = false, wasSquarePressed = false, wasTrianglePressed = false;
    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;
    private boolean wasDpadLeftPressed = false;
    private boolean wasDpadRightPressed = false;

    // ===== SERVO SETTINGS =====
    private final double SERVO_STOP_POSITION = 0.5;
    private final double SERVO_FORWARD_POWER = 1.0;
    private final double SERVO_REVERSE_POWER = 0.0;
   
    private ElapsedTime servoTimer = new ElapsedTime();
    private ElapsedTime dpadNudgeTimer = new ElapsedTime();
   
    private final double FORWARD_TIME_MS = 450.0;
    private final double BELT_DELAY_MS = 500.0; // Wait 500ms before belt turns on
    private final double NUDGE_TIME_MS = 100.0; // 100ms nudge duration
   
    private int servoState = 0; // 0: Stopped, 1: Forward, 2: Backward (Wait for belt), 3: Dpad Nudge
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

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
               
        while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            idle();
        }
       
        waitForStart();

            // --- Subsystems ---
            if (gamepad1.x && !wasSquarePressed) intakeOn = !intakeOn;
            intakeMotor.setPower(intakeOn ? -1.0 : 0.0);
            wasXPressed = gamepad1.x;

            if (gamepad1.b && !wasCirclePressed) shooterOn = !shooterOn;
            if (gamepad1.right_bumper && !wasR1Pressed) desiredShooterPower = 0.655;
            if (gamepad1.right_trigger > 0.5 && !wasR2Pressed) desiredShooterPower = 0.56;
            leftShooter.setPower(shooterOn ? desiredShooterPower : 0);
            rightShooter.setPower(shooterOn ? desiredShooterPower : 0);
            wasCirclePressed = gamepad1.b; wasR1Pressed = gamepad1.right_bumper; wasR2Pressed = gamepad1.right_trigger > 0.5;
           
                // --- Shooter Power Incrementation ---
                // --- 1. Preset Selection Logic ---

                if (gamepad1.right_bumper && !wasR1Pressed) {
                    desiredShooterPower = 0.655;
                    double currentBasePreset = 0.655; //High Power
                   
                    // D-pad LEFT: Reset to whichever preset was last selected
                    if (gamepad1.dpad_left && !wasDpadLeftPressed) {
                    desiredShooterPower = currentBasePreset;
                    }
                }

                if (gamepad1.right_trigger > 0.5 && !wasR2Pressed) {
                    desiredShooterPower = 0.56;
                    double currentBasePreset = 0.56; //Low Power
                   
                    // D-pad LEFT: Reset to whichever preset was last selected
                    if (gamepad1.dpad_left && !wasDpadLeftPressed) {
                    desiredShooterPower = currentBasePreset;
                    }
                }

                // --- 2. Manual Incrementation (DE-NESTED) ---

                // D-pad UP: Increase Power
                if (gamepad1.dpad_up && !wasDpadUpPressed) {
                    desiredShooterPower += 0.03;
                }

                // D-pad DOWN: Decrease Power
                if (gamepad1.dpad_down && !wasDpadDownPressed) {
                    desiredShooterPower -= 0.03;
                }

                // --- 3. Final Motor Commands & Variable Updates ---
                leftShooter.setPower(shooterOn ? desiredShooterPower : 0);
                rightShooter.setPower(shooterOn ? desiredShooterPower : 0);

                // Update ALL "wasPressed" states for the next loop iteration
                wasR1Pressed = gamepad1.right_bumper;
                wasR2Pressed = gamepad1.right_trigger > 0.5; // Saves trigger state as boolean
                wasDpadUpPressed = gamepad1.dpad_up;
                wasDpadDownPressed = gamepad1.dpad_down;
                wasDpadLeftPressed = gamepad1.dpad_left;

            // --- MANUAL SERVO NUDGE (D-Pad Right - 100ms bump) ---
            if (gamepad1.dpad_up && !wasDpadRightPressed && servoState == 0) {
                servo.setPosition(SERVO_REVERSE_POWER);
                servoState = 3; // Nudge state
                dpadNudgeTimer.reset();
            }

            // --- Servo/Belt System (L1 Cycle) ---
            if (gamepad1.left_bumper && !wasL1Pressed && servoState == 0) {
                if (!isExtended) {
                    servo.setPosition(SERVO_REVERSE_POWER);
                    servoState = 2; // Phase 1: Moving servo, waiting to turn on belt
                    isExtended = true;
                } else {
                    beltMotorOn = false;
                    servo.setPosition(SERVO_FORWARD_POWER);
                    servoState = 1; // Phase 1: Retracting servo
                    isExtended = false;
                }
                servoTimer.reset();
            }
            wasL1Pressed = gamepad1.left_bumper;
           
            // Apply belt motor power based on the toggle state
            beltMotor.setPower(beltMotorOn ? -0.75 : 0.0);

            // --- Servo & Belt State Machine ---
            if (servoState == 1) {
                // Handling the L1 Forward Cycle (closing)
                if (servoTimer.milliseconds() >= FORWARD_TIME_MS) {
                    servo.setPosition(SERVO_STOP_POSITION);
                    servoState = 0;
                }
            } else if (servoState == 2) {
                // Handling the L1 Backward Cycle (opening) + 500ms belt wait
                if (servoTimer.milliseconds() >= BELT_DELAY_MS) {
                    beltMotorOn = true; // Turn on belt after 500ms
                }
               
                // Stop the servo motor after its specific travel time
                if (servoTimer.milliseconds() >= FORWARD_TIME_MS) {
                    servo.setPosition(SERVO_STOP_POSITION);
                    // We don't exit the state until the belt is triggered if BELT_DELAY > FORWARD_TIME
                    if (servoTimer.milliseconds() >= BELT_DELAY_MS) {
                        servoState = 0;
                    }
                }
            } else if (servoState == 3) {
                // Handling the Dpad Nudge timing (100ms)
                if (dpadNudgeTimer.milliseconds() >= NUDGE_TIME_MS) {
                    servo.setPosition(SERVO_STOP_POSITION);
                    servoState = 0;
                }
            }
           
            // --- Servo Stopping Mechanism --- (WORK IN PROGRESS DO NOT EDIT UNDERNEATH THIS LINE)
            if (gamepad1.a && !wasXPressed) {
                anchor.setPosition(0);
                anchor.setDirection(Servo.Direction.FORWARD);
                anchor.setPosition(0.2);
                sleep(300);
            }
           
            // --- Shoot Full Speed with all Voltage ---
            if (gamepad1.y && !wasTrianglePressed) {
                double rawPower = 1;
                long startTime = System.currentTimeMillis();
                double targetVoltage = rawPower * 13;

                double currentBatteryVoltage = BatteryPowerFinder.getVoltage();
                double power = targetVoltage / currentBatteryVoltage;

                // Cap power at 1.0 to avoid errors
                power = Math.min(power, 1.0);

                leftShooter.setPower(power);
                rightShooter.setPower(power);
            }
           
            // --- Left Trigger Function but i forgot what it supposed to do... ---


            telemetry.addData("Servo State", servoState);
            telemetry.addData("Belt Motor", beltMotorOn ? "ON" : "OFF");
            telemetry.update();
        }
    }
