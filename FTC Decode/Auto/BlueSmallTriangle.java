package org.firstinspires.ftc.teamcode.Autonomous.Blue_Side;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Blue Small Triangle", group = "Autonomous")
public class BlueSmallTriangle extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor leftShooter, rightShooter, intakeMotor, beltMotor;
    private VoltageSensor BatteryPowerFinder;
    private Servo servo;

    @Override
    public void runOpMode() {
        // ==== Hardware Mapping ====
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Added shooter mapping (otherwise it crashes!)
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");

        beltMotor = hardwareMap.get(DcMotor.class, "beltMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        BatteryPowerFinder = hardwareMap.get(VoltageSensor.class, "Control Hub");
        servo = hardwareMap.get(Servo.class, "servo");

        // ==== Encoder Setup ====
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            driveb(-6,1);
            pivot(24,0.5);
            preShoot();
            shoot_long(5000, 0.625);
            afterShoot();
            drivef(-34.5, 1);
            pivot(-120,1);
            intake(-1.0);
            drivef(-20, 0.5);
            drivef(-20, 0.5);
            drivef(40,1);
            pivot(116,0.5);
            drivef(35.0,1);
            preShoot();
            shoot_long(4000, 0.625);
            afterShoot();
            drivef(-45.6,1);
            stopRobot();


        }

    }

    // ==== set DriveModes

    private void setDriveMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }
    public void drivef(double distance, double speed) {
        //frontRight.setDirection(DcMotor.Direction.REVERSE);
        //backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int moveCounts = (int)(distance * 30.01843216);
        frontLeft.setTargetPosition((frontLeft.getCurrentPosition() + moveCounts) * -1);
        frontRight.setTargetPosition((frontRight.getCurrentPosition() + moveCounts) * -1);
        backLeft.setTargetPosition((backLeft.getCurrentPosition() + moveCounts) * -1);
        backRight.setTargetPosition((backRight.getCurrentPosition() + moveCounts) * -1);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() && frontLeft.isBusy()) {
            telemetry.addData("Status", "Driving...");
            telemetry.update();
        }

        stopRobot();
        sleep(750);
    }
    public void driveb(double distance, double speed) {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int moveCounts = (int)(distance * 30.01843216);
        frontLeft.setTargetPosition((frontLeft.getCurrentPosition() + moveCounts) * 1);
        frontRight.setTargetPosition((frontRight.getCurrentPosition() + moveCounts) * 1);
        backLeft.setTargetPosition((backLeft.getCurrentPosition() + moveCounts) * 1);
        backRight.setTargetPosition((backRight.getCurrentPosition() + moveCounts) *1);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() && frontLeft.isBusy()) {
            telemetry.addData("Status", "Driving...");
            telemetry.update();
        }

        stopRobot();
        sleep(750);
    }

    public void pivot(int degrees, double speed) {
        int TICKSPERDEGREE = 13 * degrees;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (TICKSPERDEGREE < 0) {
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.REVERSE);

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + TICKSPERDEGREE);
            backLeft.setTargetPosition(frontLeft.getCurrentPosition() + TICKSPERDEGREE);
            frontRight.setTargetPosition(frontLeft.getCurrentPosition() - TICKSPERDEGREE);
            backRight.setTargetPosition(frontLeft.getCurrentPosition() - TICKSPERDEGREE);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(speed);
            backLeft.setPower(speed);
            frontRight.setPower(speed);
            backRight.setPower(speed);

            while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {}
            stopRobot();
            sleep(750);

        } else if (TICKSPERDEGREE > 0) {
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.FORWARD);

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - TICKSPERDEGREE);
            backLeft.setTargetPosition(frontLeft.getCurrentPosition() - TICKSPERDEGREE);
            frontRight.setTargetPosition(frontLeft.getCurrentPosition() + TICKSPERDEGREE);
            backRight.setTargetPosition(frontLeft.getCurrentPosition() + TICKSPERDEGREE);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(speed);
            backLeft.setPower(speed);
            frontRight.setPower(speed);
            backRight.setPower(speed);

            while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {}
            stopRobot();
            sleep(750);

        } else {
            sleep(3000);
        }


    }

    public void strafe2(double distance, double speed) {
        int moveCounts = (int)(distance * 54.57); // Check your math here, this is very low for standard wheels!
        //Positive is to the left

        frontLeft.setTargetPosition((frontLeft.getCurrentPosition() - moveCounts) * 1);
        frontRight.setTargetPosition((frontRight.getCurrentPosition() + moveCounts) * 1);
        backLeft.setTargetPosition((backLeft.getCurrentPosition() + moveCounts) * 1);
        backRight.setTargetPosition((backRight.getCurrentPosition() - moveCounts) * 1);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() && frontLeft.isBusy()) {
            telemetry.addData("Status", "Strafing...");
            telemetry.update();
            idle();
        }

        stopRobot();
    }

    public void turn(double degrees, double speed) {
        int moveCounts = (int)(degrees * 31.5396);
        // this function pivots off of ONE wheel (backRight) we can fix this by making a separet pivot for each wheel we want to use or incorporating an if loop
        //Negative is  Clock Wise
        if (moveCounts < 0) {
            frontLeft.setTargetPosition((frontLeft.getCurrentPosition() + moveCounts) * 1);
            frontRight.setTargetPosition((frontRight.getCurrentPosition() + moveCounts) * -1);
            backLeft.setTargetPosition((backLeft.getCurrentPosition() + moveCounts) * 1);
            backRight.setTargetPosition((backRight.getCurrentPosition() + moveCounts) * -1);

            setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower((speed)*1);
            frontRight.setPower((speed) * 1);
            backLeft.setPower((speed)*1);
            backRight.setPower((speed) * 1);
            //Positive is Counter Clock Wise
        } else if (moveCounts > 0) {
            frontLeft.setTargetPosition((frontLeft.getCurrentPosition() + moveCounts) * -1);
            frontRight.setTargetPosition((frontRight.getCurrentPosition() + moveCounts) * 1);
            backLeft.setTargetPosition((backLeft.getCurrentPosition() + moveCounts) * -1);
            backRight.setTargetPosition((backRight.getCurrentPosition() + moveCounts) * 1);

            setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower((speed)*1);
            frontRight.setPower((speed) * 1);
            backLeft.setPower((speed)*1);
            backRight.setPower((speed) * 1);
        } else {
            sleep(500);
        }

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (moveCounts>0){
            while (opModeIsActive() && frontRight.isBusy() || backRight.isBusy());{
                //telemetry.addData("Status", "Turning...");
                //telemetry.addData("Current Position", frontLeft.getCurrentPosition());
                //telemetry.update();
                idle();
            }

        }
        if (moveCounts<0){
            while (opModeIsActive() && frontLeft.isBusy() || backLeft.isBusy());{
                //telemetry.addData("Status", "Turning...");
                //telemetry.addData("Current Position", frontLeft.getCurrentPosition());
                //telemetry.update();
                idle();
            }

        }
        stopRobot();
    }

    public void shoot_long(int durationMillis, double speed) {
        double rawPower = speed;
        long startTime = System.currentTimeMillis();
        double targetVoltage = rawPower * 12.4761905;
        double belt_power = -0.5;
        double intake_power = -0.7;
        // Removed the infinite while(opModeIsActive) loop that was here
        while (opModeIsActive() && (System.currentTimeMillis() - startTime < durationMillis)) {

            double currentBatteryVoltage = BatteryPowerFinder.getVoltage();
            double power = targetVoltage / currentBatteryVoltage;

            // Cap power at 1.0 to avoid errors
            power = Math.min(power, 1.0);

            leftShooter.setPower(power);
            rightShooter.setPower(power);

            sleep(2000);

            beltMotor.setPower(belt_power);
            intakeMotor.setPower(intake_power);

            telemetry.addData("Shooting", "Voltage: %.2f", currentBatteryVoltage);
            telemetry.update();
        }
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        intakeMotor.setPower(0);
    }

    public void shoot_short(int durationMillis, double speed) {
        double rawPower = speed;
        long startTime = System.currentTimeMillis();
        double targetVoltage = rawPower * 11.45862069;
        double belt_power = -0.5;
        double intake_power = -0.7;

        // Removed the infinite while(opModeIsActive) loop that was here
        while (opModeIsActive() && (System.currentTimeMillis() - startTime < durationMillis)) {

            double currentBatteryVoltage = BatteryPowerFinder.getVoltage();
            double power = targetVoltage / currentBatteryVoltage;

            // Cap power at 1.0 to avoid errors
            power = Math.min(power, 1.0);

            leftShooter.setPower(power);
            rightShooter.setPower(power);

            sleep(2000);

            beltMotor.setPower(belt_power);
            intakeMotor.setPower(intake_power);


            telemetry.addData("Shooting", "Voltage: %.2f", currentBatteryVoltage);
            telemetry.update();
        }
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        intakeMotor.setPower(0);
    }

    public void strafe(int inches, double speed) {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int COUNTS_PER_INCH = 49;
        int moveCounts = (inches * COUNTS_PER_INCH);

        if (moveCounts < 0) {
            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + moveCounts);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + moveCounts);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + moveCounts);
            backRight.setTargetPosition(backRight.getCurrentPosition() + moveCounts);

        } else if (moveCounts > 0) {
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backRight.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + moveCounts);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + moveCounts);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + moveCounts);
            backRight.setTargetPosition(backRight.getCurrentPosition() + moveCounts);

        } else {
            sleep(1000);
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() && frontLeft.isBusy()) { }
        stopRobot();
        sleep(750);
    }

    public void intake(double intake_power) {
        intakeMotor.setPower(intake_power);

    }

    public void preShoot() {
        servo.setPosition(0.0);
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(0.4);
        sleep(50);
    }

    public void afterShoot() {
        servo.setPosition(0.4);
        intakeMotor.setPower(0);
        beltMotor.setPower(0);
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(0.0);
        sleep(50);
    }

    public void belt(int durationMillis, double belt_power) {
        beltMotor.setPower(belt_power);
        sleep(durationMillis);
    }

    public void shootersOn(int durationMillis, double shooter_power) {
        leftShooter.setPower(shooter_power);
        rightShooter.setPower(shooter_power);
        sleep(durationMillis);
    }

    public void stopRobot() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
