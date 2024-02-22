package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.io.*;
import java.nio.file.*;

@TeleOp(name = "SinglePersonDrive")
public class SinglePersonDrive extends LinearOpMode {

	// Drive constants
	final int BASE_SPEED = 1500;
	final double MAX_BOOST = 0.6; // boost maxes out at an additional 60% of the base speed
	final double STRAFE_MULT = 1.41;

	// Arm constants
	final double SHOULDER_SPEED = 0.6;
	final double ARM_MAX = 1.35;
	final double ARM_MIN = .63;

	final double SLIDE_SPEED = 0.9;
	final double SLIDE_OPTIMAL_POS = -300;

	final double WRIST_MAX = .9;

	// Claw constants
	final double CLAW_OPEN_POWER = 0.5;
	final double CLAW_CLOSE_POWER = 0.5;
	final double CLAW_CLOSE_RESIDUAL_POWER = 0.1;
	

	// Drive motors
	private DcMotorEx BackLeft;
	private DcMotorEx FrontLeft;
	private DcMotorEx FrontRight;
	private DcMotorEx BackRight;

	// Arm motors
	private DcMotorEx Shoulder;
	private DcMotorEx Slide;
	private DcMotorEx Winch;
	
	// Servos
	private Servo DroneLauncher;
	private Servo clawLeft;
	private Servo clawRight;
	private Servo wrist;
	
	// Sensors
	private TouchSensor magnet;
	private AnalogInput shoulderPot;
	private IMU imu;
	
	// Other variables
	private boolean slideFrozen;

	@Override
	public void runOpMode() throws InterruptedException {
		// Assign drive motors
		BackLeft = hardwareMap.get(DcMotorEx.class, "m1");
		FrontLeft = hardwareMap.get(DcMotorEx.class, "m2");
		FrontRight = hardwareMap.get(DcMotorEx.class, "m3");
		BackRight = hardwareMap.get(DcMotorEx.class, "m4");

		// Assign arm motors
		Shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
		Slide = hardwareMap.get(DcMotorEx.class, "lift");
		Winch = hardwareMap.get(DcMotorEx.class, "winch");
		
		// Assign servos
		DroneLauncher = hardwareMap.get(Servo.class, "drone_servo");
		clawLeft = hardwareMap.get(Servo.class, "clawLeft");
		clawRight = hardwareMap.get(Servo.class, "clawRight");
		wrist = hardwareMap.get(Servo.class, "wrist");
			
		// Assign sensors
		magnet = hardwareMap.get(TouchSensor.class, "magnet");
		shoulderPot = hardwareMap.get(AnalogInput.class, "shoulder_pot");
		imu = hardwareMap.get(IMU.class, "imu");
		
		// Set zero power behaviours
		BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    	Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		// Wait for the start button to be pressed
		waitForStart();

		// Reset robot heading on startup (not initialization)
		imu.resetYaw();
		double savedHeading = getSavedHeading();
		
		// set servo start position
		clawLeft.setPosition(0.62);
		clawRight.setPosition(0.38);
		wrist.setPosition(WRIST_MAX);

		while (opModeIsActive()) {
			// Reset yaw when start button is pressed so that a restart is not needed if the yaw should be reset again.
			if (gamepad1.start) {
				imu.resetYaw();
			}

			
			// Gamepad variables
			double leftStickXGP1 = gamepad1.left_stick_x;
			double leftStickYGP1 = gamepad1.left_stick_y;
			double rightStickXGP1 = gamepad1.right_stick_x;
			double rightStickYGP1 = gamepad1.right_stick_y;
			
			double leftStickYGP2 = gamepad2.left_stick_y;
			double rightStickYGP2 = gamepad2.right_stick_y;


			// Get the speed the bot would go with the joystick pushed all the way
			double maxSpeed = calcMaxSpeed(gamepad1.right_trigger - gamepad1.left_trigger, BASE_SPEED, MAX_BOOST);

			// Get the heading of the bot (the angle it is facing) in radians
			double botHeading = (savedHeading + imu .getRobotYawPitchRollAngles() .getYaw(AngleUnit.RADIANS));


			// Virtually rotate the joystick by the negative angle of the robot
			double rotatedX =
				leftStickXGP1 * Math.cos(botHeading) -
				leftStickYGP1 * Math.sin(botHeading);
			double rotatedY =
				leftStickXGP1 * Math.sin(botHeading) +
				leftStickYGP1 * Math.cos(botHeading);
			rotatedX *= STRAFE_MULT; // strafing is slower than rolling, bump speed


			// Set the power of the wheels based off the new joystick coordinates
			// y+x+stick <- [-1,1]
            mult = gamepad1.left_bumper ? 0 : 1;
			BackLeft.setVelocity(
				(rotatedY + rotatedX - rightStickXGP1) * maxSpeed * mult
			);
			FrontLeft.setVelocity(
				(rotatedY - rotatedX - rightStickXGP1) * maxSpeed * mult
			);
			FrontRight.setVelocity(
				(-rotatedY - rotatedX - rightStickXGP1) * maxSpeed * mult
			);
			BackRight.setVelocity(
				(-rotatedY + rotatedX - rightStickXGP1) * maxSpeed * mult
			);
			

			// Set the power of the shoulder based off left joystick y
			double shoulderPower = leftStickYGP1 * SHOULDER_SPEED;
			
			if (shoulderPot.getVoltage() <= ARM_MIN) // Dont move shoulder down if below min
				shoulderPower = Math.min(shoulderPower, 0);

			if (shoulderPot.getVoltage() >= ARM_MAX) // Dont move shoulder up if above max
				shoulderPower = Math.max(shoulderPower, 0);

			Shoulder.setPower((gamepad1.left_bumper ? shoulderPower : 0));
			
			// Set the power of the slide based off right joystick y
			double slide_power = rightStickYGP1 * SLIDE_SPEED;

            Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (magnet.isPressed()) slideFrozen = true; // Freeze slide if magnet on
            if (slide_power < 0) slideFrozen = false; // Unfreeze slide if extending
            if (slideFrozen) slide_power = Math.min(slide_power, 0); // If slide is frozen dont let it move backward

            Slide.setPower((gamepad1.left_bumper ? slide_power : 0));

			// Open claws with respective triggers, close with respective bumpers
			if (gamepad1.right_trigger > 0) clawLeft.setPosition(.62);
			else if (gamepad1.right_bumper) clawLeft.setPosition(.2);
	
			if (gamepad1.right_trigger > 0) clawRight.setPosition(.38);
			else if (gamepad1.right_bumper) clawRight.setPosition(.8);
			

			// Move wrist down with B, and up with A
			if (gamepad2.a) wrist.setPosition(0);
			if (gamepad2.b) wrist.setPosition(WRIST_MAX);
			if (gamepad2.dpad_up && gamepad2.dpad_left) wrist.setPosition(1);
			

			// Launch drone with Y, reload drone launcher with X
			if (gamepad1.x) DroneLauncher.setPosition(0);
			if (gamepad1.y) DroneLauncher.setPosition(0.5);
			

			// Telemetry
			telemetry.addData("Speed Mod:", maxSpeed);
			telemetry.addData("Shoulder Encoder:", Shoulder.getCurrentPosition());
			telemetry.addData("Shoulder Pot:", shoulderPot.getVoltage());
			telemetry.addData("Heading (radians):", botHeading);

			telemetry.update();
		}
	}

	/**
	 * if boost trigger unpressed, return base_speed,
	 * else return base_speed + boost amount
	 */
	double calcMaxSpeed(double triggerVal, int BASE_SPEED, double MAX_BOOST) {
		double boostRatio = triggerVal * MAX_BOOST;
		double boostSpeed = boostRatio * BASE_SPEED;
		return BASE_SPEED + boostSpeed;
	}

	double getSavedHeading() {
		try {
			BufferedReader br = new BufferedReader(new FileReader("heading.txt"));
			String line = br.readLine();

			br.close();

			//Files.newBufferedWriter("heading.txt" , StandardOpenOption.TRUNCATE_EXISTING);

			telemetry.addData("Retrieved saved heading:", true);
			return Double.parseDouble(line);
		} catch(Exception e) {
			telemetry.addData("Retrieved saved heading:", false);
			return 0d;
		}
	}
}
