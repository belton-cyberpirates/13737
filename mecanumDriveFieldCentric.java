package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FieldCentricDrive")
public class MecanumDriveFieldCentric extends LinearOpMode {
	//SECTION - Constants
		//SECTION Drive contants
			final int BASE_SPEED = 1500;
			final double MAX_BOOST = 0.6; // boost maxes out at an additional 60% of the base speed
		//!SECTION - End drive constants

		//SECTION - Arm Constants
			final double SHOULDER_SPEED = 0.6;
			final double SLIDE_SPEED = 0.9;
			final double STRAFE_MULT = 1.41;
		//!SECTION - End arm constats

		//SECTION - Claw constants
			final double CLAW_OPEN_POWER = 0.5;
			final double CLAW_CLOSE_POWER = 0.5;
			final double CLAW_CLOSE_RESIDUAL_POWER = 0.1;
		//!SECTION - End claw constants
	//!SECTION - End constands
	
	//SECTION - Variable init
		//SECTION - Drive Motors
			private DcMotorEx BackLeft;
			private DcMotorEx FrontLeft;
			private DcMotorEx FrontRight;
			private DcMotorEx BackRight;
		//!SECTION - End Drive motors

		//SECTION - Arm Motors
			private DcMotorEx Shoulder;
			private DcMotorEx Slide;
			private DcMotorEx Winch;
		//!SECTION - End arm motors

		//SECTION - Servos
			private Servo DroneLauncher;
			private Servo clawLeft;
			private Servo clawRight;
			private Servo wrist;
		//!SECTION - End servos

		//NOTE - IMU
		private IMU imu;
	//!SECTION - End variable initialzation

	@Override
	public void runOpMode() throws InterruptedException {
		//SECTION - Define Variables
			//SECTION - Drive motors
				BackLeft = hardwareMap.get(DcMotorEx.class, "m1");
				FrontLeft = hardwareMap.get(DcMotorEx.class, "m2");
				FrontRight = hardwareMap.get(DcMotorEx.class, "m3");
				BackRight = hardwareMap.get(DcMotorEx.class, "m4");
			//!SECTION - End drive motors

			//SECTION - Arm Motors
				Shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
				Slide = hardwareMap.get(DcMotorEx.class, "lift");
				Winch = hardwareMap.get(DcMotorEx.class, "winch");
			//!SECTION - End arm motors
			
			//SECTION - Servos
				//DroneLauncher = hardwareMap.get(Servo.class, "drone_servo");
				clawLeft = hardwareMap.get(Servo.class, "clawLeft");
				clawRight = hardwareMap.get(Servo.class, "clawRight");
				wrist = hardwareMap.get(Servo.class, "wrist");
			//!SECTION - End servos

			//NOTE - IMU
			imu = hardwareMap.get(IMU.class, "imu");

		//!SECTION - End variable definitions

		//NOTE - Set the zero power behaviour
		BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		// Wait for the start button to be pressed
		waitForStart();

		//NOTE - Reset robot heading on startup (not initialization)
		//NOTE - MAKE SURE ROBOT IS FACING FORWARD BEFORE HITTING START!
		imu.resetYaw();
		
		//NOTE set servo start position
		clawLeft.setPosition(0.5);
		clawRight.setPosition(0.5);
		wrist.setPosition(1);

		while (opModeIsActive()) {
			//NOTE - Reset Yaw on start button press so that a restart is not needed if Yaw should be reset again.
			if (gamepad1.start) {
				imu.resetYaw();
			}
			
			//SECTION GP1
				double leftStickXGP1 = gamepad1.left_stick_x;
				double leftStickYGP1 = gamepad1.left_stick_y;
				double rightStickXGP1 = gamepad1.right_stick_x;
				double rightStickYGP1 = gamepad1.right_stick_y;
			//!SECTION End GP1
			
			//SECTION GP2
				double leftStickYGP2 = gamepad2.left_stick_y;
				double rightStickYGP2 = gamepad2.right_stick_y;
			//!SECTION End GP2

			//SECTION - Base
				double maxSpeed = calcMaxSpeed(gamepad1.right_trigger - gamepad1.left_trigger, BASE_SPEED, MAX_BOOST);

				//NOTE - Get the heading of the bot (the angle it is facing) in radians
				double botHeading = imu
					.getRobotYawPitchRollAngles()
					.getYaw(AngleUnit.RADIANS);

				//NOTE - Virtually rotate the joystick by the negative angle of the robot
				double rotatedX =
					leftStickXGP1 * Math.cos(botHeading) -
					leftStickYGP1 * Math.sin(botHeading);
				double rotatedY =
					leftStickXGP1 * Math.sin(botHeading) +
					leftStickYGP1 * Math.cos(botHeading);
				rotatedX *= STRAFE_MULT; // strafing is slower than rolling, bump speed

				// Set the power of the wheels based off the new joystick coordinates
				// y+x+stick <- [-1,1]

				BackLeft.setVelocity(
					(rotatedY + rotatedX - rightStickXGP1) * maxSpeed
				);
				FrontLeft.setVelocity(
					(rotatedY - rotatedX - rightStickXGP1) * maxSpeed
				);
				FrontRight.setVelocity(
					(-rotatedY - rotatedX - rightStickXGP1) * maxSpeed
				);
				BackRight.setVelocity(
					(-rotatedY + rotatedX - rightStickXGP1) * maxSpeed
				);
			//!SECTION - End Base

			//SECTION - Arm
				//NOTE - Set the power of the arm motors
				Shoulder.setPower(leftStickYGP2 * SHOULDER_SPEED);
				Slide.setPower(rightStickYGP2 * SLIDE_SPEED);
				
			//!SECTION - End Arm

			//SECTION - Claws
			if (gamepad2.left_trigger > 0) clawLeft.setPosition(.62);
			else if (gamepad2.left_bumper) clawLeft.setPosition(.2);
	
			if (gamepad2.right_trigger > 0) clawRight.setPosition(.38);
			else if (gamepad2.right_bumper) clawRight.setPosition(.8);
			//!SECTION End claws

			if (gamepad2.a) wrist.setPosition(0);
			if (gamepad2.b) wrist.setPosition(1);
			
			//if (gamepad1.x) DroneLauncher.setPosition(0);
			//if (gamepad1.y) DroneLauncher.setPosition(0.5);

			if (gamepad2.x) Winch.setPower(-1);
			else if (gamepad2.y) Winch.setPower(1);
			else Winch.setPower(0);
			
			//SECTION - Telemetry
				telemetry.addData("Speed Mod:", maxSpeed);
				telemetry.addData(
					"Shoulder:",
					Shoulder.getCurrentPosition()
				);
				telemetry.addData("Heading (radians):", botHeading);

				telemetry.update();
			//!SECTION - End telemetry
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
}
