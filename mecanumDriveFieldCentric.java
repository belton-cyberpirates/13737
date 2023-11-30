package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
		//SECTION - Drive Contants
			final int BASE_SPEED = 1500;
			final double MAX_BOOST = 0.6; // boost maxes out at an additional 60% of the base speed
		//!SECTION - End drive constants

		//SECTION - Arm Constants
			final int SHOULDER_MIN_POS = 140;
			final int ELBOW_MIN_POS = 5;
			final double SHOULDER_SPEED = 0.5;
			final double ELBOW_SPEED = 0.5;
			final double STRAFE_MULT = 1.41;
		//!SECTION - End arm constats
	//!SECTION - End constands
	
	//SECTION - Variable init
		//SECTION - Drive Motors
			private DcMotorEx MBackLeft;
			private DcMotorEx MBackRight;
			private DcMotorEx MFrontLeft;
			private DcMotorEx MFrontRight;
		//!SECTION - End Drive motors

		//SECTION - Arm Motors
			private DcMotorEx MShoulderLeft;
			private DcMotorEx MShoulderRight;
			private DcMotorEx MElbowLeft;
			private DcMotorEx MElbowRight;
		//!SECTION - End arm motors

		//SECTION - Servos
			private CRServo clawLeft;
			private CRServo clawRight;
		//!SECTION - End arm motors

		//NOTE - IMU
		private IMU imu;
	//!SECTION - End variable initialzation

	@Override
	public void runOpMode() throws InterruptedException {
		//SECTION - Define Variables
			//SECTION - Drive motors
				MBackLeft = hardwareMap.get(DcMotorEx.class, "m1");
				MFrontLeft = hardwareMap.get(DcMotorEx.class, "m2");
				MFrontRight = hardwareMap.get(DcMotorEx.class, "m3");
				MBackRight = hardwareMap.get(DcMotorEx.class, "m4");
			//!SECTION - End drive motors

			//SECTION - Arm Motors
				MShoulderLeft = hardwareMap.get(DcMotorEx.class, "left_shoulder");
				MShoulderRight = hardwareMap.get(DcMotorEx.class, "right_shoulder");
				MElbowLeft = hardwareMap.get(DcMotorEx.class, "left_elbow");
				MElbowRight = hardwareMap.get(DcMotorEx.class, "right_elbow");
			//!SECTION - End arm motors
			
			//SECTION - Servos
				clawLeft = hardwareMap.get(CRServo.class, "s1");
				clawRight = hardwareMap.get(CRServo.class, "s2");
			//!SECTION - End servos

			//NOTE - IMU
			imu = hardwareMap.get(IMU.class, "imu");
		//!SECTION - End variable definitions

		//NOTE - Set the zero power behaviour
		MBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		MBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		MFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		MFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		// Arm constants
		final int SHOULDER_MIN_POS = 140;
		final int ELBOW_MIN_POS = 5;
		final double SHOULDER_SPEED = 0.5;
		final double ELBOW_SPEED = 0.5;
		final double STRAFE_MULT = 1.2;
		final double CLAW_OPEN_POWER = 0.5;
		final double CLAW_CLOSE_POWER = 0.5;
		final double CLAW_CLOSE_RESIDUAL_POWER = 0.1;
		double clawLeftPassivePower = 0;
		double clawRightPassivePower = 0;

		// Drive contants
		final int BASE_SPEED = 1500;
		final double MAX_BOOST = 0.6; // boost maxes out at an additional 60% of the base speed

		// Wait for the start button to be pressed
		waitForStart();

		//NOTE - Reset robot heading on startup (not initialization)
		//NOTE - MAKE SURE ROBOT IS FACING FORWARD BEFORE HITTING START!
		imu.resetYaw();

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
					leftStickXGP1 *
					Math.cos(botHeading) -
					leftStickYGP1 *
					Math.sin(botHeading);
				double rotatedY =
					leftStickXGP1 *
					Math.sin(botHeading) +
					leftStickYGP1 *
					Math.cos(botHeading);
				rotatedX *= STRAFE_MULT; // strafing is slower than rolling, bump speed

				//NOTE - Set the power of the wheels based off the new joystick coordinates
				//NOTE - y+x+stick <- [-1,1]
				//NOTE - Define caluclated stick values so we don't repeat the same equations
				double YPX = rotatedY + rotatedX - rightStickXGP1;
				double YMX = rotatedY - rotatedX - rightStickXGP1;

				MBackLeft.setVelocity(
					YPX * maxSpeed
				);
				MFrontLeft.setVelocity(
					YMX * maxSpeed
				);
				MBackRight.setVelocity(
					-YPX * maxSpeed
				);
				MFrontRight.setVelocity(
					-YMX * maxSpeed
				);
			//!SECTION - End Base

			//SECTION - Arms
				//NOTE - Set the power of the arm motors
				MShoulderLeft.setPower(-leftStickYGP2 * SHOULDER_SPEED);
				MShoulderRight.setPower(leftStickYGP2 * SHOULDER_SPEED);
				MElbowLeft.setPower(rightStickYGP2 * ELBOW_SPEED);
				MElbowRight.setPower(-rightStickYGP2 * ELBOW_SPEED);
			//!SECTION - End Arms
			
			//SECTION - Arms
				if (gamepad2.a) {
					if (gamepad2.left_trigger > 0) {
						clawLeft.setPower(CLAW_CLOSE_POWER);
						clawLeftPassivePower = CLAW_CLOSE_RESIDUAL_POWER;
					}
					if (gamepad2.right_trigger > 0) {
						clawRight.setPower(CLAW_CLOSE_POWER * -1); // inverse right claw power
						clawRightPassivePower = CLAW_CLOSE_RESIDUAL_POWER * -1; // inverse right claw power
					}
				}
				else if (gamepad2.b) {
					if (gamepad2.left_trigger > 0) {
						clawLeft.setPower(-CLAW_OPEN_POWER);
						clawLeftPassivePower = 0;
					}
					if (gamepad2.right_trigger > 0) {
						clawRight.setPower(-CLAW_OPEN_POWER * -1; // inverse right claw power
						clawRightPassivePower = 0;
					}
				}
				else {
					clawLeft.setPower(clawLeftPassivePower);
					clawRight.setPower(clawRightPassivePower);
				}
			//!SECTION - End claws

			//SECTION - Telemetry
				telemetry.addData("Speed Mod:", maxSpeed);
				telemetry.addData(
					"Left Shoulder:",
					MShoulderLeft.getCurrentPosition()
				);
				telemetry.addData(
					"Right Shoulder:",
					MShoulderRight.getCurrentPosition()
				);
				telemetry.addData("Left Elbow:", MElbowLeft.getCurrentPosition());
				telemetry.addData("Right Elbot:", MElbowRight.getCurrentPosition());
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
