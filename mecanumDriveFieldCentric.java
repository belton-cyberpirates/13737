package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="MechanumDriveFieldCentric")

public class MechanumDriveFieldCentric extends LinearOpMode {

	// Initialize base motor variables
	private DcMotorEx MBackLeft;
	private DcMotorEx MBackRight;
	private DcMotorEx MFrontLeft;
	private DcMotorEx MFrontRight;

	// Initialize arm motor variables
	private DcMotorEx MShoulderLeft;
	private DcMotorEx MShoulderRight;
	private DcMotorEx MElbowLeft;
	private DcMotorEx MElbowRight;

    // Initialize IMU variable
    private IMU imu;
	
	@Override
	public void runOpMode() throws InterruptedException {

		// Attach drive motors
		MBackLeft = hardwareMap.get(DcMotorEx.class, "m1");
		MFrontLeft = hardwareMap.get(DcMotorEx.class, "m2");
		MFrontRight = hardwareMap.get(DcMotorEx.class, "m3");
		MBackRight = hardwareMap.get(DcMotorEx.class, "m4");

		// Attach arm motors
		MShoulderLeft = hardwareMap.get(DcMotorEx.class, "left_shoulder");
		MShoulderRight = hardwareMap.get(DcMotorEx.class, "right_shoulder");
		MElbowLeft = hardwareMap.get(DcMotorEx.class, "left_elbow");
		MElbowRight = hardwareMap.get(DcMotorEx.class, "right_elbow");
		
        // Attach IMU
		imu = hardwareMap.get(IMU.class, "imu");
		
        // Set the zero power behaviour, or what the motor should when at zero power
		MBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		MBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		MFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		MFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Arm constants
		final int SHOULDER_MIN_POS = 140;
		final int ELBOW_MIN_POS = 5;
		final double SHOULDER_SPEED = 0.5;
		final double ELBOW_SPEED = 0.5;
		final double STRAFE_MULT = 1.41;
		
        // Drive contants
		final int BASE_SPEED = 1500;
		final double MAX_BOOST = 0.6; // boost maxes out at an additional 60% of the base speed
		
		waitForStart();
			
        // Reset robot heading on startup
        // MAKE SURE ROBOT IS FACING FORWARD WHEN IT IS STARTED!
        imu.resetYaw();

		while (opModeIsActive()) {
			double leftStickXGP1 = gamepad1.left_stick_x;
			double leftStickYGP1 = gamepad1.left_stick_y;
			double rightStickXGP1 = gamepad1.right_stick_x;
			double rightStickYGP1 = gamepad1.right_stick_y;

			double leftStickYGP2 = gamepad2.left_stick_y;
			double rightStickYGP2 = gamepad2.right_stick_y;

			double maxSpeed = calcMaxSpeed(gamepad1.right_trigger);

            // Get the heading of the bot (the angle it is facing) in radians
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Virtually rotate the joystick by the negative angle of the robot
            final double rotatedX = leftStickX * Math.cos(-botHeading) - 
                          leftStickY * Math.sin(-botHeading);
            
            final double rotatedY = leftStickX * Math.sin(-botHeading) + 
                          leftStickY * Math.cos(-botHeading);

            rotatedX *= STRAFE_MULT;  // strafing is slower than rolling, bump speed
            
            // Set the power of the wheels based off the new joystick coordinates
			// y+x+stick <- [-1,1]
            MBackLeft.setVelocity((rotatedY + rotatedX + rightStickX) * maxSpeed);
            MFrontLeft.setVelocity(( rotatedY - rotatedX + rightStickX) * maxSpeed);
            MBackRight.setVelocity((-rotatedY + rotatedX + rightStickX) * maxSpeed);
            MFrontRight.setVelocity((-rotatedY - rotatedX + rightStickX) * maxSpeed);
			//? should all of the rightStickX be the same? should they be pos or neg?

            // Set the power of the arm motors
			MShoulderLeft.setPower(-leftStickYGP2 * SHOULDER_SPEED);
			MShoulderRight.setPower(leftStickYGP2 * SHOULDER_SPEED);
			MElbowLeft.setPower(rightStickYGP2 * ELBOW_SPEED);
			MElbowRight.setPower(-rightStickYGP2 * ELBOW_SPEED);
			
			// Display velocity
			telemetry.addData("Speed Mod:", maxSpeed);
			telemetry.addData("Left Shoulder:", MShoulderLeft.getCurrentPosition());
			telemetry.addData("Right Shoulder:", MShoulderRight.getCurrentPosition());
			telemetry.addData("Left Elbow:", MElbowLeft.getCurrentPosition());
			telemetry.addData("Right Elbot:", MElbowRight.getCurrentPosition());
			telemetry.addData("Heading (radians):", botHeading);
			
			telemetry.update();
		}
	}

	/// if boost trigger unpressed, return base_speed,
	/// else return base_speed + boost amount
	double calcMaxSpeed(double triggerVal) {
		double boostRatio = triggerVal * MAX_BOOST;
		double boostSpeed = boostRatio * BASE_SPEED;
		return BASE_SPEED + boostSpeed;
	}
}