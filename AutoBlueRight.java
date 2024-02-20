package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Set;
import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import org.firstinspires.ftc.teamcode.DriveMotors;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "AutoBlueRight", preselectTeleOp="MecanumDriveFieldCentric")
public class AutoBlueRight extends Auto {
	/**
	 * This function is executed when this Op Mode is initialized from the Driver Station.
	 */
	@Override
	public void runOpMode() {
		Initialize();

		waitForStart();

		int position = camera.GetPropPos();

		if (opModeIsActive()) { // <----------------------------------------------------------------
			MotorSetup(); // arm between 0 and -2500

			switch(position) {
				case 0:
					// Move to the left spike mark
					driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 1));
					driveMotors.Turn(-45);
					driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 0.2));
					// Drop purple pixel
					intake.OpenRight(350);
					// Move to a starting point for scoring / parking auto
					driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * 0.2));
					driveMotors.Turn(45);
					driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 1.2));
					// Rotate towards board
					driveMotors.Turn(-90);
					break;
				case 1:
					// Move to the center spike mark
					driveMotors.Move(Direction.LEFT, (int)(Config.TILE_LENGTH * .2));
					driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 1.11));
					// Drop pixel
					intake.OpenRight(350);
					// Move to a starting point for scoring / parking auto
					driveMotors.Move(Direction.LEFT, (int)(Config.TILE_LENGTH * .8));
					driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * .11));
					// Rotate towards board
					driveMotors.Turn(-90);
					break;
				case 2:
					// Move to the right spike mark
					driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * .7));
					driveMotors.Move(Direction.RIGHT, (int)(Config.TILE_LENGTH * .6));
					// Drop pixel
					intake.OpenRight(350);
					// Move to a starting point for scoring / parking auto
					driveMotors.Move(Direction.LEFT, (int)(Config.TILE_LENGTH * .6));
					driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 1.2));
					// Rotate towards board
					driveMotors.Turn(-90);
					break;
			}
		}
		saveHeading();
	}
}
