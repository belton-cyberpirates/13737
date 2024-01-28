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


@Autonomous(name = "AutoRedRight")
public class AutoRedRight extends LinearOpMode {
	private TfodProcessor tfod;
  private VisionPortal visionPortal;

	private static final int CameraResoX = 640;

	// TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
	private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/teampiece.tflite";

	// Define the labels recognized in the model for TFOD (must be in training order!)
	private static final String[] LABELS = {
		"BLUE",
		"RED",
	};
	
	private DriveMotors driveMotors;
	private Arm arm;
	private Servo wrist;
	private Servo clawLeft;
	private Servo clawRight;


	/**
	 * Set reliable initial configuration for robot motors
	 */
	public void MotorSetup() {
		CloseClaw();
		MoveWrist(.85);
		arm.DropArm();
		sleep(1000);
		arm.Initialize();
	}


	/**
	 * Attempt to recognize the AprilTag and return which tag we see
	 */
	// private int RunAprilTagDetection() {
		// for (int numFramesWithoutDetection = 0; numFramesWithoutDetection < Config.MAX_NUM_FRAMES_NO_DETECTION; numFramesWithoutDetection++) {
		//   // Calling getDetectionsUpdate() will only return an object if there was a new frame
		//   // processed since the last time we called it. Otherwise, it will return null. This
		//   // enables us to only run logic when there has been a new frame, as opposed to the
		//   // getLatestDetections() method which will always return an object.
		//   ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

		//   // add camera stats to telemetry
		//   telemetry.addData("FPS", camera.getFps());
		//   telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
		//   telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
		//   telemetry.update();

		//   // If we haven't seen a tag for a few frames, lower the decimation
		//   // so we can hopefully pick one up if we're e.g. far back
		//   if(numFramesWithoutDetection >= Config.NUM_FRAMES_BEFORE_LOW_DECIMATION) {
		// 	  aprilTagDetectionPipeline.setDecimation(Config.DECIMATION_LOW);
		//   }

		//   if (detections == null || detections.size() == 0) {
		// 	// pass if there's no new frame or we didn't detect anything
		// 	sleep(20); // give the camera a chance to process the frame
		// 	continue;
		//   }

		//   // If we're here, we have a new frame and we detected at least one tag
		//   // turn on high decimation to increase the frame rate
		//   aprilTagDetectionPipeline.setDecimation(Config.DECIMATION_HIGH);


		//   AprilTagDetection detection = detections.get(0);
		//   telemetry.addLine(String.format("\n!!---[[ DETECTED ID=%d]]---!!", detection.id));
		//   telemetry.addLine(String.format("Translation X: %.2f", detection.pose.x));
		//   telemetry.addLine(String.format("Translation Y: %.2f", detection.pose.y));
		//   telemetry.addLine(String.format("Translation Z: %.2f", detection.pose.z));
		//   telemetry.addLine(String.format("Rotation Yaw: %.2f", detection.pose.yaw));
		//   telemetry.addLine(String.format("Rotation Pitch: %.2f", detection.pose.pitch));
		//   telemetry.addLine(String.format("Rotation Roll: %.2f", detection.pose.roll));
		//   telemetry.addLine(String.format("Attempts: %d", numFramesWithoutDetection));
		//   telemetry.update();

		//   return detection.id;
		// }

		// telemetry.addLine(String.format("!!---[[ NO TAG DETECTED AFTER %d ATTEMPTS ]]---!!", Config.MAX_NUM_FRAMES_NO_DETECTION));
		// telemetry.update();
		// return -1; // no tag detected; godspeed
	// }


	/**
	 * This function is executed when this Op Mode is initialized from the Driver Station.
	 */
	@Override
	public void runOpMode() {
		//WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
		//camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
		//aprilTagDetectionPipeline = new AprilTagDetectionPipeline(Config.TAGSIZE, Config.FX, Config.FY, Config.CX, Config.CY);

		//camera.setPipeline(aprilTagDetectionPipeline);
		//camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
		// {
		// 	@Override
		// 	public void onOpened()
		// 	{
		// 		camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
		// 	}

		// 	@Override
		// 	public void onError(int errorCode)
		// 	{
		// 	  telemetry.addData("Error", "Camera failed to open with error " + errorCode);
		// 	  telemetry.update();
		// 	}
		// });

		// argument order *must* be fr-fl-bl-br
		driveMotors = new DriveMotors(
			hardwareMap.get(DcMotorEx.class, "m3"),
			hardwareMap.get(DcMotorEx.class, "m2"),
			hardwareMap.get(DcMotorEx.class, "m1"),
			hardwareMap.get(DcMotorEx.class, "m4")
		);
		
		arm = new Arm(
			hardwareMap.get(DcMotorEx.class, "shoulder"),
			hardwareMap.get(DcMotorEx.class, "lift")
		);
		
		clawLeft = hardwareMap.get(Servo.class, "clawLeft");
		clawRight = hardwareMap.get(Servo.class, "clawRight");
		wrist = hardwareMap.get(Servo.class, "wrist");

		CloseClaw();
		wrist.setPosition(0);
		
		initTfod();
		int position = GetPropPos();
	  
		telemetry.addData("position", position);
		telemetry.update();

		waitForStart();

		if (opModeIsActive()) { // <----------------------------------------------------------------
			MotorSetup(); // arm between 0 and -2500

			switch(position) {
				case 0:

					// Move to the right spike mark
					driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 1));
					driveMotors.Turn(-45);
					driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 0.225));
					// Drop purple pixel
					OpenClaw(false, true);
					MoveWrist(0);
					sleep(350);

					// Move to a starting point for scoring / parking auto
					driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * 0.225));
					driveMotors.Turn(45);
					driveMotors.Move(Direction.RIGHT, (int)(Config.TILE_LENGTH * 0.9));
					// Rotate towards board
					driveMotors.Turn(90);
					break;
				case 1:
					// Move to the center spike mark
					driveMotors.Move(Direction.RIGHT, (int)(Config.TILE_LENGTH * .1));
					driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 1.11));
					// Drop pixel
					OpenClaw(false, true);
					MoveWrist(0);
					sleep(350);
					// Move to a starting point for scoring / parking auto
					driveMotors.Move(Direction.RIGHT, (int)(Config.TILE_LENGTH * .7));
					driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * .11));
					// Rotate towards board
					driveMotors.Turn(90);
					break;
				case 2:
					// Move to the left spike mark
					driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * .7));
					driveMotors.Move(Direction.RIGHT, (int)(Config.TILE_LENGTH * .5));
					// Drop pixel
					OpenClaw(false, true);
					MoveWrist(0);
					sleep(350);
					
					// Move to a starting point for scoring / parking auto
					driveMotors.Move(Direction.RIGHT, (int)(Config.TILE_LENGTH * .45));
					// Rotate towards board
					driveMotors.Turn(90);
					break;
			}

			arm.MoveShoulder(-1150);  
			switch(position) {
			case 0:
				driveMotors.Move(Direction.LEFT, (int)(Config.TILE_LENGTH * 0.3));
				break;
			case 1:
				driveMotors.Move(Direction.RIGHT, (int)(Config.TILE_LENGTH * 0.12));
				break;
			case 2:
				driveMotors.Move(Direction.RIGHT, (int)(Config.TILE_LENGTH * 0.2));
				break;
			}
			driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 1));

			arm.MoveSlide(-570, true);
			MoveWrist(0.5);
			sleep(450);
			OpenClaw(true, false);
			sleep(350);
			driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * .2));
			driveMotors.Move(Direction.RIGHT, (int)(Config.TILE_LENGTH * 1.5));
		}
	}

	
	public void MoveWrist(double position) {
		wrist.setPosition(position);
	}
	
	
	public void OpenClaw() {
		OpenClaw(true, true);
	}
	

	public void OpenClaw(boolean openLeft, boolean openRight) {
		if (openLeft) clawLeft.setPosition(Config.CLAW_LEFT_OPEN);
		if (openRight) clawRight.setPosition(Config.CLAW_RIGHT_OPEN);
	}
	
	
	public void CloseClaw() {
		CloseClaw(true, true);
	}


	public void CloseClaw(boolean closeLeft, boolean closeRight) {
		if (closeLeft) clawLeft.setPosition(Config.CLAW_LEFT_CLOSE);
		if (closeRight) clawRight.setPosition(Config.CLAW_RIGHT_CLOSE);
	}
	
	private void initTfod() {

		// Create the TensorFlow processor by using a builder.
		tfod = new TfodProcessor.Builder()
			.setModelFileName(TFOD_MODEL_FILE)
			.setModelLabels(LABELS)
			.build();

		// Create the vision portal by using a builder.
		VisionPortal.Builder builder = new VisionPortal.Builder();

		// Set the camera (webcam vs. built-in RC phone camera).
		builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

		// Set and enable the processor.
		builder.addProcessor(tfod);

		// Build the Vision Portal, using the above settings.
		visionPortal = builder.build();
	}

	private int GetPropPos() { // 0 = left, 1 = center, 2 = right
		Recognition recognition = highestConfidence();

		if (recognition != null) {
			double propX = (recognition.getLeft() + recognition.getRight()) / 2 ;

			if ((CameraResoX / 2) > propX) {
				return 1;
			}
			return 2;
		}
		return 0;
	}

	private Recognition highestConfidence() {
		Recognition recognition = null;

		for (int i = 0; i <= 1000; i++) {
			List<Recognition> currentRecognitions = tfod.getRecognitions();

			if (currentRecognitions.size() < 1) {
				sleep(10);
				continue;
			}
			
			// Select highest confidence object
			for (Recognition potentialRecognition : currentRecognitions) {

				if (recognition != null && recognition.getConfidence() > potentialRecognition.getConfidence()) {
					continue;
				}
				recognition = potentialRecognition;
			}
			return recognition;
		}
		return recognition;
	}
}
