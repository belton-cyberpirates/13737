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
//import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import org.firstinspires.ftc.teamcode.DriveMotors;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.Config;


@Autonomous(name = "AutoBlueLeft")
public class AutoBlueLeft extends LinearOpMode {
	private OpenCvCamera camera;
	//private AprilTagDetectionPipeline aprilTagDetectionPipeline;
	private TfodProcessor myTfodProcessor;
	private VisionPortal myVisionPortal;
	private boolean USE_WEBCAM = true;
	
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

		waitForStart();

		if (opModeIsActive()) { // <----------------------------------------------------------------
			MotorSetup();
			
			arm.MoveShoulder(-5);
			driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 1.1));
			
			// driveMotors.Move(Direction.RIGHT, (int)(Config.TILE_LENGTH * .2));
			// OpenClaw(false, true);
			// driveMotors.Move(Direction.LEFT, (int)(Config.TILE_LENGTH * .2));
			
			driveMotors.Turn(90);
			
			driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * 0.1));
			OpenClaw(false, true);
			sleep(500);
			arm.MoveShoulder(-95);
			driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 1.85));
			
			arm.MoveSlide(-750, true);
			OpenClaw();
			sleep(350);
			driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * .5));
			
			arm.MoveShoulder(-120);
			arm.MoveSlide(-50, true);
			MoveWrist(0);
			
			driveMotors.Move(Direction.RIGHT, (int)(Config.TILE_LENGTH * 1));
			driveMotors.Turn(-90);
			driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * .2));
			arm.MoveShoulder(0);
		}
	}


	private void Park(int target) {
		telemetry.addData("Parking", String.format("PARKING IN SPOT %d", target));

		switch(target) {
			case 1:
			driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * .5));
			break;
			
			case 2:
			driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * .5));
			break;
			
			case 3:
			driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 1.5));
			break;

			default:
			telemetry.addLine(String.format("ERROR: Target %d not in range 1-3", target));
			//telemetry.addLine(String.format("PARKING IN DEFAULT SPOT (%d)", Config.DEFAULT_PARKING_SPOT));
			//Park(Config.DEFAULT_PARKING_SPOT);
			break;
		}
		telemetry.update();
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
		TfodProcessor.Builder myTfodProcessorBuilder;
		VisionPortal.Builder myVisionPortalBuilder;

		// First, create a TfodProcessor.Builder.
		myTfodProcessorBuilder = new TfodProcessor.Builder();
		// Set the name of the file where the model can be found.
		myTfodProcessorBuilder.setModelFileName("CenterStage.tflite");
		// Set the full ordered list of labels the model is trained to recognize.
		myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("ball", "cube"));
		// Set the aspect ratio for the images used when the model was created.
		myTfodProcessorBuilder.setModelAspectRatio(16 / 9);
		// Create a TfodProcessor by calling build.
		myTfodProcessor = myTfodProcessorBuilder.build();
		// Next, create a VisionPortal.Builder and set attributes related to the camera.
		myVisionPortalBuilder = new VisionPortal.Builder();
		if (USE_WEBCAM) {
		// Use a webcam.
		myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
		} else {
		// Use the device's back camera.
		myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
		}
		// Add myTfodProcessor to the VisionPortal.Builder.
		myVisionPortalBuilder.addProcessor(myTfodProcessor);
		// Create a VisionPortal by calling build.
		myVisionPortal = myVisionPortalBuilder.build();
	}
	
	private void telemetryTfod() {
		List<Recognition> myTfodRecognitions;
		Recognition myTfodRecognition;
		float x;
		float y;

		// Get a list of recognitions from TFOD.
		myTfodRecognitions = myTfodProcessor.getRecognitions();
		telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
		// Iterate through list and call a function to display info for each recognized object.
		for (Recognition myTfodRecognition_item : myTfodRecognitions) {
			myTfodRecognition = myTfodRecognition_item;
			// Display info about the recognition.
			telemetry.addLine("");
			// Display label and confidence.
			// Display the label and confidence for the recognition.
			telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
			// Display position.
			x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
			y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
			// Display the position of the center of the detection boundary for the recognition
			telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
			// Display size
			// Display the size of detection boundary for the recognition
			telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
		}
	}
}
