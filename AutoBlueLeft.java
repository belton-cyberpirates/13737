package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
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
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.tfod.Tfod;
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
  private DriveMotors driveMotors;
  private Arm arm;
  private CRServo clawLeft;
  private CRServo clawRight;


  /**
   * Set reliable initial configuration for robot motors
   */
  public void MotorSetup() { // TODO add claws to motor setup
	clawLeft.setDirection(DcMotor.Direction.REVERSE);
	CloseClaw(clawLeft, clawRight);
	arm.DropArm();
	sleep(500);
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
	  hardwareMap.get(DcMotorEx.class, "left_shoulder"),
	  hardwareMap.get(DcMotorEx.class, "right_shoulder"),
	  hardwareMap.get(DcMotorEx.class, "right_elbow")
	);
	clawLeft = hardwareMap.get(CRServo.class, "s1");
	clawRight = hardwareMap.get(CRServo.class, "s2");

	waitForStart();

	if (opModeIsActive()) { // <----------------------------------------------------------------
	  MotorSetup();
	  arm.Move(25);
	  driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 1.1));
	  driveMotors.Turn(-90);
	  driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 1.5));
	  driveMotors.Move(Direction.LEFT, (int)(Config.TILE_LENGTH * 1));
	  driveMotors.Turn(90);
	  driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * .2));
	  arm.Move(0);
	  sleep(1000);
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


  /**
   * Open the given claw(s)
   */
  public void OpenClaw(DcMotor... claws) {
	for (DcMotor claw : claws) {
	  claw.setPower(-0.3);
	  sleep(200);
	  claw.setPower(0);
	}
  }


  /**
   * Close the given claw(s)
   */
  public void CloseClaw(CRServo... claws) {
	for (CRServo claw : claws) {
	  claw.setPower(0.9);
	  sleep(500);
	  claw.setPower(0.3);
	}
  }
}
