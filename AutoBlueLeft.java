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


@Autonomous(name = "AutoBlueLeft", preselectTeleOp="MecanumDriveFieldCentric")
public class AutoBlueLeft extends LinearOpMode {
  private OpenCvCamera camera;
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
	sleep(1500);
	arm.Initialize();
  }

  /**
   * This function is executed when this Op Mode is initialized from the Driver Station.
   */
  @Override
  public void runOpMode() {

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
	  MotorSetup(); // arm between 0 and -2500
	  
	  arm.MoveShoulder(-1750);
	  driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * .1));
	  sleep(200);
	  driveMotors.Move(Direction.LEFT, (int)(Config.TILE_LENGTH * 1));
	  driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 1.2));
	  
	  driveMotors.Turn(-90);
	  
	  driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * .9));
	  
	  arm.MoveSlide(-300, true);
	  OpenClaw(true, false);
	  sleep(350);
	  driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * .775));
	  
	  
	  arm.MoveShoulder(-2500);
	  arm.MoveSlide(-50);
	  MoveWrist(1);
	  
	  driveMotors.Turn(180);
	  arm.MoveShoulder(-200, true);
	  OpenClaw(false, true);
	  MoveWrist(0);
	  arm.MoveShoulder(-350);
	  
	  driveMotors.Move(Direction.RIGHT, (int)(Config.TILE_LENGTH * 1.6));
	  driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * .6));
	  
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
  
  /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
            .setModelFileName(TFOD_MODEL_FILE)
            // With the following lines commented out, the default TfodProcessor Builder
            // will load the default model for the season. To define a custom model to load, 
            // choose one of the following:
            //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
            //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
            //.setModelAssetName(TFOD_MODEL_ASSET)
            //.setModelFileName(TFOD_MODEL_FILE)

            // The following default settings are available to un-comment and edit as needed to 
            // set parameters for custom models.
            .setModelLabels(LABELS)
            //.setIsModelTensorFlow2(true)
            //.setIsModelQuantized(true)
            //.setModelInputSize(300)
            //.setModelAspectRatio(16.0 / 9.0)

            .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //.setCameraResolution(new Size(CameraResoX, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()
  
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
