package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import org.firstinspires.ftc.teamcode.DriveMotors;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.ParkingSpot;
import org.firstinspires.ftc.teamcode.Config;

@Autonomous(name = "AutoLeft")
public class AutoLeft extends LinearOpMode {

  private OpenCvCamera camera;
  private AprilTagDetectionPipeline aprilTagDetectionPipeline;
  private DriveMotors driveMotors;
  private Arm arm;
  private DcMotor claw;

  private boolean CameraError = false;


  int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
  camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
  aprilTagDetectionPipeline = new AprilTagDetectionPipeline(Config.TAGSIZE, Config.FX, Config.FY, Config.CX, Config.CY);

  camera.setPipeline(aprilTagDetectionPipeline);
  camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
  {
      @Override
      public void onOpened()
      {
          camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
      }

      @Override
      public void onError(int errorCode)
      {
        telemetry.addData("Error", "Camera failed to open with error " + errorCode);
        telemetry.update();
        CameraError = true;
      }
  });


  /**
   * Set reliable initial configuration for robot motors
   */
  private void MotorSetup() {
    arm.DropArm();
    sleep(200);
    arm.Initialize();
    claw.setPower(0.5);
  }


  /**
   * Attempt to recognize the AprilTag and return which tag we see
   */
  private int RunDetection() {
    for (int numFramesWithoutDetection = 0; numFramesWithoutDetection < Config.MAX_NUM_FRAMES_NO_DETECTION; numFramesWithoutDetection++) {
      // Calling getDetectionsUpdate() will only return an object if there was a new frame
      // processed since the last time we called it. Otherwise, it will return null. This
      // enables us to only run logic when there has been a new frame, as opposed to the
      // getLatestDetections() method which will always return an object.
      ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

      // add camera stats to telemetry
      telemetry.addData("FPS", camera.getFps());
      telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
      telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
      telemetry.update();

      // If we haven't seen a tag for a few frames, lower the decimation
      // so we can hopefully pick one up if we're e.g. far back
      if(numFramesWithoutDetection >= Config.NUM_FRAMES_BEFORE_LOW_DECIMATION) {
          aprilTagDetectionPipeline.setDecimation(Config.lDECIMATION_LOW);
      }

      if (detections == null == detections.size() == 0) {
        // pass if there's no new frame or we didn't detect anything
        sleep(20); // give the camera a chance to process the frame
        continue;
      }

      // If we're here, we have a new frame and we detected at least one tag
      // turn on high decimation to increase the frame rate
      aprilTagDetectionPipeline.setDecimation(Config.DECIMATION_HIGH);


      AprilTagDetection detection = detections.get(0);
      telemetry.addLine(String.format("\n!!---[[ DETECTED ID=%d]]---!!", detection.id));
      telemetry.addLine(String.format("Translation X: %.2f", detection.pose.x));
      telemetry.addLine(String.format("Translation Y: %.2f", detection.pose.y));
      telemetry.addLine(String.format("Translation Z: %.2f", detection.pose.z));
      telemetry.addLine(String.format("Rotation Yaw: %.2f", detection.pose.yaw));
      telemetry.addLine(String.format("Rotation Pitch: %.2f", detection.pose.pitch));
      telemetry.addLine(String.format("Rotation Roll: %.2f", detection.pose.roll));
      telemetry.update();

      return detection.id;
    }

    telemetry.addLine(String.format("!!---[[ NO TAG DETECTED AFTER %d ATTEMPTS ]]---!!", Config.MAX_NUM_FRAMES_NO_DETECTION));
    telemetry.update();
    return -1; // no tag detected; godspeed
  }


  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {

    // argument order *must* be fr-fl-bl-br
    driveMotors = new DriveMotors(
      hardwareMap.get(DcMotor.class, "m2"),
      hardwareMap.get(DcMotor.class, "m3"),
      hardwareMap.get(DcMotor.class, "m4"),
      hardwareMap.get(DcMotor.class, "m1")
    );
    
    arm = new Arm(
      hardwareMap.get(DcMotor.class, "arm1"),
      hardwareMap.get(DcMotor.class, "arm2")
    );
    claw = hardwareMap.get(DcMotor.class, "claw");

    // Wait for the game to start (driver presses PLAY)
    //? Can we do detection before this?
    waitForStart();

    telemetry.setMsTransmissionInterval(50);
    MotorSetup();

    if (opModeIsActive()) {
      for (;;) {
        int parkingSpot = CameraError ? -1 : RunDetection(); // retrieve our expected parking spot (or -1 if we failed to open the camera)
        sleep(1000);
      }
      // begin autonomous
      CloseClaw(); // grab initial cone
      arm.Move(Config.CRUISING_HEIGHT);

      // move to high pole
      driveMotors.Move(Direction.FORWARD, Config.INITIAL_CORRECTION + (int)(2*Config.TILE_LENGTH));
      
      // deposit cone
      driveMotors.Turn(45);
      arm.Move(Config.HIGH_POLE_HEIGHT, true);
      driveMotors.Move(Direction.FORWARD, Config.BUMP);
      arm.Move(Config.MID_POLE_HEIGHT);
      sleep(500);
      OpenClaw();
      driveMotors.Move(Direction.BACKWARD, Config.BUMP);
  
      // go for 2nd cone
      driveMotors.Turn(-135);
      driveMotors.Move(Direction.FORWARD, Config.TILE_LENGTH);
      arm.Move(Config.SIDE_STACK_HEIGHT, true);
      driveMotors.Move(Direction.FORWARD, Config.BUMP);
      arm.Move(Config.SIDE_STACK_HEIGHT - 3);
      sleep(500);
      CloseClaw();
      arm.Move(Config.MID_POLE_HEIGHT, true);
      
      
      // place 2nd cone
      driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * 1.2));
      arm.Move(Config.TOP, true);
      driveMotors.Turn(135);
      driveMotors.Move(Direction.FORWARD, Config.BUMP);
      arm.Move(Config.MID_POLE_HEIGHT);
      sleep(500);
      OpenClaw();
      driveMotors.Move(Direction.BACKWARD, Config.BUMP);
      driveMotors.Turn(-45);
      arm.Move(Config.CRUISING_HEIGHT, true);

      // Third cone?
      
      // Park
      Park(parkingSpot);
    }
  }
  
  
  private void Park(int target) {
    telemetry.addData("Parking", String.format("PARKING IN SPOT %d", target));

    switch(target) {
    case 1:
      driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * 1.6));
      break;
      
    case 2:
      driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * .3));
      break;
      
    case 3:
      driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * .3));

    default:
      telemetry.addLine(String.format("ERROR: Target %d not in range 1-3", target));
      telemetry.addLine(String.format("PARKING IN DEFAULT SPOT (%d)", Config.DEFAULT_PARKING_SPOT))
      driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * 1.6));
      break;
    }
    telemetry.update();
  }
  
  private void OpenClaw() {
    sleep(200);
    claw.setPower(-0.3);
    sleep(200);
    claw.setPower(0);
  }
  
  
  private void CloseClaw() {
    claw.setPower(0.9);
    sleep(200);
    claw.setPower(0.3);
  }
}