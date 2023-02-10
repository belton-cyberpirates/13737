package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Set;
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

import org.firstinspires.ftc.teamcode.DriveMotors;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.ParkingSpot;
import org.firstinspires.ftc.teamcode.Config;

@Autonomous(name = "AutoLeft"/*, preselectTeleOp = "xDriveCode"*/)
public class AutoLeft extends LinearOpMode {
  public VuforiaCurrentGame vuforiaPOWERPLAY;
  public Tfod tfod;
  public DriveMotors driveMotors;
  public Arm arm;
  public DcMotor claw;
  public static BNO055IMU imu;
  public static BNO055IMU.Parameters imuParameters;
  int ParkingPosition;
  Recognition recognition;
  ParkingSpot parkingSpot = ParkingSpot.EYES;
  


  /**
   * Set reliable initial configuration for robot motors
   */
  public void MotorSetup() {
    arm.DropArm();
    sleep(500);
    CloseClaw();
    arm.Initialize();
   }
   
  /**
   * Describe this function...
   */
  public void TFInitialize() {
    vuforiaPOWERPLAY.initialize(
        "", // vuforiaLicenseKey
        hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
        "", // webcamCalibrationFilename
        false, // useExtendedTracking
        true, // enableCameraMonitoring
        VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
        0, // dx
        0, // dy
        0, // dz
        AxesOrder.XZY, // axesOrder
        90, // firstAngle
        90, // secondAngle
        0, // thirdAngle
        true); // useCompetitionFieldTargetLocations
    // Set isModelTensorFlow2 to true if you used a TensorFlow 2 tool,
    // such as ftc-ml, to create the model. Set isModelQuantized to
    // true if the model is quantized. Models created with ftc-ml are
    // quantized. Set inputSize to the image size corresponding to
    // the model. If your model is based on SSD MobileNet v2 320x320,
    // the image size is 300 (srsly!). If your model is based on
    // SSD MobileNet V2 FPNLite 320x320, the image size is 320.
    // If your model is based on SSD MobileNet V1 FPN 640x640 or
    // SSD MobileNet V2 FPNLite 640x640, the image size is 640.
    tfod.useModelFromFile("EGRR.tflite", JavaUtil.createListWith("EYES", "GEARS", "GEARS", "ROBOTS"), true, true, 320);
    // tfod.useModelFromFile("13737TF2.tflite", JavaUtil.createListWith("EYES", "GEARS", "ROBOTS"), true, true, 320);
    tfod.initialize(vuforiaPOWERPLAY, (float) 0.7, true, true);
    tfod.activate();
    tfod.setZoom(1, 16 / 9);
    telemetry.addData("tfready", "camera dots");
    telemetry.update();
  }
  

 /**
 * Describe this function...
 */
  public void doTF() {
  List<Recognition> recognitions = tfod.getRecognitions();
  
    int index;
    for (int attempt = 0; attempt < 100; attempt++) {
      recognitions = tfod.getRecognitions();
      if (JavaUtil.listLength(recognitions) == 0) {
        telemetry.addData("TF", "none detected");
      } else {
        index = 0;
        for (Recognition recognition_item : recognitions) {
          recognition = recognition_item;
          displayInfo(index);
          index = index + 1;
          telemetry.addData("Detected", recognition.getLabel());
        }
        telemetry.update();
        return; // found object, can stop early
      }
      telemetry.update();
    }
  }
  
  private void displayInfo(int i) {
    telemetry.addData("label" + i, recognition.getLabel());
    if (recognition.getLabel().equals("ROBOTS")) {
      parkingSpot = ParkingSpot.ROBOTS;
    } else if (recognition.getLabel().equals("GEARS")) {
      parkingSpot = ParkingSpot.GEARS;
    } else {
      parkingSpot = ParkingSpot.EYES;
    }
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    vuforiaPOWERPLAY = new VuforiaCurrentGame();
    tfod = new Tfod();

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

    TFInitialize();
    waitForStart();
    try {
    if (opModeIsActive()) { // <----------------------------------------------------------------
      doTF();
      MotorSetup();
      arm.Move(Config.CRUISING_HEIGHT);

      // move to MID pole
      driveMotors.Move(Direction.FORWARD, Config.INITIAL_CORRECTION + (int)(2.05*Config.TILE_LENGTH));
      
      // deposit cone
      driveMotors.Turn(130);
      arm.Move(Config.MID_POLE_HEIGHT, true);
      driveMotors.Move(Direction.FORWARD, (int)(Config.BUMP*1.1));
      arm.Move(Config.SIDE_STACK_HEIGHT);
      sleep(500);
      OpenClaw();
      driveMotors.Move(Direction.BACKWARD, (int)(Config.BUMP*1.2));
      driveMotors.Turn(136); //130 + 140 = 270 (90*3=270)

      // go for 2nd cone
      OpenClaw();
      arm.Move(Config.SIDE_STACK_HEIGHT, true);
      driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * .98));
      CloseClaw();
      arm.Move(Config.LOW_POLE_HEIGHT, true);
      driveMotors.Move(Direction.BACKWARD, (int)(Config.TILE_LENGTH * .48));
      
      
      // place 2nd cone
      driveMotors.Turn(-90);
      arm.Move(Config.LOW_POLE_HEIGHT + 20);
      driveMotors.Move(Direction.FORWARD, (int)(Config.BUMP*.3));
      arm.Move(Config.LOW_POLE_HEIGHT - 25, true);
      OpenClaw();
      driveMotors.Move(Direction.BACKWARD, (int)(Config.BUMP*0.5));
      arm.Move(Config.CRUISING_HEIGHT, true);
 
      // Park
        Park(parkingSpot);
    }
    
    } catch(Exception e) {
      // this will catch if AutoLeft cant run
      telemetry.addData("Error", "Something went wrong running AutoLeft");
      telemetry.update();
    }
    vuforiaPOWERPLAY.close();
    tfod.close();
  }
  
  public void Park(ParkingSpot target) {
    try {
      switch(target) {
      case EYES:
        telemetry.addData("Parking", "PARKING EYES");
        driveMotors.Move(Direction.RIGHT, (int)(Config.TILE_LENGTH * 0.5));
        driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * .25));
        break;
        
      case GEARS:
        telemetry.addData("Parking", "PARKING GEARS");
        driveMotors.Move(Direction.LEFT, (int)(Config.TILE_LENGTH * 0.5));
        driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * .25));
        break;
        
      case ROBOTS:
        telemetry.addData("Parking", "PARKING ROBOTS");
        driveMotors.Move(Direction.LEFT, (int)(Config.TILE_LENGTH * 1.5));
        driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * .25));
      }
      telemetry.update();
    } catch(Exception e) {
        telemetry.addData("Park", "Couldn't Park");
        telemetry.update();
    }
  }
  
  public void OpenClaw() {
    sleep(200);
    claw.setPower(-0.3);
    sleep(200);
    claw.setPower(0);
  }
  
  
  public void CloseClaw() {
    claw.setPower(0.9);
    sleep(500);
    claw.setPower(0.3);
  }
}