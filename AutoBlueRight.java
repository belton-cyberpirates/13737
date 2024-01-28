package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import org.firstinspires.ftc.teamcode.DriveMotors;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.TensorflowCamera;



@Autonomous(name = "AutoBlueRight")
public class AutoBlueRight extends LinearOpMode {
  private TensorflowCamera camera;
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
	MoveWrist(0);
	arm.DropArm();
	sleep(1000);
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
	
	//initTfod();
	int position = camera.GetPropPos();
	  
	telemetry.addData("position", position);
	telemetry.update();
	
	waitForStart();

	if (opModeIsActive()) { // <----------------------------------------------------------------
	  MotorSetup();
	  
	  switch(position) {
		case 0:
			// Move to the left spike mark
			driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 1));
			driveMotors.Turn(-45);
			driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 0.2));
			// Drop purple pixel
			OpenClaw(false, true);
			sleep(350);
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
			OpenClaw(false, true);
			sleep(350);
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
			OpenClaw(false, true);
			sleep(350);
			// Move to a starting point for scoring / parking auto
			driveMotors.Move(Direction.LEFT, (int)(Config.TILE_LENGTH * .6));
			driveMotors.Move(Direction.FORWARD, (int)(Config.TILE_LENGTH * 1.2))
			// Rotate towards board
			driveMotors.Turn(-90);
			break;
	  }
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
}
