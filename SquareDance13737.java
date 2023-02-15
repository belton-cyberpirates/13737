package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.DriveMotors;

@Autonomous

public class SquareDance13737 extends LinearOpMode {
  
  private DriveMotors driveMotors;
  private Arm arm;
  private DcMotor claw;

  private void MotorSetup() {
    arm.DropArm();
    sleep(200);
    arm.Initialize();
    sleep(1000);
    claw.setPower(0.5);
   }
   
     @Override
  public void runOpMode() {
    // argument order *must* be fr-fl-bl-br
    driveMotors = new DriveMotors(
      hardwareMap.get(DcMotorEx.class, "m2"),
      hardwareMap.get(DcMotorEx.class, "m3"),
      hardwareMap.get(DcMotorEx.class, "m4"),
      hardwareMap.get(DcMotorEx.class, "m1")
    );
    
    arm = new Arm(
      hardwareMap.get(DcMotorEx.class, "arm1"),
      hardwareMap.get(DcMotorEx.class, "arm2")
    );
    claw = hardwareMap.get(DcMotor.class, "claw");

    waitForStart();
    if (opModeIsActive()) {
      MotorSetup();
      
      // Clap our hands
      OpenClaw();
      CloseClaw();
      
      // Do-Si-Do
      driveMotors.Move(Direction.FORWARD, Config.TILE_LENGTH);
      driveMotors.Move(Direction.BACKWARD, Config.TILE_LENGTH);
      
      // Do a twirl
      driveMotors.Turn(360);
      
      // Do another twirl
      driveMotors.Turn(-360);
      
      // Wave your arms
      arm.Move(Config.LOW_POLE_HEIGHT);
      sleep(1500);
      arm.Move(Config.MID_POLE_HEIGHT);
      sleep(1500);
      arm.Move(Config.HIGH_POLE_HEIGHT);
      sleep(3000);
      
      arm.Move(Config.BOTTOM);
      sleep(1500);
      arm.Move(Config.CRUISING_HEIGHT);
      sleep(1500);
      arm.Move(Config.TOP);
      sleep(1500);
    }
  }
  
  
  private void OpenClaw() {
    sleep(250);
    claw.setPower(-0.3);
    sleep(250);
    claw.setPower(0);
  }
  
  
  private void CloseClaw() {
    claw.setPower(0.9);
    sleep(200);
    claw.setPower(0.3);
  }
}