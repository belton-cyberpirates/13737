package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.Config;


public class DriveMotors {
  private DcMotor frontLeft;
  private DcMotor frontRight;
  private DcMotor backLeft;
  private DcMotor backRight;
  static Orientation angles;


  public DriveMotors(DcMotor frontRight,
                     DcMotor frontLeft,
                     DcMotor backLeft,
                     DcMotor backRight) {
    this.frontRight = frontRight;
    this.frontLeft = frontLeft;
    this.backLeft = backLeft;
    this.backRight = backRight;
  }
  
  
  private void Reset() {
    this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }
  
  
  private void SetToRunPosition() {
    this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }
  
  
  private void MotorInit() {
    this.Reset();
    this.SetTargetPositions(0, 0, 0, 0);
    this.SetToRunPosition();
  }
  private void MotorInitTurn() {
    this.Reset();
    this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
  }

  
  private void SetPower(double power, double compensation) {
    this.frontLeft.setPower(power * compensation);
    this.frontRight.setPower(power * compensation);
    this.backLeft.setPower(power);
    this.backRight.setPower(power);
  }

  private void SetPower(double power) {
    this.SetPower(power, 1);
  }
  
  
  private void SetTargetPositions(int fr, int fl, int bl, int br) {
    this.frontRight.setTargetPosition(fr);
    this.frontLeft.setTargetPosition(fl);
    this.backLeft.setTargetPosition(bl);
    this.backRight.setTargetPosition(br);
  }


 /**
   * Pivots around the back-right wheel as if turning
   * @param angle the angle of the expected pivot
   */
  public void Pivot(int angle) {
    int distance = (int)((angle * Config.PIVOT_TICKS_PER_360_DEG) / 360);
    this.MotorInit();

    this.frontLeft.setPower(Config.AUTO_PIVOT_POWER);
    this.backRight.setPower(0.9); // become rock

    // set target positions manually to keep front-right and back-left dead
    this.frontLeft.setTargetPosition(distance);
    this.backRight.setTargetPosition(0); // DON'T MOVE

    while (!!(this.frontLeft.isBusy() ||
    this.backRight.isBusy() )) {}
  }
  
  
  public void Move(Direction direction, int distance) {
    Move(direction, distance, false);
  }


  public void Move(Direction direction, int distance, boolean ramp) {
    this.MotorInit();

    switch(direction) {
      case FORWARD:
        this.SetTargetPositions(-distance, distance, distance, -distance);
        break;
  
      case BACKWARD:
        this.SetTargetPositions(distance, -distance, -distance, distance);
        break;
  
      case LEFT:
      this.SetTargetPositions(-distance, -distance, distance, distance);
      break;
  
      case RIGHT:
        this.SetTargetPositions(distance, distance, -distance, -distance);
        break;
        
      case FRONT_RIGHT:
        this.SetTargetPositions(0, distance, 0, -distance);
        break;
        
      case BACK_LEFT:
        this.SetTargetPositions(0, -distance, 0, distance);
        break;
        
      case FRONT_LEFT:
        this.SetTargetPositions(-distance, 0, distance, 0);
        break;
        
      case BACK_RIGHT:
        this.SetTargetPositions(distance, 0, -distance, 0);
        break;
    }
    // while motors are running, correct power with gyro angle
    this.SetPower(Config.MIN_SPEED);
    this.WaitForMotors(distance, direction, ramp);
  }
  
  
  public void Turn(int angle) {
    int distance = (int)((angle * Config.TICKS_PER_360_DEG) / 360);
    
    this.MotorInit();
    this.SetPower(Config.MIN_SPEED);
    this.SetTargetPositions(distance, distance, distance, distance);
    
    this.WaitForMotors(angle);
  }


  private void WaitForMotors(int distance) {
    this.WaitForMotors(distance);
  }


  /**
   * Wait until motion is complete
   * @param distance target distance meant to be reached
   */
  private void WaitForMotors(int distance) {
    angles = AutoLeft.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    int initialPos = (int)angles.firstAngle;
    while ((this.frontLeft.isBusy() ||
            this.frontRight.isBusy() ||
            this.backLeft.isBusy() ||
            this.backRight.isBusy() )) {}
  }
}
