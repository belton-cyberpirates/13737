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

  
  private void SetPower(double power) {
    this.frontLeft.setPower(power);
    this.frontRight.setPower(power);
    this.backLeft.setPower(power);
    this.backRight.setPower(power);
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
    this.MotorInit();
    angles = AutoLeft.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double initialPos = angles.firstAngle;
    // AutoLeft.imu.resetYaw(); //should work according to FTC but doesnt?
    
    this.SetPower(Config.MIN_SPEED);
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
    this.WaitForMotors(distance, direction);
  }


  private void GyroCorrect(Direction direction, double initial) {
    angles = AutoLeft.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double correction = (angles.firstAngle - initial) * Config.GYRO_CORRECTION;
    //left drift is positive angle, right is negative
    switch(direction) {
      case FORWARD:
        this.setCorrection(-correction, correction, correction, -correction);
        break;

      case BACKWARD:
        this.setCorrection(correction, -correction, -correction, correction);
        break;

      case LEFT:
        this.setCorrection(-correction, -correction, correction, correction);
        break;

      case RIGHT:
        this.setCorrection(correction, correction, -correction, -correction);
        break;

      case FRONT_RIGHT:
        this.setCorrection(0, -correction, 0, correction);
        break;

      case FRONT_LEFT:
        this.setCorrection(correction, 0, -correction, 0);
        break;

      case BACK_LEFT:
        this.setCorrection(0, correction, 0, -correction);
        break;

      case BACK_RIGHT:
        this.setCorrection(-correction, 0, correction, 0);
        break;

    }
  }


  private void setCorrection(double fr, double fl, double bl, double br) {
    this.frontRight.setPower(this.frontRight.getPower() + fr);
    this.frontLeft.setPower(this.frontLeft.getPower() + fl);
    this.backLeft.setPower(this.backLeft.getPower() + bl);
    this.backRight.setPower(this.backRight.getPower() + br);
  }
  
  
  public void Turn(int angle) {
    int distance = (int)((angle * Config.TICKS_PER_360_DEG) / 360);
    
    this.MotorInit();
    this.SetPower(Config.MIN_SPEED);
    this.SetTargetPositions(distance, distance, distance, distance);
    
    this.WaitForMotors(angle);
  }


  public void TurnGyro(int angle) {
    this.MotorInitTurn();
    angles = AutoLeft.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    
    int initialAngle = (int)(angles.firstAngle);
    int target = initialAngle + angle;
    if (target >= 180) {
      target -= 360;
    } else if (target <= -180) {
      target += 360;
    }
    
    while(!(angles.firstAngle == target)) {
      angles = AutoLeft.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      this.SetPower(Config.MIN_SPEED);
    }
    this.SetPower(0);
    
  }
  

  /**
   * power should follow quadratic curve with mins at 0.1 and max at 0.9
   * @param x current distance/position
   * @param distance total target distance
   * @return y=c*x(x-distance) + 0.1 where c is calculated as 4(MIN-MAX)/(d^2)
   */
  private double GetPower(int x, int distance) {
    double speed = -x*(x-distance) + Config.MIN_SPEED;
    return speed > Config.MAX_SPEED ? Config.MAX_SPEED : speed; // do not go higher than MAX_SPEED
  }



  private void WaitForMotors(int distance) {
    this.WaitForMotors(distance, null);
  }


  /**
   * Wait until motion is complete
   * @param distance target distance meant to be reached
   */
  private void WaitForMotors(int distance, Direction direction) {
    angles = AutoLeft.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    int initialPos = (int)angles.firstAngle;
    while (!!(this.frontLeft.isBusy() ||
              this.frontRight.isBusy() ||
              this.backLeft.isBusy() ||
              this.backRight.isBusy() )) {
                // assume back wheels are indicative of whole movement
                int x = Math.max(Math.abs(this.backLeft.getCurrentPosition()), Math.abs(this.backRight.getCurrentPosition()));
                this.SetPower(this.GetPower(x, distance));
                if (direction != null) {
                  GyroCorrect(direction, initialPos);
                }
              }
  }
}
