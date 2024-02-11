package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.Config;


public class DriveMotors {
  private DcMotorEx frontLeft;
  private DcMotorEx frontRight;
  private DcMotorEx backLeft;
  private DcMotorEx backRight;
  static Orientation angles;

  private LinearOpMode auto;


  public DriveMotors(LinearOpMode auto) {
    this.auto = auto;
    this.frontRight = auto.hardwareMap.get(DcMotorEx.class, Config.FRONT_RIGHT_WHEEL_NAME);
    this.frontLeft = auto.hardwareMap.get(DcMotorEx.class, Config.FRONT_LEFT_WHEEL_NAME);
    this.backLeft = auto.hardwareMap.get(DcMotorEx.class, Config.BACK_LEFT_WHEEL_NAME);
    this.backRight = auto.hardwareMap.get(DcMotorEx.class, Config.BACK_RIGHT_WHEEL_NAME);
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
  
  
  private void SetZeroBehaviour() {
    this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }


  private void MotorInit() {
    this.Reset();
    this.SetZeroBehaviour();
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

  
  private void setVelocity(int velocity) {
    this.frontLeft.setVelocity(velocity);
    this.frontRight.setVelocity(velocity);
    this.backLeft.setVelocity(velocity);
    this.backRight.setVelocity(velocity);
  }


  private void SetTargetPositions(int fr, int fl, int bl, int br) {
    this.frontRight.setTargetPosition(fr);
    this.frontLeft.setTargetPosition(fl);
    this.backLeft.setTargetPosition(bl);
    this.backRight.setTargetPosition(br);
  }
  
  
  public void Move(Direction dir, int dist) {
    Move(dir, dist, 1);
  }
  

  public void Move(Direction direction, int distance, double mult) {
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
    // while motors are running, wait
    this.setVelocity((int)(Config.CRUISE_SPEED * mult));
    this.WaitForMotors();
  }
  
  
  public void Turn(int angle) {
    int distance = (int)((angle * Config.TICKS_PER_360_DEG) / 360);
    
    this.MotorInit();
    this.setVelocity(Config.CRUISE_SPEED);
    this.SetTargetPositions(distance, distance, distance, distance);
    
    this.WaitForMotors();
  }


  /**
   * Wait until motion is complete
   * @param distance target distance meant to be reached
   */
  private void WaitForMotors() {
    while ((this.frontLeft.isBusy() ||
            this.frontRight.isBusy() ||
            this.backLeft.isBusy() ||
            this.backRight.isBusy() )) {}
  }
}
