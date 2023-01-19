package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.Config;


public class DriveMotors {
  private Tread leftTread;
  private Tread rightTread;
  static Orientation angles;


  public DriveMotors(DcMotor frontRight,
                     DcMotor frontLeft,
                     DcMotor backLeft,
                     DcMotor backRight) {
    this.leftTread = new Tread(frontLeft, backLeft);
    this.rightTread = new Tread(frontRight, backRight, true); // right tread runs in reverse
  }

  
  private void SetPower(double power) {
    this.leftTread.SetPower(power);
    this.rightTread.SetPower(power);
  }


  public void Move(Direction direction, int distance) {
    switch(direction) {
      case FORWARD:
        this.leftTread.Move(distance);
        this.rightTread.Move(distance);
        break;
  
      case BACKWARD:
        this.leftTread.Move(-distance);
        this.rightTread.Move(-distance);
        break;
    }
    this.WaitForMotors(distance);
  }
  
  
  public void Turn(int angle) {
    int distance = (int)((angle * Config.TICKS_PER_360_DEG) / 360);
    
    this.leftTread.Move(distance);
    this.rightTread.Move(-distance); // turning right means right tread moves backward
    
    this.WaitForMotors(angle);
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


  /**
   * Wait until motion is complete
   * @param distance target distance meant to be reached
   */
  private void WaitForMotors(int distance) {
    while (!!(this.leftTread.IsBusy() ||
              this.rightTread.IsBusy())) {
                // assume back wheels are indicative of whole movement
                int x = Math.abs(this.leftTread.GetCurrentPosition());
                this.SetPower(this.GetPower(x, distance));
                if (direction != null) {
                  GyroCorrect(direction, initialPos);
                }
              }
  }
}