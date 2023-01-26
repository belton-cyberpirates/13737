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

  
  private void Update() {
    this.leftTread.Update();
    this.rightTread.Update();
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
   * Wait until motion is complete
   * @param distance target distance meant to be reached
   */
  private void WaitForMotors(int distance) {
    while (!!(this.leftTread.IsBusy() ||
              this.rightTread.IsBusy())) {
                this.Update();
              }
  }
}