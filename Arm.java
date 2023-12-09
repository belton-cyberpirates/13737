package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Direction;


public class Arm {
  Telemetry telemetry;
  private DcMotorEx leftShoulder;
  private DcMotorEx rightShoulder;
  private DcMotorEx leftElbow;
  private DcMotorEx rightElbow;

  public Arm(DcMotorEx leftShoulder, DcMotorEx rightShoulder, DcMotorEx leftElbow, DcMotorEx rightElbow) {
    this.leftShoulder = leftShoulder;
    this.rightShoulder = rightShoulder;
    this.leftElbow = leftElbow;
    this.rightElbow = rightElbow;
  }
  
  public void DropArm(){
    this.arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.arm1.setPower(-0.95);
    this.arm2.setPower(0.95);
  }

  
  public void Initialize() {
    this.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.arm1.setTargetPosition(0);
    this.arm2.setTargetPosition(0);
    this.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }
  
  private void setVelocity(int velocity) {
    this.arm1.setVelocity(velocity);
    this.arm2.setVelocity(velocity);
  }
  
  public void Move(int position) {
    this.setVelocity(Config.ARM_VELOCITY);
    
    this.arm1.setTargetPosition(position);
    this.arm2.setTargetPosition(-position);
  }
  

public void Move(int position, boolean waitForDone) {
    this.Move(position);
    
    if (waitForDone){
      WaitForMotors();
    }
  }

  public void Move(int position, boolean waitForDone, int newVelocity) {
    this.setVelocity(newVelocity);

    this.arm1.setTargetPosition(position);
    this.arm2.setTargetPosition(-position);
    
    if (waitForDone){
      WaitForMotors();
    }

  }
  
  private void WaitForMotors() {
    while (this.arm1.isBusy() || this.arm2.isBusy()) {}
    this.setVelocity(Config.ARM_VELOCITY);
  }
}