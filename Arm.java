package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Direction;


public class Arm {
  Telemetry telemetry;
  private DcMotor arm1;
  private DcMotor arm2;
  
  public Arm(DcMotor arm1, DcMotor arm2) {
    this.arm1 = arm1;
    this.arm2 = arm2;
  }
  
  public void DropArm(){
    this.arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    this.arm1.setPower(-.95);
    this.arm2.setPower(.95);
  }

  
  public void Initialize() {
    this.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    this.arm1.setTargetPosition(0);
    this.arm2.setTargetPosition(0);
    this.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }
  
  private void SetPower(float power) {
    this.arm1.setPower(power);
    this.arm2.setPower(power);
  }
  
  public void Move(int position) {
    final float power = .95f /*position < 10 ? 0.5f : 0.9f*/;
    this.SetPower(power);
    
    this.arm1.setTargetPosition(position);
    this.arm2.setTargetPosition(-position);
  }
  
  
    public void Move(int position, boolean waitForDone) {
    this.Move(position);
    
    if (waitForDone){
      WaitForMotors();
    }
  }
  
  private void WaitForMotors() {
        // TODO: do we need this "!!"?
    while (this.arm1.isBusy() || this.arm2.isBusy()) {}
  }
}