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
  private DcMotorEx Elbow;
  private DcMotorEx[] motors;

  // leftShoulder, rightShoulder, leftElbow, rightElbow
  public Arm(DcMotorEx leftShoulder, DcMotorEx rightShoulder, DcMotorEx Elbow) {
	this.leftShoulder = leftShoulder;
	this.rightShoulder = rightShoulder;
	this.Elbow = Elbow;

	// reverse left motors
	this.rightShoulder.setDirection(DcMotor.Direction.REVERSE);

	// create list of motors to make code cleaner
	this.motors = new DcMotorEx[]{this.leftShoulder, this.rightShoulder, this.Elbow};
  }
  
  public void DropArm() {
	for(DcMotorEx motor : this.motors) motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	for(DcMotorEx motor : this.motors) motor.setPower(-.5);
  }

  
  public void Initialize() {
	for(DcMotorEx motor : this.motors) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	for(DcMotorEx motor : this.motors) motor.setTargetPosition(0);
	for(DcMotorEx motor : this.motors) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }
  
  private void setVelocity(int velocity) {
	for(DcMotorEx motor : this.motors) motor.setVelocity(velocity);
  }
  
  public void Move(int position) {
	this.setVelocity(Config.ARM_VELOCITY);
	for(DcMotorEx motor : this.motors) motor.setTargetPosition(position);
  }
  

public void Move(int position, boolean waitForDone) {
	this.Move(position);
	
	if (waitForDone)
	  WaitForMotors();
  }

  public void Move(int position, boolean waitForDone, int tempVelocity) {
	this.setVelocity(tempVelocity);

	for(DcMotorEx motor : this.motors) motor.setTargetPosition(position);
	
	if (waitForDone)
	  WaitForMotors();
  }
  
  private void WaitForMotors() {
	while (this.leftShoulder.isBusy() || this.rightShoulder.isBusy() || this.Elbow.isBusy()) {}
	this.setVelocity(Config.ARM_VELOCITY);
  }
}