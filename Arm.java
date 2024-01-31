package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Direction;


public class Arm {
	Telemetry telemetry;
	private DcMotorEx Shoulder;
	private DcMotorEx Slide;
	private DcMotorEx[] motors;

	public Arm() {
		this.Shoulder =  hardwareMap.get(DcMotorEx.class, Config.SHOULDER_NAME);
		this.Slide = hardwareMap.get(DcMotorEx.class, Config.SLIDE_NAME);

		// create list of motors to make code cleaner
		this.motors = new DcMotorEx[]{this.Shoulder, this.Slide};
  	}
  
	public void DropArm() {
		for(DcMotorEx motor : this.motors) motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.Shoulder.setPower(.3);
	}

  
	public void Initialize() {
		for(DcMotorEx motor : this.motors) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		for(DcMotorEx motor : this.motors) motor.setTargetPosition(0);
		for(DcMotorEx motor : this.motors) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}
  
	private void setVelocity(int shoulderVelocity, int slideVelocity) {
		this.Shoulder.setVelocity(shoulderVelocity);
		this.Slide.setVelocity(slideVelocity);
	}
  
	public void MoveShoulder(int position) {
		this.setVelocity(Config.ARM_VELOCITY, Config.SLIDE_VELOCITY);
		this.Shoulder.setTargetPosition(position);
	}

	public void MoveShoulder(int position, boolean waitForDone) {
		this.MoveShoulder(position);
		
		if (waitForDone) WaitForMotors();
	}

	public void MoveShoulder(int position, boolean waitForDone, int tempArmVelocity, int tempSlideVelocity) {
		this.setVelocity(tempArmVelocity, Config.SLIDE_VELOCITY);

		this.Shoulder.setTargetPosition(position);
		
		if (waitForDone) WaitForMotors();
	}


	public void MoveSlide(int position) {
		this.setVelocity(Config.ARM_VELOCITY, Config.SLIDE_VELOCITY);
		this.Slide.setTargetPosition(position);
	}

	public void MoveSlide(int position, boolean waitForDone) {
		this.MoveSlide(position);
		
		if (waitForDone) WaitForMotors();
	}

	public void MoveSlide(int position, boolean waitForDone, int tempSlideVelocity) {
		this.setVelocity(Config.ARM_VELOCITY, tempSlideVelocity);

		this.Slide.setTargetPosition(position);
		
		if (waitForDone) WaitForMotors();
	}
	
	private void WaitForMotors() {
		while (this.Shoulder.isBusy() || this.Slide.isBusy()) {}
		
		this.setVelocity(Config.ARM_VELOCITY, Config.SLIDE_VELOCITY);
	}
}