package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Direction;


public class Arm extends LinearOpMode {
	private DcMotorEx Shoulder;
	private DcMotorEx Slide;
	private DcMotorEx[] motors;

	public Arm(shoulder, slide) {
		this.Shoulder = shoulder;
		this.Slide = slide;

		// create list of motors to make code cleaner
		motors = new DcMotorEx[]{this.Shoulder, this.Slide};
  	}
  	
  
  
	public void DropArm() {
		for(DcMotorEx motor : motors) motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		Shoulder.setPower(.3);
	}

  
	public void Initialize() {
		for(DcMotorEx motor : motors) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		for(DcMotorEx motor : motors) motor.setTargetPosition(0);
		for(DcMotorEx motor : motors) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}
  
	private void setVelocity(int shoulderVelocity, int slideVelocity) {
		Shoulder.setVelocity(shoulderVelocity);
		Slide.setVelocity(slideVelocity);
	}
  
	public void MoveShoulder(int position) {
		setVelocity(Config.ARM_VELOCITY, Config.SLIDE_VELOCITY);
		Shoulder.setTargetPosition(position);
	}

	public void MoveShoulder(int position, boolean waitForDone) {
		MoveShoulder(position);
		
		if (waitForDone) WaitForMotors();
	}

	public void MoveShoulder(int position, boolean waitForDone, int tempArmVelocity, int tempSlideVelocity) {
		setVelocity(tempArmVelocity, Config.SLIDE_VELOCITY);

		Shoulder.setTargetPosition(position);
		
		if (waitForDone) WaitForMotors();
	}


	public void MoveSlide(int position) {
		setVelocity(Config.ARM_VELOCITY, Config.SLIDE_VELOCITY);
		Slide.setTargetPosition(position);
	}

	public void MoveSlide(int position, boolean waitForDone) {
		MoveSlide(position);
		
		if (waitForDone) WaitForMotors();
	}

	public void MoveSlide(int position, boolean waitForDone, int tempSlideVelocity) {
		setVelocity(Config.ARM_VELOCITY, tempSlideVelocity);

		Slide.setTargetPosition(position);
		
		if (waitForDone) WaitForMotors();
	}
	
	private void WaitForMotors() {
		while (Shoulder.isBusy() || Slide.isBusy()) {}
		
		setVelocity(Config.ARM_VELOCITY, Config.SLIDE_VELOCITY);
	}
}