package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Direction;


public class Arm {
	private LinearOpMode auto;
	private DcMotorEx shoulder;
	private DcMotorEx slide;
	private DcMotorEx[] motors;


	public Arm(LinearOpMode auto) {
		this.auto = auto;
		this.shoulder = auto.hardwareMap.get(DcMotorEx.class, Config.SHOULDER_NAME);
		this.slide = auto.hardwareMap.get(DcMotorEx.class, Config.SLIDE_NAME);

		// create list of motors to make code cleaner
		motors = new DcMotorEx[]{ this.shoulder, this.slide };
  	}

  
	public void DropArm() {
		for(DcMotorEx motor : motors) motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		shoulder.setPower(.3);
	}

  
	public void Initialize() {
		for(DcMotorEx motor : motors) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		for(DcMotorEx motor : motors) motor.setTargetPosition(0);
		for(DcMotorEx motor : motors) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}
  
	private void setVelocity(int shoulderVelocity, int slideVelocity) {
		shoulder.setVelocity(shoulderVelocity);
		slide.setVelocity(slideVelocity);
	}
  
	public void MoveShoulder(int position) {
		setVelocity(Config.ARM_VELOCITY, Config.SLIDE_VELOCITY);
		shoulder.setTargetPosition(position);
	}

	public void MoveShoulder(int position, boolean waitForDone) {
		MoveShoulder(position);
		
		if (waitForDone) WaitForMotors();
	}

	public void MoveShoulder(int position, boolean waitForDone, int tempArmVelocity, int tempSlideVelocity) {
		setVelocity(tempArmVelocity, Config.SLIDE_VELOCITY);

		shoulder.setTargetPosition(position);
		
		if (waitForDone) WaitForMotors();
	}


	public void MoveSlide(int position) {
		setVelocity(Config.ARM_VELOCITY, Config.SLIDE_VELOCITY);
		slide.setTargetPosition(position);
	}

	public void MoveSlide(int position, boolean waitForDone) {
		MoveSlide(position);
		
		if (waitForDone) WaitForMotors();
	}

	public void MoveSlide(int position, boolean waitForDone, int tempSlideVelocity) {
		setVelocity(Config.ARM_VELOCITY, tempSlideVelocity);

		slide.setTargetPosition(position);
		
		if (waitForDone) WaitForMotors();
	}
	
	private void WaitForMotors() {
		while (shoulder.isBusy() || slide.isBusy()) {}
		
		setVelocity(Config.ARM_VELOCITY, Config.SLIDE_VELOCITY);
	}
}