package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config;
import java.util.List;

public class Intake extends LinearOpMode {
	private Servo wrist;
	private Servo clawLeft;
	private Servo clawRight;
	
	
	public Intake(wrist, clawLeft, clawRight) {
		this.wrist = wrist;
		this.clawLeft = clawLeft;
		this.clawRight = clawRight;
	}
	

	public void MoveWrist(double position) {
		MoveWrist(position, 0);
	}
	

	public void MoveWrist(double position, int wait) {
		wrist.setPosition(position);
		sleep(wait);
	}
	
	
	public void OpenClaws(int wait) {
		OpenClaws(true, true, wait);
	}


	public void OpenLeft(int wait) {
		OpenClaws(true, false, wait);
	}


	public void OpenRight(int wait) {
		OpenClaws(false, true, wait);
	}


	public void CloseLeft(int wait) {
		CloseClaws(true, false, wait);
	}


	public void CloseRight(int wait) {
		CloseClaws(false, true, wait);
	}
	
	
	public void CloseClaws(int wait) {
		CloseClaws(true, true, wait);
	}
	
  
	private void OpenClaws(boolean openLeft, boolean openRight, int wait) {
		if (openLeft) clawLeft.setPosition(Config.CLAW_LEFT_OPEN);
		if (openRight) clawRight.setPosition(Config.CLAW_RIGHT_OPEN);
		sleep(wait);
	}
  
  
	private void CloseClaws(boolean closeLeft, boolean closeRight, int wait) {
		if (closeLeft) clawLeft.setPosition(Config.CLAW_LEFT_CLOSE);
		if (closeRight) clawRight.setPosition(Config.CLAW_RIGHT_CLOSE);
		sleep(wait);
	}
}