package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.DriveMotors;
import org.firstinspires.ftc.teamcode.ObjectDetection;

import java.util.List;

public abstract class Auto extends LinearOpMode {
	public DriveMotors driveMotors;
	public Arm arm;
	public Intake intake;
	public ObjectDetection camera;
	
	/**
	 * Initialize classes used by autos
	 */
	public void Initialize() {
		driveMotors = new DriveMotors(
            hardwareMap.get(DcMotorEx.class, Config.FRONT_RIGHT_WHEEL_NAME),
		    hardwareMap.get(DcMotorEx.class, Config.FRONT_LEFT_WHEEL_NAME),
		    hardwareMap.get(DcMotorEx.class, Config.BACK_LEFT_WHEEL_NAME),
		    hardwareMap.get(DcMotorEx.class, Config.BACK_RIGHT_WHEEL_NAME)
        );

		arm = new Arm(
            hardwareMap.get(DcMotorEx.class, Config.SHOULDER_NAME),
		    hardwareMap.get(DcMotorEx.class, Config.SLIDE_NAME)
        );

		intake = new Intake(
            hardwareMap.get(Servo.class, Config.WRIST_NAME),
		    hardwareMap.get(Servo.class, Config.CLAW_LEFT_NAME),
		    hardwareMap.get(Servo.class, Config.CLAW_RIGHT_NAME)
        );

		camera = new ObjectDetection(
            hardwareMap.get(CameraName.class, Config.CAMERA_NAME)
        );
	}

	/**
	 * Set reliable initial configuration for robot motors
	 */
	public void MotorSetup() {
		intake.CloseClaws(0);
		intake.MoveWrist(0);
		arm.DropArm();
		sleep(1500);
		arm.Initialize();
	}
}
