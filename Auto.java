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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
import org.firstinspires.ftc.teamcode.Heading;


import java.util.List;

public abstract class Auto extends LinearOpMode {
	protected DriveMotors driveMotors;
	protected Arm arm;
	protected Intake intake;
	protected ObjectDetection camera;
	protected IMU imu;
	protected Heading heading;
	
	/**
	 * Initialize classes used by autos
	 */
	protected void Initialize() {
		driveMotors = new DriveMotors(this);
		arm = new Arm(this);
		intake = new Intake(this);
		camera = new ObjectDetection(this);

		imu = hardwareMap.get(IMU.class, "imu");
		imu.resetYaw();
	}

	/**
	 * Set reliable initial configuration for robot motors
	 */
	protected void MotorSetup() {
		intake.CloseClaws(0);
		intake.MoveWrist(0);
		arm.DropArm();
		sleep(1500);
		arm.Initialize();
	}

	protected void saveHeading() {
		double _heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
		this.heading.setHeading(_heading);
		/*
		String strHeading = Double.toString(heading);

		File file = new File("heading");
		FileWriter fw = new FileWriter(file, false);
		fw.flush();

		fw.write(strHeading);

		writer.close();*/
	}
}
