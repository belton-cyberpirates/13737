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
     * Set reliable initial configuration for robot motors
     */
    private void motorSetup() {
	    CloseClaw();
	    MoveWrist(0);
        arm.DropArm();
        sleep(1500);
        arm.Initialize();
    }
    

    public void Initialize() {
        driveMotors = new DriveMotors();
        arm = new Arm();
        intake = new Intake();
        camera = new ObjectDetection();

        camera.Initialize();
    }
}
