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
import org.firstinspires.ftc.teamcode.Config;
import java.util.List;

public class Intake {
    private LinearOpMode auto;
    private Servo wrist;
    private Servo clawLeft;
    private Servo clawRight;
    

    public Intake(LinearOpMode auto) {
        this.auto = auto;
        this.wrist = auto.hardwareMap.get(Servo.class, Config.WRIST_NAME);
        this.clawLeft = auto.hardwareMap.get(Servo.class, Config.CLAW_LEFT_NAME);
        this.clawRight = auto.hardwareMap.get(Servo.class, Config.CLAW_RIGHT_NAME);
    }


    public void MoveWrist(double position) {
        MoveWrist(position, 0);
    }
    

    public void MoveWrist(double position, int wait) {
        wrist.setPosition(position);
        auto.sleep(wait);
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
        auto.sleep(wait);
    }
  
  
    private void CloseClaws(boolean closeLeft, boolean closeRight, int wait) {
        if (closeLeft) clawLeft.setPosition(Config.CLAW_LEFT_CLOSE);
        if (closeRight) clawRight.setPosition(Config.CLAW_RIGHT_CLOSE);
        auto.sleep(wait);
    }
}