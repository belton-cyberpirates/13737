package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

@TeleOp(name = "xDriveOutreach")
public class xDriveOutreach extends LinearOpMode {

  private DcMotorEx m1;
  private DcMotorEx m2;
  private DcMotorEx m3;
  private DcMotorEx m4;
  private DcMotorEx arm2;
  private DcMotorEx claw;
  private AnalogInput armpot;
  private DcMotorEx arm1;
  private AnalogInput clawpot;
  private double clawComp = 0;

  private void setArmPower(double power) {
    arm1.setPower(power);
    arm2.setPower(power);
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int stayClosed;
    double compensation = 1;
    boolean boostEnabled = false;

    m1 = hardwareMap.get(DcMotorEx.class, "m1");
    m2 = hardwareMap.get(DcMotorEx.class, "m2");
    m3 = hardwareMap.get(DcMotorEx.class, "m3");
    m4 = hardwareMap.get(DcMotorEx.class, "m4");
    arm2 = hardwareMap.get(DcMotorEx.class, "arm2");
    claw = hardwareMap.get(DcMotorEx.class, "claw");
    armpot = hardwareMap.get(AnalogInput.class, "armpot");
    arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
    clawpot = hardwareMap.get(AnalogInput.class, "clawpot");

    waitForStart();
    // Put initialization blocks here.
    m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    arm2.setDirection(DcMotorSimple.Direction.REVERSE);
    claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    stayClosed = 0;
    double speed = 750;
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        telemetry.addData("clawComp", clawComp);
        double rightTriggerCorrection = gamepad1.right_trigger > 0.2 ? gamepad1.right_trigger -0.2 : 0; // if right trigger, speed up
        double leftTriggerCorrection  = gamepad1.left_trigger > 0.2 ? 0.2 - (gamepad1.left_trigger * .7) : 0; // if left trigger, slow down
        double armCorrection = armpot.getVoltage() < 0.715 ? - 0.3 : 0; // if arm up, slow down

        double SpeedupVal = 1 + boostEnabled ? (rightTriggerCorrection + leftTriggerCorrection + armCorrection) : 0;

        telemetry.addData("gamepad info", gamepad2);

        m2.setVelocity((int)(((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * SpeedupVal) * speed * compensation));
        m3.setVelocity((int)(((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * SpeedupVal) * speed));
        m4.setVelocity((int)(((-gamepad1.left_stick_y + -gamepad1.left_stick_x + gamepad1.right_stick_x) * SpeedupVal) * speed));
        m1.setVelocity((int)(((gamepad1.left_stick_y + -gamepad1.left_stick_x + gamepad1.right_stick_x) * SpeedupVal) * speed * compensation));
        if (gamepad2.right_stick_y < 0) {
          setArmPower(-(gamepad2.right_stick_y * .9));
        } else if (gamepad2.right_stick_y > 0) {
          setArmPower(-(gamepad2.right_stick_y * .5));
        } else if (gamepad2.left_bumper) {
          setArmPower(-0.5);
        } else if (gamepad2.right_bumper) {
          if (armpot.getVoltage() < 0.5) {
            setArmPower(0.3)
          } else {
            setArmPower(0.9);
          }
        } else if (armpot.getVoltage() > 1.45) {
          setArmPower(0)
        } else {
          setArmPower(0.1)
        }
        if (clawpot.getVoltage() < 1.35 + clawComp) {
          claw.setPower(0.3);
        } else if (gamepad2.right_trigger > 0) {
          claw.setPower(0.9);
          stayClosed = 1;
        } else if (clawpot.getVoltage() > 1.45 + clawComp && gamepad2.left_trigger > 0) {
          claw.setPower(-0.5);
          stayClosed = 0;
        } else if (stayClosed == 1) {
          claw.setPower(0.5);
        } else {
          claw.setPower(0);
        }
        
        telemetry.addData("boost", SpeedupVal);
        telemetry.update();
      }
    }
  }
}
