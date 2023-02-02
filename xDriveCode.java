package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

@TeleOp(name = "xDriveCode")
public class xDriveCode extends LinearOpMode {

  private DcMotor m1;
  private DcMotor m2;
  private DcMotor m3;
  private DcMotor m4;
  private DcMotor arm2;
  private DcMotor claw;
  private AndroidTextToSpeech androidTextToSpeech;
  private AnalogInput armpot;
  private DcMotor arm1;
  private AnalogInput clawpot;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int SlowdownRate;
    int stayClosed;
    int slowness;
    double compensation = 1;

    m1 = hardwareMap.get(DcMotor.class, "m1");
    m2 = hardwareMap.get(DcMotor.class, "m2");
    m3 = hardwareMap.get(DcMotor.class, "m3");
    m4 = hardwareMap.get(DcMotor.class, "m4");
    arm2 = hardwareMap.get(DcMotor.class, "arm2");
    claw = hardwareMap.get(DcMotor.class, "claw");
    androidTextToSpeech = new AndroidTextToSpeech();
    armpot = hardwareMap.get(AnalogInput.class, "armpot");
    arm1 = hardwareMap.get(DcMotor.class, "arm1");
    clawpot = hardwareMap.get(AnalogInput.class, "clawpot");

    waitForStart();
    // Put initialization blocks here.
    SlowdownRate = 2;
    m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    arm2.setDirection(DcMotorSimple.Direction.REVERSE);
    claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    stayClosed = 0;
    slowness = 2;
    androidTextToSpeech.initialize();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        double rightTriggerCorrection = gamepad1.right_trigger > 0.2 ? gamepad1.right_trigger -0.2 : 0; // if right trigger, speed up
        double leftTriggerCorrection  = gamepad1.left_trigger > 0.2 ? 02 - (gamepad1.left_trigger * .7) : 0; // if left trigger, slow down
        double armCorrection = armpot.getVoltage() < 0.715 ? - 0.3 : 0; // if arm up, slow down

        double SpeedupVal = 1 + rightTriggerCorrection + leftTriggerCorrection + armCorrection;

        m2.setPower(((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * SpeedupVal) / slowness * compensation);
        m3.setPower(((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * SpeedupVal) / slowness);
        m4.setPower(((-gamepad1.left_stick_y + -gamepad1.left_stick_x + gamepad1.right_stick_x) * SpeedupVal) / slowness);
        m1.setPower(((gamepad1.left_stick_y + -gamepad1.left_stick_x + gamepad1.right_stick_x) * SpeedupVal) / slowness * compensation);
        if (gamepad2.right_stick_y < 0) {
          arm1.setPower(-(gamepad2.right_stick_y / 2));
          arm2.setPower(-(gamepad2.right_stick_y / 2));
        } else if (gamepad2.right_stick_y > 0) {
          arm1.setPower(-(gamepad2.right_stick_y / 3));
          arm2.setPower(-(gamepad2.right_stick_y / 3));
        } else if (gamepad2.left_bumper) {
          arm1.setPower(-0.5);
          arm2.setPower(-0.5);
        } else if (gamepad2.right_bumper) {
          if (armpot.getVoltage() < 0.5) {
            arm1.setPower(0.3);
            arm2.setPower(0.3);
          } else {
            arm1.setPower(0.9);
            arm2.setPower(0.9);
          }
        } else if (armpot.getVoltage() > 1.45) {
          arm1.setPower(0);
          arm2.setPower(0);
        } else {
          arm1.setPower(0.1);
          arm2.setPower(0.1);
        }
        if (clawpot.getVoltage() < 1.15) {
          claw.setPower(0.3);
        } else if (gamepad2.right_trigger > 0) {
          claw.setPower(0.9);
          stayClosed = 1;
        } else if (clawpot.getVoltage() > 1.25 && gamepad2.left_trigger > 0) {
          claw.setPower(-0.5);
          stayClosed = 0;
        } else if (stayClosed == 1) {
          claw.setPower(0.5);
        } else {
          claw.setPower(0);
        }
        // telemetry.addData("arm1", arm1.getPower());
        // telemetry.addData("m1", m1.getCurrentPower());
        // telemetry.addData("m2", m2.getCurrentPower());
        // telemetry.addData("m3", m3.getCurrentPower());
        // telemetry.addData("m4", m4.getCurrentPower());
        // telemetry.addData("ArmPot", armpot.getVoltage());
        // telemetry.addData("ClawPot", clawpot.getVoltage());
        telemetry.update();
      }
      // Put run blocks here.
    }

    androidTextToSpeech.close();
  }
}
