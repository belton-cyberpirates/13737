package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Config;


public class Tread {
    private DcMotor front;
    private DcMotor back;
    private int direction;


    public Tread(DcMotor front, DcMotor back) {
        this.direction = 1;
        this.front = front;
        this.back = back;
    }


    public Tread(DcMotor front, DcMotor back, boolean reverse) {
        this.front = front;
        this.back = back;

        this.direction = reverse ? -1 : 1;
    }


    private void Init() {
        this.Reset();
        this.SetTargetPosition(0);
        this.SetToRunPosition();
    }


    private void Reset() {
        this.front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    private void SetTargetPosition(int distance) {
        this.front.setTargetPosition(this.direction * distance);
        this.back.setTargetPosition(this.direction * distance);
    }


    private void SetToRunPosition() {
        this.front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public int GetCurrentPosition() {
        return this.front.getCurrentPosition();
    }


    public boolean IsBusy() {
        return this.front.isBusy() || this.back.isBusy();
    }

    public void Move(int distance) {
        this.Init();
        this.SetPower(Config.MIN_SPEED);
        this.SetTargetPosition(distance);
    }


    public void SetPower(double power) {
        this.front.setPower(power);
        this.back.setPower(power);
    }
  }