package org.firstinspires.ftc.teamcode;

public class Tread {
    private DcMotor front;
    private DcMotor back;


    public Tread(DcMotor front, DcMotor back) {
        this.front = front;
        this.back = back;
    }


    public boolean IsBusy() {
        return this.front.isBusy() || this.back.isBusy();
    }


    public void Reset() {
        this.front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void SetPower(double power) {
        this.front.setPower(power);
        this.back.setPower(power);
    }


    public void SetToRunPosition() {
        this.front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void SetTargetPosition(int distance) {
        this.front.setTargetPosition(distance);
        this.back.setTargetPosition(distance);
    }