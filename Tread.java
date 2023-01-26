package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Config;


public class Tread {
    private DcMotor front;
    private DcMotor back;
    private DcMotor encoder; // not a DcMotor, but will work for now
    private int direction;

    // PID loop values
    private double error_prior = 0;
    private double integral_prior = 0;
    private double bias = 0;


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


    public int GetCurrentPosition() {
        return this.encoder.getCurrentPosition();
    }


    public boolean IsBusy() {
        int pos = this.encoder.getCurrentPosition();
        return pos != this.targetPosition;
    }

    public void Move(int distance) {
        this.SetTargetPosition(this.direction * distance);
    }


    public void Update() {
        double error = this.targetPosition - this.encoder.getCurrentPosition();
        double integral = integral_prior + error;
        double derivative = (error - error_prior);
        double power = Config.KP*error + Config.KI*integral + Config.KD*derivative + bias;

        this.front.setPower(this.direction * power);
        this.back.setPower(this.direction * power);

        this.error_prior = error;
        this.integral_prior = integral;
    }
  }