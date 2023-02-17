package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Config;


public class Tread {
    private DcMotorEx front;
    private DcMotorEx back;
    private DcMotorEx encoder;
    private int direction;
    private int targetPosition;

    // PID loop values
    private double error_prior = 0;
    private double integral_prior = 0;
    private double bias = 0;


    public Tread(DcMotorEx front, DcMotorEx back, DcMotorEx encoder) {
        this.front = front;
        this.back = back;
        this.encoder = encoder;
        
        this.direction = 1;
    }

    public int GetCurrentPosition() {
        return this.encoder.getCurrentPosition();
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

    private boolean isBusy() {
        // theres probably a shorter way to do this
        // returns false if higher than target and direction is 1, or if lower than target and direction is -1
        return !(((encoder.getCurrentPosition() > targetPosition) && (direction == 1)) ||
                 ((encoder.getCurrentPosition() < targetPosition) && (direction == -1)));
    }

    public Tread(DcMotorEx front, DcMotorEx back, DcMotorEx encoder, boolean reverse) {
        this.front = front;
        this.back = back;
        this.encoder = encoder;

        this.direction = reverse ? -1 : 1;
    }

    public void SetPower(double power) {
        this.front.setPower(power);
        this.back.setPower(power);
    }
  }