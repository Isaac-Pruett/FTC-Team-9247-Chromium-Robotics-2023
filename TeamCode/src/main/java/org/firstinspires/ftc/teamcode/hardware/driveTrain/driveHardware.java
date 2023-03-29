package org.firstinspires.ftc.teamcode.hardware.driveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// I recommend initializing this as "robot" or "driveBase", one must initialize all names before opModes start. this is basically just an interface.
public class driveHardware {
    public DcMotor leftOdo;
    public DcMotor rightOdo;
    public DcMotor centerOdo;

    public double driveSpeed = 1;
    public DcMotorSimple LF;
    public DcMotorSimple RF;
    public DcMotorSimple RB;
    public DcMotorSimple LB;

    public void setMecanumPower(double x, double y, double r){

        double denominator = 1;
        double avg = Math.abs(x) + Math.abs(y) + Math.abs(r);
        if (avg > 1){
            denominator = avg;
        }

        LF.setPower(((y + x + r) / denominator) * driveSpeed);
        LB.setPower(((y - x + r) / denominator) * driveSpeed);
        RF.setPower(((y - x - r) / denominator) * driveSpeed);
        RB.setPower(((y + x - r) / denominator) * driveSpeed);
    }

    public void stopMotors(){
        setMecanumPower(0,0,0);
    }
}
