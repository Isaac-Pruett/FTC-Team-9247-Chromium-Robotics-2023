package org.firstinspires.ftc.teamcode.PIDUtils;

import com.qualcomm.robotcore.util.Range;

public class PID {
    double p = 0;
    double i = 0;
    double d = 0;
    double f = 0;

    double k_p, k_i, k_d;

    private long deltaTimeMs = 1;
    private long lastTimeMs = System.currentTimeMillis();
    private double lastError = 0.0;

    public void setPIDCoefficients(double k_p, double k_i, double k_d){
        this.k_p = k_p;
        this.k_d = k_d;
        this.k_i = k_i;
    }

    public double calcPID(double target, double feedback){
        double error = target - feedback;
        return calcPID(error);
    }

    public double calcPID(double error){
        if (deltaTimeMs < 1) {
            deltaTimeMs = 1;
        }

        p = k_p * error;
        i += k_i * (error * (double)(deltaTimeMs));
        d = k_d * (error - lastError) / (double)(deltaTimeMs);

        //i = Range.clip(i, -0.1, 0.1); // i think the following is better????

        i = Range.clip(i, -1, 1);

        lastError = error;

        deltaTimeMs = System.currentTimeMillis() - lastTimeMs;
        lastTimeMs = System.currentTimeMillis();

        return pidVals();
    }

    public double pidVals(){
        return p + i + d + f;
    }
}
