package org.firstinspires.ftc.teamcode.hardware.driveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PIDUtils.PID;
import org.firstinspires.ftc.teamcode.position_handler.pose;

public class driveTrain extends odometry_position_handler{

    public pose position = new pose();

    public double movement_threshold = 0;
    public double rotate_threshold = 0 * (Math.PI*2);

    public PID pidX = new PID();
    public PID pidY = new PID();
    public PID pidR = new PID();

    public LinearOpMode currentOpMode;

    public void moveVectorRAD(double angle, double magnitude, double heading){ // for use with a linear op mode
        pose target = new pose();

        target.x = Math.cos(angle) * magnitude;
        target.y = Math.sin(angle) * magnitude;
        target.theta = heading;

        boolean within_thresholds = ((target.x - position.x) > movement_threshold) || ((target.y - position.y) > movement_threshold) || ((target.theta - position.theta) > rotate_threshold);

        if (within_thresholds){
            while (within_thresholds && (!currentOpMode.isStopRequested())){

                updatePosition(position);
                within_thresholds = ((target.x - position.x) > movement_threshold) || ((target.y - position.y) > movement_threshold) || ((target.theta - position.theta) > rotate_threshold);

                double Dist_y = target.y - position.y;
                double Dist_x = target.x - position.x;
                double angle_to_target = Math.atan2(Dist_y, Dist_x);

                double adjusted_angle = angle_to_target - (position.theta-(Math.PI/2));

                double adjusted_normal_target_heading = (((target.theta - position.theta) + 2*Math.PI) % (2*Math.PI));

                double rotate_power_angle = 0;

                if (adjusted_normal_target_heading == 0){
                    rotate_power_angle = position.theta;

                }else if(adjusted_normal_target_heading > Math.PI){
                    rotate_power_angle = target.theta - position.theta - 2*Math.PI;

                }else if(adjusted_normal_target_heading < Math.PI){
                    rotate_power_angle = target.theta;

                }else if(adjusted_normal_target_heading == Math.PI){
                    rotate_power_angle = Math.PI;

                }


                double x_power = pidX.calcPID(Math.cos(adjusted_angle));
                double y_power = pidY.calcPID(Math.sin(adjusted_angle));
                double r_power = pidR.calcPID((rotate_power_angle/Math.PI));

                setMecanumPower(x_power, y_power, r_power);

            }
        }

        stopMotors();
    }

    public void moveVectorDEG(double angle, double magnitude, double heading){
        moveVectorRAD(Math.toRadians(angle), magnitude, Math.toRadians(heading));
    }

    public void moveCoordsRAD(double x, double y, double heading){
        double dist_x = x - position.x;
        double dist_y = y - position.y;
        double magnitude = Math.hypot(dist_x, dist_y);

        double angle = Math.atan2(dist_y, dist_x);

        moveVectorRAD(angle, magnitude, heading);
    }

    public void moveCoordsDEG(double x, double y, double heading){
        moveCoordsRAD(x, y, Math.toRadians(heading));
    }
}
