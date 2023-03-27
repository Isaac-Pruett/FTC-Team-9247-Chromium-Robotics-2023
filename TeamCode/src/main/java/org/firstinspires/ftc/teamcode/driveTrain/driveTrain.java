package org.firstinspires.ftc.teamcode.driveTrain;

import org.firstinspires.ftc.teamcode.PIDUtils.PID;
import org.firstinspires.ftc.teamcode.position_handler.pose;

public class driveTrain extends odometry_position_handler{

    public static pose robot = new pose();

    public double movement_threshold = 0;
    public double rotate_threshold = 0 * (Math.PI*2);

    public PID pidX = new PID();
    public PID pidY = new PID();
    public PID pidR = new PID();



    public void moveVectorRAD(double angle, double magnitude, double heading){
        pose target = new pose();

        target.x = Math.cos(angle) * magnitude;
        target.y = Math.sin(angle) * magnitude;
        target.theta = heading;

        if (((target.x - robot.x) > movement_threshold) || ((target.y - robot.y) > movement_threshold) || ((target.theta - robot.theta) > rotate_threshold)){
            while (((target.x - robot.x) > movement_threshold) || ((target.y - robot.y) > movement_threshold) || ((target.theta - robot.theta) > rotate_threshold)){
                updatePosition(robot);

                double Dist_y = target.y - robot.y;
                double Dist_x = target.x - robot.x;
                double angle_to_target = Math.atan2(Dist_y, Dist_x);

                double adjusted_angle = angle_to_target - (robot.theta-(Math.PI/2));

                double adjusted_normal_target_heading = (((target.theta - robot.theta) + 2*Math.PI) % (2*Math.PI));

                double rotate_power_angle = 0;

                if (adjusted_normal_target_heading == 0){
                    rotate_power_angle = robot.theta;

                }else if(adjusted_normal_target_heading > Math.PI){
                    rotate_power_angle = target.theta - robot.theta - 2*Math.PI;

                }else if(adjusted_normal_target_heading < Math.PI){
                    rotate_power_angle = target.theta;

                }else if(adjusted_normal_target_heading == Math.PI){
                    rotate_power_angle = Math.PI;

                }

                pidX.calcPID(Math.cos(adjusted_angle));
                pidY.calcPID(Math.sin(adjusted_angle));
                pidR.calcPID(rotate_power_angle/Math.PI);

                double x_power = pidX.pidVals();
                double y_power = pidY.pidVals();
                double r_power = pidR.pidVals();

                setMecanumPower(x_power, y_power, r_power);

            }
        }

        stopMotors();
    }

    public void moveVectorDEG(double angle, double magnitude, double heading){
        moveVectorRAD(Math.toRadians(angle), magnitude, Math.toRadians(heading));
    }

    public void moveCoordsRAD(double x, double y, double heading){
        double dist_x = x - robot.x;
        double dist_y = y - robot.y;
        double magnitude = Math.hypot(dist_x, dist_y);

        double angle = Math.atan2(dist_y, dist_x);

        moveVectorRAD(angle, magnitude, heading);
    }

    public void moveCoordsDEG(double x, double y, double heading){
        moveCoordsRAD(x, y, Math.toRadians(heading));
    }
}
