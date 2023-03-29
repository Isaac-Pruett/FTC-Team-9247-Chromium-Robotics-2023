package org.firstinspires.ftc.teamcode.hardware.driveTrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.position_handler.pose;

public class odometry_position_handler extends driveConstants{

    private int PrevCenterPos = 0;
    private int PrevRightPos = 0;
    private int PrevLeftPos = 0;

    public void resetEncoders(){
        centerOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        centerOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void updatePosition(pose robot){

        int center_pos = centerOdo.getCurrentPosition();
        int right_pos = rightOdo.getCurrentPosition();
        int left_pos = leftOdo.getCurrentPosition();

        int deltaC = (center_pos - PrevCenterPos);
        int deltaR = (right_pos - PrevRightPos);
        int deltaL = (left_pos - PrevLeftPos);

        double phi = (deltaL-deltaR) / ODOMETRY_COUNTS_PER_MILLIMETER / TRACK_WIDTH; // phi == delta theta.

        double deltaMiddle = (double)((deltaL + deltaR) / 2) / ODOMETRY_COUNTS_PER_MILLIMETER;
        double deltaPerp = (deltaC - (FORWARD_OFFSET * phi)) / ODOMETRY_COUNTS_PER_MILLIMETER;

        double deltaX = deltaMiddle * Math.cos(robot.theta) - deltaPerp * Math.sin(robot.theta);
        double deltaY = deltaMiddle * Math.sin(robot.theta) - deltaPerp * Math.cos(robot.theta);

        robot.x += deltaX;
        robot.y += deltaY;
        robot.theta += phi;

        robot.theta = (robot.theta % 2 * Math.PI);

        //Heading = Heading % (2*Math.PI);

        PrevCenterPos = center_pos;
        PrevLeftPos = left_pos;
        PrevRightPos = right_pos;
    }

}
