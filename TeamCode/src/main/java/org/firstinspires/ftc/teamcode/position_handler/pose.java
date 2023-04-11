package org.firstinspires.ftc.teamcode.position_handler;

public class pose {
    public double x = 0;
    public double y = 0;

    public double theta = 0;

    public double f;
    public double g;
    public double h;

    
    public double getDistanceTo(pose pos){
        double dist_x = pos.x - this.x;
        double dist_y = pos.y - this.y;
        return Math.hypot(dist_x, dist_y);
    }

    public double getAngleTo(pose pos){
        double dist_x = pos.x - this.x;
        double dist_y = pos.y - this.y;
        double angle = Math.atan2(dist_y, dist_x);

        //(angle-(Math.PI));
        return angle;

    }

}
