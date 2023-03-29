package org.firstinspires.ftc.teamcode.hardware.driveTrain;


public class driveConstants extends driveHardware{

    public static double TRACK_WIDTH = 400; // in mm
    public static double FORWARD_OFFSET = 30; // in mm

    //public static double WHEEL_DIAMETER = 200; // in mm.

    public static double ODOMETRY_COUNTS_PER_REVOLUTION = 8192; //revcoder CPR
    public static double ODOMETRY_WHEEL_DIAMETER = 35; // in mm.

    public static double ODOMETRY_COUNTS_PER_MILLIMETER = ODOMETRY_COUNTS_PER_REVOLUTION / (ODOMETRY_WHEEL_DIAMETER*Math.PI);
}
