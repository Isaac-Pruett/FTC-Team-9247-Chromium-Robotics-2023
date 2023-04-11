package org.firstinspires.ftc.teamcode.hardware.driveTrain;


public class driveConstants extends driveHardware{

    public static double TRACK_WIDTH = 233.4; // in mm
    public static double FORWARD_OFFSET = -9.5; // in mm

    //public static double WHEEL_DIAMETER = 200; // in mm.

    public static double ODOMETRY_COUNTS_PER_REVOLUTION = 8192.0; //revcoder CPR
    public static double ODOMETRY_WHEEL_DIAMETER = 35.0; // in mm.

    public static double ODOMETRY_COUNTS_PER_MILLIMETER = ODOMETRY_COUNTS_PER_REVOLUTION / (ODOMETRY_WHEEL_DIAMETER*Math.PI);


    public static double TRACK_WIDTH_INCHES = 9.1875;

    public static double FORWARD_OFFSET_INCHES = -0.375;

    public static double ODOMETRY_WHEEL_DIAMETER_INCHES = (35.0/25.4);

    public static double ODOMETRY_COUNTS_PER_INCH = ODOMETRY_COUNTS_PER_REVOLUTION / (ODOMETRY_WHEEL_DIAMETER_INCHES * Math.PI);


}
