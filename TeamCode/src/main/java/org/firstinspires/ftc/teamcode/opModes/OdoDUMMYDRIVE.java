package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.driveTrain.driveTrain;
import org.firstinspires.ftc.teamcode.position_handler.pose;

import java.util.List;


@TeleOp(name="Odometry DRIVE", group= "tests")
public class OdoDUMMYDRIVE extends OpMode {

    private DcMotor LeftOdo;
    private DcMotor RightOdo;
    private DcMotor CenterOdo;

    private DcMotorSimple MotorLeftFront;
    private DcMotorSimple MotorLeftBack;
    private DcMotorSimple MotorRightFront;
    private DcMotorSimple MotorRightBack;

    private ElapsedTime runtime = new ElapsedTime();
    driveTrain driveBase = new driveTrain();


    boolean moved = false;

    @Override
    public void init() {

        LeftOdo = hardwareMap.get(DcMotor.class, "LeftOdowheel");
        RightOdo = hardwareMap.get(DcMotor.class, "RightOdowheel");
        CenterOdo = hardwareMap.get(DcMotor.class, "CenterOdowheel");


        MotorLeftFront = hardwareMap.get(DcMotorSimple.class, "LF");
        MotorRightFront = hardwareMap.get(DcMotorSimple.class, "RF");
        MotorRightBack = hardwareMap.get(DcMotorSimple.class, "RB");
        MotorLeftBack = hardwareMap.get(DcMotorSimple.class, "LB");

        MotorLeftFront.setDirection(DcMotor.Direction.FORWARD); //set all directions, left side == FORWARD
        MotorRightFront.setDirection(DcMotor.Direction.REVERSE);
        MotorRightBack.setDirection(DcMotor.Direction.REVERSE);
        MotorLeftBack.setDirection(DcMotor.Direction.FORWARD);


        driveBase.LF = MotorLeftFront;
        driveBase.RF = MotorRightFront;
        driveBase.LB = MotorLeftBack;
        driveBase.RB = MotorRightBack;

        driveBase.centerOdo = CenterOdo;
        driveBase.rightOdo = RightOdo;
        driveBase.leftOdo = LeftOdo;


        driveBase.movement_threshold = 25; // reminder : in mm
        driveBase.rotate_threshold = Math.toRadians(10);


        driveBase.resetEncoders();


        driveBase.pidX.setPIDCoefficients(1, 0, 0);
        driveBase.pidY.setPIDCoefficients(1, 0, 0);
        driveBase.pidR.setPIDCoefficients(.5, 0, 0);


        driveBase.position.x = 0;
        driveBase.position.y = 0;
        driveBase.position.theta = 0;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        driveBase.driveSpeed = 0.5;



    }


    @Override
    public void loop() {

        pose testTargetPose = new pose();

        testTargetPose.x = 500.0;
        testTargetPose.y = 0.0;
        testTargetPose.theta = 0.0;

        driveBase.updatePosition(driveBase.position);

        telemetry.addData("THETA", Math.toDegrees(driveBase.position.theta));
        telemetry.addData("X", driveBase.position.x);
        telemetry.addData("Y", driveBase.position.y);

        telemetry.addData("angleTo", Math.toDegrees(driveBase.position.getAngleTo(testTargetPose)));
        telemetry.addData("distanceToX", Math.cos(driveBase.position.getAngleTo(testTargetPose))*driveBase.position.getDistanceTo(testTargetPose));
        telemetry.addData("distanceToY", Math.sin(driveBase.position.getAngleTo(testTargetPose))*driveBase.position.getDistanceTo(testTargetPose));
        telemetry.update();

        driveBase.setMecanumPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

    }

    @Override
    public void stop(){

    }
}
