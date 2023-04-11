package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.driveTrain.driveTrain;
import org.firstinspires.ftc.teamcode.position_handler.pose;

import java.util.List;

@TeleOp(name="Odometry TEST", group= "tests")
public class OdoTestsLinear extends LinearOpMode {

    private DcMotor LeftOdo;
    private DcMotor RightOdo;
    private DcMotor CenterOdo;

    private DcMotorSimple MotorLeftFront;
    private DcMotorSimple MotorLeftBack;
    private DcMotorSimple MotorRightFront;
    private DcMotorSimple MotorRightBack;

    private ElapsedTime runtime = new ElapsedTime();
    driveTrain driveBase = new driveTrain();




    @Override
    public void runOpMode() throws InterruptedException {
        initialize_robot();
        waitForStart();
        if(opModeIsActive()){
            //driveBase.moveCoordsDEG(1500, 0, 0);
            while (opModeIsActive()){

                driveBase.updatePosition(driveBase.position);

                telemetry.addData("THETA", Math.toDegrees(driveBase.position.theta));
                telemetry.addData("X", driveBase.position.x);
                telemetry.addData("Y", driveBase.position.y);
                telemetry.update();
            }
        }
    }

    public void initialize_robot() {

        driveBase.currentOpMode = this;

        LeftOdo = hardwareMap.get(DcMotor.class, "LeftOdowheel");
        RightOdo = hardwareMap.get(DcMotor.class, "RightOdowheel");
        CenterOdo = hardwareMap.get(DcMotor.class, "CenterOdowheel");

        LeftOdo.setDirection(DcMotorSimple.Direction.REVERSE);
        RightOdo.setDirection(DcMotorSimple.Direction.FORWARD);
        CenterOdo.setDirection(DcMotorSimple.Direction.FORWARD);

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


        driveBase.movement_threshold = 10; // reminder : in mm
        driveBase.rotate_threshold = Math.toRadians(5);


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
}
