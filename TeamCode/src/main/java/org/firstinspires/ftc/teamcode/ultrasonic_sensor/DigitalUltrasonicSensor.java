package org.firstinspires.ftc.teamcode.ultrasonic_sensor;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DigitalUltrasonicSensor {

    private final double MAX_DISTANCE_POSSIBLE = 144 * 25.4; //inches to mm
    private final double MILLIMETERS_PER_SECOND = 343000;

    DigitalChannel echoPin;
    DigitalChannel triggerPin;



    public void initialize_sensor(DigitalChannel echo, DigitalChannel trigger){
        this.echoPin = echo;
        this.triggerPin = trigger;
    }

    public double getDistance(){
        echoPin.setMode(DigitalChannel.Mode.INPUT);
        triggerPin.setMode(DigitalChannel.Mode.OUTPUT);
        double distance = 0;
        ElapsedTime runtime = new ElapsedTime(0);

        triggerPin.setState(true);
        while (runtime.seconds() < 0.00001){ //sleep for trigger to actually trigger
        }
        triggerPin.setState(false);
        runtime.reset();

        while(!echoPin.getState()){
            runtime.reset();
        }

        while (distance < MAX_DISTANCE_POSSIBLE && (echoPin.getState())){
            distance = runtime.seconds() * MILLIMETERS_PER_SECOND / 2; //(distanceToObject = 1/2(s/t)
        }

        return distance;
    }
}
