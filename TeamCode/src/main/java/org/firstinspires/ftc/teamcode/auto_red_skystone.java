package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Retro -Red Skystone ", group = "Concept")
//@Disabled
public class auto_red_skystone extends BaseLinearOpMode {

    // THIS DOESN'T WORK - CAN'T OVERRIDE init() in LinearOpMode, need to find way to set
    // BRIDGE_DIRECTION for RED vs BLUE autonomous programs
    /*
    @Override
    public void init(){
        //for RED side, bridge is located at a heading of 90deg from start position
        double BRIDGE_DIRECTION = 90.0;
    }

     */

    @Override
    public void runOpMode() {

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "front_sensor_color");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "front_sensor_color");

        // wait for the INIT button to be pressed
        waitForStart();

        initRetroRobot();
        servoIncrement(robot.brickClaw, 1, 0, 1);
        // encoderDrive(.40, 31.5, 0, 100);
        // servoIncrement(robot.brickClaw, -1, 0, 1);

        //  while (robot.brickClaw.getPosition() > 0) {
        //  }
        //  encoderDrive(.40, -3, 0, 100);
        telemetry.update();

        float colorvalue = (sensorColor.red() * sensorColor.green()) / (sensorColor.blue() * sensorColor.blue());

        if (colorvalue >= 3) {
            encoderShift(.25, -9.75);
            colorvalue = (sensorColor.red() * sensorColor.green()) / (sensorColor.blue() * sensorColor.blue());
            if (colorvalue >= 3) {
                encoderShift(.25, -9.75);
            }
        }
        encoderDrive(.40, 2.5, 0, 100);
        servoIncrement(robot.brickClaw, -1, 0, 1);
        sleep(2000);
        telemetry.addData("colorvalue ", colorvalue);

    }

}