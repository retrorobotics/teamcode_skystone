/* Copyright (c) 2017 FIRST. All rights reserved.

 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Thread.sleep;

/**
* This file provides basic Telop driving for the RobotRuckus 2018/219 season.
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcontroller.external.samples.RR_HardwarePushbot;

import static java.lang.Thread.sleep;

/**
* This file provides basic Telop driving for the RobotRuckus 2018/219 season.

*/

@TeleOp(name="RR Teleop", group="Pushbot")
//@Disabled
public class Teleop_Skystone extends OpMode {

   /* Declare OpMode members. */
   RR_HardwarePushbot robot       = new RR_HardwarePushbot(); // use the class created to define a Pushbot's hardware

    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    //static final double     DRIVE_GEAR_REDUCTION    = 0.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.85 ;     // For figuring circumference --was 3.5
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    PIDFCoefficients pidOrig;
    PIDFCoefficients pidNew;
    PIDFCoefficients pidModified;

   /*
    * Code to run ONCE when the driver hits INIT
    */

   @Override
   public void init() {
       /* Initialize the hardware variables.
        * The init() method of the hardware class does all the work here
        */
       robot.init(hardwareMap);

       /*
        * prep for adjustments to built-in motor PID controller to allow for smoother lift operation
        */
       DcMotorEx liftMotor;
       double NEW_P = 6.0;
       double NEW_I = 1.75;
       double NEW_D = 2.0;
       double NEW_F = 0.0;

       liftMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "brick_Lift");

       // get the PID coefficients for the RUN_USING_ENCODER  modes.
       pidOrig = liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

       // change coefficients using methods included with DcMotorEx class.
       pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
       liftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

       // re-read coefficients and verify change.
       pidModified = liftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

       // Send telemetry message to signify robot waiting;
       telemetry.addData("Say", "Hello Retro");    //
   }

   /*
    * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    */
   @Override
   public void init_loop() {

       robot.brickLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       while (robot.brickLift.getCurrentPosition() != 0) {
           //do nothing
       }
       robot.brickLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       // display info to user.
       telemetry.addData("Runtime", "%.03f", getRuntime());
       telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.0f, %.0f",
               pidOrig.p, pidOrig.i, pidOrig.d, pidOrig.f);
       telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.0f",
               pidModified.p, pidModified.i, pidModified.d, pidOrig.f);
       telemetry.update();


       //test wheel orientation
       /*
       telemetry.addData("Left Front", "GO");
       robot.leftFrontDrive.setPower(0.5);
       try {
           sleep(1000);
       } catch (InterruptedException e) {
           e.printStackTrace();}
       robot.leftFrontDrive.setPower(0);

       telemetry.addData("Right Front", "GO");
       robot.rightFrontDrive.setPower(0.5);
           try {
               sleep(1000);
           } catch (InterruptedException e) {
               e.printStackTrace();}
       robot.rightFrontDrive.setPower(0);

       telemetry.addData("Left Back", "GO");
       robot.leftBackDrive.setPower(0.5);
               try {
                   sleep(1000);
               } catch (InterruptedException e) {
                   e.printStackTrace();}
       robot.leftBackDrive.setPower(0);

       telemetry.addData("Right Back", "GO");
       robot.rightBackDrive.setPower(0.5);
                   try {
                       sleep(1000);
                   } catch (InterruptedException e) {
                       e.printStackTrace();}
       robot.rightBackDrive.setPower(0);
*/

   }

   /*
    * Code to run ONCE when the driver hits PLAY
    */
   @Override

   public void start() {}

       boolean yLastPass = false;
       boolean drive_mode = false; // FALSE is holonomic drive, TRUE is tank drive

       double leftFront = 0;
       double rightFront = 0;
       double leftBack = 0;
       double rightBack = 0;


   /*
    * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    */
   @Override
   public void loop() {
       double maxServoPosition = 1;
       double minServoPosition = 0;

       int armBaseMovement = 2;
       int armElbowMovement = 2;
       //   double speedAdj = .90;

       //option for slow speed driving
       boolean speedslow = gamepad1.left_bumper || gamepad1.right_bumper;
       //  speedAdj = speedslow?0.2:0.6;

       float gamepad1LeftY = gamepad1.left_stick_y;
       float gamepad1LeftX = gamepad1.left_stick_x;
       float gamepad1RightX = gamepad1.right_stick_x;
       float gamepad1RightY = gamepad1.right_stick_y;

       float gamepad2LeftY = (float) scaleInput(gamepad2.left_stick_y);
       float gamepad2RightY = (float) scaleInput(gamepad2.right_stick_y);
       float gamepad2LeftX = (float) scaleInput(gamepad2.left_stick_x);
       float gamepad2RightX = (float) scaleInput(gamepad2.right_stick_x);

       gamepad1LeftY = (float) scaleInput(gamepad1LeftY);
       gamepad1LeftX = (float) scaleInput(gamepad1LeftX);
       gamepad1RightX = (float) scaleInput(gamepad1RightX);

       //toggle drive mode
       boolean aPressed = gamepad1.y;
       if (aPressed && !yLastPass) {
           drive_mode = !drive_mode;
       }
       yLastPass = aPressed;

       if (drive_mode) {
           // og code - tank driving
           leftFront = (gamepad1LeftY);
           rightFront = (-gamepad1RightY);
           leftBack = (gamepad1LeftY);
           rightBack = (-gamepad1RightY);
       } else {

           /*following code runs too slow
           final double x = Math.pow(gamepad1LeftX, 3.0);
           final double y = Math.pow(gamepad1LeftY, 3.0);

           final double rotation = Math.pow(gamepad1RightX, 3.0);
           final double direction = Math.atan2(x, y);
           final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

           leftFront = -(speed * Math.sin(direction + Math.PI / 4.0) + rotation);
           rightFront = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
           leftBack = -(speed * Math.cos(direction + Math.PI / 4.0) + rotation);
           rightBack = speed * Math.sin(direction + Math.PI / 4.0) - rotation;
*/
           // holonomic drive
           leftFront = -(gamepad1LeftY + gamepad1LeftX * 0.35 - (-gamepad1RightX * .7));
           rightFront = gamepad1LeftY - gamepad1LeftX * 0.35 + (-gamepad1RightX * .7);
           leftBack = -(gamepad1LeftY - gamepad1LeftX * 0.35 - (-gamepad1RightX * .7));
           rightBack = gamepad1LeftY + gamepad1LeftX * 0.35 + (-gamepad1RightX * .7);

       }

       leftFront = Range.clip(leftFront, -1, 1);
       rightFront = Range.clip(rightFront, -1, 1);
       leftBack = Range.clip(leftBack, -1, 1);
       rightBack = Range.clip(rightBack, -1, 1);

       // Send telemetry message to signify robot waiting;
       telemetry.addData("L_front", leftFront);
       telemetry.addData("R_front", rightFront);
       telemetry.addData("L_back", leftBack);
       telemetry.addData("R_back", rightBack);
       telemetry.addData("Y buttom", gamepad1.y);
       telemetry.addData("drive mode", drive_mode);

       //set SLOW and FAST speeds here
       double slowspeedset = 0.2;
       double fastspeedset = 0.45;

       //implement speed based on whether a bumper is pressed
       if(speedslow) {
           robot.leftFrontDrive.setPower(leftFront * slowspeedset);
           robot.rightFrontDrive.setPower(rightFront * slowspeedset);
           robot.leftBackDrive.setPower(leftBack * slowspeedset);
           robot.rightBackDrive.setPower(rightBack * slowspeedset);
       }else{robot.leftFrontDrive.setPower(leftFront * fastspeedset);
           robot.rightFrontDrive.setPower(rightFront * fastspeedset);
           robot.leftBackDrive.setPower(leftBack * fastspeedset);
           robot.rightBackDrive.setPower(rightBack * fastspeedset);}

       //
       //code for cascade lift
       //
       int position = robot.brickLift.getCurrentPosition();
//        int position2 = robot.bricklift2.getCurrentPosition();

       //code for running to a set position
       /*
       robot.brickLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       robot.brickLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       int upPosition = 200;
       int downPosition = 0;


           if (gamepad2.left_bumper) {
               robot.brickLift.setTargetPosition(upPosition);
           } else if (gamepad2.right_bumper) {
               robot.brickLift.setTargetPosition(downPosition);
           }

           if (robot.brickLift.isBusy()) {
               robot.brickLift.setPower(0.6);
           } else {
               robot.brickLift.setPower(0);
           }
*/

       //OLD CODE FOR RUNNING LIFT

       //motor power for up/down motion
       double power_up = 0.7;
       double power_down = -0.1;
       double power_hold = 0;

       //left bumper is DOWN
       if (gamepad2.left_bumper) {
         robot.brickLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           robot.brickLift.setPower(power_down);
       }
       //right bumper is UP
       else if (gamepad2.right_bumper) {
           robot.brickLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           robot.brickLift.setPower(power_up);
       }
       //hold position
       else {
           robot.brickLift.setTargetPosition(robot.brickLift.getCurrentPosition());
           robot.brickLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           robot.brickLift.setPower(power_hold);
       }
       telemetry.addData("lift power", robot.brickLift.getPower());
       telemetry.addData("lift position", robot.brickLift.getCurrentPosition());
       /*
       if (gamepad2LeftY > .01){
           robot.bricklift2.setTargetPosition(position2+100);
           robot.brickLift.setTargetPosition(position+100);
           robot.brickLift.setPower(gamepad2RightY);
           robot.bricklift2.setPower(gamepad2RightY);
           robot.brickLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           robot.bricklift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       }
       else if (gamepad2LeftY < -.01)
           robot.bricklift2.setTargetPosition(position2-100);
           robot.brickLift.setTargetPosition(position-100);
           robot.brickLift.setPower(gamepad2RightY);
           robot.bricklift2.setPower(gamepad2RightY);
       robot.brickLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       robot.bricklift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

*/

       /* OLD CODE FOR MOVING CLAW
       if (gamepad2.b) {
           servoIncrement(robot.brickClaw, .2);
       } else if (gamepad2.a) {
           servoIncrement(robot.brickClaw, -.2);
       } else if (gamepad2.x) {

       }
*/
             double maxClawServoPosition = .36;

       if (gamepad2RightY > 0) {
           servoIncrement(robot.Foundation_Pull_1, .02);
           if (gamepad2RightY < 0) {
               servoIncrement(robot.Foundation_Pull_1, -.02);
           }

           if (gamepad2LeftY > 0) {
               armBasePosition += armBaseMovement;
           } else if (gamepad2LeftY < 0) {
               armBasePosition -= armBaseMovement;
           }

           if (gamepad2RightX > 0) {

           }

       }

       if (gamepad2LeftX > 0) {
           armElbowPosition -= armElbowMovement;
       } else if (gamepad2LeftX < 0) {
           armElbowPosition += armElbowMovement;
       }

       if (gamepad2.y) {
           servoIncrement(robot.brickClaw, 1, 0, 1);
           //  servoIncrement( robot.Foundation_Pull_2, .02, minServoPosition, maxServoPosition);
       }

       if (gamepad2.a) {
           servoIncrement(robot.brickClaw, -1, 0, 1);
           // servoIncrement( robot.Foundation_Pull_2, -.02, minServoPosition, maxServoPosition);
       }

   }

       /*
       }

       if ( gamepad2.y ){
           servoIncrement( robot.armAClaw, -.02, minServoPosition, maxClawServoPosition);
      */



   @Override
   public void stop() {
   }

   void isArmLimitReached(){
       //return !robot.armBaseLimit.getState();
   }


   double scaleInput(double dVal)  {
       double[] scaleArray = { 0.0, 0.03, 0.05, 0.07, 0.10, 0.13, 0.15, 0.21,
               0.26, 0.31, 0.38, 0.45, 0.54, 0.57, 0.63, 0.71, 0.76 };

       // get the corresponding index for the scaleInput array.
       int index = (int) (dVal * 16.0);

       // index should be positive.
       if (index < 0) {
           index = -index;
       }

       // index cannot exceed size of array minus 1.
       if (index > 16) {
           index = 16;
       }

       // get value from the array.
       double dScale = 0.0;
       if (dVal < 0) {
           dScale = -scaleArray[index];
       } else {
           dScale = scaleArray[index];
       }

       // return scaled value.
       return dScale;
   }

   /**
    * Move a servo by the selected increment.
    * @param servo: servo to move
    * @param increment: amount to move it
    */
   private void servoIncrement(Servo servo, double increment){
       servoIncrement( servo, increment, 0, 1);
   }

   /**
    * Move a servo by the selected increment within the provided bounds
    * @param servo: serve to move
    * @param increment: amount to move it
    * @param minServoPosition: minimum position the servo can be moved to
    * @param maxServoPosition: maximum position the servo can be moved to
    */
   private void servoIncrement(Servo servo, double increment, double minServoPosition, double maxServoPosition){
       boolean allowMove = false;
       if ( increment > 0 && servo.getPosition() < maxServoPosition){
           allowMove = true;
       } else if ( increment < 0 && servo.getPosition() > minServoPosition ){
           allowMove = true;
       }

       if ( allowMove ) {
           servo.setPosition(servo.getPosition() + increment);
           try {
               sleep(50);
           } catch (InterruptedException e) {
               e.printStackTrace();

           }
       }
   }
}
