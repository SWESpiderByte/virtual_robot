package org.firstinspires.ftc.teamcode.spiderb2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "autonomous1 - *Nandita*", group = "SB^2 Two Wheel")
public class NanditaAuto extends LinearOpMode {

    // declare motors and servos
    DcMotorEx left = (DcMotorEx)hardwareMap.dcMotor.get("left_motor");
    DcMotorEx right = (DcMotorEx)hardwareMap.dcMotor.get("right_motor");
    Servo backServo = hardwareMap.servo.get("back_servo");

    public void runOpMode() {
        // Reverse the direction of the left motor so we can easily adjust negative and positive values
        left.setDirection(DcMotor.Direction.REVERSE);

        // Code pauses here until the "Start" button is pressed on the Driver Station phone or simulation
        waitForStart();

        // Move Forward for 1 second
        moveForward(1000);

        // Turn Left for 3 seconds
        turnLeft(3000);

        //Move Backward for 2 seconds
        moveBackward(2000);

        //move servo to position .7
        moveServo(0.7);
        //backServo.position(0.7)

        // Turn right for 3.5 seconds
        turnRight(3500);

        //Forwards for 2.5 seconds
        moveForward(2500);

        //move servo to position .5
        moveServo(0.5);
        //backServo.position(0.5)

        // STOP



    }//end of runOpMode


    public void printMessage(String message) {
        telemetry.addData(message,"");
        telemetry.update();
    }

    public void moveForward(int time){
        printMessage("Moving Forward");
        left.setPower(1);
        right.setPower(1);
        sleep(time);
        left.setPower(0);
        right.setPower(0);
    }

    //create other methods here
    public void turnLeft(int time){
        printMessage("Turning Left");
        left.setPower(-1);
        right.setPower(1);
        sleep(time);
        left.setPower(0);
        right.setPower(0);
    }
    public void moveBackward(int time){
        printMessage("Moving Backward");
        left.setPower(-1);
        right.setPower(-1);
        sleep(time);
        left.setPower(0);
        right.setPower(0);
    }
    public void moveServo(double position){
        printMessage("Moving Servo");
        backServo.setPosition(position);

    }
    public void turnRight(int time){
        printMessage("Turning Right");
        left.setPower(1);
        right.setPower(-1);
        sleep(time);
        left.setPower(0);
        right.setPower(0);
    }

}
