package org.firstinspires.ftc.teamcode.spiderb2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Example Autonomous OpMode. Demonstrates open-loop control of motors & servos
 *
 */
@Autonomous(name = "two wheel autonomous example", group = "SB^2 Two Wheel")
public class AutonomousExample extends LinearOpMode {

    // Define class variables here - will accessible in all AutonomousExample functions

    // How does wiring correspond to code?
    // The hardware configuration configured on the Robot Controller needs to match
    // the names used in the HardwareMap to access motors, servos, sensors, etc.
    DcMotorEx left = (DcMotorEx)hardwareMap.dcMotor.get("left_motor");
    DcMotorEx right = (DcMotorEx)hardwareMap.dcMotor.get("right_motor");
    Servo backServo = hardwareMap.servo.get("back_servo");

    public void runOpMode() {
        // Reverse the direction of the left motor so + power values = forward, - power values = backward
        left.setDirection(DcMotor.Direction.REVERSE);

        // Telemetry = data printed out on driver station phone,
        // SUPER useful for seeing what the code is doing
        telemetry.addData("Press Start to Continue","");
        telemetry.update();

        // Code pauses here until the "Start" button is pressed on the Driver Station phone or simulation
        waitForStart();

        // 1) Move forward full speed
        // Left and right wheels full power forward
        printMessage("Moving forward");
        left.setPower(1);
        right.setPower(1);
        sleep(1000);

        // 2) Turn left while moving forward
        // Left wheel half power forward, right wheel full power forward
        printMessage("Turning left");
        left.setPower(0.5);
        right.setPower(1.0);
        sleep(3000);

        // 3) Turn right in place
        // Left wheel forward, right wheel backward
        printMessage("Turning right in place");
        left.setPower(0.5);
        right.setPower(-0.5);
        sleep(1500);

        // 4) Drive backwards
        printMessage("Driving backwards");
        left.setPower(-0.5);
        right.setPower(-0.5);
        sleep(1000);

        // 4) Stop
        // Turn off both motors
        printMessage("Stop");
        left.setPower(0);
        right.setPower(0);
        sleep(2000);

        // 5) Sweep servo
        printMessage("Moving servo");
        double position = 0;
        // Gradually move servo from position 0.0 to 1.0
        for (int i = 0; i <= 10; i++) {
            printMessage("Servo position = " + Double.toString(position));
            // Move servo to new position
            backServo.setPosition(position);
            sleep(1000);
            // Increment position by 0.1 (equivalent to position = position + 0.1)
            // What position is being set for step i? position = 0.1 * i;
            // e.g. i = 3, position = 0.1 + 0.1 + 0.1 = 0.1 * 3 = 0.1 * i
            position += 0.1;
        }
    }

    /**
     * This is a helper method to send a telemetry message to
     * the Driver Station phone for debugging. For convenience,
     * update telemetry is called after updating the telemetry message.
     *
     * @param message   string message to display in telemetry
     */
    public void printMessage(String message) {
        telemetry.addData(message,"");
        telemetry.update();
    }

}
