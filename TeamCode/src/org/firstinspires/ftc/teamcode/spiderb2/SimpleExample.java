package org.firstinspires.ftc.teamcode.spiderb2;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Example Autonomous OpMode. Demonstrates driving forward
 */
@Autonomous(name = "simple drive forward", group = "SB^2 Two Wheel")
public class SimpleExample extends LinearOpMode {

    // Define class variables here, string names in "" need to match hardware configuration
    private final DcMotorEx left = (DcMotorEx)hardwareMap.dcMotor.get("left_motor");
    private final DcMotorEx right = (DcMotorEx)hardwareMap.dcMotor.get("right_motor");

    public void runOpMode() {
        // Reverse the direction of the left motor so + power values = forward, - power values = backward
        left.setDirection(DcMotor.Direction.REVERSE);

        // Telemetry = data printed out on driver station phone, SUPER useful for seeing what the code is doing
        telemetry.addData("Press Start to Continue","");
        telemetry.update();

        // Code pauses here until the "Start" button is pressed on the Driver Station phone or simulation
        waitForStart();

        // Move forward full speed - left and right wheels full power forward
        left.setPower(1);
        right.setPower(1);
        sleep(1000);
    }
}
