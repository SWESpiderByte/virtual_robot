package org.firstinspires.ftc.teamcode.spiderb2;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.*;


/**
 * Example Autonomous OpMode. Demonstrates driving forward
 */
@Autonomous(name = "wall following", group = "SB^2 Mechanum Wheel")
public class LineFollowing extends LinearOpMode {

    // Define class variables here, string names in "" need to match hardware configuration
    private final DcMotorEx front_left = (DcMotorEx)hardwareMap.dcMotor.get("front_left_motor");
    private final DcMotorEx front_right = (DcMotorEx)hardwareMap.dcMotor.get("front_right_motor");
    private final DcMotorEx back_left = (DcMotorEx)hardwareMap.dcMotor.get("back_left_motor");
    private final DcMotorEx back_right = (DcMotorEx)hardwareMap.dcMotor.get("back_right_motor");
    private final ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
    private final DistanceSensor front_distance = hardwareMap.get(DistanceSensor.class, "front_distance");
    private final DistanceSensor back_distance = hardwareMap.get(DistanceSensor.class, "back_distance");
    private final DistanceSensor left_distance = hardwareMap.get(DistanceSensor.class, "left_distance");
    private final DistanceSensor right_distance = hardwareMap.get(DistanceSensor.class, "right_distance");
    private final BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");


    private final double goal_distance = 35.0;
    private double goal_heading = -90;

    public void runOpMode() {
        imu.initialize(new BNO055IMU.Parameters());
        // Reverse the direction of the left motor so + power values = forward, - power values = backward
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);

        // Telemetry = data printed out on driver station phone, SUPER useful for seeing what the code is doing
        telemetry.addData("Press Start to Continue","");
        telemetry.update();

        // Code pauses here until the "Start" button is pressed on the Driver Station phone or simulation
        waitForStart();

        while (opModeIsActive())
        {
            if (front_distance.getDistance(DistanceUnit.CM) < goal_distance)
            {
                // Turn
                // Get heading
                double heading = 0;
                Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                heading = orientation.firstAngle * 180.0 / Math.PI;
                telemetry.addData("heading", heading);
                telemetry.update();
//
                while (heading > goal_heading)
                {
                    orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                    telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);
                    telemetry.update();
                    heading = orientation.firstAngle * 180.0 / Math.PI;
                    front_left.setPower(1);
                    back_left.setPower(1);
                    front_right.setPower(-1);
                    back_right.setPower(-1);
                }
                goal_heading -= 89;
            }
            else {
                // Get distance from left wall
                double current_distance = left_distance.getDistance(DistanceUnit.CM);

                //double distance_error = current_distance - goal_distance;
                if (current_distance > goal_distance) {
                    // go left
                    telemetry.addData("going left", current_distance);
                    front_left.setPower(0);
                    back_left.setPower(1);
                    front_right.setPower(1);
                    back_right.setPower(0);
                } else if (current_distance < goal_distance) {
                    // go right
                    telemetry.addData("going right", current_distance);

                    front_left.setPower(1);
                    back_left.setPower(0);
                    front_right.setPower(0);
                    back_right.setPower(1);
                } else {
                    // go straight
                    telemetry.addData("going straight", current_distance);

                    front_left.setPower(1);
                    back_left.setPower(1);
                    front_right.setPower(1);
                    back_right.setPower(1);
                }
                telemetry.update();
                sleep(50);
            }
        }
    }

    public String getColor()
    {
        String color = "unknown";
        double red = colorSensor.red();
        double green = colorSensor.green();
        double blue = colorSensor.blue();

        if (green > 200)
        {
            color = "yellow";
        }
        else if (red > 200)
        {
            color = "red";
        }
        else if (blue > 100)
        {
            color = "blue";
        }

        telemetry.addData("red", colorSensor.red());
        telemetry.addData("green", colorSensor.green());
        telemetry.addData("blue", colorSensor.blue());
        telemetry.addData("color", color);
        telemetry.update();

        return color;

    }
}
