package org.firstinspires.ftc.teamcode.spiderb2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TankDrive")
public class TankDrive extends LinearOpMode {

    DcMotor left = null;
    DcMotor right = null;

    public void runOpMode() {
        left = hardwareMap.dcMotor.get("left_motor");
        right = hardwareMap.dcMotor.get("right_motor");
        left.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            left.setPower(-gamepad1.left_stick_y);
            right.setPower(-gamepad1.right_stick_y);
        }

    }
}
