package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class ColorSensorTest extends RobotLinearOpMode{
public void runOpMode() {
    waitForStart();
    while (opModeIsActive()) {
        colorSensor();

        telemetry.addData("Color Sensor", colorSensor());
        telemetry.update();
    }
}
}
