package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
public class AprilTagTesting extends RobotLinearOpMode {

    @Override
    public void runOpMode() {


        telemetry.update();

        initAprilTag();

        waitForStart();
        while (opModeIsActive()) {

            getAprilTags();

        }
    }
}
