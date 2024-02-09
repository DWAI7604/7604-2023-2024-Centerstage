package org.firstinspires.ftc.teamcode;

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
