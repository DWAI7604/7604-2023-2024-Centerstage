package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@Autonomous
public class AutoBlue extends RobotLinearOpMode{

    //Declaration of drive motors
    DcMotor rightFrontDriveMotor;
    DcMotor leftFrontDriveMotor;
    DcMotor rightBackDriveMotor;
    DcMotor leftBackDriveMotor;
    OpenCvWebcam webcam;
    SkystoneDeterminationExample.SkystoneDeterminationPipeline pipeline;
    SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition snapshotAnalysis = SkystoneDeterminationExample.SkystoneDeterminationPipeline.SkystonePosition.LEFT; // default
    @Override
    public void runOpMode() {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationExample.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {}
        });

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        sleep(2000);
        snapshotAnalysis = pipeline.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        switch (snapshotAnalysis)
        {
            case LEFT:
            {
                /* Your autonomous code */
                encoderDrive(.3,10,MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(.4,20,MOVEMENT_DIRECTION.FORWARD);
                encoderDrive(.3,20, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(.4, 80, MOVEMENT_DIRECTION.STRAFE_LEFT);
                telemetry.addData("Where", "Left");
                telemetry.update();
            }

            case RIGHT:
            {
                /* Your autonomous code */
                encoderDrive(.3,10,MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(.4,20,MOVEMENT_DIRECTION.FORWARD);
                encoderDrive(.3,20, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(.4, 80, MOVEMENT_DIRECTION.STRAFE_LEFT);
                telemetry.addData("Where", "Right");
                telemetry.update();
            }

            case CENTER:
            {
                /* Your autonomous code*/
                encoderDrive(.4,20,MOVEMENT_DIRECTION.FORWARD);
                encoderDrive(.3,20, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(.4, 90, MOVEMENT_DIRECTION.STRAFE_LEFT);
                telemetry.addData("Where", "Center");
                telemetry.update();
            }
        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive())
        {
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        declareHardwareProperties();

        waitForStart();
        //Robot should end up exactly where it started, in the same orientation, if there is a difference then one motor is incorrect and needs to be adjusted
        //Sleep commands implemented to give time to record distance and orientation after each movement


        leftBackDriveMotor.setPower(0);
        leftFrontDriveMotor.setPower(0);
        rightBackDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
       /* encoderDrive(1,24,MOVEMENT_DIRECTION.STRAFE_RIGHT);

        encoderDrive(1,24,MOVEMENT_DIRECTION.REVERSE);

        encoderDrive(1,24,MOVEMENT_DIRECTION.STRAFE_LEFT);*/

    }
}

