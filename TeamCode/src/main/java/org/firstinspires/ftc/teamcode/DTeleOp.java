package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="DTeleOp", group = "Linear OpMode")
public class DTeleOp extends RobotLinearOpMode {

    //Declares the variables of the motors, make sure naming matches RobotLinearOpMode
    private DcMotor leftFrontDriveMotor = null;
    private DcMotor rightFrontDriveMotor = null;
    private DcMotor leftBackDriveMotor = null;
    private DcMotor rightBackDriveMotor = null;
    private DcMotor intakeMotor = null;
    double halfPower;



    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        //Uses RobotLinearOpMode method to declare all hardware properties
        declareHardwareProperties();

        leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {


            exponentialDrive();
            intakeControl();
            planeLauncher();
            hangerControl();

            if (isStopRequested()) {
                leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }




        }



    }

    public void standardDrive() {

        double leftFrontMotorPower;
        double rightFrontMotorPower;
        double leftBackMotorPower;
        double rightBackMotorPower;

        double axial = -gamepad1.right_stick_y; //forward & back
        double lateral = gamepad1.right_stick_x; //strafe
        double yaw = gamepad1.left_stick_x; //turning

        leftFrontMotorPower = axial - lateral + yaw;
        rightFrontMotorPower = axial + lateral - yaw;
        leftBackMotorPower = axial + lateral + yaw;
        rightBackMotorPower = axial - lateral - yaw;

        leftFrontDriveMotor.setPower(leftFrontMotorPower);
        rightFrontDriveMotor.setPower(rightFrontMotorPower);
        leftBackDriveMotor.setPower(leftBackMotorPower);
        rightBackDriveMotor.setPower(rightBackMotorPower);

        if (gamepad1.dpad_down) {
            leftFrontDriveMotor.setPower(leftFrontMotorPower/2);
            rightFrontDriveMotor.setPower(rightFrontMotorPower/2);
            leftBackDriveMotor.setPower(leftBackMotorPower/2);
            rightBackDriveMotor.setPower(rightBackMotorPower/2);
        }


    }

    public void exponentialDrive() {

        double leftFrontMotorPower;
        double rightFrontMotorPower;
        double leftBackMotorPower;
        double rightBackMotorPower;
        double slow = 0;
        double standard = 0;



        double axial = -Math.atan(gamepad1.left_stick_y); //forward and back power
        double lateral = Math.atan(gamepad1.left_stick_x); //left and right power
        double yaw = gamepad1.right_stick_x; //turning







        leftFrontMotorPower = axial - lateral + (.8*yaw);
        rightFrontMotorPower = axial - lateral - (.8*yaw);
        leftBackMotorPower = axial + lateral + (.8*yaw);
        rightBackMotorPower = axial + lateral - (.8*yaw);

            if (gamepad1.dpad_up) {
                leftFrontMotorPower = axial - lateral + (.8*yaw);
                rightFrontMotorPower = axial - lateral - (.8*yaw);
                leftBackMotorPower = axial + lateral + (.8*yaw);
                rightBackMotorPower = axial + lateral - (.8*yaw);
            } else if (gamepad1.dpad_down) {
                leftFrontMotorPower = .4 * (axial - lateral + (.8*yaw));
                rightFrontMotorPower = .4 * (axial - lateral - (.8*yaw));
                leftBackMotorPower = .4 * (axial + lateral + (.8*yaw));
                rightBackMotorPower = .4 * (axial + lateral - (.8*yaw));
            } else if (gamepad1.dpad_left && gamepad1.dpad_right) {
                leftFrontMotorPower = 4 * (axial - lateral + (.8*yaw));
                rightFrontMotorPower = 4 * (axial - lateral - (.8*yaw));
                leftBackMotorPower = 4 * (axial + lateral + (.8*yaw));
                rightBackMotorPower = 4 * (axial + lateral - (.8*yaw));
            }



        leftFrontDriveMotor.setPower(leftFrontMotorPower);
        rightFrontDriveMotor.setPower(rightFrontMotorPower);
        leftBackDriveMotor.setPower(leftBackMotorPower);
        rightBackDriveMotor.setPower(rightBackMotorPower);





    }

    public void juliansBullshit() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double controllerThreshold = 0.5;

        double joystickAngle = -Math.atan2(-gamepad1.right_stick_x, gamepad1.right_stick_y);
        //double joystickAngle = Math.atan2(gamepad1.right_stick_x, gamepad1.right_stick_y);




        // Send calculated power to wheels
        // This is strafe drive (left and right)

        if (gamepad1.left_stick_x < -controllerThreshold){ // turning
            leftFrontDriveMotor.setPower(gamepad1.left_stick_x * halfPower);
            leftBackDriveMotor.setPower(-gamepad1.left_stick_x * halfPower);
            rightFrontDriveMotor.setPower(-gamepad1.left_stick_x * halfPower);
            rightBackDriveMotor.setPower(gamepad1.left_stick_x * halfPower);
        } else if (gamepad1.left_stick_x > controllerThreshold) { // turning
            leftFrontDriveMotor.setPower(gamepad1.left_stick_x * halfPower);
            leftBackDriveMotor.setPower(-gamepad1.left_stick_x * halfPower);
            rightFrontDriveMotor.setPower(-gamepad1.left_stick_x * halfPower);
            rightBackDriveMotor.setPower(gamepad1.left_stick_x * halfPower);
        } else {
            double joystickMagnitude = Math.sqrt(gamepad1.right_stick_y * gamepad1.right_stick_y + gamepad1.right_stick_x * gamepad1.right_stick_x);
            double leftFront = Math.sin(joystickAngle + Math.PI / 4) * joystickMagnitude;//leftFront and rightBack are always the same so this makes it more efficient
            leftFrontDriveMotor.setPower(-leftFront * halfPower);
            rightBackDriveMotor.setPower(leftFront * halfPower);
            double rightFront = Math.sin(joystickAngle - Math.PI / 4) * joystickMagnitude;//leftFront and rightBack are always the same so this makes it more efficient
            rightFrontDriveMotor.setPower(rightFront * halfPower);
            leftBackDriveMotor.setPower(-rightFront * halfPower);
        }
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    public void intakeControl(){

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (gamepad1.right_bumper) {
            intakeMotor.setPower(-1);
            exponentialDrive();
        }
        while (gamepad1.left_bumper) {
            intakeMotor.setPower(1);
            exponentialDrive();
        }

        if (gamepad1.left_bumper != true && gamepad1.right_bumper != true) {
            intakeMotor.setPower(0);
            exponentialDrive();
        }

    }
    public void hPower(){

        if (gamepad1.right_trigger > 0.2){

            halfPower = 0.5;

        } else {

            halfPower = 1.0;

        }


        }

        public void planeLauncher() {

            Servo planeLauncher = null;

            planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");


            



            if (gamepad1.x) {
                planeLauncher.setPosition(0.15);


            }
        }

        public void hangerControl() {
        DcMotor Hanger = null;


        Hanger = hardwareMap.get(DcMotor.class, "Hanger");

        Hanger.setPower(gamepad1.right_trigger);
        Hanger.setPower(-gamepad1.left_trigger);




        }
    public void declareHardwareProperties() {


        rightFrontDriveMotor = hardwareMap.get(DcMotor.class, "rightFrontDriveMotor");
        leftFrontDriveMotor = hardwareMap.get(DcMotor.class, "leftFrontDriveMotor");
        rightBackDriveMotor = hardwareMap.get(DcMotor.class, "rightBackDriveMotor");
        leftBackDriveMotor = hardwareMap.get(DcMotor.class, "leftBackDriveMotor");

        rightFrontDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }



}


