package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="DTeleOp", group = "Linear OpMode")
public class DTeleOp extends RobotLinearOpMode {

    //Declares the variables of the motors, make sure naming matches RobotLinearOpMode
    private DcMotor leftFrontDriveMotor = null;
    private DcMotor rightFrontDriveMotor = null;
    private DcMotor leftBackDriveMotor = null;
    private DcMotor rightBackDriveMotor = null;
    double halfPower;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        //Uses RobotLinearOpMode method to declare all hardware properties
        declareHardwareProperties();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            standardDrive();



        }

    }

    public void standardDrive() {

        double leftFrontMotorPower;
        double rightFrontMotorPower;
        double leftBackMotorPower;
        double rightBackMotorPower;

        double axial = gamepad1.right_stick_y;
        double lateral = gamepad1.right_stick_x;
        double yaw = gamepad1.left_stick_x;

        leftFrontMotorPower = axial + lateral + yaw;
        rightFrontMotorPower = axial - lateral - yaw;
        leftBackMotorPower = axial - lateral + yaw;
        rightBackMotorPower = axial + lateral - yaw;

        leftFrontDriveMotor.setPower(leftFrontMotorPower);
        rightFrontDriveMotor.setPower(rightFrontMotorPower);
        leftBackDriveMotor.setPower(leftBackMotorPower);
        rightBackDriveMotor.setPower(rightBackMotorPower);
    }

    public void exponentialDrive() {

        double leftFrontMotorPower;
        double rightFrontMotorPower;
        double leftBackMotorPower;
        double rightBackMotorPower;


        double axial = gamepad1.right_stick_y;
        double lateral = gamepad1.right_stick_x;
        double yaw = gamepad1.left_stick_x;


        if(axial < 0) {
             axial = -(Math.pow(axial, 2));
        } else if(axial > 0) {
            axial = Math.pow(axial, 2);
        }

        if(lateral < 0) {
            lateral = -(Math.pow(lateral, 2));
        } else if(lateral > 0) {
            lateral = Math.pow(lateral, 2);
        }

        if(yaw < 0) {
            yaw = -(Math.pow(yaw, 2));
        } else if(yaw > 0) {
            yaw = Math.pow(yaw, 2);
        }

        leftFrontMotorPower = axial + lateral + yaw;
        rightFrontMotorPower = axial - lateral - yaw;
        leftBackMotorPower = axial - lateral + yaw;
        rightBackMotorPower = axial + lateral - yaw;

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
    public void hPower(){

        if (gamepad1.right_trigger > 0.2){

            halfPower = 0.5;

        } else {

            halfPower = 1.0;

        }

    }

}


