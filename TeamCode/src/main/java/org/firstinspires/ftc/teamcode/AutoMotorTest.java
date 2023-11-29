package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous
public class AutoMotorTest extends RobotLinearOpMode{

    //Declaration of drive motors
    DcMotor rightFrontDriveMotor;
    DcMotor leftFrontDriveMotor;
    DcMotor rightBackDriveMotor;
    DcMotor leftBackDriveMotor;

    public void runOpMode() {

        declareHardwareProperties();

        waitForStart();
        //Robot should end up exactly where it started, in the same orientation, if there is a difference then one motor is incorrect and needs to be adjusted
        //Sleep commands implemented to give time to record distance and orientation after each movement
        encoderDrive(1,2,MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(.5,90,MOVEMENT_DIRECTION.STRAFE_LEFT);

        leftBackDriveMotor.setPower(0);
        leftFrontDriveMotor.setPower(0);
        rightBackDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
       /* encoderDrive(1,24,MOVEMENT_DIRECTION.STRAFE_RIGHT);

        encoderDrive(1,24,MOVEMENT_DIRECTION.REVERSE);

        encoderDrive(1,24,MOVEMENT_DIRECTION.STRAFE_LEFT);*/

    }
}
