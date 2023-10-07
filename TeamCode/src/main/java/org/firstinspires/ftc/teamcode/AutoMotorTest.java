package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoMotorTest extends RobotLinearOpMode{

    //Declaration of drive motors
    DcMotor rightFrontDriveMotor;
    DcMotor leftFrontDriveMotor;
    DcMotor rightBackDriveMotor;
    DcMotor leftBackDriveMotor;

    public void runOpMode() {

        declareHardwareProperties();

        //Robot should end up exactly where it started, in the same orientation, if there is a difference then one motor is incorrect and needs to be adjusted
        //Sleep commands implemented to give time to record distance and orientation after each movement
        encoderDrive(1,24,MOVEMENT_DIRECTION.FORWARD);
        sleep(5000);
        encoderDrive(1,24,MOVEMENT_DIRECTION.STRAFE_RIGHT);
        sleep(5000);
        encoderDrive(1,24,MOVEMENT_DIRECTION.REVERSE);
        sleep(5000);
        encoderDrive(1,24,MOVEMENT_DIRECTION.STRAFE_LEFT);
        sleep(5000);
    }
}
