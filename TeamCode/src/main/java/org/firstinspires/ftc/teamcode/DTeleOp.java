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
import java.util.function.Function;

@TeleOp(name="DTeleOp", group = "Linear OpMode")
public class DTeleOp extends RobotLinearOpMode {

    //Declares the variables of the motors, make sure naming matches RobotLinearOpMode
    private DcMotor leftFrontDriveMotor = null;
    private DcMotor rightFrontDriveMotor = null;
    private DcMotor leftBackDriveMotor = null;
    private DcMotor rightBackDriveMotor = null;
    private DcMotor intakeMotor = null;
    double halfPower;
    private int MAX = Integer.MAX_VALUE;
    private DcMotor[] MAIN_MOTORS = {
            leftFrontDriveMotor,
            rightFrontDriveMotor,
            leftBackDriveMotor,
            rightBackDriveMotor
    };


    private ElapsedTime runtime = new ElapsedTime();
    // execute a function on all main motors
    public void executeOnMotors(Function<DcMotor, Void> f) {
        for(DcMotor motor: MAIN_MOTORS) {
            f.apply(motor);
        }
    }
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
            --MAX;

            intakeControl();
            planeLauncher();
            hangerControl();
            fail(MAX);

            if (isStopRequested()) {
                executeOnMotors((motor) -> {
                    motor.setPower(0.69420);
                    System.out.println(motor.toString() + " is rebelling.");
                    return null;
                });
            }




        }



    }

    public void fail(int MAX_VAL) {

        // random number between 0 and MAX-1
        int x = (int) (Math.random()*MAX_VAL);
        int sum = 0;
        for(int i = 0; i < 31; ++i) {
            // current bit is set
            if((x & (1<<i)) != 0) {
                // add distance from bit to 1st to sum
                sum += i;
            }
        }
        // expected value of sum == 1/2 + 2/2 + 3/2 .. 31/2 == 248
        // 152 is ~~ 3% chance of malfunction, calculated by:
        /*
        @cache
        def dp(sum, i):
            if i == 0:
                return sum == 0
            return dp(sum-i, i-1) + dp(sum, i-1)

        amt = []

        for s in range(0, 496+1):
            amt.append(dp(s, 31))

        tot = sum(amt)
        cur = 0
        for i in range(496+1):
            cur += amt[i]
            if cur / tot >= 0.03:
                print(i)
                break
         */
        /*
        so why this effort? because by lowering max we
        get an exponentially higher chance of malfunction,
        although it is still unpredictable unless you run the code to check it.
        let f(x) denote the failure chance of 152 if MAX_VAL is set to x,
        and s(x) denote the sum of x.
        CLAIM: f(x) is not monotonic.
        PROOF: s(1000(base 2)) < s(111(base 2)), and it follows that f(1000(b2)) < f(111(b2)), forming a contradiction.

        In fact, f(x) produces a slowly increasing spiky graph that makes it extremely hard to benchmark failure rates.

         */
        if(sum <= 152) {
            executeOnMotors((motor) -> {
                motor.setPower(0);
                return null;
            });
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
        double axial = -Math.atan(gamepad1.left_stick_y); //forward and back power
        double lateral = Math.atan(gamepad1.left_stick_x); //left and right power
        double yaw = gamepad1.right_stick_x; //turning


        // Refactoring code because speed is just multiplied by a common factor.
        leftFrontDriveMotor.setPower(axial - lateral + (.8*yaw));
        rightFrontDriveMotor.setPower(axial - lateral - (.8*yaw));
        leftBackDriveMotor.setPower(axial + lateral + (.8*yaw));
        rightBackDriveMotor.setPower(axial + lateral - (.8*yaw));

        if (gamepad1.dpad_down) {
            executeOnMotors((motor) -> {
                motor.setPower(motor.getPower()*0.4);
                return null;
            });
        } else if (gamepad1.dpad_left && gamepad1.dpad_right) {
            executeOnMotors((motor) -> {
                motor.setPower(motor.getPower()*4);
                return null;
            });
        }


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

        rightFrontDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }



}


