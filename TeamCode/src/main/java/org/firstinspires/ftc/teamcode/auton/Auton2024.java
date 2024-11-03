package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.DeviceNames;

/**
 * Works for Mecanum
 * <p>
 * Ensure all 4 motors are of the same configuration (SKU 5203-2402-0019)
 * <p>
 * Ensure all motors have encoders connected
 * <p>
 * Ensure wheels are configured in X shape
 * <p>
 * Measure width and change if needed
 */
@Autonomous(name = "auton_2024", group = "furious_frog_2024")
public class Auton2024 extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    //28 * 20 / (2ppi * 4.125)
    //chassis width. Normally used for turn. Ignore if only forward/strafing motion is needed
    Double width = 16.0; //inches

    //See encoder resolution https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/?srsltid=AfmBOoruBKEaSkcIwfFpy1t6rZOtf4fzb0cfTNLmTL8Pr3JVCrC2-ln3
    //this is encoder pulse per rotation on the motor. The value 537.7 shown on web-site is after multiplying by gear-ratio
    Integer cpr = 28; //counts per rotation
    //Mentioned in "Gear Ratio" section of above link
    Double gearRatio = 19.2;
    //diameter of wheel in inches
    Double diameter = 3.77;
    Double cpi = (cpr * gearRatio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))

    Double bias = 0.8;//this will be measured after calibration. This is to counteract friction etc.
    Double strafeBias = 0.9;//change to adjust only strafing movement
    //
    Double conversion = cpi * bias;
    Boolean exit = false;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.dcMotor.get(DeviceNames.MOTOR_FRONT_LEFT);
        frontRight = hardwareMap.dcMotor.get(DeviceNames.MOTOR_FRONT_RIGHT);
        backLeft = hardwareMap.dcMotor.get(DeviceNames.MOTOR_BACK_LEFT);
        backRight = hardwareMap.dcMotor.get(DeviceNames.MOTOR_BACK_RIGHT);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);//If your robot goes backward, switch this from right to left
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);//If your robot goes backward, switch this from right to left

        waitForStart();

        moveToPosition(20, .2);//Don't change this line, unless you want to calibrate with different speeds
    }

    /*
  This function's purpose is simply to drive forward or backward.
  To drive backward, simply make the inches input negative.
   */
    public void moveToPosition(double inches, double speed) {
        //
        int move = (int) (Math.round(inches * conversion));
        //
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        //
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);
        //
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            if (exit) {
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
                return;
            }
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        return;
    }

    /*
   This function uses the encoders to strafe left or right.
   Negative input for inches results in left strafing.
    */
    public void strafeToPosition(double inches, double speed) {
        //
        int move = (int) (Math.round(inches * cpi * strafeBias));
        //
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
        //
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);
        //
        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        return;
    }
}