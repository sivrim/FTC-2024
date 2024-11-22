package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ArmDown extends LinearOpMode {
    public static double ARM1_ANGLE_TO_ENCODER = 3 * 8375/90;
    public static double ARM2_ANGLE_TO_ENCODER = 2680/90;

    public static int ARM_1_MOVE_DOWN_1_ANGLE = 300;
    public static int ARM_2_MOVE_BACK_1_ANGLE = 135;
    public static double ARM1_POWER = 1.0;
    public static double ARM2_POWER = 1.0;

    public void runOpMode() {
    }

    public void moveArmToPosition(DcMotorSimple.Direction dir, int encoderPos, DcMotor motor, ElapsedTime runtime) {

        motor.setDirection(dir);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Arm motor position at very start", motor.getCurrentPosition());

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(encoderPos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1.0);
        runtime.reset();
        int timeout = 10;
        telemetry.addData("Running to",  " %7d ", encoderPos);

        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (motor.isBusy() )) {
        }

        telemetry.addData("After motion, Currently at",  " at %7d ",
                motor.getCurrentPosition());
        telemetry.update();
        sleep(100);
    }

}
