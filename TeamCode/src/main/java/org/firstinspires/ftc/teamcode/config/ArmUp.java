package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ArmUp extends LinearOpMode {
    public static double ARM1_ANGLE_TO_ENCODER = 5000/45;
    public static double ARM2_ANGLE_TO_ENCODER = 2698/90;

    public static double MAX_CLAW_OPEN = 0.3;
    public static double MAX_CLAW_CLOSE = 0.5;
    public static double MAX_WRIST_OPEN = 1.0;
    public static double MAX_WRIST_CLOSE = 0.1;
    public static double TURN_M45 = -45;

    public static double TURN_45 = 45;

    public static int ARM_1_MOVE_BACK_1_ANGLE = 105;
    public static int ARM_2_MOVE_BACK_1_ANGLE = 165;
    public static int ARM_1_MOVE_BACK_2_ANGLE = 10;
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
        runtime.reset();
        int timeout = 5;
        telemetry.addData("Running to",  " %7d ", encoderPos);

        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (motor.isBusy() )) {
        }

        telemetry.addData("After motion, Currently at",  " at %7d ",
                motor.getCurrentPosition());
        telemetry.update();
        sleep(1000);
    }

}
