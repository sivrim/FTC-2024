package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ArmUp extends LinearOpMode {
    public static double ARM1_ANGLE_TO_ENCODER = 3287/90;
    public static double ARM2_ANGLE_TO_ENCODER = 1380/90;


    public static double MAX_CLAW_OPEN = 0.6;//close
    public static double MAX_CLAW_CLOSE = 0.0; //actually open
    public static double MAX_WRIST_UP = 0.6;//goes up
    public static double MAX_WRIST_DOWN = 0.8;// go toward ground
    public static double MAX_WRIST_START = 0.0;

    public static int SAMPLE_1_ANGLE_ARM_1_AWAY_FROM_FLOOR = 30;

    public static int SAMPLE_1_ANGLE_ARM_2_MOVE_BACK_1 = 20;
    public static int SAMPLE_1_ANGLE_ARM_1_MOVE_BACK_1 = 115;

    public static int SAMPLE_1_ANGLE_ARM_2_MOVE_BACK_2 = 150;
    public static int SAMPLE_1_ANGLE_ARM_1_MOVE_BACK_2 = 28;

    public static int ARM_1_SAMPLE_PICK_ANGLE = 135;
    public static int ARM_1_SAMPLE_234_DROP_ANGLE = 122;


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
        sleep(20);
    }

}
