package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class ArmUp extends LinearOpMode {
    public static double ARM1_ANGLE_TO_ENCODER = 3287/90;
    public static double ARM2_ANGLE_TO_ENCODER = 1380/90;


    public static double MAX_CLAW_OPEN = 0.6;//close
    public static double MAX_CLAW_CLOSE = 0.0; //actually open
    public static double MAX_WRIST_UP = 0.6;//goes up
    public static double MAX_WRIST_DOWN = 0.8;// go toward ground
    public static double MAX_WRIST_START = 0.0;

    //increase this from dashboard if you want to stop program at that point to observe
    public static int TEST_SLEEP_TIME_BEFORE_ARM_MOVE = 1;
    public static int TEST_SLEEP_TIME_AFTER_ARM_MOVE = 1;
    public static int SAMPLE_1_ANGLE_ARM_1_AWAY_FROM_FLOOR = 30;

    public static int SAMPLE_1_ANGLE_ARM_2_MOVE_BACK_1 = 15;
    public static int SAMPLE_1_ANGLE_ARM_1_MOVE_BACK_1 = 182;

    public static int SAMPLE_1_ANGLE_ARM_2_MOVE_BACK_2 = 155;
    public static int SAMPLE_1_ANGLE_ARM_1_MOVE_BACK_2 = 55;

    public static int SAMPLE_2_ARM_1_SAMPLE_PICK_ANGLE_1 = 128;
    public static int SAMPLE_2_ARM_1_DROP_ANGLE_1 = 110;
    public static int SAMPLE_2_ARM_1_DROP_ANGLE_2 = 30;
    public static double DROP_SENSOR_DISTANCE = 4.5;

    public void runOpMode() {
    }

    public void moveArmToPosition(DcMotorSimple.Direction dir, int encoderPos, DcMotor motor, ElapsedTime runtime) {
        moveArmToPosition(dir, encoderPos, motor, runtime, null, 1);
    }
        public void moveArmToPosition(DcMotorSimple.Direction dir, int encoderPos, DcMotor motor,
                                      ElapsedTime runtime, Rev2mDistanceSensor distanceSensor, double power) {
        sleep(TEST_SLEEP_TIME_BEFORE_ARM_MOVE);
        motor.setDirection(dir);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Arm motor position at very start", motor.getCurrentPosition());

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(encoderPos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        runtime.reset();
        int timeout = 6;
        telemetry.addData("Running to",  " %7d ", encoderPos);

        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (motor.isBusy() )) {
            if(distanceSensor != null) {
                double distance = distanceSensor.getDistance(DistanceUnit.INCH);

                if(distance < 15){
                    System.out.println("Distance is ........ " + distance);
                    telemetry.addData("Distance is ........ ", distance);
                    break;
                }

                if(distance < DROP_SENSOR_DISTANCE) {
                    motor.setPower(0);
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    telemetry.addLine("Will break........................");
                    break;
                }
            }
        }

        telemetry.addData("After motion, Currently at " , motor.getCurrentPosition());

        sleep(TEST_SLEEP_TIME_AFTER_ARM_MOVE);
    }

}
