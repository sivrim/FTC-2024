package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DeviceNames;
import org.firstinspires.ftc.teamcode.auton.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "aaa")
public class AAuton24Left extends LinearOpMode {
    public static double FORWARD_1 = 6;
    public static double STRAFE_LEFT_GO_TO_BASKET = 18;
    public static int ARM_1_MOVE_BACK_1_ANGLE = 90;
    public static int ARM_2_MOVE_BACK_1_ANGLE = 120;
    public static int ARM_1_MOVE_BACK_2_ANGLE = 45;
    public static double ARM1_POWER = 1.0;
    public static double ARM2_POWER = 1.0;

    public static final double FORWARD_2 = 6;

    public static double MAX_CLAW_OPEN = 0.3;
    public static double MAX_CLAW_CLOSE = 0.5;
    public static double MAX_WRIST_OPEN = 0.15;
    public static double MAX_WRIST_CLOSE = 0.75;
    public static double TURN_M45 = -45;

    public static double TURN_45 = 45;

    public static double ARM1_ANGLE_TO_ENCODER = 5000/45;
    public static double ARM2_ANGLE_TO_ENCODER = 2698/90;


    private ElapsedTime runtime = new ElapsedTime();

    DcMotor armMotor = null;
    Servo clawServo;
    Servo wristServo;
    DcMotor armMotor2;


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        armMotor = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM);
        armMotor2 = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM2);
        clawServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_CLAW);
        clawServo.setPosition(MAX_CLAW_CLOSE);
        wristServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_WRIST);
        wristServo.setPosition(MAX_WRIST_CLOSE);

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD_1)
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .strafeLeft(STRAFE_LEFT_GO_TO_BASKET)
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .forward(FORWARD_2)
                .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .back(FORWARD_2)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(trajectory1);

        drive.followTrajectory(trajectory2);

        drive.turn(Math.toRadians(TURN_M45));

        // Reset the motor encoder so that it reads zero ticks
        armMotor2.setPower(ARM2_POWER);
        armMotor.setPower(ARM1_POWER);

        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_1_MOVE_BACK_1_ANGLE * ARM1_ANGLE_TO_ENCODER), armMotor);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_2_MOVE_BACK_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor2);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM_1_MOVE_BACK_2_ANGLE * ARM1_ANGLE_TO_ENCODER), armMotor);
        wristServo.setPosition(MAX_WRIST_OPEN);
//        drive.turn(Math.toRadians(TURN_45));
//
//        drive.followTrajectory(trajectory3);
//
//        drive.followTrajectory(trajectory4);
//
//
//        drive.turn(Math.toRadians(TURN_M45));
//
//        drive.turn(Math.toRadians(TURN_45));
//
//        drive.followTrajectory(trajectory3);
//
//        drive.followTrajectory(trajectory4);
    }

    private void moveArmToPosition(DcMotorSimple.Direction dir, int encoderPos, DcMotor motor) {
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
