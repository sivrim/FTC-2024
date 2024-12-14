package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DeviceNames;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "aaa")
@Disabled
public class Auton2024 extends LinearOpMode {
    public static double ARM1_ANGLE_TO_ENCODER = 3287/90;
    public static double ARM2_ANGLE_TO_ENCODER = 1380/90;


    public static double MAX_CLAW_OPEN = 0.6;//close
    public static double MAX_CLAW_CLOSE = 0.0; //actually open
    public static double MAX_WRIST_UP = 0.6;//goes up
    public static double MAX_WRIST_DOWN = 0.8;// go toward ground
    public static double MAX_WRIST_START = 0.0;

    public static int ARM_1_MOVE_UP_AT_START_ANGLE = 10;
    public static int SAMPLE_1_DROP_ARM_2_MOVE_BACK_1_ANGLE = 20;
    public static int SAMPLE_1_DROP_ARM_1_MOVE_BACK_1_ANGLE = 90;
    public static int SAMPLE_1_DROP_ARM_2_MOVE_BACK_2_ANGLE = 30;

    public static int SAMPLE_2_PICK_ARM_1_MOVE_DOWN_1_ANGLE = 45;
    public static int SAMPLE_2_PICK_ARM_2_MOVE_DOWN_1_ANGLE = 90;

    public static double ARM1_POWER = 1.0;
    public static double ARM2_POWER = 1.0;

    public static double FORWARD_FROM_START_STEP_1 = 16;
    public static double STRAFE_LEFT_GO_TO_BASKET_SAMPLE_1 = 7;
    public static double STRAFE_RIGHT_GO_TO_SAMPLE2 = 4.5;
    public static double FORWARD_FROM_DROP = 7;
    public static double BACK_STEP_3 = 15;

    public static double TURN_RATIO = 1;
    public static double ANGLE = 45;
    public static int SLEEP_DROP = 1000;

    public static double ARM_MOTOR_2_POWER = 0.5;

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor armMotor = null;Servo clawServo;Servo wristServo;DcMotor armMotor2;SampleMecanumDrive drive;

    @Override
    public void runOpMode() {

        initMotors();

        TrajectorySequence trajToArmStretch = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(FORWARD_FROM_START_STEP_1)
                .strafeLeft(STRAFE_LEFT_GO_TO_BASKET_SAMPLE_1)
                .turn(Math.toRadians(-1  * ANGLE * TURN_RATIO))
                .build();

        TrajectorySequence trajToSample1Drop = drive.trajectorySequenceBuilder(trajToArmStretch.end())
                .back(BACK_STEP_3)
                .build();

        TrajectorySequence trajToSample2Pick = drive.trajectorySequenceBuilder(trajToSample1Drop.end())
                .forward(FORWARD_FROM_DROP)
                .turn(Math.toRadians(ANGLE * TURN_RATIO))
                .strafeRight(STRAFE_RIGHT_GO_TO_SAMPLE2)
                .build();

        TrajectorySequence trajToSample2Drop = drive.trajectorySequenceBuilder(trajToSample2Pick.end())
                .turn(Math.toRadians(-1 * ANGLE * TURN_RATIO))
                .strafeLeft(STRAFE_RIGHT_GO_TO_SAMPLE2)
                .back(FORWARD_FROM_DROP)
                .build();

        TrajectorySequence trajToSample3Pick = drive.trajectorySequenceBuilder(trajToSample2Drop.end())
                .forward(FORWARD_FROM_DROP)
                .turn(Math.toRadians(ANGLE * TURN_RATIO))
                .strafeRight(STRAFE_RIGHT_GO_TO_SAMPLE2 -10)
                .build();

        TrajectorySequence trajToSample3Drop = drive.trajectorySequenceBuilder(trajToSample3Pick.end())
                .strafeRight(10)
                .turn(Math.toRadians(-1 * ANGLE * TURN_RATIO))
                .strafeLeft(STRAFE_RIGHT_GO_TO_SAMPLE2)
                .back(FORWARD_FROM_DROP)
                .build();

        clawServo.setPosition(MAX_CLAW_CLOSE);

        /////////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        if(isStopRequested()) return;
        /////////////////////////////////////////////////////////////////////////////////////

        clawServo.setPosition(MAX_CLAW_CLOSE);
        //move so we do not drag the arm on floor
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int) (ARM_1_MOVE_UP_AT_START_ANGLE * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        clawServo.setPosition(MAX_CLAW_CLOSE);

        drive.followTrajectorySequence(trajToArmStretch);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(SAMPLE_1_DROP_ARM_2_MOVE_BACK_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(SAMPLE_1_DROP_ARM_1_MOVE_BACK_1_ANGLE * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(SAMPLE_1_DROP_ARM_2_MOVE_BACK_2_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wristServo.setPosition(MAX_WRIST_UP);
        sleep(500);
        drive.followTrajectorySequence(trajToSample1Drop);
        clawServo.setPosition(MAX_CLAW_OPEN);

        telemetry.addLine("BEFORE sample 2 pick trajectory");
        drive.followTrajectorySequence(trajToSample2Pick);
        telemetry.addLine("After sample 2 pick trajectory");
        telemetry.update();
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(SAMPLE_2_PICK_ARM_1_MOVE_DOWN_1_ANGLE * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        telemetry.addLine("After arm1 sample 2 pick trajectory");
        telemetry.update();
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(SAMPLE_2_PICK_ARM_2_MOVE_DOWN_1_ANGLE * ARM2_ANGLE_TO_ENCODER), armMotor2, runtime);
        telemetry.addLine("After arm2 sample 2 pick trajectory");
        telemetry.update();
        wristServo.setPosition(MAX_WRIST_UP);
        clawServo.setPosition(MAX_CLAW_OPEN);

    }

    @NonNull
    private void initMotors() {
        drive = new SampleMecanumDrive(hardwareMap);
        armMotor = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM);
        armMotor2 = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM2);
        clawServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_CLAW);
        clawServo.setPosition(MAX_CLAW_CLOSE);
        wristServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_WRIST);
        wristServo.setPosition(MAX_WRIST_START);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        armMotor2.setPower(ARM_MOTOR_2_POWER);
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

        sleep(100);
    }


}
