package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DeviceNames;
import org.firstinspires.ftc.teamcode.auton.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "AAutonServoTest", group = "aaa")
public class AutonServoTest extends LinearOpMode {
    public static double MAX_CLAW_OPEN = 0.2;
    public static double MAX_CLAW_CLOSE = 1.0;
    public static double MAX_WRIST_OPEN = 0.0;
    public static double MAX_WRIST_CLOSE = 1.0;

    public static boolean fineGrained = true;

    private ElapsedTime runtime = new ElapsedTime();

    Servo clawServo;
    Servo wristServo;

    @Override
    public void runOpMode() {
        clawServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_CLAW);
        wristServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_WRIST);

        clawServo.setPosition(MAX_CLAW_CLOSE);

        wristServo.setPosition(MAX_WRIST_CLOSE);
        sleep(2000);

        wristServo.setPosition(MAX_WRIST_OPEN);
        sleep(2500);

        if(fineGrained){
            wristServo.setDirection(Servo.Direction.REVERSE);
            for(double i = 0.1; i<=1 ;i = i + 0.9){
                wristServo.setPosition(i);
                sleep(100);
            }


            wristServo.setDirection(Servo.Direction.FORWARD);
            for(double i = 0.1; i<=0.9 ;i = i + 0.1){
                wristServo.setPosition(i);
                sleep(100);
            }
        }

        waitForStart();

        if(isStopRequested()) return;

    }

}
