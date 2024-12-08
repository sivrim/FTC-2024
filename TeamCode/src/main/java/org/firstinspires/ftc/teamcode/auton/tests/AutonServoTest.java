package org.firstinspires.ftc.teamcode.auton.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DeviceNames;

@Config
@TeleOp(name = "AAutonServoTest", group = "aaa")
public class AutonServoTest extends LinearOpMode {
    public static double MAX_CLAW_OPEN = 0.1;//close
    public static double MAX_CLAW_CLOSE = 1.0; //actually open
    public static double MAX_WRIST_OPEN = 0.0;//goes up
    public static double MAX_WRIST_CLOSE = 1.0;// go toward ground

    public static boolean fineGrained = false;

    private ElapsedTime runtime = new ElapsedTime();

    Servo clawServo;
    Servo wristServo;

    @Override
    public void runOpMode() {
        clawServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_CLAW);

        wristServo = hardwareMap.get(Servo.class, DeviceNames.SERVO_WRIST);

        telemetry.addData("clawservo", clawServo.getPosition());

        clawServo.setPosition(MAX_CLAW_CLOSE);
        telemetry.addData("clawservo", clawServo.getPosition());

        sleep(2000);
        clawServo.setPosition(MAX_CLAW_OPEN);
        telemetry.addData("clawservo", clawServo.getPosition());
        telemetry.update();

        sleep(2000);
        wristServo.setPosition(MAX_WRIST_CLOSE);//in
        sleep(2000);
        wristServo.setPosition(MAX_WRIST_OPEN);//out
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
