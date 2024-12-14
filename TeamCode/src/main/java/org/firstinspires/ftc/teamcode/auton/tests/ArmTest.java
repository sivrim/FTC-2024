package org.firstinspires.ftc.teamcode.auton.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DeviceNames;
import org.firstinspires.ftc.teamcode.config.ArmUp;

@Config
@Autonomous(group = "aaa")
@Disabled
public class ArmTest extends ArmUp {

    DcMotor armMotor = null;
    DcMotor armMotor2;

    public static int ARM1_UP = 160;
    public static int ARM1_DOWN = 135;

    public static int ARM1_UP_1 = 160;

    @Override
    public void runOpMode() {
        armMotor = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM);
        armMotor2 = hardwareMap.get(DcMotor.class, DeviceNames.MOTOR_ARM2);

        waitForStart();

        ElapsedTime runtime = new ElapsedTime();

        if(isStopRequested()) return;

        moveArmToPosition(DcMotorSimple.Direction.FORWARD, (int)(ARM1_UP * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);
        sleep(1000);
        moveArmToPosition(DcMotorSimple.Direction.REVERSE, (int)(ARM1_DOWN * ARM1_ANGLE_TO_ENCODER), armMotor, runtime);

    }

}
