package org.firstinspires.ftc.teamcode.auton.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.DeviceNames;
import org.firstinspires.ftc.teamcode.auton.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    //copied from https://www.gobilda.com/swingarm-odometry-pod-48mm-wheel/?srsltid=AfmBOornkcHyeGvkNrrewYYHbajbadFXsurIn_iwA2b_VCRhD6z5YVvL
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = .95; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    //TODO
    public static double LATERAL_DISTANCE = 12.455; // in; distance between the left and right wheels
    //TODO . Negative if in the back. the distance from the center of rotation to the middle wheel.
    public static double FORWARD_OFFSET = 1; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 1.01; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.014; // Multiplier in the Y direction

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    private List<Integer> lastEncPositions, lastEncVels;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        //TODO get the names
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, DeviceNames.MOTOR_FRONT_LEFT));//"leftEncoder"
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, DeviceNames.MOTOR_FRONT_RIGHT));//"rightEncoder"
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, DeviceNames.MOTOR_BACK_RIGHT));//"frontEncoder"

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos) *X_MULTIPLIER,
                encoderTicksToInches(rightPos) *X_MULTIPLIER,
                encoderTicksToInches(frontPos) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) frontEncoder.getCorrectedVelocity();

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(frontVel);

        return Arrays.asList(
                encoderTicksToInches(leftVel) * X_MULTIPLIER,
                encoderTicksToInches(rightVel) * X_MULTIPLIER,
                encoderTicksToInches(frontVel) * Y_MULTIPLIER
        );
    }
}
