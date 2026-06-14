package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.533827)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.01,   // was 0.03 — much lower for lighter robot
                    0,
                    0.005,  // add kD to dampen oscillation
                    0.008   // was 0.015
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.2,    // fine correction near endpoint
                    0,
                    0.008,  // damping
                    0.0003
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.3,    // was 0.8 — way too aggressive for lighter robot
                    0,
                    0.02,   // add damping
                    0.005
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    1.0,    // was 2.5
                    0,
                    0.05,
                    0.0003
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.04,   // was 0.1
                    0,
                    0.0001, // was 0.00035
                    0.6,
                    0.008   // was 0.015
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.01,
                    0,
                    0.000002,
                    0.6,
                    0.005
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0003);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.8)
            .rightFrontMotorName("right_front_motor")
            .rightRearMotorName("right_back_motor")
            .leftRearMotorName("left_back_motor")
            .leftFrontMotorName("left_front_motor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5)
            .strafePodX(0.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odoX")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            4.0,
            2.5,
            0.05,
            3000,
            1.5,    // was 1.25 — stronger braking force
            10,
            0.6     // was 0.8 — start braking much earlier (60% through path)
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants) // Pinpoint handles IMU too
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}