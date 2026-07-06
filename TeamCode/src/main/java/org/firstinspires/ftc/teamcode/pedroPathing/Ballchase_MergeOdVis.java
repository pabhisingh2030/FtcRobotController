package org.firstinspires.ftc.teamcode.pedroPathing;

/*
 * Ballchase_MergeOdVis.java
 * ──────────────────────────
 * Autonomous OpMode — chases a single yellow ball using the Limelight 3A
 * neural detector, integrated with Pedro Pathing for odometry-aware motion.
 *
 * Sequence:
 *   1. Detect the ball (highest ta if multiple visible)
 *   2. Rotate in place until tx = 0 (ball perfectly centered)
 *   3. Drive straight forward toward the ball
 *   4. Stop when ta >= TA_STOP_THRESHOLD (ball fills enough of the frame)
 */

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Autonomous(name = "Ball Chase Merge OdVis", group = "Autonomous")
public class Ballchase_MergeOdVis extends LinearOpMode {

    // ═══════════════════════════════════════════════════════════════
    //  CONSTANTS — tune these in the garage
    // ═══════════════════════════════════════════════════════════════

    // TODO: set to your Roboflow neural detector pipeline slot number
    static final int    PIPELINE_NEURAL   = 1;

    // Stop driving when the ball fills with % of the camera frame
    static final double TA_STOP_THRESHOLD = 10.0;

    // Minimum confidence for a valid ball detection (0-100)
    static final double CONFIDENCE_THRESHOLD = 53.0;

    // How centered the ball needs to be before we start driving forward
    static final double TX_TOLERANCE      = 2.0;

    // Max turn power when zeroing tx
    static final double TURN_SPEED        = 0.35;

    // Drive power when moving toward the ball
    static final double DRIVE_SPEED       = 0.4;

    // Class name from your Roboflow label map
    static final String BALL_CLASS        = "Ball";

    // ═══════════════════════════════════════════════════════════════
    //  HARDWARE
    // ═══════════════════════════════════════════════════════════════

    private Limelight3A limelight;
    private Follower    follower;

    // ═══════════════════════════════════════════════════════════════
    //  MAIN
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void runOpMode() {

        // Initialize Pedro Pathing Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE_NEURAL);
        limelight.start();

        telemetry.setMsTransmissionInterval(11);

        telemetry.addData("Status", "Ready — Press Play");
        telemetry.update();

        waitForStart();

        // Start Pedro's teleop drive mode for manual control via powers
        follower.startTeleopDrive();

        try {
            while (opModeIsActive()) {
                follower.update();
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {

                    // Find the best ball detection (highest ta)
                    LLResultTypes.DetectorResult ball = getBestBall(result);

                    if (ball != null) {
                        double tx = ball.getTargetXDegrees(); // horizontal angle
                        double ta = ball.getTargetArea();     // % of frame filled
                        double conf = ball.getConfidence();   // confidence %

                        telemetry.addData("Ball tx", "%.2f°", tx);
                        telemetry.addData("Ball ta", "%.2f%%", ta);
                        telemetry.addData("Ball conf", "%.1f%%", conf);

                        // ── STOP condition — ball is close enough ───────────
                        if (ta >= TA_STOP_THRESHOLD) {
                            follower.setTeleOpDrive(0, 0, 0, true);
                            telemetry.addData("Status", "Ball reached — STOPPED");
                            telemetry.update();

                            // Loop to hold position until OpMode is stopped by user
                            while (opModeIsActive()) {
                                follower.update();
                                telemetry.addData("Status", "Complete");
                                telemetry.addData("Final Area", "%.2f%%", ta);
                                telemetry.update();
                            }
                            return;
                        }

                        // ── STEP 1: Zero tx — rotate until ball is centered ─
                        if (Math.abs(tx) > TX_TOLERANCE) {
                            // Scale turn power proportionally to tx
                            double turnPower = (tx / 30.0) * TURN_SPEED;
                            turnPower = Math.max(-TURN_SPEED, Math.min(TURN_SPEED, turnPower));

                            // Send rotation to Follower (robot-centric)
                            follower.setTeleOpDrive(0, 0, turnPower, true);

                            telemetry.addData("Status", "Centering... tx=%.2f°", tx);
                        } else {
                            // ── STEP 2: Ball centered — drive straight forward ─
                            // Scale drive power down as ta increases
                            double drivePower = DRIVE_SPEED * (1.0 - (ta / TA_STOP_THRESHOLD));
                            drivePower = Math.max(0.15, drivePower);

                            // Send forward power to Follower (robot-centric)
                            follower.setTeleOpDrive(drivePower, 0, 0, true);

                            telemetry.addData("Status", "Driving forward... ta=%.2f%%", ta);
                        }

                    } else {
                        // No ball visible — rotate slowly to scan
                        follower.setTeleOpDrive(0, 0, TURN_SPEED * 0.4, true);
                        telemetry.addData("Status", "Scanning for ball...");
                    }

                } else {
                    // No Limelight data — stop and wait
                    follower.setTeleOpDrive(0, 0, 0, true);
                    telemetry.addData("Status", "Waiting for Limelight...");
                }

                telemetry.update();
            }
        } finally {
            limelight.stop();
        }
    }

    private LLResultTypes.DetectorResult getBestBall(LLResult result) {
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

        LLResultTypes.DetectorResult best = null;
        double bestTa = -1;

        for (LLResultTypes.DetectorResult det : detections) {
            if (!det.getClassName().equalsIgnoreCase(BALL_CLASS)) continue;
            if (det.getConfidence() < CONFIDENCE_THRESHOLD) continue;
            if (det.getTargetArea() > bestTa) {
                bestTa = det.getTargetArea();
                best   = det;
            }
        }

        return best;
    }
}
