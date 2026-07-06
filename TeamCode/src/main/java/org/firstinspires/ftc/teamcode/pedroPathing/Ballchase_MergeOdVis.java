package org.firstinspires.ftc.teamcode.pedroPathing;

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

    // Pipeline slot on the Limelight dashboard
    static final int    PIPELINE_NEURAL      = 1;

    // Rotate until tx is within this range (-5 to +5 degrees)
    static final double TX_TOLERANCE         = 5.0;

    // How fast the robot rotates
    static final double TURN_SPEED           = 0.35;

    // Minimum confidence to count as a valid ball detection
    static final double CONFIDENCE_THRESHOLD = 53.0;

    // Must match the class name in your Roboflow model exactly
    static final String BALL_CLASS           = "Ball";

    private Limelight3A limelight;
    private Follower    follower;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE_NEURAL);
        limelight.start();

        telemetry.addData("Status", "Ready — Press Play");
        telemetry.update();

        waitForStart();

        follower.startTeleopDrive();

        try {
            while (opModeIsActive()) {
                follower.update();

                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {

                    LLResultTypes.DetectorResult ball = getBestBall(result);

                    if (ball != null) {
                        double tx = ball.getTargetXDegrees(); // negative = left, positive = right

                        telemetry.addData("Ball tx", "%.2f°", tx);

                        if (tx > TX_TOLERANCE) {
                            // Ball is to the right — rotate right
                            follower.setTeleOpDrive(0, 0, TURN_SPEED, true);
                            telemetry.addData("Status", "Rotating RIGHT");

                        } else if (tx < -TX_TOLERANCE) {
                            // Ball is to the left — rotate left
                            follower.setTeleOpDrive(0, 0, -TURN_SPEED, true);
                            telemetry.addData("Status", "Rotating LEFT");

                        } else {
                            // tx is between -5 and +5 — stop!
                            follower.setTeleOpDrive(0, 0, 0, true);
                            telemetry.addData("Status", "Ball centered — DONE");
                        }

                    } else {
                        // No ball detected — stay still
                        follower.setTeleOpDrive(0, 0, 0, true);
                        telemetry.addData("Status", "No ball detected");
                    }

                } else {
                    // No Limelight data — stay still
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

        LLResultTypes.DetectorResult best   = null;
        double                       bestTa = -1;

        for (LLResultTypes.DetectorResult det : detections) {
            if (!det.getClassName().equalsIgnoreCase(BALL_CLASS)) continue;
            if (det.getConfidence() < CONFIDENCE_THRESHOLD)       continue;
            if (det.getTargetArea() > bestTa) {
                bestTa = det.getTargetArea();
                best   = det;
            }
        }

        return best;
    }
}