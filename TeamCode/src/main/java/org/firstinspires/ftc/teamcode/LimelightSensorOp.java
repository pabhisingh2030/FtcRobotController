package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;


@TeleOp(name = "Limelight Color + Distance (Fixed)", group = "Sensor")
public class LimelightSensorOp extends LinearOpMode {

    private static final double K_AREA = 24.0;

    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(50);

        try {
            limelight.pipelineSwitch(0);
        } catch (Exception e) {
            telemetry.addLine("Pipeline set failed (check SDK)");
        }

        telemetry.addLine("Limelight Ready. Waiting for start...");
        telemetry.update();

        waitForStart();

        limelight.start();

        try {
            while (opModeIsActive()) {

                LLStatus status = limelight.getStatus();
                LLResult result = limelight.getLatestResult();

                if (status != null) {
                    telemetry.addData("Pipeline", status.getPipelineIndex());
                } else {
                    telemetry.addLine("No Status");
                }

                if (result != null && result.isValid()) {
                    double tx = result.getTx();
                    double ty = result.getTy();
                    double ta = result.getTa();

                    telemetry.addLine("--- Main Target ---");
                    telemetry.addData("Offset", "X: %.2f, Y: %.2f", tx, ty);
                    telemetry.addData("Area (%)", "%.2f", ta);

                    List<LLResultTypes.DetectorResult> targets = result.getDetectorResults();

                    if (targets != null && !targets.isEmpty()) {
                        int i = 0;
                        for (LLResultTypes.DetectorResult t : targets) {
                            double area = t.getTargetArea();
                            if (area > 0.001) {
                                double distIn = K_AREA / Math.sqrt(area);
                                telemetry.addData("Target " + i, "Dist: %.1f in", distIn);
                            } else {
                                telemetry.addData("Target " + i, "Area too small");
                            }
                            i++;
                        }
                    } else {
                        telemetry.addLine("No individual targets");
                    }

                } else {
                    telemetry.addLine("No Target Detected");
                }

                telemetry.update();
                idle();
            }

        } finally {
            limelight.stop();
        }
    }
}
