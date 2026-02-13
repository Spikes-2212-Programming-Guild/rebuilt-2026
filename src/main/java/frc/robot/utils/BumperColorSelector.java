package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.vision.RobotRelativeVisionSource;
import java.util.Optional;

public class BumperColorSelector {

    // Configure your Limelight pipelines here
    public static final int RED_BUMPER_PIPELINE = -1;
    public static final int BLUE_BUMPER_PIPELINE = -1;
    public static final int FUEL_PIPELINE = -1;

    private final RobotRelativeVisionSource limelight;

    public BumperColorSelector(RobotRelativeVisionSource limelight) {
        this.limelight = limelight;
    }

    /**
     * Sets the pipeline to track the OPPONENT'S bumpers (for avoidance/defense).
     */
    public void setPipelineToOpponent() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            // We are Red, so look for Blue bumpers
            limelight.setPipelineIndex(BLUE_BUMPER_PIPELINE);
        } else {
            // We are Blue (or invalid), so look for Red bumpers
            limelight.setPipelineIndex(RED_BUMPER_PIPELINE);
        }
    }

    /**
     * Sets the pipeline to track FUEL (Yellow).
     */
    public void setPipelineToFuel() {
        limelight.setPipelineIndex(FUEL_PIPELINE);
    }
}
