package frc.robot.utils.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RobotRelativeLimelightCamera implements RobotRelativeVisionSource {

    private final String tableName;

    public RobotRelativeLimelightCamera(String tableName) {
        this.tableName = tableName;
    }

    @Override
    public boolean hasValidTarget() {
        return getTable().getEntry("tv").getDouble(0) == 1.0;
    }

    @Override
    public double getHorizontalOffset() {
        return getTable().getEntry("tx").getDouble(0);
    }

    @Override
    public double getTargetArea() {
        return getTable().getEntry("ta").getDouble(0);
    }

    @Override
    public void setPipelineIndex(int index) {
        getTable().getEntry("pipeline").setNumber(index);
    }

    private NetworkTable getTable() {
        return NetworkTableInstance.getDefault().getTable(tableName);
    }
}
