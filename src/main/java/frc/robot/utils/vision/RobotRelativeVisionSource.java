package frc.robot.utils.vision;

public interface RobotRelativeVisionSource {

    /**
     * @return true if the camera currently sees a valid target.
     */
    boolean hasValidTarget();

    /**
     * @return the horizontal offset to the target (tx) in degrees.
     */
    double getHorizontalOffset();

    /**
     * @return the target area (ta) as a percentage of the image.
     */
    double getTargetArea();

    /**
     * Sets the vision processing pipeline index.
     *
     * @param index the pipeline ID to switch to.
     */
    void setPipelineIndex(int index);
}
