package frc.robot.commands.advancedcommands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.difficultcommands.CollectionToPosition;
import frc.robot.commands.difficultcommands.VisionDriveAlignCommand;
import frc.robot.commands.simplecommands.SimpleIntake;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.CollectionMovement;
import frc.robot.subsystems.SwerveDrivetrain;

public class FullyAutoCollect extends SequentialCommandGroup {

    private static final double COLLECTION_SPEED = -1.0;

    public FullyAutoCollect(Collection collection, CollectionMovement movement,
                            SwerveDrivetrain swerve) {
        addCommands(
                new CollectionToPosition(
                        movement,
                        () -> CollectionMovement.CollectionMovementPose.MAX_POSE.neededPose
                ),
                new SimpleIntake(collection).alongWith(
                        new VisionDriveAlignCommand(
                                swerve,
                                this::calculateForwardSpeed,
                                () -> 0.0
                        )
                ).finallyDo(
                        () -> CommandScheduler.getInstance().schedule(
                                new CollectionToPosition(
                                        movement,
                                        () -> CollectionMovement.CollectionMovementPose.MIN_POSE.neededPose)
                        )
                )
        );
    }

    /**
     * Determines the forward speed. Driving continues as long as targets are visible.
     *
     * @return the collection speed if a target is found, otherwise 0.
     */
    private double calculateForwardSpeed() {
        double tv = NetworkTableInstance.getDefault().getTable("limelight")
                .getEntry("tv").getDouble(0);

        if (tv == 1.0) {
            return COLLECTION_SPEED;
        }
        return 0.0;
    }
}
