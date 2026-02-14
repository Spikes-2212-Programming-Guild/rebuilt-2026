package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.difficultcommands.CollectionToPosition;
import frc.robot.commands.simplecommands.SimpleIntake;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.CollectionMovement;
import frc.robot.commands.difficultcommands.VisionDriveAlignCommand;
import frc.robot.subsystems.SwerveDrivetrain;
import java.util.function.Supplier;

public class CollectionAutoAlign extends SequentialCommandGroup {

    public CollectionAutoAlign(Collection collection, CollectionMovement collectionMovement,
                               SwerveDrivetrain swerve, Supplier<Double> forwardVelocity,
                               Supplier<Double> strafeVelocity) {
        addCommands(
                new CollectionToPosition(
                        collectionMovement, () -> CollectionMovement.CollectionMovementPose.MAX_POSE.neededPose
                ),
                new SimpleIntake(collection).alongWith(new VisionDriveAlignCommand(swerve, forwardVelocity, strafeVelocity))
        );
    }
}
