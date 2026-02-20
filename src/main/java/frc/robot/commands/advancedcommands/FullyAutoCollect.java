package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.difficultcommands.CollectionToPosition;
import frc.robot.commands.difficultcommands.VisionDriveAlignCommand;
import frc.robot.commands.simplecommands.SimpleIntake;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.CollectionMovement;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.vision.RobotRelativeLimelightCamera;
import frc.robot.utils.vision.RobotRelativePhotonCamera;
import frc.robot.utils.vision.RobotRelativeVisionSource;
import frc.robot.utils.BumperColorSelector;
import frc.robot.utils.RobotRelativeCollisionGuard;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class FullyAutoCollect extends SequentialCommandGroup {

    private static final double COLLECTION_SPEED = -1.0;

    private final RobotRelativeVisionSource safetyCam;
    private final RobotRelativeVisionSource intakeCam;
    private final RobotRelativeCollisionGuard collisionGuard;
    private final BumperColorSelector pipelineSelector;

    public FullyAutoCollect(Collection collection, CollectionMovement movement,
                            SwerveDrivetrain swerve) {

        this.safetyCam = new RobotRelativeLimelightCamera("limelight");
        this.intakeCam = new RobotRelativePhotonCamera("photoncamera");
        this.collisionGuard = new RobotRelativeCollisionGuard(safetyCam);
        this.pipelineSelector = new BumperColorSelector(safetyCam);

        addCommands(
                runOnce(pipelineSelector::setPipelineToOpponent),

                new CollectionToPosition(
                        movement,
                        () -> CollectionMovement.CollectionMovementPose.MAX_POSE.neededPose
                ),

                new SimpleIntake(collection).alongWith(
                        new VisionDriveAlignCommand(
                                swerve,
                                intakeCam,
                                this::calculateSafeSpeed,
                                () -> 0.0
                        )
                ).finallyDo(
                        () -> CommandScheduler.getInstance().schedule(
                                new CollectionToPosition(
                                        movement,
                                        () -> CollectionMovement.CollectionMovementPose.MIN_POSE.neededPose
                                )
                        )
                )
        );
    }

    private double calculateSafeSpeed() {
        if (collisionGuard.isPathBlocked()) {
            return 0.0;
        }

        if (intakeCam.hasValidTarget()) {
            return COLLECTION_SPEED;
        }

        return 0.0;
    }
}
