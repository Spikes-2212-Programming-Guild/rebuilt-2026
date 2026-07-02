package frc.robot.commands.advancedcommands;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.Drive;
import frc.robot.commands.swerve.RotateAccordingAprilTags;
import frc.robot.subsystems.intake.Collection;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

public class TuneToAprilTag extends SequentialCommandGroup {

    private static final double SPEED_TO_MOVE_COLLECTION = 0.5;

    public TuneToAprilTag(Drivetrain drivetrain, VisionService visionService, Shooter shooter, Kicker kicker,
                          SpinningMagazine spinningMagazine, Collection collection, double rotationSpeed) {
        addCommands(
                new Drive(drivetrain, () -> 0.0, () -> 0.0, () -> rotationSpeed, true, true)
                        .until(visionService::hasTarget),
                new RotateAccordingAprilTags(drivetrain, () -> 0.0, visionService, true),
                new ParallelCommandGroup(
                        new ShootToHub(shooter, kicker, spinningMagazine, visionService)
//                        new MoveGenericSubsystem(collection, SPEED_TO_MOVE_COLLECTION)
                )

        );
    }
}
