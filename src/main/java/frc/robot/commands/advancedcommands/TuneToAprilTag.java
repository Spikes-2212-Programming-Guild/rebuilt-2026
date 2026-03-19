package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.Drive;
import frc.robot.commands.swerve.RotateAccordingAprilTags;
import frc.robot.subsystems.shoot.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

public class TuneToAprilTag extends SequentialCommandGroup {

    public TuneToAprilTag(Drivetrain drivetrain, VisionService visionService, Shooter shooter, Kicker kicker,
                          SpinningMagazine spinningMagazine, double rotationSpeed) {
        addCommands(
                new Drive(drivetrain, () -> 0.0, () -> 0.0, () -> rotationSpeed, true, true)
                        .until(visionService::hasTarget),
                new RotateAccordingAprilTags(drivetrain, () -> 0.0, visionService, true),
                new ShootToHub(shooter, kicker, spinningMagazine, visionService)

        );
    }
}
