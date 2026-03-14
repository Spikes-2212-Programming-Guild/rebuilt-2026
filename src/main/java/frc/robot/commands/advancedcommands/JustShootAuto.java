package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.Drive;
import frc.robot.subsystems.shoot.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

public class JustShootAuto extends SequentialCommandGroup {

    public JustShootAuto(Drivetrain drivetrain, Shooter shooter, Kicker transport, SpinningMagazine spinningMagazine,
                         VisionService visionService) {
        addCommands(
                new Drive(drivetrain, drivetrain::getXSpeed, drivetrain::getYSpeed, drivetrain::getRotationSpeed,
                        false, true),
                new TuneAndShoot(shooter, transport, spinningMagazine, visionService));
    }
}
