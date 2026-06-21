package frc.robot.autonomous.autoincode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.advancedcommands.TuneToAprilTag;
import frc.robot.commands.swerve.Drive;
import frc.robot.subsystems.intake.Collection;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

import java.util.function.Supplier;

public class JustShootAuto extends SequentialCommandGroup {

    //@TODO change supplier values after calibration

    public JustShootAuto(Drivetrain drivetrain, Shooter shooter, Kicker transport, SpinningMagazine spinningMagazine,
                         VisionService visionService, Collection collection, Supplier<Double> speed) {
        addCommands(
                new Drive(drivetrain, () -> 0.0, () -> 0.0, () -> 0.0, false, false).
                        withTimeout(1),
                new TuneToAprilTag(drivetrain ,visionService, shooter, transport, spinningMagazine, collection,
                        speed.get()).withTimeout(4));
    }
}
