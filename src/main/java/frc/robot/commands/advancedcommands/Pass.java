package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shoot.JustShoot;
import frc.robot.commands.storage.Spin;
import frc.robot.commands.storage.Transport;
import frc.robot.subsystems.shoot.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;

import java.util.function.Supplier;

public class Pass extends ParallelCommandGroup {

    public Pass(Shooter shooter, Supplier<Double> shootingSpeed,
                SpinningMagazine spinningMagazine,
                Kicker transport
    ) {
        addCommands(
                new Spin(spinningMagazine),
                new Transport(transport),
                new JustShoot(shooter, shootingSpeed)
        );
    }
}
