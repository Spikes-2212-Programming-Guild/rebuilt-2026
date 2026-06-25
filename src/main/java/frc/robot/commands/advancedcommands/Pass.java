package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.shoot.JustShoot;
import frc.robot.commands.storage.SpinMagazine;
import frc.robot.commands.storage.Transport;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;

import java.util.function.Supplier;

public class Pass extends ParallelCommandGroup {

    public Pass(Shooter shooter, Supplier<Double> shootingSpeed, SpinningMagazine spinningMagazine,
                Kicker kicker) {

        addCommands(
                new SpinMagazine(spinningMagazine),
                new Transport(kicker),
                new JustShoot(shooter, shootingSpeed)
        );
    }
}
