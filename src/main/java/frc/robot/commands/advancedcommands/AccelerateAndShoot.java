package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shoot.ShootWithPID;
import frc.robot.commands.storage.Spin;
import frc.robot.commands.storage.Transport;
import frc.robot.subsystems.shoot.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;

import java.util.function.Supplier;

public class AccelerateAndShoot extends SequentialCommandGroup {

    public AccelerateAndShoot(Shooter shooter, Supplier<Double> shootingSpeed,
                              SpinningMagazine spinningMagazine,
                              Kicker transport
    ) {
        addCommands(
                new ShootWithPID(shooter, shootingSpeed), // accelerate shooter
                new ParallelCommandGroup(
                        new Spin(spinningMagazine),
                        new Transport(transport),
                        new ShootWithPID(shooter, shootingSpeed)
                )
        );
    }
}
