package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.collection.CollectionJoint;
import frc.robot.subsystems.collection.Roller;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;

import java.util.function.Supplier;

public class CollectAndPass extends ParallelCommandGroup {

    public CollectAndPass(Roller roller, CollectionJoint collectionJoint, Shooter shooter,
                          Supplier<Double> shootingSpeed, SpinningMagazine spinningMagazine, Kicker kicker) {
        addCommands(
                new Collect(roller, collectionJoint),
                new Pass(shooter, shootingSpeed, spinningMagazine, kicker)
        );
    }
}
