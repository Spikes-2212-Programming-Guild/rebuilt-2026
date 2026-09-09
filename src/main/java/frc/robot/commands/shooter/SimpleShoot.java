package frc.robot.commands.shooter;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.Supplier;

public class SimpleShoot extends MoveGenericSubsystem {

    public SimpleShoot(Shooter shooter, Supplier<Double> speed) {
        super(shooter, speed);
    }

    public SimpleShoot(Supplier<Double> speed) {
        super(Shooter.getInstance(), speed);
    }
}
