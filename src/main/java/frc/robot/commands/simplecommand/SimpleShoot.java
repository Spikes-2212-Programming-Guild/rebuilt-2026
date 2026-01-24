package frc.robot.commands.simplecommand;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class SimpleShoot extends MoveGenericSubsystem {

    public SimpleShoot(Shooter shooter, Supplier<Double> speed) {
        super(shooter, speed);
    }
}
