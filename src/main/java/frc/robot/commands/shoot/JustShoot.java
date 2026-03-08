package frc.robot.commands.shoot;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.shoot.Shooter;

import java.util.function.Supplier;

public class JustShoot extends MoveGenericSubsystem {

    public JustShoot(Shooter shooter, Supplier<Double> speed) {
        super(shooter, speed);
    }
}
