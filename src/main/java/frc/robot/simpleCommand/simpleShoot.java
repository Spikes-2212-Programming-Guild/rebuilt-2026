package frc.robot.simpleCommand;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class simpleShoot extends MoveGenericSubsystem {

    public simpleShoot(Shooter shooter, Supplier<Double> speed) {
        super(shooter, speed);
    }
}
