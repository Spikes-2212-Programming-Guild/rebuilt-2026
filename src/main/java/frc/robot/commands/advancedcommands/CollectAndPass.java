package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.shooter.SimpleShoot;
import frc.robot.commands.spindexer.SpinMagazine;
import frc.robot.commands.spindexer.Transport;

public class CollectAndPass extends ParallelCommandGroup {

    public CollectAndPass(){
        addCommands(
                new Collect(),
                new SpinMagazine(),
                new Transport(),
                new SimpleShoot(() -> 0.4)
        );
    }
}
