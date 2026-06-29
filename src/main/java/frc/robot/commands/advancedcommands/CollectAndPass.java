package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CollectAndPass extends SequentialCommandGroup {

    public CollectAndPass(){
        addCommands(
                new Collect(),
                new Shoot()
        );
    }
}
