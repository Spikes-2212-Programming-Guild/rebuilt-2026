package frc.robot.autonomous.autoincode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.Drive;
import frc.robot.subsystems.swerve.Drivetrain;

public class GoAndWaitAuto extends SequentialCommandGroup {

    public GoAndWaitAuto(Drivetrain drivetrain) {
        addCommands(new Drive(drivetrain, () -> 0.2, () -> 0.0, () -> 0.0, false, false).
                withTimeout(3));
    }
}
