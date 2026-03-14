package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.Drive;
import frc.robot.subsystems.swerve.Drivetrain;

public class GoAndWaitAuto extends SequentialCommandGroup {

    public GoAndWaitAuto(Drivetrain drivetrain) {
        addCommands(new Drive(drivetrain, drivetrain::getXSpeed, drivetrain::getYSpeed,drivetrain::getRotationSpeed,
                false, true).
                withTimeout(3));
    }
}
