package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.Drive;
import frc.robot.commands.swerve.RotateAccordingAprilTags;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

public class RotateToTag extends SequentialCommandGroup {

    public RotateToTag(double rotationSpeed) {
        Drivetrain drivetrain = Drivetrain.getInstance();
        addCommands(
                new Drive(
                        drivetrain,
                        () -> 0.0,
                        () -> 0.0,
                        () -> rotationSpeed,
                        true,
                        true
                )
                        .until(VisionService.getInstance()::hasTarget),
                new RotateAccordingAprilTags(
                        drivetrain,
                        () -> 0.0,
                        () -> VisionService.getInstance().getX(),
                        true
                )
        );
    }
}
