package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shoot.ShootWithPID;
import frc.robot.commands.swerve.Drive;
import frc.robot.commands.swerve.RotateAccordingAprilTags;
import frc.robot.subsystems.shoot.Shooter;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

public class ShootMyGuy extends SequentialCommandGroup {

    private static final double SPEED_TO_DRIVE_X_AXIS = 1.0;
    private static final double TIME_TO_DRIVE = 1.0;
    private static final double SPEED_TO_DRIVE_ROTATIONAL_AXIS = 1.0;
    private static final double LINEAR_EQUATION_M_FACTOR = 0.4;
    private static final double LINEAR_EQUATION_B_FACTOR = 2.63;

    public ShootMyGuy(Drivetrain drivetrain, VisionService visionService, Shooter shooter) {
        addCommands(
                new Drive(drivetrain, () -> SPEED_TO_DRIVE_X_AXIS, () -> 0.0,
                        () -> 0.0, true, true).withTimeout(TIME_TO_DRIVE),
                new Drive(drivetrain, () -> 0.0, () -> 0.0, () -> SPEED_TO_DRIVE_ROTATIONAL_AXIS, true,
                        true).until(visionService::hasTarget),
                new RotateAccordingAprilTags(drivetrain, () -> 0.0, visionService, true),
                new ShootWithPID(shooter,
                        () -> (LINEAR_EQUATION_M_FACTOR * visionService.getX() + LINEAR_EQUATION_B_FACTOR))
        );
    }
}
