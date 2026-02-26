package frc.robot.commands.difficultcommands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import frc.robot.com.spikes2212.command.drivetrains.swerve.commands.RotateModulesWithPID;

public class ModuleRotateWithPID extends RotateModulesWithPID {

    public ModuleRotateWithPID(SwerveDrivetrain drivetrain, Rotation2d frontLeftDesiredAngle,
                               Rotation2d frontRightDesiredAngle, Rotation2d backLeftDesiredAngle,
                               Rotation2d backRightDesiredAngle) {
        super(drivetrain, frontLeftDesiredAngle, frontRightDesiredAngle, backLeftDesiredAngle, backRightDesiredAngle);
    }
}
