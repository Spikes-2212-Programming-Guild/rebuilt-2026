package frc.robot.commands.difficultcommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.VisionDriveAlign;
import frc.robot.utils.vision.RobotRelativeVisionSource;

import java.util.function.Supplier;

public class VisionDriveAlignCommand extends Command {

    private final VisionDriveAlign visionUtil;
    private final PIDController controller;

    public VisionDriveAlignCommand(SwerveDrivetrain swerve, RobotRelativeVisionSource intakeCam, Supplier<Double> vX,
                                   Supplier<Double> vY) {
        this.visionUtil = new VisionDriveAlign(swerve, vX, vY);
        this.controller = visionUtil.getController();
    }

    @Override
    public void execute() {
        visionUtil.align(controller, false, false);
    }
}
