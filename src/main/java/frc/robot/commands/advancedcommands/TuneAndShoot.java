package frc.robot.commands.advancedcommands;

import com.spikes2212.dashboard.SpikesLogger;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shoot.ShootWithPID;
import frc.robot.commands.storage.Spin;
import frc.robot.commands.storage.Transport;
import frc.robot.commands.swerve.Drive;
import frc.robot.commands.swerve.RotateAccordingAprilTags;
import frc.robot.subsystems.shoot.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;

public class TuneAndShoot extends SequentialCommandGroup {

    private static final double LINEAR_EQUATION_M_FACTOR = -0.4;
    private static final double LINEAR_EQUATION_B_FACTOR = 2.63;
    public static final double DISTANCE_FROM_CAMERA_TO_SHOOTER = 0.5;

    private static final double FIRST_WAIT_TIME = 0.1;
    private static final double SECOND_WAIT_TIME = 10;

    private final SpikesLogger logger = new SpikesLogger();

    public TuneAndShoot(Shooter shooter, Kicker transport, SpinningMagazine spinningMagazine,
                        Drivetrain drivetrain, VisionService visionService) {
        addCommands(
                logger.logCommand("check"),
                new Drive(drivetrain, () -> 0.0, () -> 0.0, () -> 1.0, true, true)
                        .until(visionService::hasTarget),
                new RotateAccordingAprilTags(drivetrain, () -> 0.0, visionService, true),
                new ShootWithPID(shooter,
                        () -> {
                            if (visionService.hasTarget()) {
                                return -(LINEAR_EQUATION_M_FACTOR *
                                        (visionService.getZ() + DISTANCE_FROM_CAMERA_TO_SHOOTER) +
                                        LINEAR_EQUATION_B_FACTOR);
                            }
                            return 1.0;
                        },
                        FIRST_WAIT_TIME) {

                    @Override
                    public void end(boolean i) {}
                },
                logger.logCommand("Finished Acceleration"),
                new ParallelCommandGroup(
                        new Spin(spinningMagazine),
                        new Transport(transport),
                        new ShootWithPID(shooter,
                                () -> {
                                    if (visionService.hasTarget()) {
                                        return -(LINEAR_EQUATION_M_FACTOR *
                                                (visionService.getZ() + DISTANCE_FROM_CAMERA_TO_SHOOTER) +
                                                LINEAR_EQUATION_B_FACTOR);
                                    }
                                    return 1.0;
                                }, SECOND_WAIT_TIME) {

                            @Override
                            public boolean isFinished() {
                                return false;
                            }

                            @Override
                            public void end(boolean b) {
                                super.end(b);
                                logger.log("done");
                            }
                        }
                )
        );
    }
}
