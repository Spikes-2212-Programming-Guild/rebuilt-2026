package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShootWithPID;
import frc.robot.commands.spindexer.SpinMagazine;
import frc.robot.commands.spindexer.SpinKicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.utils.VisionService;

public class ShootToHub extends SequentialCommandGroup {

    private static final double LINEAR_EQUATION_M_FACTOR = 0.4;
    private static final double LINEAR_EQUATION_B_FACTOR = 2.63;
    private static final double DISTANCE_FROM_CAMERA_TO_SHOOTER = 0.5;

    private static final double FIRST_WAIT_TIME = 0.1;
    private static final double SECOND_WAIT_TIME = 10;

    public ShootToHub(Shooter shooter, Kicker kicker, SpinningMagazine spinningMagazine,
                      VisionService visionService) {
        addCommands(
                new ShootWithPID(shooter,
                        () -> (LINEAR_EQUATION_M_FACTOR * (visionService.getZ() + DISTANCE_FROM_CAMERA_TO_SHOOTER) +
                                LINEAR_EQUATION_B_FACTOR),
                        FIRST_WAIT_TIME) {

                    @Override
                    public void end(boolean i) {
                    }
                },
                new ParallelCommandGroup(
                        new SpinMagazine(spinningMagazine),
                        new SpinKicker(kicker),
                        new ShootWithPID(shooter,
                                () -> (LINEAR_EQUATION_M_FACTOR *
                                        (visionService.getZ() + DISTANCE_FROM_CAMERA_TO_SHOOTER) +
                                        LINEAR_EQUATION_B_FACTOR), SECOND_WAIT_TIME) {

                            @Override
                            public boolean isFinished() {
                                return false;
                            }

                        }
                )
        );
    }
}
