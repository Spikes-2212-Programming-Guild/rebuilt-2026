package frc.robot.commands.advancedcommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shoot.ShootWithPID;
import frc.robot.commands.storage.SpinMagazine;
import frc.robot.commands.storage.Transport;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.utils.VisionService;

import java.util.function.Supplier;

public class ShootToHub extends SequentialCommandGroup {

    private static final double LINEAR_EQUATION_M_FACTOR = 0.636;
    private static final double LINEAR_EQUATION_B_FACTOR = 1.49;
    private static final double DISTANCE_FROM_CAMERA_TO_SHOOTER = 0;

    private static final double FIRST_WAIT_TIME = 0.5;
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
//                        DriverStation.reportError("helllo", false);
                        System.out.println("halo");
                    }
                },
                new ParallelCommandGroup(
                        new SpinMagazine(),
                        new Transport(kicker),
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
