package frc.robot.util.odometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record StandardDeviations(double translationX, double translationY, double rotation) {

    public static StandardDeviations EMPTY_STANDARD_DEVIATIONS
            = new StandardDeviations(0,0,0);

    public Matrix<N3, N1> toMatrix() {
        return VecBuilder.fill(
                translationX,
                translationY,
                rotation
        );
    }
}
