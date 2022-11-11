package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;

public class ControlBoard {
    Joystick joy = new Joystick(0);

    public Translation2d scale(double s, Translation2d tAxes) {
        return new Translation2d(tAxes.getX() * s, tAxes.getY() * s);
    }

    public static Translation2d fromPolar(Rotation2d direction, double magnitude){
    	return new Translation2d(direction.getCos() * magnitude, direction.getSin() * magnitude);
    }

    public double norm(Translation2d tAxes) {
        return Math.hypot(tAxes.getX(), tAxes.getY());
    }
    //6.62/6.71


    public double getSwerveRotation() {
        double rotAxis = joy.getRawAxis(4);

        if (Math.abs(rotAxis) < 0.15) {
            return 0.0;
        } else {
            return Settings.SwerveConstants.maxAngularVelocity * (rotAxis - (Math.signum(rotAxis) * 0.15)) / (1 - 0.15);
        }
    }

    public Translation2d getSwerveTranslation() {
        double forwardAxis = joy.getRawAxis(0);
        double strafeAxis = joy.getRawAxis(1);

        // System.out.println(forwardAxis);

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);
        // System.out.println(tAxes);

        if (Math.abs(norm(tAxes)) < 0.15) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.getX(), tAxes.getY());
            Translation2d deadband_vector = fromPolar(deadband_direction, 0.15);

            // System.out.println(deadband_direction);

            double scaled_x = tAxes.getX() - (deadband_vector.getX()) / (1 - deadband_vector.getX());
            double scaled_y = tAxes.getY() - (deadband_vector.getY()) / (1 - deadband_vector.getY());
            return scale(Settings.SwerveConstants.maxSpeed, new Translation2d(scaled_x, scaled_y));
        }
    }
}
