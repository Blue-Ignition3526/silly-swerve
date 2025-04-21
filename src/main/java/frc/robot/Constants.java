package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import lib.team3526.constants.CTRECANDevice;
import lib.team3526.constants.PIDFConstants;
import lib.team3526.constants.SwerveModuleOptions;
import lib.team3526.utils.SwerveChassis;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;

public class Constants {
    //* Logging options
    public static final class Logging {
        public static final boolean kDebug = true;
    }

    //* Swerve Drive
    public static final class SwerveDrive {
        //* Gyroscope (Pigeon 2.0)
        public static final CTRECANDevice kGyroDevice = new CTRECANDevice(34);

        public static final double kJoystickDeadband = 0.1;
        //* Physical model of the robot
        public static final class PhysicalModel {
            //* MAX DISPLACEMENT SPEED (and acceleration)
            public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(5);
            public static final LinearAcceleration kMaxAcceleration = MetersPerSecond.per(Second).of(kMaxSpeed.in(MetersPerSecond));

            //* MAX ROTATIONAL SPEED (and acceleration)
            public static final AngularVelocity kMaxAngularSpeed = DegreesPerSecond.of(360);
            public static final AngularAcceleration kMaxAngularAcceleration = DegreesPerSecond.per(Second).of(kMaxAngularSpeed.in(DegreesPerSecond));

            // Drive wheel diameter
            public static final Distance kWheelDiameter = Inches.of(4);

            // Gear ratios
            public static final double kDriveMotorGearRatio = 1.0 / 6.12; // 6.12:1 Drive
            public static final double kTurningMotorGearRatio = 1.0 / 12.8; // 12.8:1 Steering

            // Conversion factors (Drive Motor)
            public static final double kDriveEncoder_RotationToMeter = kDriveMotorGearRatio * kWheelDiameter.in(Meters) * 2 * Math.PI;
            public static final double kDriveEncoder_RPMToMeterPerSecond = kDriveEncoder_RotationToMeter / 60.0;

            // Conversion factors (Turning Motor)
            public static final double kTurningEncoder_RotationToRadian = kTurningMotorGearRatio * 2.0 * Math.PI;
            public static final double kTurningEncoder_RPMToRadianPerSecond = kTurningEncoder_RotationToRadian / 60.0;

            // Robot Without bumpers measures
            public static final Distance kTrackWidth = Inches.of(23.08);
            public static final Distance kWheelBase = Inches.of(22.64);
    
            // Create a kinematics instance with the positions of the swerve modules
            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(SwerveChassis.sizeToModulePositions(kTrackWidth.in(Meters), kWheelBase.in(Meters)));

            // Rotation lock PIDF Constants
            public static final PIDFConstants kHeadingControllerPIDConstants = new PIDFConstants(0.1, 0.0, 0.0);

            // Physical model constants
            public static final double kRobotMassKg = 46;
        }

        //* Swerve modules configuration
        public static final class SwerveModules {
            //* PID
            public static final PIDFConstants kTurningPIDConstants = new PIDFConstants(0.5);

            //* Global offset
            public static final Angle kGlobalOffset = Degrees.of(0);

            //* Swerve modules options
            public static final SwerveModuleOptions kFrontLeftOptions = new SwerveModuleOptions()
                .setDriveMotorID(2)
                .setTurningMotorID(3)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(4))
                .setName("Front Left");

            public static final SwerveModuleOptions kFrontRightOptions = new SwerveModuleOptions()
                .setDriveMotorID(5)
                .setTurningMotorID(6)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(7))
                .setName("Front Right");

            public static final SwerveModuleOptions kBackLeftOptions = new SwerveModuleOptions()
                .setDriveMotorID(8)
                .setTurningMotorID(9)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(10))
                .setName("Back Left");

            public static final SwerveModuleOptions kBackRightOptions = new SwerveModuleOptions()
                .setDriveMotorID(11)
                .setTurningMotorID(12)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(13))
                .setName("Back Right");
        }

        //* AUTONOMOUS
        public static final class Autonomous {
            public static final PIDConstants kTranslatePIDConstants = new PIDConstants(5.0, 0.0, 0.0);
            public static final PIDConstants kRotatePIDConstants = new PIDConstants(5.0, 0.0, 0.0);
            public static final LinearVelocity kMaxSpeedMetersPerSecond = MetersPerSecond.of(1);
        }
    }
}
