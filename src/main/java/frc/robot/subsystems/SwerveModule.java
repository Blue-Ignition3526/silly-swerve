package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.team3526.constants.PIDFConstants;
import lib.team3526.constants.SwerveModuleOptions;
import static edu.wpi.first.units.Units.*;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {
    //* Options for the module
    public final SwerveModuleOptions options;

    //* Motors
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    //* Encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    //* PID Controller for turning
    private final SparkClosedLoopController turningPID;

    //* Absolute encoder
    private final CANcoder absoluteEncoder;

    //* Target state
    private SwerveModuleState targetState = new SwerveModuleState();

    /**
     * Create a new swerve module with the provided options
     * @param options
     */
    public SwerveModule(SwerveModuleOptions options) {
        // Store the options
        this.options = options;

        // * Drive motor
        this.driveMotor = new SparkMax(options.driveMotorID, MotorType.kBrushless);

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig
            .smartCurrentLimit(40)
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12);
        driveConfig.encoder
            .positionConversionFactor(Constants.SwerveDrive.PhysicalModel.kDriveEncoder_RotationToMeter)
            .velocityConversionFactor(Constants.SwerveDrive.PhysicalModel.kDriveEncoder_RPMToMeterPerSecond);

        this.driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // * Turn motor
        this.turningMotor = new SparkMax(options.turningMotorID, MotorType.kBrushless);

        SparkMaxConfig turningMotorConfig = new SparkMaxConfig();
        turningMotorConfig
            .smartCurrentLimit(20)
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12);
        turningMotorConfig.encoder
            .positionConversionFactor(Constants.SwerveDrive.PhysicalModel.kTurningEncoder_RotationToRadian)
            .velocityConversionFactor(Constants.SwerveDrive.PhysicalModel.kTurningEncoder_RPMToRadianPerSecond);
        turningMotorConfig.closedLoop
            .p(Constants.SwerveDrive.SwerveModules.kTurningPIDConstants.kP)
            .i(Constants.SwerveDrive.SwerveModules.kTurningPIDConstants.kI)
            .d(Constants.SwerveDrive.SwerveModules.kTurningPIDConstants.kD)
            .positionWrappingInputRange(0, 2 * Math.PI)
            .positionWrappingEnabled(true);

        this.turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        this.driveEncoder = driveMotor.getEncoder();
        this.turningEncoder = turningMotor.getEncoder();

        this.turningPID = turningMotor.getClosedLoopController();

        // Configure the absolute encoder
        this.absoluteEncoder = new CANcoder(options.absoluteEncoderDevice.getDeviceID(), options.absoluteEncoderDevice.getCanbus());
        this.absoluteEncoder.getAbsolutePosition().waitForUpdate(20, true);

        // Reset the encoders
        resetEncoders();
        
    }

    /**
     * Get the absolute encoder turn position
     * @return
     */
    public Angle getAbsoluteEncoderPosition() {
        return Radians.of((absoluteEncoder.getAbsolutePosition().refresh().getValue().in(Rotations) * 2 * Math.PI) * (this.options.absoluteEncoderInverted ? -1.0 : 1.0));
    }

    /**
     * Reset the drive encoder (set the position to 0)
     */
    public void resetDriveEncoder() {
        this.driveEncoder.setPosition(0);
    }

    /**
     * Reset the turning encoder (set the position to the absolute encoder's position)
     */
    public void resetTurningEncoder() {
        this.turningEncoder.setPosition(getAbsoluteEncoderPosition().in(Radians));
    }
    
    /**
     * Reset the drive and turning encoders
     */
    public void resetEncoders() {
        resetDriveEncoder();
        resetTurningEncoder();
    }

    /**
     * Get the current angle of the module
     * @return
     */
    public Angle getAngle() {
        return Radians.of(this.turningEncoder.getPosition() % (2 * Math.PI));
    }

    /**
     * Set the target state of the module
     * @param state The target state
     */
    public void setTargetState(SwerveModuleState state) {
        setTargetState(state, false);
    }

    /**
     * Set the target state of the module
     * @param state The target state
     * @param force If true, the module will ignore the current speed and turn to the target angle
     */
    public void setTargetState(SwerveModuleState state, boolean force) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001 || force) {
            stop();
            return;
        }

        state.optimize(Rotation2d.fromRadians(getAngle().in(Radians)));

        this.targetState = state;

        driveMotor.set(state.speedMetersPerSecond / Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond));
        turningPID.setReference(state.angle.getRadians(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    /**
     * Stop the module (set the speed of the motors to 0)
     */
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    /**
     * Get the target state of the module
     * @return
     */
    public SwerveModuleState getTargetState() {
        return this.targetState;
    }

    /**
     * Get the real state of the module
     * @return
     */
    public SwerveModuleState getRealState() {
        return new SwerveModuleState(
            this.driveEncoder.getVelocity(),
            Rotation2d.fromRadians(this.getAngle().in(Radians))
        );
    }

    /**
     * Get the position of the module
     * @return
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            this.driveEncoder.getPosition(),
            Rotation2d.fromRadians(this.getAngle().in(Radians))
        );
    }

    public void periodic() {
        Logger.recordOutput("SwerveDrive/" + this.options.name + "/MotEncoderDeg", this.getAngle().in(Degrees));
        Logger.recordOutput("SwerveDrive/" + this.options.name + "/AbsEncoderDeg", this.getAbsoluteEncoderPosition().in(Degrees));
        Logger.recordOutput("SwerveDrive/" + this.options.name + "/RealState", this.getRealState());
        Logger.recordOutput("SwerveDrive/" + this.options.name + "/TargetState", this.getTargetState());
    }
}
