#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
import wpimath.units
import rev

kWheelRadius = wpimath.units.inchesToMeters(4)
kEncoderResolution = 4096
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau


class SwerveModule:
    def __init__(
        self,
        driveMotorId: int,
        steerMotorId: int,
    ) -> None:
        """Constructs a SwerveModule with a drive motor, steer motor, drive encoder and steer encoder.

        :param driveMotorId:    CAN id for the drive motor.
        :param steerMotorId:    CAN id for the steer motor.
        """
        self.driveMotor = rev.CANSparkMax(driveMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.steerMotor = rev.CANSparkMax(steerMotorId, rev.CANSparkLowLevel.MotorType.kBrushless)

        self.driveEncoder = self.driveMotor.getEncoder()
        self.steerEncoder = self.steerMotor.getEncoder()

        # Gains are for example purposes only - must be determined for your own robot!
        self.drivePIDController = wpimath.controller.PIDController(1, 0, 0)

        # Gains are for example purposes only - must be determined for your own robot!
        self.steerPIDController = wpimath.controller.ProfiledPIDController(
            1,
            0,
            0,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity,
                kModuleMaxAngularAcceleration,
            ),
        )

        # Gains are for example purposes only - must be determined for your own robot!
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0.5)

        # Set the distance per pulse for the drive encoder. We can simply use the
        # distance traveled for one rotation of the wheel divided by the encoder
        # resolution.
        self.driveEncoder.setPositionConversionFactor(
            math.tau * kWheelRadius / 7.13
        )

        # Set the distance (in this case, angle) in radians per pulse for the steer encoder.
        # This is the the angle through an entire rotation (2 * pi) divided by the
        # encoder resolution.
        self.steerEncoder.setPositionConversionFactor(math.tau / 11.3143)

        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.steerPIDController.enableContinuousInput(-math.pi, math.pi)

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getVelocity(),
            wpimath.geometry.Rotation2d(self.steerEncoder.getPosition()),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getVelocity(),
            wpimath.geometry.Rotation2d(self.steerEncoder.getPosition()),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        encoderRotation = wpimath.geometry.Rotation2d(self.steerEncoder.getVelocity())

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        state.speed *= (state.angle - encoderRotation).cos()

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getVelocity(), state.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(state.speed)

        # Calculate the steer motor output from the steer PID controller.
        turnOutput = self.steerPIDController.calculate(
            self.steerEncoder.getPosition(), state.angle.radians()
        )

        turnFeedforward = self.turnFeedforward.calculate(
            self.steerPIDController.getSetpoint().velocity
        )

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.steerMotor.setVoltage(turnOutput + turnFeedforward)