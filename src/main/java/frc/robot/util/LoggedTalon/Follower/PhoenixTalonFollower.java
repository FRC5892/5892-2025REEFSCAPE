// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.LoggedTalon.Follower;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

public record PhoenixTalonFollower(int canid, MotorAlignmentValue opposeDirection) {}
