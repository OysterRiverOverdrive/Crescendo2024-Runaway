package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class FeederSubsystem extends SubsystemBase {
  // Creating Both Motors
  private CANSparkMax m_feedLeftMotor =
      new CANSparkMax(RobotConstants.FeederLeftCanId, MotorType.kBrushless);
  private CANSparkMax m_feedRightMotor =
      new CANSparkMax(RobotConstants.FeederRightCanId, MotorType.kBrushless);

  // Create color sensor and limit switch
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final AnalogInput limitSwitch = new AnalogInput(0);
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  // Establish Color Matches
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color OrangeTarget = new Color(0.546, 0.363, 0.091);

  public FeederSubsystem() {
    m_feedRightMotor.follow(m_feedLeftMotor, true);

    m_colorMatcher.addColorMatch(OrangeTarget);
  }

  public void InFeederCmd() {
    m_feedLeftMotor.set(RobotConstants.FeederInSpeed);
  }

  public void OutFeederCmd() {
    m_feedLeftMotor.set(RobotConstants.FeederOutSpeed);
  }

  public void StopFeederCmd() {
    m_feedLeftMotor.stopMotor();
  }

  public void ToShooterCmd() {
    m_feedLeftMotor.set(RobotConstants.FeederToShooterSpeed);
  }

  public Boolean getColorSensor() {

    Color detectedColor = m_colorSensor.getColor();

    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == OrangeTarget && match.confidence > 0.85) {

      return true;

    } else {

      return false;
    }
  }

  public Boolean getLimitSwitch() {

    if (limitSwitch.getValue() <= RobotConstants.LimitSwtichActivation) {

      return true;

    } else {

      return false;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note Detected (Color)", getColorSensor());
    // Used for Limit Switch when on Robot
    // SmartDashboard.putBoolean("Note Detected (Limit)", getLimitSwitch());
    // SmartDashboard.putNumber("Limit Switch Tuning", limitSwitch.getValue());
  }
}
