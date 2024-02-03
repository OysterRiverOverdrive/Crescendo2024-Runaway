package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.RobotConstants;

public class FeederSubsystem extends SubsystemBase {

  private CANSparkMax m_leadMotor;
  private CANSparkMax m_followMotor;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final AnalogInput limitSwitch = new AnalogInput(0);
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color OrangeTarget = new Color(0.546, 0.363, 0.091);


public FeederSubsystem() {
        m_leadMotor = new CANSparkMax(4, MotorType.kBrushless);
        m_followMotor = new CANSparkMax(1,MotorType.kBrushless);
        m_followMotor.follow(m_leadMotor);
        m_colorMatcher.addColorMatch(OrangeTarget);
    }

public void InFeederCmd(){
    m_leadMotor.set(RobotConstants.FeederIn);
}

public void OutFeederCmd(){
    m_leadMotor.set(RobotConstants.FeederOut);
}

public void StopFeederCmd(){
  m_leadMotor.set(RobotConstants.StopFeeder);
}



    @Override
    public void periodic() {
 

    Color detectedColor = m_colorSensor.getColor();

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == OrangeTarget && match.confidence > 0.85 ){

        colorString = "Orange";
  
      } else {
  
        colorString = "Unknown";
  
      }

      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Green", detectedColor.green);
      SmartDashboard.putNumber("Blue", detectedColor.blue);
      SmartDashboard.putNumber("Confidence", match.confidence);
      SmartDashboard.putString("Detected Color", colorString);
  
      SmartDashboard.putNumber("Limit Switch", limitSwitch.getValue());
      boolean clicked;
      if (limitSwitch.getValue() <= 193) {
        clicked = true;
      } else {
        clicked = false;
      }
      SmartDashboard.putBoolean("Clicked", clicked);


  
    }
}



