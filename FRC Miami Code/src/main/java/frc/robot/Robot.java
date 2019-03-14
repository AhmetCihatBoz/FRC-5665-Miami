//|  Bu kod FRC 2019 Deep Space, Miami Regional'da Sparc 5665 tarafından kullanılması üzere, takım programcıları tarafından yazılmıştır.|
//|  Bu kod Sparc 5665 tarafından tüm takımların faydalanması adına açık kaynak kodlu yazılmıştır ve değiştirmeye-düzenlemeye açıktır.  |
//|  Sparc 5665, tüm takımlara her konuda destek vermeye hazırdır ve tüm takımlara başarılar diler.                                     |
//|  -Sparc 5665 Programlama Ekibi-                                                ahmetcihatboz@gmail.com / ahmetcihatboz@hotmail.com  | 
//|___________________________________________________________________________________________Sparc 5665 - Ahmet Cihat BOZ______________|

package frc.robot; //IterativeRobot standart template kodu.
import edu.wpi.cscore.UsbCamera;  //Gerekli kütüphaneleri çağırdık.
import edu.wpi.first.wpilibj.*; //Gerekli kütüphaneleri çağırdık.
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;  //Gerekli kütüphaneleri çağırdık.
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //Gerekli kütüphaneleri çağırdık.
import com.ctre.phoenix.*;  //Gerekli kütüphaneleri çağırdık.
import com.ctre.phoenix.motorcontrol.FeedbackDevice;  //Gerekli kütüphaneleri çağırdık.
import com.ctre.phoenix.motorcontrol.can.TalonSRX; //Gerekli kütüphaneleri çağırdık.
//ŞU ANDA KULLANILMIYOR!! import edu.wpi.first.wpilibj.AnalogGyro;  //Gerekli kütüphaneleri çağırdık.
//ŞU ANDA KULLANILMIYOR!! import edu.wpi.first.wpilibj.ADXRS450_Gyro; //Gerekli kütüphaneleri çağırdık.
import edu.wpi.first.wpilibj.SPI; //Gerekli kütüphaneleri çağırdık.
import edu.wpi.first.wpilibj.command.Command; //Gerekli kütüphaneleri çağırdık.
import edu.wpi.first.wpilibj.command.Scheduler; //Gerekli kütüphaneleri çağırdık.
public class Robot extends IterativeRobot {
  private static final String kDefaultAuto = "Default"; //IterativeRobot standart template kodu.
  private static final String kCustomAuto = "My Auto";  //IterativeRobot standart template kodu.
  private String m_autoSelected;  //IterativeRobot standart template kodu.
  private final SendableChooser<String> m_chooser = new SendableChooser<>();  //IterativeRobot standart template kodu.
  /******************************************************************************************************************************************************/
  TalonSRX _talon1 = new TalonSRX(1);   //Şase sürüş sisteminin motorlarını (sürücülerini) tanıttık. Parantezde yazan sayılar sürücülerin CAN ID'leri.  (CAN ID'leri Phoenix Software'den değiştirilebilir.)
	TalonSRX _talon2 = new TalonSRX(8);   //Şase sürüş sisteminin motorlarını (sürücülerini) tanıttık. Parantezde yazan sayılar sürücülerin CAN ID'leri.  (CAN ID'leri Phoenix Software'den değiştirilebilir.)
	TalonSRX _talon3 = new TalonSRX(9);   //Şase sürüş sisteminin motorlarını (sürücülerini) tanıttık. Parantezde yazan sayılar sürücülerin CAN ID'leri.  (CAN ID'leri Phoenix Software'den değiştirilebilir.)
	TalonSRX _talon4 = new TalonSRX(10);  //Şase sürüş sisteminin motorlarını (sürücülerini) tanıttık. Parantezde yazan sayılar sürücülerin CAN ID'leri.  (CAN ID'leri Phoenix Software'den değiştirilebilir.)
	//VictorSP _victor1 = new VictorSP(0);  //Şimdilik boş.
	//VictorSP _victor2 = new VictorSP(1);  //Şimdilik boş.
	//VictorSP _victor3 = new VictorSP(2);  //Şimdilik boş.
	//VictorSP _victor4 = new VictorSP(3);  //Şimdilik boş.
	Spark _sparkYukselme = new Spark(4); //Yükselme sisteminin motor sürücüsü. (Parantez içindeki değer, sürücünün roboRIO üzerindeki hangi pwm kanalına takılı olduğunu gösteriyor.)
	Spark _sparkTopAlma = new Spark(5); //Top alma sisteminin motor sürücüsü. (Parantez içindeki değer, sürücünün roboRIO üzerindeki hangi pwm kanalına takılı olduğunu gösteriyor.)
	Spark _sparkDisk = new Spark(6); //Disk sisteminin motor sürücüsü.                         		
	Joystick _joyStick = new Joystick(0);	//Kumandamızı (joystick) tanımladık, eğer 2 kumanda kullanılacaksa iki kumanda tanımlanmalı, ikinicisinin parantezine 1 yazılmalı.
  DoubleSolenoid deneme = new DoubleSolenoid(0,1);  //Pnömatik sistemde kullanmak üzere iki yönlü pnömatik valfimizi (solenoid) tanımladık. Parantez içindeki 1 ve 0, valflerin tetik kablolarının PCM'de ki 0 ve 1 numaralı porta bağlı olduğunu gösteriyor.
  //ŞU ANDA KULLANILMIYOR!! private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0; //RoboRIO üzerindeki SPI portunu tanımladık. (Gyro için)
  //ŞU ANDA KULLANILMIYOR!! private static final double kVoltsPerDegreePerSecond = 0.0128;  //Gyro için gerekli olabilecek bir ayar (sensitivity), denedikten sonra ihtiyaç yoksa sil.
  //ŞU ANDA KULLANILMIYOR!! private ADXRS450_Gyro _gyro = new ADXRS450_Gyro(kGyroPort); //Önceden belirlediğimiz portta yeni bir Gyro tanımladık.
  public static Vision vision;  //Görüntü işlemede kullanmak üzere vision'u tanıttık.
  /******************************************************************************************************************************************************/
  StringBuilder _sb = new StringBuilder();  //TalonSRX ile PID kontrolde kullanmak üzere hazır bulunan kodlar. (İleri seviye)
  int _loops = 0; //TalonSRX ile PID kontrolde kullanmak üzere hazır bulunan kodlar. (İleri seviye) 
  boolean _lastButton1 = false; //TalonSRX ile PID kontrolde kullanmak üzere hazır bulunan kodlar. (İleri seviye) 																						
	double targetPositionRotations;	//TalonSRX ile PID kontrolde kullanmak üzere hazır bulunan kodlar. (İleri seviye)
  
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto); //IterativeRobot standart template kodu.
    m_chooser.addOption("My Auto", kCustomAuto);  //IterativeRobot standart template kodu.
    SmartDashboard.putData("Auto choices", m_chooser);  //IterativeRobot standart template kodu.
    /******************************************************************************************************************************************************/
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(); //Kameradan görüntü alabilmek için kamerayı tanımladık.
    camera.setResolution(640, 480); //Kameradan gelen görüntünün gecikmesini azaltmak amacıyla çözünürlüğünü kıstık.
    //ŞU ANDA KULLANILMIYOR!! _gyro.calibrate();  //Gyro'muzu robotu her ilk enable ettiğimizde bir defa olacak şekilde kalibre ediyoruz. KALİBRE EDİLİRKEN ROBOT HAREKET ETTİRİLMEMELİDİR. (İŞLEM 1 SANİYE SÜRER)
    /******************************************************************************************************************************************************/
    _talon1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,Constants.kPIDLoopIdx,Constants.kTimeoutMs);  //Buradan aşağısı TalonSRX ile PID kontrolde kullanmak üzere hazır bulunan kodlar. (İleri seviye)						  
    _talon2.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,Constants.kPIDLoopIdx,Constants.kTimeoutMs);							 
    _talon3.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,Constants.kPIDLoopIdx,Constants.kTimeoutMs);							
    _talon4.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,Constants.kPIDLoopIdx,Constants.kTimeoutMs);					
    _talon1.setSensorPhase(Constants.kSensorPhase);																			
    _talon2.setSensorPhase(Constants.kSensorPhase);																								
    _talon3.setSensorPhase(Constants.kSensorPhase);																								
    _talon4.setSensorPhase(Constants.kSensorPhase);																								
    _talon1.setInverted(Constants.kMotorInvert);																											
    _talon2.setInverted(Constants.kMotorInvert);
    _talon3.setInverted(Constants.kMotorInvert);
    _talon4.setInverted(Constants.kMotorInvert);
    _talon1.configNominalOutputForward(0, Constants.kTimeoutMs);
    _talon1.configNominalOutputReverse(0, Constants.kTimeoutMs);
    _talon1.configPeakOutputForward(1, Constants.kTimeoutMs);
    _talon1.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    _talon2.configNominalOutputForward(0, Constants.kTimeoutMs);
    _talon2.configNominalOutputReverse(0, Constants.kTimeoutMs);
    _talon2.configPeakOutputForward(1, Constants.kTimeoutMs);
    _talon2.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    _talon3.configNominalOutputForward(0, Constants.kTimeoutMs);
    _talon3.configNominalOutputReverse(0, Constants.kTimeoutMs);
    _talon3.configPeakOutputForward(1, Constants.kTimeoutMs);
    _talon3.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    _talon4.configNominalOutputForward(0, Constants.kTimeoutMs);
    _talon4.configNominalOutputReverse(0, Constants.kTimeoutMs);
    _talon4.configPeakOutputForward(1, Constants.kTimeoutMs);
    _talon4.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    _talon1.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    _talon2.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    _talon3.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    _talon4.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    _talon1.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    _talon1.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    _talon1.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    _talon1.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
    _talon2.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    _talon2.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    _talon2.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    _talon2.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
    _talon3.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    _talon3.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    _talon3.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    _talon3.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
    _talon4.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    _talon4.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    _talon4.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    _talon4.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
    int absolutePosition1 = _talon1.getSensorCollection().getPulseWidthPosition();
    int absolutePosition2 = _talon2.getSensorCollection().getPulseWidthPosition();
    int absolutePosition3 = _talon3.getSensorCollection().getPulseWidthPosition();
    int absolutePosition4 = _talon4.getSensorCollection().getPulseWidthPosition();
    absolutePosition1 &= 0xFFF;
    if (Constants.kSensorPhase) { absolutePosition1 *= -1; }
    if (Constants.kMotorInvert) { absolutePosition1 *= -1; }		
    _talon1.setSelectedSensorPosition(absolutePosition1, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    _talon2.setSelectedSensorPosition(absolutePosition2, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    _talon3.setSelectedSensorPosition(absolutePosition3, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    _talon4.setSelectedSensorPosition(absolutePosition4, Constants.kPIDLoopIdx, Constants.kTimeoutMs); //Buradan yukarısı TalonSRX ile PID kontrolde kullanmak üzere hazır bulunan kodlar. (İleri seviye)				
}
  
  @Override
  public void robotPeriodic() {
  }
  
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();  //IterativeRobot standart template kodu.
    System.out.println("Auto selected: " + m_autoSelected); //IterativeRobot standart template kodu.
    /******************************************************************************************************************************************************/
  }
  
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic(); //Otonom periyodunda robotu teleop modunda kullanabilmek için teleop fonksiyonunu çağırdık.
    Scheduler.getInstance().run();  //Scheduler'i başlatıyoruz.
  }
  
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();  //Scheduler'i başlatıyoruz.		
		vision.testPixy1(); //Pixy test fonksiyonunu, seri haberleşme ekranından Pixy değerlerini okumak için çağırıyoruz.
    //ŞU ANDA KULLANILMIYOR!! System.out.println("Açı:  "+_gyro.getAngle());  //Gyro'dan okunan değerler seri haberleşme ekranında yazdırılıyor.
  }
  
  @Override
  public void testPeriodic() {
  }
}

//___________________________________________________________Faydalı Kaynaklar________________________________________________________
//Pixy ► Bilgilendirme: https://www.chiefdelphi.com/t/pixy-with-roborio/158939/2
//Pixy ► Kullanıcı tarafından yazılmış kütüphane: https://github.com/Team5593/pixy
//Pixy ► Pixy-roborio: https://www.chiefdelphi.com/t/pixy-i2c/141776/8
//Pixy ► SPI java: https://github.com/croadfeldt/wpilib_pixy_spi_java
//PID  ► Pid kontrol: https://frc-pdr.readthedocs.io/en/latest/control/pid_control.html
//Gyro ► Gyro bilgilendirme1: https://wpilib.screenstepslive.com/s/currentCS/m/java/l/599713-gyros-measuring-rotation-and-controlling-robot-driving-direction
//Gyro ► Gyro bilgilendirme2: https://www.chiefdelphi.com/t/integrating-gyro-code/118147/3
//Gyro ► Gyro bilgilendirme3 - PID ile Drive Straingt kod örneği: https://wiki.analog.com/first/adxrs450_gyro_board_frc/java
//____________________________________________________________________________________________________________________________________



//___________________________________________________________Notlar___________________________________________________________________
//Not1: RoboRIO ile encoder okunmak istendiğinde, roboRIO'nun sayma hızı yetersiz kalıyor. Yapılan iki denemede motor şaftı aynı açıda dönmesine rağmen
//okunan pals sayısı eşit olmuyor, bu sebeple sağlıksız ve yetersiz bir sayım gerçekleştiriliyor. Bunu önlemek için, her encoder'a sadece sayma işlemini
//gerçekleştirecek bir mikrodenetleyici bağlamak, sayma işlemini bu mikrodenetleyici ile gerçekleştirmek ve bu mikrodenetleyiciyi roboRIO ile haberleştirerek 
//mikrodenetleyicinin saydığı sayıyı roboRIO üzerinden dijital olarak okumak gerekiyor. Tercihen arduino veya raspberryPi kullanılabilir.
//Biz kendi robotumuzda encoderleri saymak için mikrodenetleyici kullanmak yerine talonSRX'lerin encoder 
//sayma ve PID döngüsü gerçekleştirme yeteneğinden faydalanıyoruz.

//Not2: Normalde Gyro'yu robotu dümdüz şekilde, sağa sola sapma olmadan sürmek ve hassas eksenel dönüşler elde etmek için kullanırız. Fakat bizim
//robotumuzda tekerleklerde encoderler olduğundan ve gerektiğinde tekerleklerin hareketini PID döngüsü ile kontrol ettiğimizden Gyro'nun hayati önemi yok.
//RoboRIO üzerinde yalnızca bir adet SPI haberleşme portu bulunduğundan ve bu portta Pixy takılı olduğundan Gyro kullanmıyoruz.
//____________________________________________________________________________________________________________________________________