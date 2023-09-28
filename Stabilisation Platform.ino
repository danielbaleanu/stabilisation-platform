#include <Wire.h>
#include <L3G4200D.h>
#include <DFRobot_LIS2DW12.h>
#include <Servo.h>

DFRobot_LIS2DW12_I2C accel;
L3G4200D gyro;

Servo Servo1;  // Crearea obiectului servo1 pentru controlul primului servomotor
Servo Servo2;  // Crearea obiectului servo2 pentru controlul celui de-al doilea servomotor

Servo Servo3;  // Crearea obiectului servo3 pentru controlul celui de-al treilea servomotor
Servo Servo4;  // Crearea obiectului servo2 pentru controlul celui de-al patrulea servomotor

#define GYROSCOPE_SENSITIVITY 70.0        // LSB/grade pe secundă
#define ACCELEROMETER_SENSITIVITY 8192.0  // LSB/g

bool SerialPrint = false;
bool function_running = true;

static bool instruction_executed = false;

const int ledPin1 = 2;
const int ledPin2 = 4;
const int ledPin3 = 7;
const int ledPin4 = 8;

float XZeroCorrection = 8;
float YZeroCorrection = 4;

float XFactorCorrectionPositive = 1.0;
float XFactorCorrectionNegative = 1.015;
float YFactorCorrectionPositive = 1.065;
float YFactorCorrectionNegative = 1.065;

float XAngleScale = -90.0;
float YAngleScale = -90.0;

int blinkCount = 0;

const long interval = 100000;

unsigned long currentMicros = 0;
unsigned long previousMicrosGyro = 0;
unsigned long previousMicrosLEDs = 0;
unsigned long previousMicrosServo = 0;

// Inițializarea variabilelor
float gyro_x, gyro_y, gyro_z;     // Citirile giroscopului
float accel_x, accel_y, accel_z;  // Citirile accelerometrului
float roll, pitch;                // Unghiurile de orientare (ruliu și tangaj)
float x_avg = 0.0, y_avg = 0.0;   // Mediile ultimelor cinci citiri ale accelerometrului
float alpha = 0.2;                // Coeficientul alfa pentru algoritmul "sensor fusion" (filtru complementar)

int MappedPosServo1;
int MappedPosServo2;
int MappedPosServo3;
int MappedPosServo4;

void TurnOnLEDs(const int& ledPin1,
                const int& ledPin2,
                const int& ledPin3,
                const int& ledPin4);

void TurnOffLEDs(const int& ledPin1,
                 const int& ledPin2,
                 const int& ledPin3,
                 const int& ledPin4);

void QuickBlinkLEDs(unsigned long& currentMicros,
                    unsigned long& previousMicrosLEDs,
                    const long& interval,
                    const int& ledPin1,
                    const int& ledPin2,
                    const int& ledPin3,
                    const int& ledPin4,
                    int& blinkCount,
                    bool& function_running);

void SerialPrintFcn(bool& SerialPrint,
                    float& roll,
                    float& pitch);

void setup() {
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);

  TurnOnLEDs(ledPin1,
             ledPin2,
             ledPin3,
             ledPin4);

  if (SerialPrint) {
    Serial.begin(115200);
  }

  while (!accel.begin()) {
    delay(1000);
  }

  TurnOffLEDs(ledPin1,
              ledPin2,
              ledPin3,
              ledPin4);

  accel.getID();

  // Resetarea chip-ului (Soft Reset)
  accel.softReset();

  // Setăm dacă dorim să achiziționăm date în mod continuu
  // (True -> dorim/False -> nu dorim)
  accel.continRefresh(true);

  /**！
     Setări pentru rata de achiziție de date de la senzor:
        eRate_0hz         		         /<Măsurătoare oprită>/
        eRate_1hz6        		         /<1.6hz, folosiți numai în modul Low-Power>/
        eRate_12hz5       		         /<12.5hz>/
        eRate_25hz          
        eRate_50hz          
        eRate_100hz         
        eRate_200hz         
        eRate_400hz       		         /<Folosiți numai în modul High-Performance>/
        eRate_800hz       		         /<Folosiți numai în modul High-Performance>/
        eRate_1k6hz       		         /<Folosiți numai în modul High-Performance>/
        eSetSwTrig        		         /<Software-ul declanșează o singură măsurătoare>/
  */
  accel.setDataRate(DFRobot_LIS2DW12::eRate_1k6hz);

  /**！
     Setări pentru intervalul de măsurare al accelerației:
        e2_g   			                   /<±2g>/
        e4_g   			                   /<±4g>/
        e8_g   			                   /<±8g>/
        e16_g  			                   /<±16g>/
  */
  accel.setRange(DFRobot_LIS2DW12::e2_g);

  /**！
     Setări de filtrare:
        eLPF 				                   (Filtru trece-jos)
        eHPF 				                   (Filtru trece-sus)
  */
  accel.setFilterPath(DFRobot_LIS2DW12::eLPF);

  /**！
     Setări lungime de bandă：
        eRateDiv_2  		               /<Frecvența/2 (până la Frecvență = 800 Hz, când Frecvența = 1600 Hz)>/
        eRateDiv_4                     /<Frecvența/4 (High-Power/Low-Power)>*
        eRateDiv_10                    /<Frecvența/10 (FTS/FTJ)>/
        eRateDiv_20                    /<Frecvența/20 (FTS/FTJ)>/
  */
  accel.setFilterBandwidth(DFRobot_LIS2DW12::eRateDiv_2);

  /**！
     Setări mod putere:
        eHighPerformance_14bit         /<Mod High-Performance, 14-bit resolution>/
        eContLowPwr4_14bit             /<Măsurătoare continuă, Mod Low-Power 4 (rezoluție 14-biți)>/
        eContLowPwr3_14bit             /<Măsurătoare continuă, Mod Low-Power 3 (rezoluție 14-biți)>/
        eContLowPwr2_14bit             /<Măsurătoare continuă, Mod Low-Power 2 (rezoluție 14-biți)>/
        eContLowPwr1_12bit             /<Măsurătoare continuă, Mod Low-Power 1 (rezoluție 12-biți)>/
        eSingleLowPwr4_14bit           /<Mod conversie singulară de date la cerere, Mod Low-Power 4 (rezoluție 14-biți)>/
        eSingleLowPwr3_14bit           /<Mod conversie singulară de date la cerere, Mod Low-Power 3 (rezoluție 14-biți)>/
        eSingleLowPwr2_14bit           /<Mod conversie singulară de date la cerere, Mod Low-Power 2 (rezoluție 14-biți)>/
        eSingleLowPwr1_12bit           /<Mod conversie singulară de date la cerere, Mod Low-Power 1 (rezoluție 12-biți)>/
        eHighPerformanceLowNoise_14bit /<Mod High-Performance, Activare reducere zgomot (rezoluție 14-biți)>/
        eContLowPwrLowNoise4_14bit     /<Măsurătoare continuă, Mod Low-Power 4, Activare reducere zgomot (rezoluție 14-biți)>/
        eContLowPwrLowNoise3_14bit     /<Măsurătoare continuă, Mod Low-Power 3, Activare reducere zgomot (rezoluție 14-biți)>/
        eContLowPwrLowNoise2_14bit     /<Măsurătoare continuă, Mod Low-Power 2, Activare reducere zgomot (rezoluție 14-biți)>/
        eContLowPwrLowNoise1_12bit     /<Măsurătoare continuă, Mod Low-Power 1, Activare reducere zgomot (rezoluție 12-biți)>/
        eSingleLowPwrLowNoise4_14bit   /<Mod conversie singulară de date la cerere, Mod Low-Power 4, Activare rducere zgomot (rezoluție 14-biți)>/
        eSingleLowPwrLowNoise3_14bit   /<Mod conversie singulară de date la cerere, Mod Low-Power 3, Activare rducere zgomot (rezoluție 14-biți)>/
        eSingleLowPwrLowNoise2_14bit   /<Mod conversie singulară de date la cerere, Mod Low-Power 2, Activare rducere zgomot (rezoluție 14-biți)>/
        eSingleLowPwrLowNoise1_12bit   /<Mod conversie singulară de date la cerere, Mod Low-Power 1, Activare rducere zgomot (rezoluție 12-biți)>/
  */
  accel.setPowerMode(DFRobot_LIS2DW12::eHighPerformanceLowNoise_14bit);



  delay(1000);

  TurnOnLEDs(ledPin1,
             ledPin2,
             ledPin3,
             ledPin4);

  // Inițializare L3G4200D
  // (gps -> grade pe secundă, fidlb -> frecvența de ieșire a datelor și lățimea de bandă)

  // gps:
  // L3G4200D_SCALE_250DPS:   	   200 gps
  // L3G4200D_SCALE_500DPS:   	   500 gps
  // L3G4200D_SCALE_2000DPS: 	     2000 gps (prestabilit)

  // fidlb:
  // L3G4200D_DATARATE_800HZ_50:   Frecvența de Ieșire a Datelor 800HZ, Tăiere 50
  // L3G4200D_DATARATE_800HZ_35:   Frecvența de Ieșire a Datelor 800HZ, Tăiere 35
  // L3G4200D_DATARATE_800HZ_30:   Frecvența de Ieșire a Datelor 800HZ, Tăiere 30
  // L3G4200D_DATARATE_400HZ_110:  Frecvența de Ieșire a Datelor 400HZ, Tăiere 110
  // L3G4200D_DATARATE_400HZ_50:   Frecvența de Ieșire a Datelor 400HZ, Tăiere 50
  // L3G4200D_DATARATE_400HZ_25:   Frecvența de Ieșire a Datelor 400HZ, Tăiere 25
  // L3G4200D_DATARATE_400HZ_20:   Frecvența de Ieșire a Datelor 400HZ, Tăiere 20
  // L3G4200D_DATARATE_200HZ_70:   Frecvența de Ieșire a Datelor 200HZ, Tăiere 70
  // L3G4200D_DATARATE_200HZ_50:   Frecvența de Ieșire a Datelor 200HZ, Tăiere 50
  // L3G4200D_DATARATE_200HZ_25:   Frecvența de Ieșire a Datelor 200HZ, Tăiere 25
  // L3G4200D_DATARATE_200HZ_12_5: Frecvența de Ieșire a Datelor 200HZ, Tăiere 12.5
  // L3G4200D_DATARATE_100HZ_25:   Frecvența de Ieșire a Datelor 100HZ, Tăiere 25
  // L3G4200D_DATARATE_100HZ_12_5: Frecvența de Ieșire a Datelor 100HZ, Tăiere 12.5 (prestabilit)
  while (!gyro.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50)) {
    delay(1000);
  }

  // Verificați scara gps selectată
  // (grade pe secundă/degrees per second)
  switch (gyro.getScale()) {
    case L3G4200D_SCALE_250DPS:
      break;
    case L3G4200D_SCALE_500DPS:
      break;
    case L3G4200D_SCALE_2000DPS:
      break;
    default:
      break;
  }

  // Verificați frecvența transmisiunii de date și lățimea de bandă
  switch (gyro.getOdrBw()) {
    case L3G4200D_DATARATE_800HZ_110:
      break;
    case L3G4200D_DATARATE_800HZ_50:
      break;
    case L3G4200D_DATARATE_800HZ_35:
      break;
    case L3G4200D_DATARATE_800HZ_30:
      break;
    case L3G4200D_DATARATE_400HZ_110:
      break;
    case L3G4200D_DATARATE_400HZ_50:
      break;
    case L3G4200D_DATARATE_400HZ_25:
      break;
    case L3G4200D_DATARATE_400HZ_20:
      break;
    case L3G4200D_DATARATE_200HZ_70:
      break;
    case L3G4200D_DATARATE_200HZ_50:
      break;
    case L3G4200D_DATARATE_200HZ_25:
      break;
    case L3G4200D_DATARATE_200HZ_12_5:
      break;
    case L3G4200D_DATARATE_100HZ_25:
      break;
    case L3G4200D_DATARATE_100HZ_12_5:
      break;
    default:
      break;
  }

  // Calibrarea giroscopului. Calibrarea trebuie efectuată la repaus.
  // Dacă nu dorim efectuarea calibrării, putem comenta linia de cod.
  gyro.calibrate();

  // Setarea pragului de senzitivitate. Prestabilit -> 3.
  // Dacă nu doriți să folosiți pragul de senzitivitate, comentați linia sau setați
  // -> 0.
  gyro.setThreshold(0);

  TurnOffLEDs(ledPin1,
              ledPin2,
              ledPin3,
              ledPin4);

  delay(1000);
}

void loop() {
  // Clipim LED-urile rapid pentru a marca încheierea etapei de inițializare și începerea funcționării propriu-zise
  // Funcția este executată și în cazul atingerii unghiurilor limită ale platformei
  QuickBlinkLEDs(currentMicros,
                 previousMicrosLEDs,
                 interval,
                 ledPin1,
                 ledPin2,
                 ledPin3,
                 ledPin4,
                 blinkCount,
                 function_running);

  // Servomotoarele pot fi comandate doar după ce funcția QuickBlinkLEDs a fost executată
  if (!function_running) {
    if (!instruction_executed) {
      Servo1.attach(11);  // Corelăm obiectul servo1 cu pinul de comandă 10
      Servo2.attach(10);  // Corelăm obiectul servo2 cu pinul de comandă 11

      Servo3.attach(6);  // Corelăm obiectul servo3 cu pinul de comandă 6
      Servo4.attach(9);  // Corelăm obiectul servo4 cu pinul de comandă 9

      instruction_executed = true;
    }

    // Citește datele de la giroscop și scalează în grade/secundă
    Vector norm = gyro.readNormalize();
    gyro_x = norm.XAxis / GYROSCOPE_SENSITIVITY;
    gyro_y = norm.YAxis / GYROSCOPE_SENSITIVITY;
    gyro_z = norm.ZAxis / GYROSCOPE_SENSITIVITY;

    // Citește datele de la accelerometru și scalează în funcție de forța g
    accel_x = accel.readAccX() / ACCELEROMETER_SENSITIVITY;
    accel_y = accel.readAccY() / ACCELEROMETER_SENSITIVITY;
    accel_z = accel.readAccZ() / ACCELEROMETER_SENSITIVITY;

    // Calculează media ultimelor cinci citiri
    x_avg = (x_avg * 4 + accel_x) / 5;
    y_avg = (y_avg * 4 + accel_y) / 5;

    // Calculază ruliul și tangajul din datele accelerometrului
    roll = atan2(y_avg, sqrt(pow(x_avg, 2) + pow(accel_z, 2))) * 180 / PI;
    pitch = atan2(-x_avg, sqrt(pow(y_avg, 2) + pow(accel_z, 2))) * 180 / PI;

    // Corectarea erorii de drift a giroscopului folosindu-ne de datele accelerometrului
    gyro_x -= roll;
    gyro_y -= pitch;

    // Corectraea suplimentară a erorii giroscopului folosindu-ne de mediile accelerometrului
    gyro_x = alpha * gyro_x + (1 - alpha) * x_avg;
    gyro_y = alpha * gyro_y + (1 - alpha) * y_avg;

    // Aplicarea unui filtru complementar pentru a combina datele de la accelerometru și giroscop
    roll = (alpha * (roll + gyro_x * 0.01) + (1 - alpha) * roll) * 1.015 + XZeroCorrection;
    pitch = (alpha * (pitch + gyro_y * 0.01) + (1 - alpha) * pitch) * 1.065 + YZeroCorrection;

    // Verificăm dacă s-a atins unghiul limită al platformei (45 de grade)
    if (abs(roll) >= 45 || abs(pitch) >= 45) {
      blinkCount = 0;
    }

    // Limităm ruliul și tangajul la o înclinație de maxim 45 de grade
    roll = constrain(roll, -45, 45);
    pitch = constrain(pitch, -45, 45);

    // Scalăm intervalul mișcării de tangaj (180°: [-90°, 90°]) cu valorile reale de comandă PWM ale servomotoarelor (determinate experimental)
    MappedPosServo1 = map(pitch, XAngleScale, abs(XAngleScale), 650, 2040);
    MappedPosServo2 = map(pitch, XAngleScale, abs(XAngleScale), 2380, 790);

    // Scalăm intervalul mișcării de ruliu (180°: [90°, -90°]) cu valorile reale de comandă PWM ale servomotoarelor (determinate experimental)
    MappedPosServo3 = map(roll, abs(YAngleScale), YAngleScale, 2390, 810);
    MappedPosServo4 = map(roll, abs(YAngleScale), YAngleScale, 680, 2090);

    // Comandăm servomotoarele în semnal PWM, în concordanță cu poziția de tangaj scalată anterior
    Servo1.writeMicroseconds(MappedPosServo1);
    Servo2.writeMicroseconds(MappedPosServo2);

    // Comandăm servomotoarele în semnal PWM, în concordanță cu poziția de ruliu scalată anterior
    Servo3.writeMicroseconds(MappedPosServo3);
    Servo4.writeMicroseconds(MappedPosServo4);

    // Afișăm ruliul și tangajul pe monitor dacă variabila booleană SerialPrint = true
    // Prestabilit: bool SerialPrint = false
    SerialPrintFcn(SerialPrint,
                   roll,
                   pitch);
  }
}

void TurnOnLEDs(const int& ledPin1,
                const int& ledPin2,
                const int& ledPin3,
                const int& ledPin4) {

  digitalWrite(ledPin1, HIGH);
  digitalWrite(ledPin2, HIGH);
  digitalWrite(ledPin3, HIGH);
  digitalWrite(ledPin4, HIGH);
}

void TurnOffLEDs(const int& ledPin1,
                 const int& ledPin2,
                 const int& ledPin3,
                 const int& ledPin4) {

  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);
  digitalWrite(ledPin4, LOW);
}

void QuickBlinkLEDs(unsigned long& currentMicros,
                    unsigned long& previousMicrosLEDs,
                    const long& interval,
                    const int& ledPin1,
                    const int& ledPin2,
                    const int& ledPin3,
                    const int& ledPin4,
                    int& blinkCount,
                    bool& function_running) {

  currentMicros = micros();

  if (currentMicros - previousMicrosLEDs >= interval) {
    if (blinkCount < 9) {
      digitalWrite(ledPin1, !digitalRead(ledPin1));
      digitalWrite(ledPin2, !digitalRead(ledPin2));
      digitalWrite(ledPin3, !digitalRead(ledPin3));
      digitalWrite(ledPin4, !digitalRead(ledPin4));

      blinkCount++;
    } else {
      TurnOnLEDs(ledPin1,
                 ledPin2,
                 ledPin3,
                 ledPin4);

      function_running = false;
    }
    previousMicrosLEDs = currentMicros;
  }
}

void SerialPrintFcn(bool& SerialPrint,
                    float& roll,
                    float& pitch) {
  if (SerialPrint) {
    Serial.print("Roll: ");
    Serial.print(int(roll));
    Serial.print(" degrees, Pitch: ");
    Serial.print(int(pitch));
    Serial.println(" degrees");
  }
}