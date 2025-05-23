#include <GyverStepper.h>
// сделать autopower после HOME
// добавить planner в будущем

#define   LASER_ONOFF_PIN    7
#define   LASER_PWM_PIN      5

#define   MOTOR0_STEP_PIN    A1
#define   MOTOR0_DIR_PIN     A0
#define   MOTOR0_EN_PIN      A2

#define   MOTOR1_STEP_PIN    A4
#define   MOTOR1_DIR_PIN     A3
#define   MOTOR1_EN_PIN      A5

#define   OPTO0_ENDSTOP_PIN  3
#define   OPTO1_ENDSTOP_PIN  2

#define   HOMESPEED          -1000
#define   HOMEGAP            50
#define   HOMEGAPDELAY       1000

#define   LASERWATCHDOG_MS   15000
#define   MOTOR_DIS_TIMEOUT  1000000


GStepper<STEPPER2WIRE> stepper0(200 * 8, MOTOR0_STEP_PIN, MOTOR0_DIR_PIN, MOTOR0_EN_PIN);
GStepper<STEPPER2WIRE> stepper1(200 * 8, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, MOTOR1_EN_PIN); //A4



#define   CMD_IDLE      (   0)
#define   CMD_HA        (1<<1)
#define   CMD_HV        (1<<2)
#define   CMD_HA_WAIT   (1<<3)
#define   CMD_HV_WAIT   (1<<4)
#define   CMD_GA        (1<<5)
#define   CMD_GA_WAIT   (1<<6)
#define   CMD_GV        (1<<7)
#define   CMD_GV_WAIT   (1<<8)
#define   CMD_ML        (1<<9)
#define   CMD_ML_WAIT   (1<<10)
#define   CMD_OPTO_STP  (1<<11)

//#define   DEBUG 1



volatile uint16_t      cmd_num = 0;
volatile uint16_t  old_cmd_num = 0;
volatile int        LaserDelay = 0;
volatile int          LaserPWM = 0;
volatile long watchdogstart_ms = 0;
volatile long  watchdogstop_ms = 0;
long                  lastmove = 0;
uint16_t        LaserStopAt_ms = 0;
long           oldprinttime_ms = 0;


void AutoEn(bool aP) {
  if (aP)  {
    stepper0.enable();
    stepper1.enable();
  } else {
    stepper0.disable();
    stepper1.disable();
  }

  #ifdef DEBUG
    if (aP)  {
      Serial.println("Motors enable");
    }
    else
    {
      Serial.println("Motors disable");
    };
  #endif
}


void setup() {

  Serial.begin(9600);

#ifdef DEBUG
  Serial.println("Started!");
  Serial.println("DEBUG Enabled");
#endif

  digitalWrite(LASER_PWM_PIN, LOW);
  pinMode(LASER_PWM_PIN, OUTPUT);
  digitalWrite(LASER_PWM_PIN, LOW);

  digitalWrite(LASER_ONOFF_PIN, LOW);
  pinMode(LASER_ONOFF_PIN, OUTPUT);
  digitalWrite(LASER_ONOFF_PIN, HIGH);

  pinMode(OPTO1_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(OPTO0_ENDSTOP_PIN, INPUT_PULLUP);

  attachInterrupt(0, opto1, RISING);
  attachInterrupt(1, opto0, RISING);

  stepper0.reverse(true);
  stepper1.reverse(true);

  stepper0.setSpeed(0);
  stepper1.setSpeed(0);

  stepper0.setMaxSpeed(16000);
  stepper0.setAcceleration(1500);

  stepper1.setMaxSpeed(16000);
  stepper1.setAcceleration(1000);

  stepper0.brake();
  stepper1.brake();

  AutoEn(false);

  delay(2000);
  digitalWrite(LASER_ONOFF_PIN, LOW);
  
 
}



void opto0() {
  if ((cmd_num & (CMD_HA_WAIT)) > 0) {
    stepper0.reset();
    cmd_num = 0;
  };
}

void opto1() {
  if ((cmd_num & (CMD_HV_WAIT)) > 0) {
    stepper1.reset();
    cmd_num = 0;
  };
}

void LaserPower(int PWM, int _watchdogdelay_ms)
{
  LaserPWM = PWM;
  if (PWM > 0) {
#ifdef DEBUG    
    Serial.println("Laser on");
#endif    
    digitalWrite(LASER_ONOFF_PIN, HIGH);
    cmd_num = cmd_num | CMD_ML_WAIT;
    watchdogstart_ms = millis();
    watchdogstop_ms  = watchdogstart_ms + _watchdogdelay_ms;
    analogWrite(LASER_PWM_PIN, PWM);
  } else {
#ifdef DEBUG    
    Serial.println("Laser off");
#endif    
    digitalWrite(LASER_ONOFF_PIN, LOW);
    cmd_num = cmd_num & (~CMD_ML_WAIT);
    analogWrite(LASER_PWM_PIN, 0);
    watchdogstart_ms = 0;
    watchdogstop_ms  = 0;
  }

}


void PrintStatus(void) {
  Serial.print(cmd_num, BIN);
  Serial.print(" A:");
  Serial.print(stepper0.getCurrent());
  Serial.print(",");
  Serial.print(stepper1.getCurrent());
  Serial.print(",");
  Serial.print(LaserPWM);
  Serial.println(";");
}

void StandardPrintStatus(void) {
  Serial.print(stepper0.getCurrent());
  Serial.print(",");
  Serial.print(stepper1.getCurrent());
  Serial.print(",");
  Serial.print(LaserPWM);
  Serial.println(";");
}

void loop() {


  if (Serial.available() != 0) {
    String cmdline = Serial.readStringUntil('\n');
    cmdline.trim();
#ifdef DEBUG
    Serial.println(cmdline);
#endif


    // $HA поиск нуля по оси V
    if (cmdline == "$HA") {

#ifdef DEBUG
      Serial.print("cmd - ");
      Serial.println(cmdline);
#endif

   //   AutoEn(true);


      stepper0.setRunMode(FOLLOW_POS);
      stepper0.setTarget( HOMEGAP, RELATIVE );

      while (stepper0.tick()) {
        stepper0.tick();
      }      
 
      stepper0.setRunMode(KEEP_SPEED);

      stepper0.setSpeed( HOMESPEED );
      cmd_num = cmd_num | CMD_HA_WAIT;// установили бит ожидания

      cmdline = "";
    };


    // $HV поиск нуля по оси V
    if (cmdline == "$HV") {

#ifdef DEBUG
      Serial.print("cmd - ");
      Serial.println(cmdline);
#endif

     // AutoEn(true);


      stepper1.setRunMode(FOLLOW_POS);
      stepper1.setTarget( HOMEGAP, RELATIVE );
      
      while (stepper1.tick()) {
        stepper1.tick();
      }


      stepper1.setRunMode(KEEP_SPEED);


      stepper1.setSpeed( HOMESPEED );
      cmd_num = cmd_num | CMD_HV_WAIT;  // установили бит ожидания

      cmdline = "";
    };

    
    stepper0.tick();
    stepper1.tick();



    // $R сброс всего
    if (cmdline == "$R") {

#ifdef DEBUG
      Serial.print("cmd - ");
      Serial.println(cmdline);
      Serial.println("Full reset");
#endif

      stepper0.reset();
      stepper1.reset();
      AutoEn(true);

      cmd_num = 0;
      LaserPower(0, 0);

      StandardPrintStatus();

      cmdline = "";
    };

    stepper0.tick();
    stepper1.tick();


    // G с лазером
    // GАxxxVyyyPzzz  -  двигаться по оси А на ххх по оси V на yyy и включиь лазер с мощностью zzz
    if ((cmdline.length() > 2) && (cmdline.substring(0, 2) == "GA")) {

      String val;

      LaserPWM   = 0;
      int Achar = 0;
      int Vchar = 0;
      int Pchar = 0;


      for (int i = 0; i < cmdline.length(); i++) {
        if (cmdline[i] == 'A') {
          Achar = i + 1;
        }
        if (cmdline[i] == 'V') {
          Vchar = i + 1;
        }
        if (cmdline[i] == 'P') {
          Pchar = i + 1;
        }
      }

      val = cmdline.substring(Achar, Vchar);
      long TargetPosA = val.toInt();
      val = cmdline.substring(Vchar, Pchar);
      long TargetPosV = val.toInt();
      val = cmdline.substring(Pchar);
      long _LaserPWM = val.toInt();

      AutoEn(true);


      stepper0.setRunMode(FOLLOW_POS);
      stepper0.setTarget( TargetPosA, RELATIVE );

      stepper1.setRunMode(FOLLOW_POS);
      stepper1.setTarget( TargetPosV, RELATIVE );

      LaserPower(_LaserPWM, LASERWATCHDOG_MS);

#ifdef DEBUG
      Serial.print("cmd - ");
      Serial.print(cmdline);
      Serial.print(" Value: ");
      Serial.print(cmdline.substring(2));
      Serial.print(" PosA:");
      Serial.print(TargetPosA);
      Serial.print(" PosV:");
      Serial.print(TargetPosV);
      Serial.print(" Laser:");
      Serial.println(LaserPWM);
#endif



      cmd_num = cmd_num | CMD_GA_WAIT;// установили бит ожидания
      cmd_num = cmd_num | CMD_GV_WAIT;// установили бит ожидания

      cmdline = "";
    };

    stepper0.tick();
    stepper1.tick();


    // MLxxxDyyy  -  включиь лазер с мощностью xxx на время yyy
    if ((cmdline.length() > 2) && (cmdline.substring(0, 2) == "ML")) {

      String val;


      uint16_t _LaserPWM   = 0;
      LaserStopAt_ms = 0;
      int Dchar = 0;

      for (int i = 3; i < cmdline.length(); i++) {
        if (cmdline[i] == 'D') {
          val = cmdline.substring(i + 1);
          LaserStopAt_ms = val.toInt()+millis();
          Dchar = i;
          break;
        }
      }

      val = cmdline.substring(2, Dchar);
      LaserPWM = val.toInt();


      LaserPower(LaserPWM, LASERWATCHDOG_MS);

 

#ifdef DEBUG
      Serial.print("cmd - ");
      Serial.print(cmdline);
      Serial.print(" Value: ");
      Serial.print(cmdline.substring(2));
      Serial.print(" ");
      Serial.print(LaserPWM);
      Serial.print(" ");
      Serial.println(LaserStopAt_ms);
#endif


      cmd_num = cmd_num & (~CMD_ML); // сбросили бит команды
      cmd_num = cmd_num | CMD_ML_WAIT;// установили бит ожидания

      cmdline = "";
    };




    if (cmdline != "") {
#ifdef DEBUG
      Serial.print("unknow - ");
      Serial.println(cmdline);
#endif
      cmd_num = CMD_IDLE; // сбросываем все команды

    }
  };


  stepper0.tick();
  stepper1.tick();

  if ((stepper0.tick() != 0) | (stepper1.tick() != 0)) {
#ifdef DEBUG
    PrintStatus();
#endif
  };

  if (stepper0.tick() == 0) {
    if ((cmd_num & CMD_GA_WAIT) != 0) {
#ifdef DEBUG
      Serial.println("Stop A   ");
#endif
      cmd_num = cmd_num & (~(CMD_GA_WAIT));  // сбрасываем бит  если остановились
    }

  }

  if (stepper1.tick() == 0) {
    if ((cmd_num & CMD_GV_WAIT) != 0) {
#ifdef DEBUG
      Serial.println("Stop V   ");
#endif
      cmd_num = cmd_num & (~(CMD_GV_WAIT));   // сбрасываем бит  если остановились
    }

  }



    if ((LaserPWM > 0) &&  ((cmd_num & CMD_GV_WAIT) == 0) && ((cmd_num & CMD_GA_WAIT) == 0) && ((cmd_num & CMD_ML_WAIT) == 0)) {
      LaserPower(0, 0);
#ifdef DEBUG
      Serial.print("Laser stop by motor A and V ");
      PrintStatus();
#endif
    };


  if ((oldprinttime_ms==0) || (millis()-oldprinttime_ms>1000)) {
    if ((stepper1.tick() != 0) || (stepper0.tick() != 0)) {
      StandardPrintStatus( );
      oldprinttime_ms=millis();
    };
  };
  
  stepper0.tick();
  stepper1.tick();

//
  if ((LaserPWM > 0) &&  ((cmd_num & CMD_GV_WAIT) == 0) && ((cmd_num & CMD_GA_WAIT) == 0) && ((cmd_num & CMD_ML_WAIT) != 0)) {
    LaserPower(0, 0);
#ifdef DEBUG
    Serial.print("Laser stop by motor/time stop  ");
    PrintStatus();
#endif
  };


  if ((LaserStopAt_ms != 0) && (LaserStopAt_ms< millis()) && ((cmd_num & CMD_ML_WAIT) != 0))  {
    LaserPower(0, 0);
    LaserStopAt_ms = 0;
#ifdef DEBUG
    cmd_num = cmd_num & (~(CMD_ML_WAIT)); 
    Serial.print("Laser off normally");
    PrintStatus();
#endif
  }  


  if ((watchdogstart_ms != 0) && (watchdogstop_ms < millis()))  {
    LaserPower(0, 0);
#ifdef DEBUG
    Serial.print("Laser off by watchdog  ");
    PrintStatus();
#endif
  }

  if   ((cmd_num & CMD_OPTO_STP) != 0)   {
#ifdef DEBUG
    Serial.print("Opto stop  ");
    PrintStatus();
#endif
    cmd_num = 0;
  }


  if   (((((old_cmd_num & CMD_GA_WAIT) > 0) || ((old_cmd_num & CMD_GV_WAIT) > 0)  || ((old_cmd_num & CMD_ML_WAIT) > 0) || ((old_cmd_num & CMD_HA_WAIT) > 0) || ((old_cmd_num & CMD_HV_WAIT) > 0)) &&
         (((cmd_num & CMD_GA_WAIT) == 0) && ((cmd_num & CMD_GV_WAIT) == 0)  && ((cmd_num & CMD_ML_WAIT) == 0) && ((cmd_num & CMD_HA_WAIT) == 0) && ((cmd_num & CMD_HV_WAIT) == 0)))) {
#ifdef DEBUG
    Serial.print(" stop condition  ");
#endif

    lastmove = millis();

    StandardPrintStatus();
  }

  if ((lastmove > 0) && ((millis() - lastmove) > MOTOR_DIS_TIMEOUT)) {

    lastmove = 0;
    AutoEn(false);
#ifdef DEBUG
    Serial.print(" autopower checkout ");
#endif
  }

  



  stepper0.tick();
  stepper1.tick();

  old_cmd_num = cmd_num;
}
