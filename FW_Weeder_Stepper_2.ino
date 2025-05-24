#include <GyverStepper.h>
#include <avr/wdt.h>
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

#define   HOME_A_SPEED       -5000
#define   HOME_V_SPEED       -100
#define   HOME_A_GAP         200
#define   HOME_V_GAP         30
 

#define   LASERWATCHDOG_MS      15000
#define   MOTOR_DIS_TIMEOUT_MS  1000000
#define   LASER_COOL_TIMER_MS   15000


GStepper<STEPPER2WIRE> stepper0(200 * 8, MOTOR0_STEP_PIN, MOTOR0_DIR_PIN, MOTOR0_EN_PIN);
GStepper<STEPPER2WIRE> stepper1(200 * 8, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, MOTOR1_EN_PIN); //A4



#define   CMD_IDLE      (   0)
#define   CMD_HA        (1<<1)
#define   CMD_HV        (1<<2)
#define   CMD_HA_WAIT   (1<<3)
#define   CMD_HV_WAIT   (1<<4)

#define   CMD_GA_WAIT   (1<<6)

#define   CMD_GV_WAIT   (1<<8)

#define   CMD_ML_WAIT   (1<<10)
#define   CMD_OPTO_STP  (1<<11)
#define   CMD_LASER_ON  (1<<12)

//#define   DEBUG 1



volatile uint16_t      cmd_num = 0;
volatile uint16_t  old_cmd_num = 0;
long                LaserDelay = 0;
volatile int          LaserPWM = 0;
long          watchdogstart_ms = 0;
long           watchdogstop_ms = 0;
long                  lastmove = 0;
long            LaserStopAt_ms = 0;
long           oldprinttime_ms = 0;
long               cooltime_ms = 0;
volatile int      prevLaserPWM = 0;
long            CoolTimeOff_ms = 0;

void reboot() {
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}



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

  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(100);


  stepper0.brake();
  stepper1.brake();

  AutoEn(false);

  delay(2000);
  digitalWrite(LASER_ONOFF_PIN, LOW);
  
 
}



void opto0() {
  if ((cmd_num & (CMD_HA_WAIT)) > 0) {
    stepper0.reset();
    cmd_num = CMD_IDLE;
  };
}

void opto1() {
  if ((cmd_num & (CMD_HV_WAIT)) > 0) {
    stepper1.reset();
    cmd_num = CMD_IDLE;
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
    cmd_num = cmd_num | CMD_LASER_ON;
    watchdogstart_ms = millis();
    watchdogstop_ms  = watchdogstart_ms + _watchdogdelay_ms;
    analogWrite(LASER_PWM_PIN, PWM);
    CoolTimeOff_ms = 0;

   
  } else {
#ifdef DEBUG    
    Serial.println("Laser off");
#endif    
 //   digitalWrite(LASER_ONOFF_PIN, LOW);
    cmd_num = cmd_num & (~CMD_LASER_ON);
    
    analogWrite(LASER_PWM_PIN, 0);
    watchdogstart_ms = 0;
    watchdogstop_ms  = 0;

    cooltime_ms = millis();
  }

}


void PrintStatus(void) {
  Serial.print(millis());
  Serial.print(" ");
  Serial.print(cmd_num, BIN);
  Serial.print(" A:");
  Serial.print(stepper0.getCurrent());
  Serial.print(",");
  Serial.print(stepper1.getCurrent());
  Serial.print(",");
  Serial.print(LaserPWM);
  Serial.print(",");
  Serial.print(CoolTimeOff_ms);
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
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    if (cmdline == "$HA") {

#ifdef DEBUG
      Serial.print("cmd - ");
      Serial.println(cmdline);
#endif

      stepper0.setRunMode(FOLLOW_POS);
      stepper0.setTarget( HOME_A_GAP, RELATIVE );

      while (stepper0.tick()) {
        stepper0.tick();
      }      
 
      stepper0.setRunMode(KEEP_SPEED);

      stepper0.setSpeed( HOME_A_SPEED );
      cmd_num = cmd_num | CMD_HA_WAIT;// установили бит ожидания

      cmdline = "";
    };


    // $HV поиск нуля по оси V
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    if (cmdline == "$HV") {

#ifdef DEBUG
      Serial.print("cmd - ");
      Serial.println(cmdline);
#endif

     // AutoEn(true);


      stepper1.setRunMode(FOLLOW_POS);
      stepper1.setTarget( HOME_V_GAP, RELATIVE );
      
      while (stepper1.tick()) {
        stepper1.tick();
      }


      stepper1.setRunMode(KEEP_SPEED);

      stepper1.setSpeed( HOME_V_SPEED );
      cmd_num = cmd_num | CMD_HV_WAIT;  // установили бит ожидания

      cmdline = "";
    };

    
    stepper0.tick();
    stepper1.tick();



     // $R сброс всего
     ////////////////////////////////////////////////////////////////////
     ////////////////////////////////////////////////////////////////////
   if (cmdline == "$R") {

#ifdef DEBUG
      Serial.print("CMD: ");
      Serial.println(cmdline);
      Serial.println("  Full reset");
#endif

      stepper0.reset();
      stepper1.reset();
      AutoEn(true);

      cmd_num = 0;
      LaserPower(0, 0);

      StandardPrintStatus();
      reboot();
      cmdline = "";
    };

    stepper0.tick();
    stepper1.tick();


    // G с лазером
    // GАxxxVyyyPzzz  -  двигаться по оси А на ххх по оси V на yyy и включиь лазер с мощностью zzz
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
    
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
    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////
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
      Serial.print("CMD: ");
      Serial.print(cmdline);
      Serial.print(" Value: ");
      Serial.print(cmdline.substring(2));
      Serial.print(" ");
      Serial.print(LaserPWM);
      Serial.print(" ");
      Serial.println(LaserStopAt_ms);
#endif


      cmd_num = cmd_num | CMD_ML_WAIT;// установили бит ожидания

      cmdline = "";
    };




    if (cmdline != "") {
#ifdef DEBUG
      Serial.print("Unknow CMD: ");
      Serial.println(cmdline);
#endif
      cmd_num = CMD_IDLE; // сбросываем все команды

    }
  };


  stepper0.tick();
  stepper1.tick();

  // сбрасываем соответствующий бит при остановке мотора 0  ось А
  if ((stepper0.tick() == 0) && ((cmd_num & CMD_GA_WAIT) != 0)) {
      cmd_num = cmd_num & (~(CMD_GA_WAIT));  // сбрасываем бит  если остановились
#ifdef DEBUG
      Serial.println("Stop A   ");
#endif
    }
  
  // сбрасываем соответствующий бит при остановке мотора 1  ось V
  if ((stepper1.tick() == 0) && ((cmd_num & CMD_GV_WAIT) != 0)) {
      cmd_num = cmd_num & (~(CMD_GV_WAIT));   // сбрасываем бит  если остановились
#ifdef DEBUG
      Serial.println("Stop V   ");
#endif
    }

    // отключаем лазер если биты отвечающие за моторы обнулены, лезер включен, бит за работу неподвижного лазера не установлен
    if ((LaserPWM > 0) &&  ((cmd_num & CMD_GV_WAIT) == 0) && ((cmd_num & CMD_GA_WAIT) == 0) && ((cmd_num & CMD_ML_WAIT) == 0)) {
      LaserPower(0, 0);
#ifdef DEBUG
      Serial.println("Laser stop by stop motors A and V ");
      PrintStatus();
#endif
    };
  
  stepper0.tick();
  stepper1.tick();

  // отключаем лазер если прошло время команды ML
  if ((LaserStopAt_ms != 0) && (LaserStopAt_ms< millis()) && ((cmd_num & CMD_ML_WAIT) != 0))  {
    LaserPower(0, 0);
    LaserStopAt_ms = 0;
    cmd_num = cmd_num & (~(CMD_ML_WAIT)); 
#ifdef DEBUG
    Serial.println("Laser off normally");
    PrintStatus();
#endif
  }  

  // watchdog лазера 
  if ((watchdogstart_ms != 0) && (watchdogstop_ms < millis()))  {
    LaserPower(0, 0);
    watchdogstart_ms = 0;
    cmd_num = CMD_IDLE;
#ifdef DEBUG
    Serial.println("Laser off by watchdog  ");
    PrintStatus();
#endif
  }
 

  // timeout для моторов - отключае после простоя
  if ((lastmove > 0) && ((millis() - lastmove) > MOTOR_DIS_TIMEOUT_MS)) {

    lastmove = 0;
    AutoEn(false);
#ifdef DEBUG
    Serial.println(" autopower checkout ");
#endif
  }

  
  // ежесекундное вывод координат мотора
  if ((oldprinttime_ms==0) || (millis()-oldprinttime_ms>1000)) {
 //   if ((stepper1.tick() != 0) || (stepper0.tick() != 0)) {
      PrintStatus();
      oldprinttime_ms=millis();
  //  };
  };


  if ((CoolTimeOff_ms == 0) && (LaserPWM>0) && (prevLaserPWM>0)) {
    CoolTimeOff_ms = millis()+LASER_COOL_TIMER_MS;
    digitalWrite(LASER_ONOFF_PIN, HIGH);    
#ifdef DEBUG
    Serial.println(" Cool timer start");
#endif    
  }

  if ((CoolTimeOff_ms > 0)  && (millis()>CoolTimeOff_ms)) {
    CoolTimeOff_ms = 0;
    digitalWrite(LASER_ONOFF_PIN, LOW);    
#ifdef DEBUG
    Serial.println(" Cool timer stop ");
#endif    
  }

  prevLaserPWM = LaserPWM;


  stepper0.tick();
  stepper1.tick();

  old_cmd_num = cmd_num;
}
