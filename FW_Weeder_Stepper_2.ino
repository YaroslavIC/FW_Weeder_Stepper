#include <GyverStepper.h>

GStepper<STEPPER2WIRE> stepper0(200 * 8, A1, A0, A2);
GStepper<STEPPER2WIRE> stepper1(200 * 8, A4, A3, A5); //A4


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



volatile uint16_t cmd_num = 0;
volatile uint16_t old_cmd_num = 0;
volatile int  LaserDelay = 0;
volatile int  LaserPWM = 0;
volatile long watchdogstart_ms = 0;
volatile long watchdogstop_ms = 0;
long lastmove = 0;


void AutoPower(bool aP) {
  stepper0.autoPower(aP);
  stepper1.autoPower(aP);

    #ifdef DEBUG
      if (aP)  {
        Serial.println("AutoPower true"); }
      else   
        { Serial.println("AutoPower false");};
     #endif   
      
        

      
}


void setup() {
  // put your setup code here, to run once:


  Serial.begin(9600);

  #ifdef DEBUG
    Serial.println("Started!");
    Serial.println("DEBUG Enabled");
  #endif

  digitalWrite(5, LOW);
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);


  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

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

   AutoPower(true);

}



void opto0() {
  if ((cmd_num & (CMD_HA_WAIT))>0) {
    stepper0.reset();
    cmd_num = 0;
  }; 
}

void opto1() {
  if ((cmd_num & (CMD_HV_WAIT))>0) {
    stepper1.reset();
    cmd_num = 0;
  }; 
}

void LaserPower(int PWM, int _watchdogdelay_ms)
{
  LaserPWM = PWM;
  if (PWM > 0) {

    cmd_num = cmd_num | CMD_ML_WAIT;
    watchdogstart_ms = millis();
    watchdogstop_ms  = watchdogstart_ms + _watchdogdelay_ms;
    analogWrite(5, PWM);
  } else {
    cmd_num = cmd_num & (~CMD_ML_WAIT);
    analogWrite(5, 0);
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

      AutoPower(false);
      
      stepper0.setRunMode(FOLLOW_POS);
      stepper0.setTarget( 50, RELATIVE );
      long stmsec = millis();
      while ((millis()-stmsec)<1000) {
        stepper0.tick();
      }
      
      stepper0.setRunMode(KEEP_SPEED);

      stepper0.setSpeed( -1000 );
      cmd_num = cmd_num | CMD_HA_WAIT;// установили бит ожидания

      cmdline = "";
    };


    // $HV поиск нуля по оси V
    if (cmdline == "$HV") {
      
      #ifdef DEBUG
        Serial.print("cmd - ");
        Serial.println(cmdline);
      #endif

      AutoPower(false);

      stepper1.setRunMode(FOLLOW_POS);
      stepper1.setTarget( 50, RELATIVE );
      long stmsec = millis();
      while ((millis()-stmsec)<1000) {
        stepper1.tick();
      }

        
      stepper1.setRunMode(KEEP_SPEED);

      AutoPower(false);
      stepper1.setSpeed( -1000 );
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
      AutoPower(true);
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

      AutoPower(false);


      stepper0.setRunMode(FOLLOW_POS);
      stepper0.setTarget( TargetPosA, RELATIVE );

      stepper1.setRunMode(FOLLOW_POS);
      stepper1.setTarget( TargetPosV, RELATIVE );

      LaserPower(_LaserPWM, 15000);

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

      LaserDelay = 0;
      LaserPWM   = 0;
      int Dchar = 0;

      for (int i = 3; i < cmdline.length(); i++) {
        if (cmdline[i] == 'D') {
          val = cmdline.substring(i + 1);
          LaserDelay = val.toInt();
          Dchar = i;
          break;
        }
      }

      val = cmdline.substring(2, Dchar);
      LaserPWM = val.toInt();

      AutoPower(false);

      #ifdef DEBUG
        Serial.print("cmd - ");
        Serial.print(cmdline);
        Serial.print(" Value: ");
        Serial.print(cmdline.substring(2));
        Serial.print(" ");
        Serial.print(LaserPWM);
        Serial.print(" ");
        Serial.println(LaserDelay);
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

  stepper0.tick();
  stepper1.tick();


  if ((LaserPWM > 0) &&  ((cmd_num & CMD_GV_WAIT) == 0) && ((cmd_num & CMD_GA_WAIT) == 0) && ((cmd_num & CMD_ML_WAIT) != 0)) {
    LaserPower(0, 0);
    #ifdef DEBUG
      Serial.print("Laser stop by motor/time stop  ");
      PrintStatus();
    #endif  
  };


  if ((watchdogstart_ms != 0) && (watchdogstop_ms < millis()))  {
    LaserPower(0, 0);
    #ifdef DEBUG
      Serial.print("Laser stop by watchdog  ");
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

  if   (((((old_cmd_num & CMD_GA_WAIT) > 0) || ((old_cmd_num & CMD_GV_WAIT) > 0)  || ((old_cmd_num & CMD_ML_WAIT) > 0))) &&
        (((cmd_num & CMD_GA_WAIT) == 0) && ((cmd_num & CMD_GV_WAIT) == 0)  && ((cmd_num & CMD_ML_WAIT) == 0))) {
    #ifdef DEBUG
      Serial.print(" stop condition  ");
    #endif  

    lastmove = millis();

    StandardPrintStatus();
  }

  if ((lastmove>0) && ((millis()-lastmove)>10000)) {

    lastmove = 0;
    AutoPower(true);
    #ifdef DEBUG
      Serial.print(" autopower checkout ");
    #endif    
  }

  

  stepper0.tick();
  stepper1.tick();

  old_cmd_num = cmd_num;
}
