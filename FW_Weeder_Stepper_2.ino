#include <GyverStepper.h>

GStepper<STEPPER2WIRE> stepper0(200*8, A1, A0,A2);
GStepper<STEPPER2WIRE> stepper1(200*8, A4, A3,A5); //A4


#define   CMD_IDLE         0
#define   CMD_HA        1<<1
#define   CMD_HV        1<<2
#define   CMD_HA_WAIT   1<<3
#define   CMD_HV_WAIT   1<<4
#define   CMD_GA        1<<5
#define   CMD_GA_WAIT   1<<6
#define   CMD_GV        1<<7
#define   CMD_GV_WAIT   1<<8
#define   CMD_ML        1<<9
#define   CMD_ML_WAIT   1<<10
#define   CMD_OPTO_STP  1<<11



volatile int  opto0_counter = 0; 
volatile int  opto1_counter = 0; 
volatile long cmd_num = 0; 
volatile int  LaserDelay = 0;
volatile int  LaserPWM = 0;
volatile long watchdogstart_ms = 0;
volatile long watchdogstop_ms = 0;



void setup() {
  // put your setup code here, to run once:


   Serial.begin(2000000);

   Serial.println("Started!");

   digitalWrite(5, LOW);
   pinMode(5, OUTPUT);
   digitalWrite(5, LOW);
   

   pinMode(2, INPUT_PULLUP);
   pinMode(3, INPUT_PULLUP);

   attachInterrupt(0, opto0, RISING);
   attachInterrupt(1, opto1, RISING);

   stepper0.reverse(true);
   stepper1.reverse(true);

   stepper0.setSpeed(0);
   stepper1.setSpeed(0);

  stepper0.setMaxSpeed(1600);
  stepper0.setAcceleration(500);   

  stepper1.setMaxSpeed(1600);
  stepper1.setAcceleration(100);   

 // stepper0.autoPower(true);
 // stepper1.autoPower(true);
  
  stepper0.brake();
  stepper1.brake();
  
 
}

void AutoPower(bool aP){
  stepper0.autoPower(aP);
  stepper1.autoPower(aP);
}

void opto0() {
  opto0_counter++;  
  stepper0.reset();
  stepper1.reset();
  cmd_num = 0;
  LaserPower(0,0);
  
  cmd_num = cmd_num | CMD_OPTO_STP;
  
  
  
}

void opto1() {
  opto1_counter++;  // + нажатие
  stepper0.reset();
  stepper1.reset();
  cmd_num = 0;
  LaserPower(0,0);

  cmd_num = cmd_num | CMD_OPTO_STP;
  
}

void LaserPower(int PWM, int _watchdogdelay_ms)
{
  LaserPWM = PWM;
  if (PWM>0) {
    
    cmd_num = cmd_num | CMD_ML_WAIT;
    watchdogstart_ms = millis(); 
    watchdogstop_ms  = watchdogstart_ms + _watchdogdelay_ms;
    analogWrite(5,PWM);
  } else {
    cmd_num = cmd_num & (~CMD_ML_WAIT); 
    analogWrite(5,0);
    watchdogstart_ms = 0;
    watchdogstop_ms  = 0;
  }
  
}




void loop() {
   /*  Serial.print(cmd_num,BIN);
    Serial.print(" ");
    Serial.print(stepper0.getCurrent());
    Serial.print(" ");
    Serial.print(stepper1.getCurrent());
    Serial.print(" ");
    Serial.print(stepper0.getSpeed());
    Serial.print(" ");
    Serial.println(stepper1.getSpeed());
     */

  if (Serial.available() != 0) {
    String cmdline = Serial.readStringUntil('\n');  
    cmdline.trim();                         
    Serial.println(cmdline);
    

    // $HA поиск нуля по оси V
    if (cmdline=="$HA") {
      cmd_num = cmd_num | CMD_HA;
      Serial.print("cmd - ");
      Serial.println(cmdline);

      AutoPower(false);
      stepper0.setRunMode(KEEP_SPEED);

      stepper0.setSpeed( -1000 );
      cmd_num = cmd_num & (~CMD_HA); // сбросили бит команды
      cmd_num = cmd_num | CMD_HA_WAIT;// установили бит ожидания
      
      cmdline = "";
    };
    

    // $HV поиск нуля по оси V
    if (cmdline=="$HV") {
      Serial.print("cmd - ");
      Serial.println(cmdline);
      stepper1.setRunMode(KEEP_SPEED);
      cmd_num = cmd_num | CMD_HV;

      AutoPower(false);
      stepper1.setSpeed( -1000 );
      cmd_num = cmd_num & (~CMD_HV); // сбросили бит команды
      cmd_num = cmd_num | CMD_HV_WAIT;  // установили бит ожидания
      
      cmdline = "";
    };


    // $R сброс всего
    if (cmdline=="$R") {
      Serial.print("cmd - ");
      Serial.println(cmdline);
      Serial.println("Full reset");

      AutoPower(true);
      stepper0.reset();
      stepper1.reset();
      cmd_num = 0;
      LaserPower(0,0);
      
      cmdline = "";
    };



    // G с лазером 
    // GАxxxVyyyPzzz  -  двигаться по оси А на ххх по оси V на yyy и включиь лазер с мощностью zzz
    if ((cmdline.length()>2) & (cmdline.substring(0,2)=="GA")) {

      String val;

      LaserPWM   = 0;
      int Achar = 0;
      int Vchar = 0;
      int Pchar = 0;


      for (int i=0; i<cmdline.length(); i++) {
        if (cmdline[i]=='A') {
          Achar = i+1;
        }
        if (cmdline[i]=='V') {
          Vchar = i+1;
        }
        if (cmdline[i]=='P') {
          Pchar = i+1;
        }
      }
      
      val = cmdline.substring(Achar,Vchar);
      long TargetPosA = val.toInt();
      val = cmdline.substring(Vchar,Pchar);
      long TargetPosV = val.toInt();
      val = cmdline.substring(Pchar);
      long _LaserPWM = val.toInt();

      AutoPower(false);
      
      
      stepper0.setRunMode(FOLLOW_POS);
      stepper0.setTarget( TargetPosA,RELATIVE );
      
      stepper1.setRunMode(FOLLOW_POS);
      stepper1.setTarget( TargetPosV,RELATIVE );

      LaserPower(_LaserPWM,15000);
      

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

      

      cmd_num = cmd_num | CMD_GA_WAIT;// установили бит ожидания
      cmd_num = cmd_num | CMD_GV_WAIT;// установили бит ожидания
      
      cmdline = "";
    };

    

    // MLxxxDyyy  -  включиь лазер с мощностью xxx на время yyy 
    if ((cmdline.length()>2) & (cmdline.substring(0,2)=="ML")) {

      String val;

      LaserDelay = 0;
      LaserPWM   = 0;
      int Dchar = 0;
      
      for (int i=3; i<cmdline.length(); i++) {
        if (cmdline[i]=='D') {
          val = cmdline.substring(i+1);
          LaserDelay = val.toInt();
          Dchar = i;
          break;
        }
      }
      
      val = cmdline.substring(2,Dchar);
      LaserPWM = val.toInt();
      
      AutoPower(false);
      
      Serial.print("cmd - ");
      Serial.print(cmdline);
      Serial.print(" Value: ");
      Serial.print(cmdline.substring(2));
      Serial.print(" ");
      Serial.print(LaserPWM);
      Serial.print(" ");
      Serial.println(LaserDelay);
      

      cmd_num = cmd_num & (~CMD_ML); // сбросили бит команды
      cmd_num = cmd_num | CMD_ML_WAIT;// установили бит ожидания
      
      cmdline = "";
    };
    



    if (cmdline!="") {
      Serial.print("unknow - ");
      Serial.println(cmdline);
      cmd_num = CMD_IDLE; // сбросываем все команды 
      
    }
  };


    stepper0.tick();
    stepper1.tick();

    if ((stepper0.tick()!=0) | (stepper1.tick()!=0)) {
      Serial.print(cmd_num,BIN);
      Serial.print(" ");
      Serial.print(stepper0.getCurrent());
      Serial.print(" ");
      Serial.print(stepper1.getCurrent());
      Serial.print(" ");
      Serial.println(LaserPWM);
    };  

    if (stepper0.tick()==0) {
      if ((cmd_num & CMD_GA_WAIT)!=0) {// сбрасываем бит  если остановились
        Serial.println("Stop A ");      
        cmd_num = cmd_num & (~CMD_GA_WAIT);
      
      }
    }
 
    if (stepper1.tick()==0) {
      if ((cmd_num & CMD_GV_WAIT)!=0) {// сбрасываем бит  если остановились
        Serial.println("Stop V ");      
        cmd_num = cmd_num & (~CMD_GV_WAIT);
      
      }
    }

    if ((LaserPWM>0) &  ((cmd_num & CMD_GV_WAIT)==0) & ((cmd_num & CMD_GA_WAIT)==0) & ((cmd_num & CMD_ML_WAIT)!=0)) {
     LaserPower(0,0);
     Serial.print("Laser stop by motor/time stop");      

      Serial.print(" ");
      Serial.print(cmd_num,BIN);
      Serial.print(" ");
      Serial.print(stepper0.getCurrent());
      Serial.print(" ");
      Serial.print(stepper1.getCurrent());
      Serial.print(" ");
      Serial.println(LaserPWM);
     
    };


   if ((watchdogstart_ms!=0) & (watchdogstop_ms < millis()))  {
     LaserPower(0,0);
     Serial.print("Laser stop by watchdog ");      

      Serial.print(" ");
      Serial.print(cmd_num,BIN);
      Serial.print(" ");
      Serial.print(stepper0.getCurrent());
      Serial.print(" ");
      Serial.print(stepper1.getCurrent());
      Serial.print(" ");
      Serial.println(LaserPWM);
     
   }

    if   ((cmd_num && CMD_OPTO_STP)!=0)   {
     Serial.print("Opto stop");      

     cmd_num = 0;
      
    }


     
   
     

}
