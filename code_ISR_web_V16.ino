
//Info:
// http://yourduino.com/docs/MegaPinOut.png 
// http://rigidtalk.com/wiki/images/8/82/RAMPS1_4schematic.png
// http://www.robotek.pk/product/arduino-mega-2560/#&gid=1&pid=3   
// https://3dtek.xyz/blogs/technical-docs/18982427-not-another-ramps-tutorial  jumper settings for ramp  

//Source Code 
// http://forum.arduino.cc/index.php?topic=48585.0   Motor interupt code base on 
// https://tutorial.cytron.io/2011/07/27/an-arduino-room-monitoring-web-server/  Web server code base on 
// http://forum.arduino.cc/index.php/topic,82416.msg619420.html#msg619420

//Idea
//http://startingelectronics.org/tutorials/arduino/ethernet-shield-web-server-tutorial/SD-card-IO/
//https://diyprojects.io/bootstrap-create-beautiful-web-interface-projects-esp8266/#.Wv97gYgvzBw

// todo 
/*  Motor Acceleration is cool ,, deceleration to add 
 *  cal a position not a movement  <OK
 *  Set 0 position   <OK
 *  save preset   <OK
 *  set  soft limit travel ... is done in the interupt , has to be done before 
 
*/

#include <SPI.h>
#include <Ethernet.h>

// Ethernet shield attached to pins 10, 11, 12, 13

#define X_DIR    A1   //fire motor   
#define X_STP    A0 
#define X_EN     38

#define Y_DIR    A7  //Azimuth motor
#define Y_STP    A6  //200step=360o.. 1/16step*200 .. 3200step pour 360degre .. 8.8step pour 1degree ... mon code fait step/2 ....17.7 pour 1degree
#define Y_EN     A2

#define Z_DIR    48   //unused 
#define Z_STP    46 
#define Z_EN     A8

#define E0_DIR    28  //Altitude motor
#define E0_STP    26 //up- down  200step=360o... 1/16step*200 .. 3200step pour 360degree... Ratio 66/12gear=5.5 ....   17600step pour 360o .... 48.8step pour 1degree ... mon code fait step/2 ....97.7 pour 1degree
#define E0_EN     24

#define E1_DIR    34    //unused 
#define E1_STP    36 
#define E1_EN     30  // my ramp board is defect on pin EN 

#define X_SPEED_MAX  10000L   //fire
#define X_SPEED_MIN  100L
#define STEP_FOR_1TURN_X 200l*8  //If doing math with integers, at least one of the numbers must be followed by an L, forcing it to be a long https://www.arduino.cc/reference/en/language/variables/data-types/long/

#define Y_SPEED_MAX  10000L   //left-rigth
#define Y_SPEED_MIN  100L
#define STEP_FOR_1TURN_Y 200l*4 

#define E0_SPEED_MAX  50000L  //upo-down
#define E0_SPEED_MIN  100L
#define STEP_FOR_1TURN_E0 200l*4


/****************GLOBAL VAR ***************/ 
    byte mac[] = {    0x0E, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

   IPAddress IPAddress_ip       (10, 188, 182, 5);
   IPAddress IPAddress_dnServer (10, 175, 237, 44);
   IPAddress IPAddress_gateway  (10, 188, 182, 254);   // the router's gateway address:
   IPAddress IPAddress_subnet   (255, 255, 255, 0);  // the subnet:
    
   // IPAddress IPAddress_ip       (192, 168, 81, 152);
   // IPAddress IPAddress_dnServer (192, 168, 81, 1);
   // IPAddress IPAddress_gateway  (192, 168, 81, 1);   // the router's gateway address:
   // IPAddress IPAddress_subnet   (255, 255, 255, 0);  // the subnet:

    
    EthernetServer server(80);

    long           serialdebug =0;
    const float   samplerate = 10000.0f;  // interrupt frequency
    
    long           X_Speed = 0;
    long           Y_Speed = 0;
    long           Z_Speed = 0;
    long           E0_Speed = 0;
    long           E1_Speed = 0;
 
    long           step_todo_X  = 0;
    long           step_todo_Y  = 0; 
    long           step_todo_Z  = 0; 
    long           step_todo_E0 = 0; 
    long           step_todo_E1 = 0; 
     
    long           position_X  = 0;
    long           position_Y  = 0; 
    long           position_Z  = 0; 
    long           position_E0 = 0; 
    long           position_E1 = 0; 

    long           Save_1_Y  = 120*17.7; 
    long           Save_2_Y  =  60*17.7; 
    long           Save_3_Y  = -60*17.7;  
    long           Save_4_Y  = -120*17.7;  
    long           Save_5_Y  =  0*17.7; 
    long           Save_6_Y  =  0*17.7; 

    long           Save_1_E0 = 10*97.7;
    long           Save_2_E0 = 10*97.7;
    long           Save_3_E0 = 30*97.7;
    long           Save_4_E0 = 30*97.7;
    long           Save_5_E0 = 0*97.7;
    long           Save_6_E0 = 0*97.7;

    long           Limit_Y_1  = -150*17.7;
    long           Limit_Y_2  =  150*17.7;
    long           Limit_E0_1  = -5*97.7;  
    long           Limit_E0_2  = 70*97.7; 

    unsigned char   Limit_overide = 0;
 
    char c = 0;           // received data
    char command[2] = "\0";  // command

    byte rip[]          = {0,0,0,0  };
    byte log_rip[20][4] = {0        };
     
/**************** SETUP ***************/ 
void setup(){
  // init pins
  pinMode(X_EN, OUTPUT);
  pinMode(X_STP, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  
  pinMode(Y_EN, OUTPUT);
  pinMode(Y_STP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  
  pinMode(Z_EN, OUTPUT);
  pinMode(Z_STP, OUTPUT);
  pinMode(Z_DIR, OUTPUT);
  
  pinMode(E0_EN, OUTPUT);
  pinMode(E0_STP, OUTPUT);
  pinMode(E0_DIR, OUTPUT);
  
  pinMode(E1_EN, OUTPUT);
  pinMode(E1_STP, OUTPUT);
  pinMode(E1_DIR, OUTPUT);

  digitalWrite(X_EN, HIGH); //
  digitalWrite(X_STP, HIGH);
  digitalWrite(X_DIR, HIGH);
  
  digitalWrite(Y_EN, HIGH); //
  digitalWrite(Y_STP, HIGH);
  digitalWrite(Y_DIR, HIGH);
  
  digitalWrite(Z_EN, HIGH); //
  digitalWrite(Z_STP, HIGH);
  digitalWrite(Z_DIR, HIGH);
  
  digitalWrite(E0_EN, HIGH); //
  digitalWrite(E0_STP, HIGH);
  digitalWrite(E0_DIR, HIGH);
  
  digitalWrite(E1_EN, HIGH);
  digitalWrite(E1_STP, HIGH);
  digitalWrite(E1_DIR, HIGH);

  delay(100);

  // initialize timer1
  noInterrupts(); // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 16000000.0f / samplerate; // compare match register for IRQ with selected samplerate
  TCCR1B |= (1 << WGM12); // CTC mode
  TCCR1B |= (1 << CS10); // no prescaler
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  interrupts(); // enable all interrupts

 /*
    Serial.begin(115200);
    while (!Serial) {   } ; // wait for serial port to connect. Needed for Leonardo only
*/
  
  //Ethernet.begin(mac, ip);
  Ethernet.begin(mac, IPAddress_ip, IPAddress_dnServer, IPAddress_gateway, IPAddress_subnet);
  server.begin();
  //Serial.print("server is at ");
 // Serial.println(Ethernet.localIP());
 
}
/**************** MOTOR INTERUPT  ***************/ 
ISR(TIMER1_COMPA_vect){// timer 1 interrupt

    static   unsigned char X_tick  = 0;
    static   unsigned char Y_tick  = 0; 
    static   unsigned char E0_tick = 0;
    
    static   uint32_t      X_wait  = 0;
    static   uint32_t      Y_wait  = 0;
    static   uint32_t      E0_wait = 0;


   if (step_todo_X !=0 ){
      digitalWrite(X_EN, LOW);                                                            // sactivate motor 
      X_Speed = X_Speed + 2;                                                              //acceleration
      if (X_Speed > X_SPEED_MAX) X_Speed=X_SPEED_MAX;                                     //if at max accel , stay there 
      X_wait += X_Speed;                                                                  //Increase X_wait by X_speed until over 65535, then tick the motor, and reset X_wait  // generate the next clock pulse on accumulator overflow
      if (X_wait > 0xffff) {                                                              //65535
          if (step_todo_X>0){ digitalWrite(X_DIR, HIGH); step_todo_X--; position_X++;}    //count step toward step_todo_X = 0 
          if (step_todo_X<0){ digitalWrite(X_DIR, LOW ); step_todo_X++; position_X--;}    //count step toward step_todo_X = 0 
          if (X_tick)       { digitalWrite(X_STP, HIGH);    }                             //step 
            else            { digitalWrite(X_STP, LOW );    }                             //step 
          X_wait = 0 ;                                                                    //done one tick , start back to wait 
          X_tick = !X_tick;                                                               //next tick reversed 
      }  
   }
   else{     
     X_Speed = X_SPEED_MIN;    
     digitalWrite(X_EN, HIGH);                                             //desactivate motor if neaded
   }
   
   if ((step_todo_Y !=0)   ){
      digitalWrite(Y_EN, LOW);                                                            // sactivate motor 
      Y_Speed = Y_Speed + 2;                                                              //acceleration
      if (Y_Speed > Y_SPEED_MAX) Y_Speed=Y_SPEED_MAX;                                     //if at max accel , stay there 
      Y_wait += Y_Speed;                                                                  //Increase X_wait by X_speed until over 65535, then tick the motor, and reset X_wait  // generate the next clock pulse on accumulator overflow
      if (Y_wait > 0xffff) {                                                              //65535
          if ((step_todo_Y>0)&&(position_Y<Limit_Y_2)){ 
              digitalWrite(Y_DIR, HIGH); 
              step_todo_Y--; 
              position_Y++;
              if (Y_tick) { digitalWrite(Y_STP, HIGH); } else { digitalWrite(Y_STP, LOW ); }  
          }     
          if ((step_todo_Y<0)&&(position_Y>Limit_Y_1)){ 
              digitalWrite(Y_DIR, LOW ); 
              step_todo_Y++; 
              position_Y--;
              if (Y_tick) { digitalWrite(Y_STP, HIGH); } else { digitalWrite(Y_STP, LOW ); }  
          }     
          Y_wait = 0 ;                                                                    //done one tick , start back to wait 
          Y_tick = !Y_tick;                                                               //next tick reversed 
      }  
   }
   else{     
     Y_Speed = Y_SPEED_MIN;    
     //digitalWrite(Y_EN, HIGH);                                             //desactivate motor if neaded
   }
   if (step_todo_E0 !=0  ){
      digitalWrite(E0_EN, LOW);                                                 //activate motor 
      E0_Speed = E0_Speed + 2;                                                  //acceleration
      if (E0_Speed > E0_SPEED_MAX) E0_Speed=E0_SPEED_MAX;                       //if at max accel , stay there 
      E0_wait += E0_Speed;                                                      //Increase X_wait by X_speed until over 65535, then tick the motor, and reset X_wait
      if (E0_wait > 0xffff) {                                                   //65535
          if ((step_todo_E0>0)&&(position_E0<Limit_E0_2)){                                                   
              digitalWrite(E0_DIR, HIGH); 
              step_todo_E0--; 
              position_E0++;
              if (E0_tick) { digitalWrite(E0_STP, HIGH); } else { digitalWrite(E0_STP, LOW ); }
          }  
          if ((step_todo_E0<0)&&(position_E0>Limit_E0_1)){                                                   
              digitalWrite(E0_DIR, LOW ); 
              step_todo_E0++; 
              position_E0--;
              if (E0_tick) { digitalWrite(E0_STP, HIGH); } else { digitalWrite(E0_STP, LOW ); }
          }  
          E0_wait = 0 ;                                                          //done one tick , start back to wait 
          E0_tick = !E0_tick;                                                    //next tick reversed 
      }  
   }
   else{     
     E0_Speed = E0_SPEED_MIN;    
     //digitalWrite(E0_EN, HIGH);                                             //desactivate motor if neaded
   }
  
}
void goto_Y(long pos){
do_step_Y(pos-(step_todo_Y+ position_Y));
}
void goto_E0(long pos){
do_step_E0(pos-(step_todo_E0+ position_E0));
}
void calibrate_all(void){
/*
  Limit_E0_1  = -200*97.7;  
  Limit_Y_1  = -400*17.7;

  do_step_E0(-106 *97.77 );
  do_step_Y(-360 *17.77 );
  
  delay(10000);
  
//  position_E0=(-30 *97.77 );
//  position_Y=(-180 *17.77);

//  do_step_E0(30 *97.77 );
//  do_step_Y(180 *17.77 );

  
//  delay(5500);

   Limit_Y_1  = -170*17.7;
   Limit_Y_2  =  170*17.7;
   Limit_E0_1  = -10*97.7;  
   Limit_E0_2  = 80*97.7; 
   */
  position_E0=( 0 );
  position_Y =( 0 );
   
}

void do_step_X(long steps){
  if(((step_todo_X + steps +0l) > 0) && (step_todo_X < 0)){ noInterrupts(); X_Speed = X_SPEED_MIN;  interrupts(); } //if change direction , reset accel to 0    //todo X_Speed = 0 only if not moving already 
  if(((step_todo_X + steps +0l) < 0) && (step_todo_X > 0)){ noInterrupts(); X_Speed = X_SPEED_MIN;  interrupts(); } //if change direction , reset accel to 0 
  noInterrupts();    step_todo_X=step_todo_X + steps;    interrupts();
}
void do_step_Y(long steps){
  if(((step_todo_Y + steps +0l) > 0) && (step_todo_Y < 0)){ noInterrupts(); Y_Speed = Y_SPEED_MIN;  interrupts(); } //if change direction , reset accel to 0 
  if(((step_todo_Y + steps +0l) < 0) && (step_todo_Y > 0)){ noInterrupts(); Y_Speed = Y_SPEED_MIN;  interrupts(); } //if change direction , reset accel to 0 
  noInterrupts();    step_todo_Y=step_todo_Y + steps;    interrupts();
}
void do_step_E0(long steps){
  if(((step_todo_E0 + steps +0l) > 0) && (step_todo_E0 < 0)){ noInterrupts(); E0_Speed = E0_SPEED_MIN;  interrupts(); } //if change direction , reset accel to 0 
  if(((step_todo_E0 + steps +0l) < 0) && (step_todo_E0 > 0)){ noInterrupts(); E0_Speed = E0_SPEED_MIN;  interrupts(); } //if change direction , reset accel to 0 
  noInterrupts();    step_todo_E0=step_todo_E0 + steps;    interrupts();
}


/**************** MAIN  ***************/ 
void loop(){
    int inByte = 0;         // incoming serial byte

    if(serialdebug){
      if (Serial.available() > 0) {
        inByte = Serial.read();
        if(inByte=='5') do_step_X (1000);  //todo , was in l 
        if(inByte=='6') do_step_Y (9);  //17.7 is a degree 
        if(inByte=='4') do_step_Y (-9);       
        if(inByte=='8') do_step_E0(-49); //97.7  is a degre
        if(inByte=='2') do_step_E0( 49); 
      }
   
      Serial.print("step_todo_X:");    Serial.print(step_todo_X);  Serial.print(" "); 
      Serial.print("position_X:");     Serial.print(position_X);   Serial.print(" "); 
      Serial.print("step_todo_Y:");    Serial.print(step_todo_Y);  Serial.print(" "); 
      Serial.print("position_Y:");     Serial.print(position_Y);   Serial.print(" "); 
      Serial.print("step_todo_E0:");   Serial.print(step_todo_E0); Serial.print(" "); 
      Serial.print("position_E0:");    Serial.print(position_E0);  Serial.print(" "); 
      Serial.println();
  }
 

EthernetClient client = server.available();
  // detect if current is the first line
  boolean current_line_is_first = true;

  if (client) {
/*     
 *    client.getRemoteIP(rip);     //need mod to lib  http://forum.arduino.cc/index.php/topic,82416.msg619420.html#msg619420
     for (unsigned char bcount= 0; bcount < 4; bcount++)     { 
        //Serial.print(rip[bcount], DEC); 
        //if (bcount<3) Serial.print(".");
        log_rip[19][bcount] = log_rip[18][bcount];
        log_rip[18][bcount] = log_rip[17][bcount];
        log_rip[17][bcount] = log_rip[16][bcount];
        log_rip[16][bcount] = log_rip[15][bcount];
        log_rip[15][bcount] = log_rip[14][bcount];
        log_rip[14][bcount] = log_rip[13][bcount];
        log_rip[13][bcount] = log_rip[12][bcount];
        log_rip[12][bcount] = log_rip[11][bcount];
        log_rip[11][bcount] = log_rip[10][bcount];
        log_rip[10][bcount] = log_rip[ 9][bcount];
        log_rip[ 9][bcount] = log_rip[ 8][bcount];
        log_rip[ 8][bcount] = log_rip[ 7][bcount];
        log_rip[ 7][bcount] = log_rip[ 6][bcount];
        log_rip[ 6][bcount] = log_rip[ 5][bcount];
        log_rip[ 5][bcount] = log_rip[ 4][bcount];
        log_rip[ 4][bcount] = log_rip[ 3][bcount];
        log_rip[ 3][bcount] = log_rip[ 2][bcount];
        log_rip[ 2][bcount] = log_rip[ 1][bcount];
        log_rip[ 1][bcount] = log_rip[ 0][bcount];
        log_rip[ 0][bcount] = rip[bcount];
     } */

    
    // an http request ends with a blank line
    boolean current_line_is_blank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
                    if(serialdebug){ Serial.print(c); }
        
        // if we've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so we can send a reply
        if (c == '\n' && current_line_is_blank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println();

          // auto reload webpage every 5 second
          client.println("<META HTTP-EQUIV=REFRESH CONTENT=3;URL=http://10.188.182.5>");
          //;url=http://example.com/
          client.println("<!-- Go fudge yourself, you filty code reader ! -->");
          
          // webpage title
          client.println("<center><p><h1>Elastic War Machine V16.0 </h1></p><center><hr> ");

          client.print("<p><h3>Altitude = <font color=indigo>");
          client.print(position_E0/97.7, 1);
          client.println("</font>&deg</p>");
          client.print("<p>Azimuth = <font color=indigo>");
          client.print(position_Y/17.7, 1);
          client.println("</font>&deg</h3></p>");

          
          // button functions
          client.println("<form  method=get name=form>");
          client.println("<button name=b value=1 type=submit style=height:30px;width:35px>10</button>");
          client.println("<br/>");
          client.println("<button name=b value=2 type=submit style=height:30px;width:35px>0.5</button>");
          client.println("<br/>");
          client.println("<button name=b value=3 type=submit style=height:30px;width:35px>25</button>");
          client.println("<button name=b value=4 type=submit style=height:30px;width:35px>10</button>");
          client.println("<button name=b value=5 type=submit style=height:30px;width:35px>0.5</button>");
          client.println("<button style=\"color:red\" name=b value=6 type=submit style=height:30px;width:35px>Fire</button>");
          client.println("<button name=b value=7 type=submit style=height:30px;width:35px>-0.5</button>");
          client.println("<button name=b value=8 type=submit style=height:30px;width:35px>-10</button>");
          client.println("<button name=b value=9 type=submit style=height:30px;width:35px>-25</button>");

          client.println("<br/>");
          client.println("<button name=b value=s type=submit style=height:30px;width:35px>-0.5</button>");
          client.println("<br/>");
          client.println("<button name=b value=t type=submit style=height:30px;width:35px>-10</button>");
          client.println("<br/>");
          client.println("</form>");
 
          client.println("<form  method=get name=form>");
          client.println("<button name=b value=a type=submit style=height:25px;width:75px>Save #1</button>");
          client.println("<button name=b value=b type=submit style=height:25px;width:180px>Recal ");
          client.print(Save_1_E0/97.7, 1);
          client.println("&deg   ");
          client.print(Save_1_Y/17.7, 1);
          client.println("&deg </button> ");
          client.println("<br/>");
          
          client.println("<button name=b value=c type=submit style=height:25px;width:75px>Save #2</button>");
          client.println("<button name=b value=d type=submit style=height:25px;width:180px>Recal ");
          client.print(Save_2_E0/97.7, 1);
          client.println("&deg   ");
          client.print(Save_2_Y/17.7, 1);
          client.println("&deg </button> ");
          client.println("<br/>");
          
          client.println("<button name=b value=e type=submit style=height:25px;width:75px>Save #3</button>");
          client.println("<button name=b value=f type=submit style=height:25px;width:180px>Recal ");
          client.print(Save_3_E0/97.7, 1);
          client.println("&deg   ");
          client.print(Save_3_Y/17.7, 1);
          client.println("&deg </button> ");
          client.println("<br/>");

          
          client.println("<button name=b value=g type=submit style=height:25px;width:75px>Save #4</button>");
          client.println("<button name=b value=h type=submit style=height:25px;width:180px>Recal ");
          client.print(Save_4_E0/97.7, 1);
          client.println("&deg   ");
          client.print(Save_4_Y/17.7, 1);
          client.println("&deg </button> ");
          client.println("<br/>");

          
          client.println("<button name=b value=i type=submit style=height:25px;width:75px>Save #5</button>");
          client.println("<button name=b value=j type=submit style=height:25px;width:180px>Recal ");
          client.print(Save_5_E0/97.7, 1);
          client.println("&deg   ");
          client.print(Save_5_Y/17.7, 1);
          client.println("&deg </button> ");
          client.println("<br/>");

          client.println("<br/>");
          client.println("<button name=b value=0 type=submit style=height:30px;width:135px>Goto Zero</button>");
          //client.println("<button name=b value=z type=submit style=height:30px;width:135px>Calibrate</button>");
          client.println("<br/>");
          client.println("</form>");
 

          client.println("<font size=\"1\">");
          client.println("<hr><center>Last Connections:<br />");

          for (int ip_count= 0; ip_count < 20; ip_count++)     { 
            for (int byte_count= 0; byte_count < 4; byte_count++)     { 
              client.print(log_rip[ip_count][byte_count] );
              if (byte_count<3) client.print(".");
            } 
            client.println("<br/>");
          } 
          client.println("</font>");
          client.println("<br/>");
          client.println("<br/>");
          client.println("<br/>");
          client.println("https://www.thingiverse.com/thing:2926187 </a>");
          client.println("<a href=http://jfpayeur.com>Source Code </a>");
          client.println("<br/>");
          client.println("<br/>");
          client.println("<br/>");
          
          client.println("</center>");
          
          break;
        }
        if (c == '\n') {
          // we're starting a new line
          current_line_is_first = false;
          current_line_is_blank = true;
        } 
        else if (c != '\r') {
          // we've gotten a character on the current line
          current_line_is_blank = false;
        }
        // get the first http request
        if (current_line_is_first && c == '=') {
          for (int i = 0; i < 1; i++) {
            c = client.read();
            command[i] = c;
          }

               if (!strcmp(command, "1")) {            do_step_E0( 10*97.7 );          }
          else if (!strcmp(command, "2")) {            do_step_E0(      49 );          }
          else if (!strcmp(command, "3")) {            do_step_Y ( 25*17.7 );          }
          else if (!strcmp(command, "4")) {            do_step_Y ( 10*17.7 );          }
          else if (!strcmp(command, "5")) {            do_step_Y (      9  );          }
          else if (!strcmp(command, "6")) {            do_step_X (     800  );          }
          else if (!strcmp(command, "7")) {            do_step_Y (     -9  );          }
          else if (!strcmp(command, "8")) {            do_step_Y (-10*17.7 );          }
          else if (!strcmp(command, "9")) {            do_step_Y (-25*17.7 );          }
          else if (!strcmp(command, "s")) {            do_step_E0(    -49  );          }
          else if (!strcmp(command, "t")) {            do_step_E0(-10*97.7 );          }
 
          
          else if (!strcmp(command, "a")) {            Save_1_Y=position_Y; Save_1_E0=position_E0;  }
          else if (!strcmp(command, "c")) {            Save_2_Y=position_Y; Save_2_E0=position_E0;  }
          else if (!strcmp(command, "e")) {            Save_3_Y=position_Y; Save_3_E0=position_E0;  }
          else if (!strcmp(command, "g")) {            Save_4_Y=position_Y; Save_4_E0=position_E0;  }
          else if (!strcmp(command, "i")) {            Save_5_Y=position_Y; Save_5_E0=position_E0;  }
          
          else if (!strcmp(command, "b")) {            goto_Y(Save_1_Y);  goto_E0(Save_1_E0);  }
          else if (!strcmp(command, "d")) {            goto_Y(Save_2_Y);  goto_E0(Save_2_E0);  }
          else if (!strcmp(command, "f")) {            goto_Y(Save_3_Y);  goto_E0(Save_3_E0);  }
          else if (!strcmp(command, "h")) {            goto_Y(Save_4_Y);  goto_E0(Save_4_E0);  }
          else if (!strcmp(command, "j")) {            goto_Y(Save_5_Y);  goto_E0(Save_5_E0);  }
          
          else if (!strcmp(command, "0")) {            goto_Y(0);    goto_E0(0);       }
          else if (!strcmp(command, "z")) {            calibrate_all();     }
          
        }
      }
    }

    delay(1);     // give the web browser time to receive the data
    client.stop();
  }


   
}


















 
