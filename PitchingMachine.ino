#include "ArduinoMotorShieldR3.h"

#define I_A     0
#define I_B     1
#define BUF_LEN 80
#define KP 3
#define KI 0.01
#define KD 0.01
#define TIMER_MAX 781 //OCR1A = [16 MHz / (2 * N * fDesired)] - 1, N is prescalar (1024)
//I put in a timer interrupt if you want one. Use the equation above and set TIMER_MAX to get fDesired.
//That is, it will call ISR(TIMER1_COMPA_vect) every 1/fDesired seconds. The default value gives 10 Hz.

inline size_t minsize(size_t x) {
    return ((BUF_LEN > x) ? x : BUF_LEN);
}


ArduinoMotorShieldR3 md;
static float targetSpeed = 0.0f;
void set_speed(float spd) {
  md.setSpeed2(-spd/100, spd/100);
}
static float error_last;
static float integral;
static float last_time;
void setup()
{
  md.init();
  md.setSpeed2(0, 0);
  md.clearBrake2();
  pinMode(ENCODER_1, INPUT); // set ENCODER_1 to input
  pinMode(ENCODER_2, INPUT); // set ENCODER_2 to input
  pinMode(I_A, INPUT);
  pinMode(I_B, INPUT);  
  InitializeInterrupt();
  interrupts();
  Serial.begin(115200); //115200 baud, 8 data bits, 1 stop bit, no parity, XON/XOFF flow control
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("");
  Serial.println("UW ECE Ideas Clinic Pitching Machine");
}

void loop()
{
  bool quit = false;
  set_speed(0);
  while (!quit) {

    if (Serial.available()) {
        char buffer[BUF_LEN];
        size_t bytes_read = Serial.readBytes(buffer, minsize(Serial.available()));    
        if (bytes_read >= 1) {
                    // Assuming no int overflows occur here and numbers are input as expected
                    int val = 0;
                    for (int i = 0; i < bytes_read; i++) {
                        if (buffer[i] < '0' || buffer[i] > '9') {
                            val = -1;
                            if (buffer[i] == 'i') {
                              char msg[80];
                              sprintf(msg, "Current draw: %u, %u", analogRead(I_A),analogRead(I_B));
                              Serial.println(msg);
                            }
                            
                            break;
                        }
                        val *= 10;
                        val += buffer[i]-'0';
                    }
                    if (val >= 0 && val <= 255) {
                        set_speed(val);
                        target_speed = val;
                    }
                          
                    if (val != -1) {
                      Serial.print(val);
                      Serial.print("\n");
                    }
            }
        }

    // PID
    float time_since_last = get_cur_time()-last_time;
    float cur_speed = get_speed();
    float error = abs(target_speed-cur_speed);
    float integral += error*time_since_last;
    float derivative = (error-error_last)/time_since_last;
    float output = KP*error+KI*integral+KD*derivative;
    error_last = error;
    last_time = get_cur_time();
     delay(50);
  }
  set_speed(0);
  return;
}

void InitializeInterrupt() //Don't mess with this function - it sets up the control registers for the IR sensor and timer interrupts
{
  cli();    // switch interrupts off while messing with their settings
  
  PCICR   = 0x02;   // Enable PCINT1 interrupt
  PCMSK1  = 0b00001100;
  
  PRR    &= ~0x04;   //Enable Timer/Counter1
  TCCR1A  = 0b00000000; //No output pins, WGM11 = 0, WGM10 = 0
  TCCR1B  = 0b00001101; //WGM12 = 1, WGM13 = 0, CS1 = 0b101 (clk/1024)
  OCR1A   = TIMER_MAX; //Set count
  TIMSK1 |= 0b00000010; //OCIE1A = 1 (interrupt enable for OCR1A)
  
  sei();    // turn interrupts back on
}

ISR(PCINT1_vect) //Encoder Interrupt Service Routine
{
  Serial.println("ISR");
char msg[80];
    sprintf(msg, "Data: %u, %u", digitalRead(ENCODER_1), digitalRead(ENCODER_2));
    Serial.println(msg);
}

ISR(TIMER1_COMPA_vect) //Timer Interrupt Service Routine
{
  //This will trigger at a frequency determined by TIMER_MAX
}


