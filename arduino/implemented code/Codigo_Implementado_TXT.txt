//Pines del display OLED
#define OLED_DC 5
//#define OLED_CS 5// CS directly grounded
#define OLED_CLK 8
#define OLED_MOSI 7
#define OLED_RESET 6
//Pines del driver TB6612
#define PWM 9
#define IN1 10
#define IN2 11
//Pines del encoder del brazo
#define ENCODER_A 4
#define ENCODER_B 2

// Boton de inicio del control
#define KEY_S 15

#include <SSD1306.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include <MsTimer2.h>  //Interrupción de temporizador

SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, 0);

double pi = 3.141592653589793; 
unsigned char Flag_Stop = 1;  //Bit de bandera de parada

/*
double K_Angulo_Pendulo = 19.0418, K_Vel_Angular_Pendulo = 66.7800, K_Angulo_Brazo = -0.3659, K_Vel_Angular_Brazo = -137.6000; // Mejor desempeño 5/12/23
double K_Angulo_Pendulo = 51.9819 , K_Vel_Angular_Pendulo = 5.3535, K_Angulo_Brazo = -2.6352 , K_Vel_Angular_Brazo = -3.4922 ; // Valores sacados del modelo. Q = [200 0 0 0; 0 200 0 0; 0 0 200 0; 0 0 0 200];			
*/

// Coeficientes de control LQR (PD) para el péndulo y el brazo
double K_Angulo_Pendulo = 19.0418, K_Vel_Angular_Pendulo = 66.7800, K_Angulo_Brazo = -0.3659, K_Vel_Angular_Brazo = -137.6000; // Mejor desempeño 5/12/23
// 						
// Ángulos y constantes de ajuste
double Cero_Angulo_Pendulo = 1.43; // Este es el valor que mide el sensor cuando el péndulo está erguido y se debe calibrar para un correcto funcionamiento.
double Cero_Angulo_Brazo = 100;

double Menu = 1, Amplitude1 = 1, Amplitude2 = 2, Amplitude3 = 2, Amplitude4 = 1;  
int Voltaje_Fuente;                                                               
double Angulo_Pendulo, Angulo_Pendulo_Medido, Angulo_Pendulo_Pasado, V_Angular_Pendulo;                                  
double Angulo_Brazo,  Angulo_Brazo_Pasado, Vel_Angular_Brazo; 
double Angulo_Brazo_Medido = Cero_Angulo_Brazo;
double Motor;

uint32_t oled_pow(uint8_t m, uint8_t n) {
  uint32_t result = 1;
  while (n--) result *= m;
  return result;
}

// Función para mostrar números en el display OLED
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len) {
  u8 t, temp;
  u8 enshow = 0;
  for (t = 0; t < len; t++) {
    temp = (num / oled_pow(10, len - t - 1)) % 10;
    oled.drawchar(x + 6 * t, y, temp + '0');
  }
}

// Función para establecer el PWM del motor
void Set_Pwm(int motor) {
  if (motor < 0) digitalWrite(IN2, HIGH), digitalWrite(IN1, LOW);    // Configura la dirección del motor y establece el PWM {CAMBIADO}
  else digitalWrite(IN2, LOW), digitalWrite(IN1, HIGH);            
  analogWrite(PWM, abs(motor));                                    
}

// Función para apagar el motor bajo ciertas condiciones
unsigned char Turn_Off() {
  unsigned char temp;
  if ( Angulo_Pendulo_Medido < (Cero_Angulo_Pendulo - 2) || Angulo_Pendulo_Medido > (Cero_Angulo_Pendulo + 2) || Flag_Stop == 1)  //Flag_Stop is set to 1 or the pendulum deviates greatly from the balance Position, turn off the motor
  {
    temp = 1;
    Flag_Stop = 1;
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, LOW); 
  } else
    temp = 0;
  return temp;
}

// Función para obtener el promedio de lecturas ADC del encoder del péndulo
u16 Get_Adc_Average(u8 ch, u8 times) {
  unsigned int temp_val = 0;
  unsigned char t;
  for (t = 0; t < times; t++) {
    temp_val += analogRead(ch);  
  }
  return temp_val / times;  
}

// Función de control principal
void control() {
  static double Voltage_All, Voltage_Count;                            
  int Temp;                                                            
  sei();                                                              
  Angulo_Pendulo_Medido = Get_Adc_Average(5, 5) * 2.0 * pi / 1040.0; // Lectura del ángulo del péndulo desde el sensor
  Angulo_Pendulo = Angulo_Pendulo_Medido - Cero_Angulo_Pendulo;        
  V_Angular_Pendulo = Angulo_Pendulo - Angulo_Pendulo_Pasado;          

  Angulo_Brazo = Angulo_Brazo_Medido - Cero_Angulo_Brazo; // Lectura del ángulo del brazo desde el sensor 
  Vel_Angular_Brazo = Angulo_Brazo - Angulo_Brazo_Pasado;         

  Motor =-K_Angulo_Pendulo*Angulo_Pendulo-K_Vel_Angular_Pendulo*V_Angular_Pendulo-K_Angulo_Brazo*Angulo_Brazo-K_Vel_Angular_Brazo*Vel_Angular_Brazo;  // Cálculo de acción de control

  Angulo_Pendulo_Pasado = Angulo_Pendulo; // Actualización de valores pasados                                                        
  Angulo_Brazo_Pasado = Angulo_Brazo;                                                                                                                                    

  if (Turn_Off() == 0)                                                                                       
    Set_Pwm(Motor * 255 / 12); // Pasar de voltaje a PWM                                                                              

  Temp = analogRead(0);                                                                                      
  Voltage_Count++;                                                                                          
  Voltage_All += Temp;                                                                                       
  if (Voltage_Count == 200) Voltaje_Fuente = Voltage_All * 0.05371 / 2, Voltage_All = 0, Voltage_Count = 0;  
}

// Configuración inicial del programa
void setup() {
  oled.ssd1306_init(SSD1306_SWITCHCAPVCC);   // Inicialización del display OLED y configuración de pines
  oled.clear();               
  pinMode(IN1, OUTPUT);       
  pinMode(IN2, OUTPUT);       
  pinMode(PWM, OUTPUT);       
  digitalWrite(IN1, 0);       
  digitalWrite(IN2, 0);       
  digitalWrite(PWM, 0);       
  pinMode(ENCODER_A, INPUT);  
  pinMode(ENCODER_B, INPUT);  
  Serial.begin(128000);       
  //Serial.begin(115200); 
  delay(200); 
  Serial.println("LABEL,Tiempo,Angulo Pendulo,Angulo Brazo,PWM Motor"); // Encabezados de PLX-DAQ
  MsTimer2::set(5, control);  // Configuración de la interrupción del temporizador       
  MsTimer2::start();                                     
  attachInterrupt(0, READ_ENCODER_A, CHANGE); // Configuración de las interrupciones de los pines del encoder
  attachPinChangeInterrupt(20, READ_ENCODER_B, CHANGE);  // 20 corresponde al pin digital 4.
}

// Bucle principal del programa
void loop() {
  if (digitalRead(KEY_S) == 0) {
    Flag_Stop = !Flag_Stop;
  } 
  // Datos enviados a PLX-DAQ
  Serial.print("DATA,TIME,");
  Serial.print(Angulo_Pendulo);
  Serial.print(",");
  Serial.print(Angulo_Brazo);
  Serial.print(",");
  Serial.println(Motor * 255 / 12);

  oled.drawstring(0, 00, "B-KP:");
  OLED_ShowNumber(30, 00, K_Angulo_Pendulo, 3);
  oled.drawstring(85, 00, "A:");
  OLED_ShowNumber(100, 00, Amplitude1, 3);

  oled.drawstring(0, 01, "B-KD:");
  OLED_ShowNumber(30, 01, K_Vel_Angular_Pendulo, 3);
  oled.drawstring(85, 01, "A:");
  OLED_ShowNumber(100, 01, Amplitude2, 3);

  oled.drawstring(0, 02, "P-KP:");
  OLED_ShowNumber(30, 02, K_Angulo_Brazo, 3);
  oled.drawstring(85, 02, "A:");
  OLED_ShowNumber(100, 02, Amplitude3, 3);

  oled.drawstring(0, 03, "P-KD:");
  OLED_ShowNumber(30, 03, K_Vel_Angular_Brazo, 3);
  oled.drawstring(85, 03, "A:");
  OLED_ShowNumber(100, 03, Amplitude4, 3);

  oled.drawstring(00, 4, "VOLTAGE:");
  oled.drawstring(71, 4, ".");
  oled.drawstring(93, 4, "V");
  OLED_ShowNumber(58, 4, Voltaje_Fuente / 100, 2);
  OLED_ShowNumber(81, 4, Voltaje_Fuente % 100, 2);

  oled.drawstring(00, 5, "Position:");
  OLED_ShowNumber(60, 5, Angulo_Brazo_Medido*100, 5); //Multiplicado x100 para una correcta visualización

  oled.drawstring(00, 6, "Cero_Angulo_Brazo:");
  OLED_ShowNumber(60, 6, Cero_Angulo_Brazo*100, 5);

  oled.drawstring(00, 7, "ADC:");
  OLED_ShowNumber(25, 7, Angulo_Pendulo_Medido*100 , 4); //Multiplicado x100 para una correcta visualización
  oled.drawstring(55, 7, "ORIGIN:");  
  OLED_ShowNumber(100, 7, Cero_Angulo_Pendulo*100, 3);

  oled.display();
}

// Funciones para leer los encoder del brazo
void READ_ENCODER_A() {
  if (digitalRead(ENCODER_A) == HIGH) {
    if (digitalRead(ENCODER_B) == LOW) Angulo_Brazo_Medido = Angulo_Brazo_Medido + 2.0 * pi / (1040.0 * 1); 
    else Angulo_Brazo_Medido = Angulo_Brazo_Medido - 2.0 * pi / (1040.0 * 1);
  } else {
    if (digitalRead(ENCODER_B) == LOW) Angulo_Brazo_Medido = Angulo_Brazo_Medido - 2.0 * pi / (1040.0 * 1);  
    else Angulo_Brazo_Medido = Angulo_Brazo_Medido + 2.0 * pi / (1040.0 * 1);
  }
}
void READ_ENCODER_B() {
  if (digitalRead(ENCODER_B) == LOW) {                                                                       
    if (digitalRead(ENCODER_A) == LOW) Angulo_Brazo_Medido = Angulo_Brazo_Medido + 2.0 * pi / (1040.0 * 1);  
    else Angulo_Brazo_Medido = Angulo_Brazo_Medido - 2.0 * pi / (1040.0 * 1);
  } else {                                                                                                   
    if (digitalRead(ENCODER_A) == LOW) Angulo_Brazo_Medido = Angulo_Brazo_Medido - 2.0 * pi / (1040.0 * 1);  
    else Angulo_Brazo_Medido = Angulo_Brazo_Medido + 2.0 * pi / (1040.0 * 1);
  }
}
