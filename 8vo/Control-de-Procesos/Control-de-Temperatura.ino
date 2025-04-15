#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin donde se conecta el sensor DS18B20
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Pin para controlar el SSR (asegúrate de que acepte señal PWM)
#define SSR_PIN 9

// Variables para PID
double Setpoint, Input, Output;
// Ajusta estos parámetros según tu sistema
double Kp = 2.0, Ki = 5.0, Kd = 1.0;

// Crea el objeto PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  sensors.begin();
  
  // Inicializa el pin del SSR
  pinMode(SSR_PIN, OUTPUT);
  
  // Define el setpoint deseado (por ejemplo, 60°C, ajusta según tu necesidad)
  Setpoint = 60.0;
  
  // Inicializa el PID
  myPID.SetMode(AUTOMATIC);
  // Ajusta el rango de salida acorde a la señal PWM (0-255)
  myPID.SetOutputLimits(0, 255);
}

void loop() {
  // Solicita al sensor la temperatura
  sensors.requestTemperatures();
  Input = sensors.getTempCByIndex(0);  // Obtiene la temperatura del primer sensor
  
  // Calcula la salida del PID
  myPID.Compute();
  
  // Aplica la salida al SSR. Si el SSR solo opera como interruptor digital,
  // podrías usar un umbral (por ejemplo, si Output > 127, encender; sino, apagar)
  analogWrite(SSR_PIN, (int)Output);
  
  // Imprime la temperatura y la salida para diagnóstico
  Serial.print("Temperatura: ");
  Serial.print(Input);
  Serial.print(" °C\tSalida PID: ");
  Serial.println(Output);
  
  delay(1000); // Espera 1 segundo entre lecturas (ajustable según necesidad)
}
