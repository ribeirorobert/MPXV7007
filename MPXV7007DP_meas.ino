
#define PIN_SENSOR  A8

// Ajuste se necessário:
const int   ADC_MAX = 1023;         // resolução do ADC (10 bits)
const float VSUPPLY = 5.0;          // alimentação do sensor (normalmente 5V)
const float VOUT_AT_0KPA = 2.5;  // ~2.5V em 0 kPa
const float SENS_V_PER_KPA = 0.285;   // ≈0.285714 V/kPa para o 7007
const float KPA_TO_CMH2O = 10.1972;        // 1 kPa = 10.1972 cmH2O

// Filtro média móvel
const int   N_AVG = 10;
float fifo[N_AVG];
int   idx = 0;
bool  filled = false;

// “Tara” (offset de zero em cmH2O)
float zeroOffset_cmh2o = 0.0;

void setup() {
  Serial.begin(115200);
  delay(100);

  for (int i = 0; i < N_AVG; i++) fifo[i] = 0;
}

void loop() {

  if (Serial.available() > 0) {
    char c = Serial.read();

    if (c == 'c') {
      zeroSensor(10);
    }
  }

  // Leitura e filtragem
  float v = readVoltage(PIN_SENSOR);
  float cmH2O = voltageToCmH2O(v) - zeroOffset_cmh2o;
  float cmH2O_f = movingAverage(cmH2O);

  // Saída
  static unsigned long t0 = 0;
  if (millis() - t0 >= 10) { // a cada 100 ms
    t0 = millis();

    Serial.println(cmH2O_f, 2);
  }
}

// -------- Funções --------
float readVoltage(byte pin) {
  int raw = analogRead(pin);
  return (raw * (VSUPPLY / ADC_MAX));
}

float voltageToCmH2O(float vout) {
  // Converte tensão -> kPa -> cmH2O
  float pkPa = (vout - VOUT_AT_0KPA) / SENS_V_PER_KPA; // kPa (positivo = P1 > P2)
  return pkPa * KPA_TO_CMH2O;                           // cmH2O
}

float movingAverage(float x) {
  fifo[idx++] = x;
  if (idx >= N_AVG) { idx = 0; filled = true; }
  int n = filled ? N_AVG : idx;
  float s = 0;
  for (int i = 0; i < n; i++) s += fifo[i];
  return s / n;
}

void zeroSensor(int samples) {
  // Mede parado e define offset (tara) em cmH2O
  float s = 0;
  for (int i = 0; i < samples; i++) {
    s += voltageToCmH2O(readVoltage(PIN_SENSOR));
    delay(5);
  }
  zeroOffset_cmh2o = s / samples;
}
