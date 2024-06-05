
#define SENSOR_LAT_DIR 16   //_SDD
#define SENSOR_FRONT_DIR 17 //_SDC
#define SENSOR_LAT_ESQ 18   //_SDA
#define SENSOR_FRONT_ESQ 5  //_SDB
#define LEDPIN 2
#define SENSOR_DIAGONAL_DIR 12 // SDIAGOD **
#define SENSOR_DIAGONAL_ESQ 27 // SDIAGOE **

#define IN1_A 33 // entrada motor A IN1
#define IN2_A 32 // entrada motor A IN2
#define IN3_B 26 // entrada motor B IN3
#define IN4_B 25 // entrada motor B IN4

#define SENSOR_LINHA_A 36 // SLA 36 E
#define SENSOR_LINHA_B 39 // SLB 39 D

#define PWMA 23 // PWMA
#define PWMB 19 // PWMB

#define MODULO_START 14 // SIN

#define SERVICE_UUID "09e15beb-7685-4c74-babc-c1be7c182b9e"
#define CHARACTERISTIC_UUID "4e9badb4-3f7b-475c-928d-67ad57c5147b"

int linha_direita = 0;
int linha_esquerda = 0;
int distancia_lateral_esquerdo = 0;
int distancia_frontal_esquerdo = 0;
int distancia_frontal_direita = 0;
int distancia_lateral_direita = 0;
int velmotor = 0;
int parametro = 0;