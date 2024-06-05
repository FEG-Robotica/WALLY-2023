#include <Arduino.h>
#include "defines.h"

const double corrd = 1;
const double corre = 1;

void mover_motor(char lado, char direcao, int velocidade)
{
  if (lado == 'd')
  {
    if (direcao == 'f')
    { // ir para frente
      digitalWrite(IN1_A, HIGH);
      digitalWrite(IN2_A, LOW);
      analogWrite(PWMA, velocidade * corrd);
    }

    if (direcao == 't')
    { // ir para trás
      digitalWrite(IN1_A, LOW);
      digitalWrite(IN2_A, HIGH);
      analogWrite(PWMA, velocidade * corrd);
    }
  }

  if (lado == 'e')
  {
    if (direcao == 'f')
    { // ir para frente
      digitalWrite(IN3_B, HIGH);
      digitalWrite(IN4_B, LOW);
      analogWrite(PWMB, velocidade);
    }

    if (direcao == 't')
    { // ir para trás
      digitalWrite(IN3_B, LOW);
      digitalWrite(IN4_B, HIGH);
      analogWrite(PWMB, velocidade);
    }
  }
}

void controlar_motores(int velocidade, char sentido)
{
  mover_motor('e', sentido, 400);
  mover_motor('d', sentido, 400);
}

void testar_PWM()
{

  int cicloDeTrabalho = 400;

  ledcWrite(0, cicloDeTrabalho);
  ledcWrite(1, cicloDeTrabalho);
}

void frente()
{
  mover_motor('e', 'f', 90);
  mover_motor('d', 'f', 90);
}

void tras()
{
  mover_motor('e', 't', 180);
  mover_motor('d', 't', 180);
}

void direita()
{
  mover_motor('e', 'f', 45);
  mover_motor('d', 'f', 90);
}

void esquerda()
{
  mover_motor('e', 'f', 90);
  mover_motor('d', 'f', 45);
}

void parar()
{
  digitalWrite(IN1_A, LOW);
  digitalWrite(IN2_A, LOW);
  digitalWrite(IN3_B, LOW);
  digitalWrite(IN4_B, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void girar_eixo()
{
  mover_motor('e', 'f', 100);
  mover_motor('d', 't', 100);
}

void girar_circulo()
{
  mover_motor('e', 'f', 100);
  mover_motor('d', 'f', 50);
}

void girar_180()
{
  mover_motor('e', 'f', 100);
  mover_motor('d', 't', 100);
  delay(300);
  parar();
}