#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>

#define TIME_STEP 256
#define QtddSensoresProx 8
#define QtddLeds 10
#define MOVEMENT_THRESHOLD 0.01
#define MAX_SPEED 6.28
#define num_boxes 10
#define BACK_UP_STEPS 10
#define TURN_STEPS 8

int main(int argc, char **argv)
{
  wb_robot_init();

  // Configure motors
  WbDeviceTag MotorEsquerdo = wb_robot_get_device("left wheel motor");
  WbDeviceTag MotorDireito = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);

  // Configure proximity sensors
  WbDeviceTag SensorProx[QtddSensoresProx];
  for (int i = 0; i < QtddSensoresProx; i++)
  {
    char sensorName[5];
    sprintf(sensorName, "ps%d", i);
    SensorProx[i] = wb_robot_get_device(sensorName);
    wb_distance_sensor_enable(SensorProx[i], TIME_STEP);
  }

  // Configure LEDs
  WbDeviceTag Leds[QtddLeds];
  for (int i = 0; i < QtddLeds; i++)
  {
    char ledName[5];
    sprintf(ledName, "led%d", i);
    Leds[i] = wb_robot_get_device(ledName);
    wb_led_set(Leds[i], 0); // Initially off
  }

  // Get references to the box nodes
  const char *box_def_names[] = {"BOX1", "BOX2", "BOX3", "BOX4", "BOX5", "BOX6", "BOX7", "BOX8", "BOX9", "BOX10"};
  WbNodeRef box_nodes[num_boxes];
  for (int i = 0; i < num_boxes; i++)
  {
    box_nodes[i] = wb_supervisor_node_get_from_def(box_def_names[i]);
  }

  double initialPositions[num_boxes][3];
  double finalPositions[num_boxes][3];

  bool backing_up = false;
  int back_up_steps = 0;
  int turn_steps = 0;

  while (wb_robot_step(TIME_STEP) != -1)
  {
    double left_motor_speed = MAX_SPEED;
    double right_motor_speed = MAX_SPEED;

    // Se o robô estiver no meio do processo de recuo e virada
    if (backing_up)
    {
      if (back_up_steps > 0)
      {
        // Recuar
        left_motor_speed = -MAX_SPEED / 2;
        right_motor_speed = -MAX_SPEED / 2;
        back_up_steps--;
      }
      else if (turn_steps > 0)
      {
        // Virar para uma direção aleatória
        left_motor_speed = (rand() % 2) ? MAX_SPEED : -MAX_SPEED;
        right_motor_speed = (rand() % 2) ? MAX_SPEED : -MAX_SPEED;
        turn_steps--;
      }
      else
      {
        // Terminou de recuar e virar, voltar ao comportamento normal
        backing_up = false;
      }
    }
    else
    {
      // Ler sensores de proximidade para evasão de obstáculos
      for (int i = 0; i < QtddSensoresProx; i++)
      {
        double sensorValue = wb_distance_sensor_get_value(SensorProx[i]);

        if (sensorValue > 120)
        { // Limiar de detecção do obstáculo
          if (i < QtddSensoresProx / 2)
          {
            // Obstáculo detectado à esquerda, vire à direita
            right_motor_speed -= (sensorValue / 4096) * MAX_SPEED;
          }
          else
          {
            // Obstáculo detectado à direita, vire à esquerda
            left_motor_speed -= (sensorValue / 4096) * MAX_SPEED;
          }
          // Se o obstáculo estiver muito perto, inicie o processo de recuo e virada
          if (sensorValue > 200)
          { // Valor alto, indicando proximidade
            backing_up = true;
            back_up_steps = BACK_UP_STEPS;
            turn_steps = TURN_STEPS;
          }
        }
      }
    }

    // Normalizar as velocidades
    left_motor_speed = fmax(fmin(left_motor_speed, MAX_SPEED), -MAX_SPEED);
    right_motor_speed = fmax(fmin(right_motor_speed, MAX_SPEED), -MAX_SPEED);

    // Definir velocidades dos motores
    wb_motor_set_velocity(MotorEsquerdo, left_motor_speed);
    wb_motor_set_velocity(MotorDireito, right_motor_speed);

    // Get initial positions of all boxes
    for (int i = 0; i < num_boxes; i++)
    {
      const double *position = wb_supervisor_node_get_position(box_nodes[i]);
      for (int j = 0; j < 3; j++)
      {
        initialPositions[i][j] = position[j];
      }
    }

    // Allow the simulation to proceed to update box positions
    wb_robot_step(TIME_STEP);

    // Check if any box has moved
    bool any_box_moved = false;
    for (int i = 0; i < num_boxes; i++)
    {
      const double *position = wb_supervisor_node_get_position(box_nodes[i]);
      for (int j = 0; j < 3; j++)
      {
        finalPositions[i][j] = position[j];
      }

      if (fabs(finalPositions[i][0] - initialPositions[i][0]) > MOVEMENT_THRESHOLD ||
          fabs(finalPositions[i][1] - initialPositions[i][1]) > MOVEMENT_THRESHOLD ||
          fabs(finalPositions[i][2] - initialPositions[i][2]) > MOVEMENT_THRESHOLD)
      {
        printf("A caixa %d se moveu.\n", i + 1);
        any_box_moved = true;
      }
    }

    if (any_box_moved)
    {
      wb_motor_set_velocity(MotorEsquerdo, 0);
      wb_motor_set_velocity(MotorDireito, 0);

      for (int i = 0; i < QtddLeds; i++)
      {
        wb_led_set(Leds[i], 1);
      }
      break;
    }
  }

  wb_robot_cleanup();
  return 0;
}
