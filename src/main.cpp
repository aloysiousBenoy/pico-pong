#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <Wire.h>
#include <MMA7660.h>
#include <U8g2lib.h>


#define SENSOR_PERIOD 50
#define BALL_UPDATE_PERIOD 20
#define DISP_Y_MAX 32
#define DISP_X_MAX 128

MMA7660 accel;
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE, PICO_DEFAULT_I2C_SCL_PIN, PICO_DEFAULT_I2C_SDA_PIN);

QueueHandle_t sensor_result;

typedef struct SensorResult
{
  int8_t x;
  int8_t y;
} SensorResult;

void task_read_sensors(void *pvParameters)
{

  int avg_x = 0;
  int avg_y = 0;
  int max_x = 10, max_y = 10;
  int count = 0;
  SensorResult res;

  int8_t x, y, z;
  while (1)
  {
    // we have to avoid pre-emption while accessing the i2c bus
    vTaskEnterCritical();
    accel.getXYZ(&x, &y, &z);
    vTaskExitCritical();

    avg_x += x;
    avg_y += y;
    if (x > max_x)
      max_x = x;
    if (y > max_y)
      max_y = y;

    count++;

    res.x = map(avg_x / count, -1 * max_x, max_x, 0, DISP_Y_MAX);
    res.y = map(avg_y / count, -1 * max_y, max_y, 0, DISP_Y_MAX);
    xQueueSend(sensor_result, &res, 0);

    if (count >= 2)
    {
      count = 0;
      avg_x = 0;
      avg_y = 0;
    }
    vTaskDelay(SENSOR_PERIOD / portTICK_PERIOD_MS);
  }
}

void task_draw_display(void *pvParameters)
{
  SensorResult result;
  const int disp_cx = DISP_X_MAX / 2;
  const int disp_cy = DISP_Y_MAX / 2;

  uint8_t ball_x = disp_cx, ball_y = disp_cy;
  int8_t ball_dx = 1, ball_dy = -1;
  uint8_t score_a = 0, score_b = 0;
  while (1)
  {
    if (xQueueReceive(sensor_result, &result, 0))
    {
      oled.clearBuffer();
      oled.drawBox(1, result.y - 1, 3, 8);
      oled.drawBox(125, result.x - 1, 3, 8);
    }
    // score board and boundary
    uint8_t cl = oled.getDrawColor();
    oled.setDrawColor(75);
    oled.drawVLine(disp_cx, 0, DISP_Y_MAX);
    oled.setDrawColor(cl);

    oled.setFont(u8g2_font_ncenB10_tr);

    oled.setCursor(disp_cx - 15, disp_cy + 7);
    oled.print(score_a);
    oled.setCursor(disp_cx + 15, disp_cy + 7);
    oled.print(score_b);

    // move ball
    ball_x += ball_dx;
    ball_y += ball_dy;

    // collision mechanics with paddles
    if (ball_x <= 4)
    {
      if (ball_y >= result.y - 1 && ball_y <= result.y + 8)
      {
        ball_dx = 1;
      }
    }
    if (ball_x >= 124)
    {
      if (ball_y >= result.x - 1 && ball_y <= result.x + 8)
      {
        ball_dx = -1;
      }
    }

    // reflect on sides
    if (ball_y <= 0)
    {
      ball_dy = 1;
    }

    if (ball_x >= DISP_X_MAX)
    {
      ball_dx = -1;
    }
    if (ball_y >= DISP_Y_MAX)
    {
      ball_dy = -1;
    }

    // reset ball and inc point
    if (ball_x <= 0)
    {
      score_b++;
      ball_x = disp_cx;
      ball_y = disp_cy;
    }
    if (ball_x >= 127)
    {
      score_a++;
      ball_x = disp_cx;
      ball_y = disp_cy;
    }

    oled.drawPixel(ball_x, ball_y);

    // we have to avoid pre-emption while accessing the i2c bus
    vTaskEnterCritical();
    oled.sendBuffer();
    vTaskExitCritical();

    vTaskDelay(BALL_UPDATE_PERIOD / portTICK_PERIOD_MS);
  }
}

void setup()
{

  Wire.setSDA(PICO_DEFAULT_I2C_SDA_PIN);
  Wire.setSCL(PICO_DEFAULT_I2C_SCL_PIN);
  Wire.setClock(100 * 1000);
  // for reasons unknown we have to initialise these in this order
  // oled first then accelerometer
  oled.begin();
  accel.init();

  sensor_result = xQueueCreate(5, sizeof(SensorResult *));

  xTaskCreate(task_read_sensors, "Read data from sensor", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);
  xTaskCreate(task_draw_display, "Draw to oled display", 5120, NULL, configMAX_PRIORITIES - 1, NULL);
}

void loop()
{
}