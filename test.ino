#include <Arduino.h>
#include <Stepper.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// Số bước cho mỗi vòng quay của động cơ (2048 bước cho 28BYJ-48 với tỉ lệ giảm tốc 64:1)
const int STEPS_PER_REVOLUTION = 2048;

// Định nghĩa các chân
#define IN1 19           // GPIO 19
#define IN2 18           // GPIO 18
#define IN3 5            // GPIO 5
#define IN4 17           // GPIO 17
#define DIRECTION_BUTTON_PIN 14  // GPIO 14 cho nút đảo chiều
#define STOP_BUTTON_PIN 27       // GPIO 27 cho nút dừng
#define DHT11_PIN 4      // GPIO 4 cho DHT11
#define SOIL_MOISTURE_PIN 34 // GPIO 34 (ADC1_6) cho độ ẩm đất
#define LED1_PIN 15      // GPIO 15 cho LED1 (nhiệt độ < 25°C)
#define LED2_PIN 25      // GPIO 25 cho LED2 (độ ẩm đất > 50%)
#define LED3_PIN 2      // GPIO 26 cho LED3 (nhiệt độ > 30°C)

// Khởi tạo đối tượng
Stepper myStepper(STEPS_PER_REVOLUTION, IN1, IN3, IN2, IN4);
#define DHTTYPE DHT11
DHT dht(DHT11_PIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 20, 4); // LCD I2C (địa chỉ 0x27, 20x4)

// Biến toàn cục cho động cơ
bool isClockwise = true; // true: thuận, false: ngược
SemaphoreHandle_t directionSemaphore;
SemaphoreHandle_t stopSemaphore;
SemaphoreHandle_t directionMutex;

// Queue cho dữ liệu cảm biến
QueueHandle_t sensorQueue;

// Cấu trúc dữ liệu cảm biến
typedef struct {
    float temperature;     // Nhiệt độ từ DHT11
    float humidity;        // Độ ẩm không khí từ DHT11
    int soil_moisture;     // Độ ẩm đất (giá trị analog)
    float soil_percentage; // Độ ẩm đất tính theo phần trăm
} sensor_data_t;

// Task xử lý hai nút nhấn
void buttonTask(void *pvParameters) {
    int lastDirectionButtonState = HIGH;
    int lastStopButtonState = HIGH;

    while (1) {
        // Đọc trạng thái hai nút
        int directionButtonState = digitalRead(DIRECTION_BUTTON_PIN);
        int stopButtonState = digitalRead(STOP_BUTTON_PIN);

        // Xử lý nút đảo chiều
        if (directionButtonState == LOW && lastDirectionButtonState == HIGH) {
            // Phát hiện cạnh xuống: đảo chiều và kích hoạt quay
            if (xSemaphoreTake(directionMutex, portMAX_DELAY) == pdTRUE) {
                isClockwise = !isClockwise;
                Serial.println(isClockwise ? "Direction Button Pressed: Set to Clockwise" : 
                                           "Direction Button Pressed: Set to Counterclockwise");
                xSemaphoreGive(directionMutex);
            }
            // Báo hiệu quay liên tục
            xSemaphoreGive(directionSemaphore);
            vTaskDelay(50 / portTICK_PERIOD_MS); // Chống dội phím
        }

        // Xử lý nút dừng
        if (stopButtonState == LOW && lastStopButtonState == HIGH) {
            // Phát hiện cạnh xuống: báo hiệu dừng
            xSemaphoreGive(stopSemaphore);
            Serial.println("Stop Button Pressed: Motor Stopped");
            vTaskDelay(50 / portTICK_PERIOD_MS); // Chống dội phím
        }

        // Cập nhật trạng thái nút
        lastDirectionButtonState = directionButtonState;
        lastStopButtonState = stopButtonState;

        vTaskDelay(10 / portTICK_PERIOD_MS); // Kiểm tra mỗi 10ms
    }
}

// Task điều khiển động cơ
void motorTask(void *pvParameters) {
    bool localIsClockwise = true;

    while (1) {
        // Kiểm tra nút dừng trước
        if (xSemaphoreTake(stopSemaphore, 0) == pdTRUE) {
            // Nút dừng được nhấn: lấy directionSemaphore để ngăn quay
            xSemaphoreTake(directionSemaphore, 0);
            continue; // Bỏ qua quay động cơ
        }

        // Kiểm tra trạng thái quay
        if (xSemaphoreTake(directionSemaphore, 0) == pdTRUE) {
            if (xSemaphoreTake(directionMutex, portMAX_DELAY) == pdTRUE) {
                localIsClockwise = isClockwise;
                xSemaphoreGive(directionMutex);
            }

            if (localIsClockwise) {
                myStepper.step(10); // 10 bước thuận
                Serial.println("Motor Task: Rotating Clockwise");
            } else {
                myStepper.step(-10); // 10 bước ngược
                Serial.println("Motor Task: Rotating Counterclockwise");
            }
            xSemaphoreGive(directionSemaphore);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // Lặp mỗi 10ms
    }
}

// Task đọc cảm biến
void sensor_task(void *pvParameters) {
    sensor_data_t sensor_data;

    while (1) {
        // Đọc độ ẩm đất
        int soil_value = analogRead(SOIL_MOISTURE_PIN);
        int soil_min = 1000; // Đất ướt
        int soil_max = 3500; // Đất khô
        sensor_data.soil_moisture = soil_value;
        sensor_data.soil_percentage = map(soil_value, soil_min, soil_max, 100, 0);
        sensor_data.soil_percentage = constrain(sensor_data.soil_percentage, 0, 100);

        // Đọc DHT11
        float h = dht.readHumidity();
        float t = dht.readTemperature();
        if (!isnan(h) && !isnan(t)) {
            sensor_data.temperature = t;
            sensor_data.humidity = h;
        } else {
            sensor_data.temperature = -1;
            sensor_data.humidity = -1;
        }

        // Gửi dữ liệu vào queue
        xQueueSend(sensorQueue, &sensor_data, portMAX_DELAY);
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Đọc mỗi 2 giây
    }
}

// Task hiển thị và điều khiển
void display_and_control_task(void *pvParameters) {
    sensor_data_t sensor_data;
    bool led1_state = false;
    bool led2_state = false;
    bool led3_state = false;

    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(LED3_PIN, OUTPUT);
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW);

    while (1) {
        // Nhận dữ liệu cảm biến
        if (xQueueReceive(sensorQueue, &sensor_data, 0) == pdTRUE) {
            // Điều khiển LED1 (nhiệt độ < 25°C)
            if (sensor_data.temperature != -1) {
                led1_state = (sensor_data.temperature < 25.0);
                digitalWrite(LED1_PIN, led1_state);
                Serial.printf("LED 1: %s (Temperature: %.1f°C)\n", 
                              led1_state ? "ON" : "OFF", sensor_data.temperature);
            }

            // Điều khiển LED2 (độ ẩm đất > 50%)
            led2_state = (sensor_data.soil_percentage > 50.0);
            digitalWrite(LED2_PIN, led2_state);
            Serial.printf("LED 2: %s (Soil Moisture: %.1f%%)\n", 
                          led2_state ? "ON" : "OFF", sensor_data.soil_percentage);

            // Điều khiển LED3 (nhiệt độ > 30°C)
            if (sensor_data.temperature != -1) {
                led3_state = (sensor_data.temperature > 30.0);
                digitalWrite(LED3_PIN, led3_state);
                Serial.printf("LED 3: %s (Temperature: %.1f°C)\n", 
                              led3_state ? "ON" : "OFF", sensor_data.temperature);
            }

            // Hiển thị trên LCD
            lcd.clear();

            // Dòng 1: Nhiệt độ
            if (sensor_data.temperature != -1) {
                char line1[21];
                snprintf(line1, sizeof(line1), "Temp: %.1f C", sensor_data.temperature);
                lcd.setCursor(0, 0);
                lcd.print(line1);
            } else {
                lcd.setCursor(0, 0);
                lcd.print("Temp: Error");
            }

            // Dòng 2: Độ ẩm không khí
            if (sensor_data.humidity != -1) {
                char line2[21];
                snprintf(line2, sizeof(line2), "Hum: %.1f %%", sensor_data.humidity);
                lcd.setCursor(0, 1);
                lcd.print(line2);
            } else {
                lcd.setCursor(0, 1);
                lcd.print("Hum: Error");
            }

            // Dòng 3: Độ ẩm đất
            char line3[21];
            snprintf(line3, sizeof(line3), "Soil: %.1f %%", sensor_data.soil_percentage);
            lcd.setCursor(0, 2);
            lcd.print(line3);

            // Dòng 4: Trạng thái động cơ
            char line4[21];
            bool isMotorRunning = (xSemaphoreTake(directionSemaphore, 0) == pdTRUE);
            bool isStopped = (xSemaphoreTake(stopSemaphore, 0) == pdTRUE);
            if (xSemaphoreTake(directionMutex, portMAX_DELAY) == pdTRUE) {
                snprintf(line4, sizeof(line4), "Motor: %s", 
                         isStopped ? "Stop" : (isMotorRunning ? (isClockwise ? "CW" : "CCW") : "Stop"));
                xSemaphoreGive(directionMutex);
            }
            if (isMotorRunning) {
                xSemaphoreGive(directionSemaphore); // Trả semaphore
            }
            if (isStopped) {
                xSemaphoreGive(stopSemaphore); // Trả semaphore
            }
            lcd.setCursor(0, 3);
            lcd.print(line4);

            // In Serial để debug
            if (sensor_data.temperature != -1 && sensor_data.humidity != -1) {
                Serial.printf("Temperature: %.1f°C, Humidity: %.1f%%\n", 
                              sensor_data.temperature, sensor_data.humidity);
            } else {
                Serial.println("Error reading DHT11");
            }
            Serial.printf("Soil Moisture (Raw): %d, Soil Moisture: %.1f%%\n", 
                          sensor_data.soil_moisture, sensor_data.soil_percentage);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Cập nhật LCD mỗi 100ms
    }
}

void setup() {
    // Khởi tạo Serial
    Serial.begin(115200);
    Serial.println("ESP32 Sensor and Stepper Control");

    // Khởi tạo LCD
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Starting...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    lcd.clear();

    // Khởi tạo DHT11
    dht.begin();

    // Cấu hình chân
    pinMode(DIRECTION_BUTTON_PIN, INPUT_PULLUP);
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(SOIL_MOISTURE_PIN, INPUT);

    // Đặt tốc độ động cơ
    myStepper.setSpeed(15); // 15 RPM

    // Tạo queue cho cảm biến
    sensorQueue = xQueueCreate(10, sizeof(sensor_data_t));
    if (sensorQueue == NULL) {
        Serial.println("Failed to create sensor queue");
        lcd.setCursor(0, 0);
        lcd.print("Queue Error");
        while (1);
    }

    // Tạo semaphore cho nút đảo chiều
    directionSemaphore = xSemaphoreCreateBinary();
    if (directionSemaphore == NULL) {
        Serial.println("Failed to create direction semaphore");
        lcd.setCursor(0, 0);
        lcd.print("Semaphore Error");
        while (1);
    }

    // Tạo semaphore cho nút dừng
    stopSemaphore = xSemaphoreCreateBinary();
    if (stopSemaphore == NULL) {
        Serial.println("Failed to create stop semaphore");
        lcd.setCursor(0, 0);
        lcd.print("Semaphore Error");
        while (1);
    }

    // Tạo mutex cho isClockwise
    directionMutex = xSemaphoreCreateMutex();
    if (directionMutex == NULL) {
        Serial.println("Failed to create direction mutex");
        lcd.setCursor(0, 0);
        lcd.print("Mutex Error");
        while (1);
    }

    // Tạo các task
    xTaskCreate(sensor_task, "Sensor Task", 2048, NULL, 1, NULL);
    xTaskCreate(display_and_control_task, "Display Task", 4096, NULL, 1, NULL);
    xTaskCreate(buttonTask, "Button Task", 2048, NULL, 1, NULL);
    xTaskCreate(motorTask, "Motor Task", 2048, NULL, 1, NULL);
}

void loop() {
    // Để trống vì FreeRTOS quản lý các task
}
