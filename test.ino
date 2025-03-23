#include <Arduino.h>
#include <DHT.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Định nghĩa chân GPIO
#define DHT11_PIN 4         // Chân cho DHT11
#define SOIL_MOISTURE_PIN 34 // Chân analog cho cảm biến độ ẩm đất (ADC1_6)
#define LED1_PIN 18         // Đèn LED 1
#define LED2_PIN 19         // Đèn LED 2
#define BUTTON1_PIN 14      // Nút nhấn điều khiển LED 1
#define BUTTON2_PIN 15      // Nút nhấn điều khiển LED 2

// Khởi tạo đối tượng DHT
#define DHTTYPE DHT11
DHT dht(DHT11_PIN, DHTTYPE);

// Khai báo Queue handle
QueueHandle_t sensorQueue;  // Queue cho dữ liệu cảm biến
QueueHandle_t buttonQueue;  // Queue cho lệnh từ nút nhấn

// Cấu trúc dữ liệu cho cảm biến
typedef struct {
    float temperature;     // Nhiệt độ từ DHT11
    float humidity;        // Độ ẩm không khí từ DHT11
    int soil_moisture;     // Độ ẩm đất (giá trị analog)
    float soil_percentage; // Độ ẩm đất tính theo phần trăm
} sensor_data_t;

// Cấu trúc dữ liệu cho trạng thái nút nhấn
typedef struct {
    int led_num;  // 1 hoặc 2 để xác định đèn LED
    int state;    // 0: tắt, 1: bật
} button_data_t;

// Task đọc cảm biến độ ẩm đất và DHT11
void sensor_task(void *pvParameters) {
    sensor_data_t sensor_data;
    
    while (1) {
        // Đọc giá trị analog từ cảm biến độ ẩm đất
        int soil_value = analogRead(SOIL_MOISTURE_PIN);
        
        // Chuyển đổi giá trị analog thành phần trăm (giả sử giá trị từ 0-4095)
        // 0 (rất ướt) -> 4095 (rất khô)
        int soil_min = 1000;  // Giá trị khi đất ướt hoàn toàn (có thể hiệu chỉnh)
        int soil_max = 3500;  // Giá trị khi đất khô hoàn toàn (có thể hiệu chỉnh)
        sensor_data.soil_moisture = soil_value;
        sensor_data.soil_percentage = map(soil_value, soil_min, soil_max, 100, 0);
        sensor_data.soil_percentage = constrain(sensor_data.soil_percentage, 0, 100);

        // Đọc dữ liệu từ DHT11
        float h = dht.readHumidity();
        float t = dht.readTemperature();
        
        if (!isnan(h) && !isnan(t)) {
            sensor_data.temperature = t;
            sensor_data.humidity = h;
        } else {
            sensor_data.temperature = -1; // Báo lỗi nếu không đọc được
            sensor_data.humidity = -1;
        }
        
        // Gửi dữ liệu vào queue
        xQueueSend(sensorQueue, &sensor_data, portMAX_DELAY);
        
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Đọc mỗi 2 giây
    }
}

// Task xử lý nút nhấn
void button_task(void *pvParameters) {
    button_data_t button_data;
    
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
    
    int last_state1 = HIGH, last_state2 = HIGH;
    
    while (1) {
        int state1 = digitalRead(BUTTON1_PIN);
        int state2 = digitalRead(BUTTON2_PIN);
        
        // Phát hiện thay đổi trạng thái nút 1
        if (state1 != last_state1) {
            if (state1 == LOW) { // Nhấn nút (mức thấp)
                button_data.led_num = 1;
                button_data.state = !digitalRead(LED1_PIN); // Đảo trạng thái
                xQueueSend(buttonQueue, &button_data, portMAX_DELAY);
            }
            last_state1 = state1;
        }
        
        // Phát hiện thay đổi trạng thái nút 2
        if (state2 != last_state2) {
            if (state2 == LOW) { // Nhấn nút (mức thấp)
                button_data.led_num = 2;
                button_data.state = !digitalRead(LED2_PIN); // Đảo trạng thái
                xQueueSend(buttonQueue, &button_data, portMAX_DELAY);
            }
            last_state2 = state2;
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS); // Chống dội phím
    }
}

// Task hiển thị dữ liệu cảm biến và điều khiển LED
void display_and_control_task(void *pvParameters) {
    sensor_data_t sensor_data;
    button_data_t button_data;
    
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    
    while (1) {
        // Nhận dữ liệu từ queue cảm biến
        if (xQueueReceive(sensorQueue, &sensor_data, 0) == pdTRUE) {
            // Hiển thị dữ liệu
            if (sensor_data.temperature != -1 && sensor_data.humidity != -1) {
                Serial.printf("Temperature: %.1f°C, Humidity: %.1f%%\n", 
                              sensor_data.temperature, sensor_data.humidity);
            } else {
                Serial.println("Error reading DHT11");
            }
            
            Serial.printf("Soil Moisture (Raw): %d, Soil Moisture: %.1f%%\n", 
                          sensor_data.soil_moisture, sensor_data.soil_percentage);
        }
        
        // Nhận dữ liệu từ queue nút nhấn
        if (xQueueReceive(buttonQueue, &button_data, 0) == pdTRUE) {
            if (button_data.led_num == 1) {
                digitalWrite(LED1_PIN, button_data.state);
                Serial.printf("LED 1: %s\n", button_data.state ? "ON" : "OFF");
            } else if (button_data.led_num == 2) {
                digitalWrite(LED2_PIN, button_data.state);
                Serial.printf("LED 2: %s\n", button_data.state ? "ON" : "OFF");
            }
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    dht.begin();
    
    // Cấu hình chân analog cho cảm biến độ ẩm đất
    pinMode(SOIL_MOISTURE_PIN, INPUT);
    
    // Tạo queues
    sensorQueue = xQueueCreate(10, sizeof(sensor_data_t));
    buttonQueue = xQueueCreate(10, sizeof(button_data_t));
    
    if (sensorQueue == NULL || buttonQueue == NULL) {
        Serial.println("Failed to create queue");
        return;
    }
    
    // Tạo các task
    xTaskCreate(sensor_task, "sensor_task", 2048, NULL, 1, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 1, NULL);
    xTaskCreate(display_and_control_task, "display_and_control_task", 2048, NULL, 1, NULL);
}

void loop() {
    // Để trống vì FreeRTOS quản lý các task
}