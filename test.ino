#include <Arduino.h>
#include <DHT.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <LiquidCrystal_I2C.h>  // Thư viện cho LCD I2C

// Định nghĩa chân GPIO
#define DHT11_PIN 4         // Chân cho DHT11
#define SOIL_MOISTURE_PIN 34 // Chân analog cho cảm biến độ ẩm đất (ADC1_6)
#define LED1_PIN 18         // Đèn LED 1 (bật khi nhiệt độ > 30°C)
#define LED2_PIN 19         // Đèn LED 2 (bật khi độ ẩm đất > 50%)

// Khởi tạo đối tượng DHT
#define DHTTYPE DHT11
DHT dht(DHT11_PIN, DHTTYPE);

// Khởi tạo đối tượng LCD I2C (địa chỉ 0x27, LCD 20x4)
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Địa chỉ I2C có thể thay đổi (0x27 hoặc 0x3F)

// Khai báo Queue handle
QueueHandle_t sensorQueue;  // Queue cho dữ liệu cảm biến

// Cấu trúc dữ liệu cho cảm biến
typedef struct {
    float temperature;     // Nhiệt độ từ DHT11
    float humidity;        // Độ ẩm không khí từ DHT11
    int soil_moisture;     // Độ ẩm đất (giá trị analog)
    float soil_percentage; // Độ ẩm đất tính theo phần trăm
} sensor_data_t;

// Task đọc cảm biến độ ẩm đất và DHT11
void sensor_task(void *pvParameters) {
    sensor_data_t sensor_data;
    
    while (1) {
        // Đọc giá trị analog từ cảm biến độ ẩm đất
        int soil_value = analogRead(SOIL_MOISTURE_PIN);
        
        // Chuyển đổi giá trị analog thành phần trăm (giả sử giá trị từ 0-4095)
        // 0 (rất ướt) -> 4095 (rất khô)
        int soil_min = 1000;  // Giá trị khi đất ướt hoàn toàn
        int soil_max = 3500;  // Giá trị khi đất khô hoàn toàn
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

// Task hiển thị dữ liệu cảm biến và điều khiển LED
void display_and_control_task(void *pvParameters) {
    sensor_data_t sensor_data;
    bool led1_state = false; // Trạng thái LED1
    bool led2_state = false; // Trạng thái LED2
    
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    
    while (1) {
        // Nhận dữ liệu từ queue cảm biến
        if (xQueueReceive(sensorQueue, &sensor_data, 0) == pdTRUE) {
            // Điều khiển LED1 dựa trên nhiệt độ (> 30°C)
            if (sensor_data.temperature != -1) {
                led1_state = (sensor_data.temperature > 30.0);
                digitalWrite(LED1_PIN, led1_state);
                Serial.printf("LED 1: %s (Temperature: %.1f°C)\n", 
                              led1_state ? "ON" : "OFF", sensor_data.temperature);
            }
            
            // Điều khiển LED2 dựa trên độ ẩm đất (> 50%)
            led2_state = (sensor_data.soil_percentage > 50.0);
            digitalWrite(LED2_PIN, led2_state);
            Serial.printf("LED 2: %s (Soil Moisture: %.1f%%)\n", 
                          led2_state ? "ON" : "OFF", sensor_data.soil_percentage);
            
            // Xóa màn hình LCD trước khi hiển thị mới
            lcd.clear();
            
            // Dòng 1: Nhiệt độ
            if (sensor_data.temperature != -1) {
                char line1[21];
                snprintf(line1, sizeof(line1), "Temperature: %.1f C", sensor_data.temperature);
                lcd.setCursor(0, 0);
                lcd.print(line1);
            } else {
                lcd.setCursor(0, 0);
                lcd.print("Temperature: Error");
            }
            
            // Dòng 2: Độ ẩm không khí
            if (sensor_data.humidity != -1) {
                char line2[21];
                snprintf(line2, sizeof(line2), "Humidity: %.1f %%", sensor_data.humidity);
                lcd.setCursor(0, 1);
                lcd.print(line2);
            } else {
                lcd.setCursor(0, 1);
                lcd.print("Humidity: Error");
            }
            
            // Dòng 3: Độ ẩm đất
            char line3[21];
            snprintf(line3, sizeof(line3), "Soil Moisture: %.1f %%", sensor_data.soil_percentage);
            lcd.setCursor(0, 2);
            lcd.print(line3);
            
            // Dòng 4: Trạng thái LED
            char line4[21];
            snprintf(line4, sizeof(line4), "LED1:%s LED2:%s", 
                     led1_state ? "ON " : "OFF", led2_state ? "ON " : "OFF");
            lcd.setCursor(0, 3);
            lcd.print(line4);
            
            // Hiển thị qua Serial để debug
            if (sensor_data.temperature != -1 && sensor_data.humidity != -1) {
                Serial.printf("Temperature: %.1f°C, Humidity: %.1f%%\n", 
                              sensor_data.temperature, sensor_data.humidity);
            } else {
                Serial.println("Error reading DHT11");
            }
            Serial.printf("Soil Moisture (Raw): %d, Soil Moisture: %.1f%%\n", 
                          sensor_data.soil_moisture, sensor_data.soil_percentage);
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    dht.begin();
    
    // Khởi tạo LCD với kích thước 20x4
    lcd.init();    // Khởi tạo LCD 20x4
    lcd.backlight();     // Bật đèn nền
    lcd.setCursor(0, 0);
    lcd.print("Starting...");
    delay(1000);         // Hiển thị thông báo khởi động
    lcd.clear();
    
    // Cấu hình chân analog cho cảm biến độ ẩm đất
    pinMode(SOIL_MOISTURE_PIN, INPUT);
    
    // Tạo queue cho cảm biến
    sensorQueue = xQueueCreate(10, sizeof(sensor_data_t));
    
    if (sensorQueue == NULL) {
        Serial.println("Failed to create sensor queue");
        lcd.setCursor(0, 0);
        lcd.print("Queue Error");
        return;
    }
    
    // Tạo các task
    xTaskCreate(sensor_task, "sensor_task", 2048, NULL, 1, NULL);
    xTaskCreate(display_and_control_task, "display_and_control_task", 2048, NULL, 1, NULL);
}

void loop() {
    // Để trống vì FreeRTOS quản lý các task
}
