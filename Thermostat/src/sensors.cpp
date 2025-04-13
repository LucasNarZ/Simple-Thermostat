#include <Arduino.h>
#include <sensors.h>
#include <esp_adc_cal.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

const double beta = 3950.0;
const double r0 = 100000.0;
const double t0 = 273.15 + 25.0;
const double rx = r0 * exp(-beta/t0);
const double vcc = 1.2;
const double R = 100000.0;
double vref_esp;

esp_adc_cal_characteristics_t adc_cal;
esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, 1100, &adc_cal);

void initSensors() {
    pinMode(NTC_PIN, INPUT);
    // pinMode(LDR_PIN, INPUT);
    pinMode(FLOOR_SENSOR_PIN, INPUT);
    analogSetPinAttenuation(NTC_PIN, ADC_0db);
    analogSetPinAttenuation(FLOOR_SENSOR_PIN, ADC_0db);

    if (adc_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
	{
		ESP_LOGI("ADC CAL", "Vref eFuse encontrado: %umV", adc_cal.vref);
        Serial.print("Vref eFuse encontrado: ");Serial.println(adc_cal.vref);
	}
	else if (adc_type == ESP_ADC_CAL_VAL_EFUSE_TP)
	{
		ESP_LOGI("ADC CAL", "Two Point eFuse encontrado");
        Serial.print("Two Point eFuse encontrado: ");Serial.println(adc_cal.vref);
	}
	else
	{
		ESP_LOGW("ADC CAL", "Nada encontrado, utilizando Vref padrao: %umV", adc_cal.vref);
        Serial.print("utilizando Vref padrao: ");Serial.println(adc_cal.vref);
	}

    vref_esp = adc_cal.vref / 1000.0;
}

bool hasFloorSensor() {
    int adc = 0;
    for (int i = 0; i < 20; i++){
        int16_t rawAdc = analogRead(FLOOR_SENSOR_PIN);
        adc += rawAdc;
        Serial.print("RAW: ");Serial.print(rawAdc);
        Serial.print("\tadc: ");Serial.println(adc);

        delay(5);
    }
    adc = adc/20;
    Serial.print("ADC: ");Serial.println(adc);

    if (adc > 800) {
        Serial.println("com sensor de piso");
        return true;
    }
    Serial.println("Sem sensor de piso");
    return false;
}

float getSensorTemperature(bool sensorPiso) {
    int16_t raw;
    float voltage, resistance, temperature;
    float tempSum = 0;
    int samples = 10;

    for (int i = 0; i < samples; i++) {
        raw = analogRead(sensorPiso ? FLOOR_SENSOR_PIN : NTC_PIN);

        voltage = (vref_esp * raw) / 4095.0;
        resistance = (vcc * R) / voltage - R;
        temperature = beta / log(resistance / rx);
        temperature -= 273.15; // Kelvin para Celsius

        tempSum += temperature;
    }

    float avg_temp = tempSum / samples;

    if ((isnan(avg_temp) || avg_temp <= 0) || isinf(avg_temp) || avg_temp > 60) {
        return false;
    }

    // Correção empírica e arredondamento
    avg_temp = avg_temp - (1 + (avg_temp < 19 ? 0.3 : avg_temp < 23 ? 0.6 : 0.8));
    avg_temp = round(avg_temp * 2) / 2;

    return avg_temp;
}

// int getLDRvalue(){
//     int ldr_reading = 0;
//     for(int i = 0; i < 50; i++){
//         ldr_reading += analogRead(LDR_PIN);
//     }
//     ldr_reading = ldr_reading/50;
//     // Serial.print("LDR: ");Serial.println(ldr_reading);
//     return ldr_reading;
// }

// avg_temp = avg_temp - (1+(avg_temp < 19? 0.3 : avg_temp < 23? 0.6:0.8))
