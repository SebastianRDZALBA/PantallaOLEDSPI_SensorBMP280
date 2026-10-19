#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
/* Configuracion I2C */
#define I2C_SDA_GPIO 21
#define I2C_SCL_GPIO 22
#define I2C_FREQUENCY 100000
/* Configuracion BMP280 */
#define BMP280_ADDRESS 0x76
#define BMP280_REG_ID 0xD0
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_CTRL 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESS 0xF7
#define BMP280_REG_CALIB 0x88
/* Configuracion conversiones */
#define PA_TO_MMHG 0.00750062f          /* 1 Pa = 0.00750062 mmHg */
#define SEA_LEVEL_PRESSURE_PA 101325.0f /* Presion a nivel del mar (ajustable) */

static const char *TAG = "BMP280";

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t bmp_handle;
/* Pines de la pantalla OLED */
#define OLED_SCLK GPIO_NUM_18
#define OLED_MOSI GPIO_NUM_23
#define OLED_RES GPIO_NUM_17
#define OLED_DC GPIO_NUM_16
#define OLED_CS GPIO_NUM_19
/* Configuracion de la pantalla */
#define OLED_WIDTH 128
#define OLED_PAGES 8
#define SPI_FREQUENCY 8000000
/* Coeficientes de calibracion */
typedef struct {
    uint16_t T1;
    int16_t T2, T3;
    uint16_t P1;
    int16_t P2, P3, P4, P5, P6, P7, P8, P9;
} bmp280_calib_t;
static bmp280_calib_t calib;
static int32_t t_fine;
static spi_device_handle_t oled_spi;
///****PROTOTIPOS BMP280****/////////
static esp_err_t read_registers(uint8_t reg, uint8_t *data,size_t length);/* Leer uno o varios registros */
static esp_err_t write_register(uint8_t reg, uint8_t value); /* Escribir un registro */
static uint16_t read_u16(const uint8_t *data);/* Convertir dos bytes little-endian a entero sin signo */
static esp_err_t i2c_init(void);/* Inicializar el bus I2C y registrar el sensor */
static esp_err_t read_calibration(void);/* Leer los coeficientes almacenados en el sensor */
static esp_err_t bmp280_init_sensor(void);/* Inicializar y configurar el BMP280 */
static float calculate_temperature(int32_t adc_T);/* Calcular temperatura a partir del ADC */
static float calculate_pressure(int32_t adc_P);/* Calcular presion en Pa usando t_fine */
static float calculate_altitude(float pressure_pa);/* Calcular altitud en metros a partir de la presion en Pa */
static esp_err_t bmp280_read_sensor(float *temperature_out,float *pressure_out,float *altitude_out);/* Leer temperatura (C), presion (mmHg) y altitud (m) */
static void send_serial_data(float temperature, float pressure, float altitude);/* Enviar las mediciones por el puerto serie */
///****PROTOTIPOS OLED****/////////
static void oled_init(void);
static void oled_reset(void);
static void oled_command(uint8_t command);
static void oled_data(uint8_t data);
static void oled_set_position(uint8_t page, uint8_t column);
static void oled_clear(void);
static void oled_write_text(uint8_t page, uint8_t column, const char *text);
static void update_oled(float temperature, float pressure, float altitude);
static const uint8_t *get_glyph(char character);
/* Fuente 5x7: letras A-Z y numeros 0-9 */
static const uint8_t font[][5] = {
    /* A-Z */
    {0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x7F,0x49,0x49,0x49,0x36}, /* B */
    {0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D */
    {0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x7F,0x09,0x09,0x09,0x01}, /* F */
    {0x3E,0x41,0x49,0x49,0x7A}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H */
    {0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01}, /* J */
    {0x7F,0x08,0x14,0x22,0x41}, /* K */
    {0x7F,0x40,0x40,0x40,0x40}, /* L */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x7F,0x09,0x09,0x09,0x06}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* R */
    {0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x01,0x01,0x7F,0x01,0x01}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V */
    {0x3F,0x40,0x38,0x40,0x3F}, /* W */
    {0x63,0x14,0x08,0x14,0x63}, /* X */
    {0x07,0x08,0x70,0x08,0x07}, /* Y */
    {0x61,0x51,0x49,0x45,0x43}, /* Z */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E}  /* 9 */
};
void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando BMP280 con I2C nativo");
    ESP_ERROR_CHECK(i2c_init());
    ESP_ERROR_CHECK(bmp280_init_sensor());
    /* Inicializar la pantalla */
    oled_init();
    /* Borrar el contenido */
    oled_clear();
    /* Escribir mensajes en diferentes renglones */
    oled_write_text(0, 0, "ESP32 SPI");
    oled_write_text(2, 0, "HOLA SEBASTIAN");
    vTaskDelay(pdMS_TO_TICKS(3000));
    oled_write_text(4, 0, "BMP280 LISTO");
    vTaskDelay(pdMS_TO_TICKS(2000));
    /* Borrar los mensajes de bienvenida antes de mostrar las mediciones */
    oled_clear();
    while (1)
    {
        float temperature = 0.0f;  /* grados C */
        float pressure = 0.0f;     /* mmHg */
        float altitude = 0.0f;     /* metros */
        esp_err_t error = bmp280_read_sensor(&temperature, &pressure, &altitude);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "Error de lectura: %s",
                     esp_err_to_name(error));
        } else {
            update_oled(temperature, pressure, altitude);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
/* Leer uno o varios registros */
static esp_err_t read_registers(uint8_t reg,uint8_t *data,size_t length)
{
    return i2c_master_transmit_receive(bmp_handle, &reg, 1, data, length, 1000);
}
/* Escribir un registro */
static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(bmp_handle, data, sizeof(data), 1000);
}
/* Convertir dos bytes little-endian a entero sin signo */
static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0]|((uint16_t)data[1] << 8);
}
/* Inicializar el bus I2C y registrar el sensor */
static esp_err_t i2c_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle)
    );
    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP280_ADDRESS,
        .scl_speed_hz = I2C_FREQUENCY,
    };
    return i2c_master_bus_add_device(bus_handle, &device_config, &bmp_handle);
}
/* Leer los coeficientes almacenados en el sensor */
static esp_err_t read_calibration(void)
{
    uint8_t data[24];
    esp_err_t error = read_registers(BMP280_REG_CALIB, data, sizeof(data));
    if (error != ESP_OK) {
        return error;
    }
    calib.T1 = read_u16(&data[0]);
    calib.T2 = (int16_t)read_u16(&data[2]);
    calib.T3 = (int16_t)read_u16(&data[4]);
    calib.P1 = read_u16(&data[6]);
    calib.P2 = (int16_t)read_u16(&data[8]);
    calib.P3 = (int16_t)read_u16(&data[10]);
    calib.P4 = (int16_t)read_u16(&data[12]);
    calib.P5 = (int16_t)read_u16(&data[14]);
    calib.P6 = (int16_t)read_u16(&data[16]);
    calib.P7 = (int16_t)read_u16(&data[18]);
    calib.P8 = (int16_t)read_u16(&data[20]);
    calib.P9 = (int16_t)read_u16(&data[22]);
    if (calib.P1 == 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}
/* Inicializar y configurar el BMP280 */
static esp_err_t bmp280_init_sensor(void)
{
    uint8_t id;
    ESP_RETURN_ON_ERROR(
        read_registers(BMP280_REG_ID, &id, 1),
        TAG, "No se pudo leer el ID"
    );
    if (id != 0x58) {
        ESP_LOGE(TAG, "ID incorrecto: 0x%02X (esperado 0x58)", id);
        return ESP_ERR_INVALID_VERSION;
    }
    ESP_LOGI(TAG, "BMP280 detectado: ID=0x%02X", id);
    ESP_RETURN_ON_ERROR(
        write_register(BMP280_REG_RESET, 0xB6),
        TAG, "Error al reiniciar el sensor"
    );
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(
        read_calibration(),
        TAG, "Error al leer la calibracion"
    );
    /* Filtro desactivado y espera de 1 segundo */
    ESP_RETURN_ON_ERROR(
        write_register(BMP280_REG_CONFIG, 0xA0),
        TAG, "Error al configurar el sensor"
    );
    /* Temperatura x1, presion x1, modo normal */
    ESP_RETURN_ON_ERROR(
        write_register(BMP280_REG_CTRL, 0x27),
        TAG, "Error al iniciar las mediciones"
    );
    return ESP_OK;
}
static float calculate_temperature(int32_t adc_T)
{
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) -
              ((int32_t)calib.T1 << 1))) *
            (int32_t)calib.T2) >> 11;
    var2 = (((((adc_T >> 4) -
               (int32_t)calib.T1) *
              ((adc_T >> 4) -
               (int32_t)calib.T1)) >> 12) *
            (int32_t)calib.T3) >> 14;
    t_fine = var1 + var2;
    return (float)((t_fine * 5 + 128) >> 8) / 100.0f;
}
/* Calcular presion en Pa usando t_fine */
static float calculate_pressure(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = (int64_t)t_fine - 128000;
    var2 = var1 * var1 * calib.P6;
    var2 += (var1 * calib.P5) << 17;
    var2 += ((int64_t)calib.P4) << 35;
    var1 = ((var1 * var1 * calib.P3) >> 8) +
           ((var1 * calib.P2) << 12);
    var1 = ((((int64_t)1 << 47) + var1) *
            calib.P1) >> 33;
    if (var1 == 0) {
        return -1.0f;
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)calib.P9 * (p >> 13) *
            (p >> 13)) >> 25;
    var2 = ((int64_t)calib.P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) +
        ((int64_t)calib.P7 << 4);
    /* El resultado esta en Pa con 8 bits fraccionarios */
    return (float)p / 256.0f;
}
/* Calcular altitud en metros a partir de la presion en Pa */
static float calculate_altitude(float pressure_pa)
{
    return 44330.0f *
           (1.0f - powf(pressure_pa / SEA_LEVEL_PRESSURE_PA, 0.1903f));
}
/* Leer temperatura (C), presion (mmHg) y altitud (m) */
static esp_err_t bmp280_read_sensor(float *temperature_out,float *pressure_out,float *altitude_out)
{
    uint8_t data[6];
    ESP_RETURN_ON_ERROR(
        read_registers(BMP280_REG_PRESS, data, sizeof(data)),
        TAG, "Error al leer las mediciones");
    int32_t adc_P = ((int32_t)data[0] << 12) |
                    ((int32_t)data[1] << 4) |
                    (data[2] >> 4);
    int32_t adc_T = ((int32_t)data[3] << 12) |
                    ((int32_t)data[4] << 4) |
                    (data[5] >> 4);
    /* La temperatura se calcula primero porque la presion usa t_fine */
    float temperature = calculate_temperature(adc_T);
    float pressure_pa = calculate_pressure(adc_P);
    if (pressure_pa < 0.0f) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    float pressure_mmhg = pressure_pa * PA_TO_MMHG;
    float altitude = calculate_altitude(pressure_pa);
    /* Mostrar los datos mediante los logs de ESP-IDF */
    ESP_LOGI(TAG, "Temperatura: %.2f C", (double)temperature);
    ESP_LOGI(TAG, "Presion: %.2f mmHg", (double)pressure_mmhg);
    ESP_LOGI(TAG, "Altitud: %.2f m", (double)altitude);
    /* Enviar los datos por el puerto serie */
    send_serial_data(temperature, pressure_mmhg, altitude);
    /* Entregar las mediciones a la funcion que llama */
    *temperature_out = temperature;
    *pressure_out = pressure_mmhg;
    *altitude_out = altitude;
    return ESP_OK;
}
/* Enviar las mediciones por el puerto serie */
static void send_serial_data(float temperature, float pressure, float altitude)
{
    printf("DATA,%.2f,%.2f,%.2f\n",(double)temperature,(double)pressure,(double)altitude);
}
///////* Configurar el bus SPI y las señales de control *////
static void oled_init(void)
{
    gpio_config_t gpio_configuracion = {
        .pin_bit_mask =
            (1ULL << OLED_RES) |
            (1ULL << OLED_DC),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_configuracion));
    spi_bus_config_t bus_config = {
        .mosi_io_num = OLED_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = OLED_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = OLED_WIDTH
    };
    ESP_ERROR_CHECK(
        spi_bus_initialize(SPI2_HOST,&bus_config,SPI_DMA_DISABLED)
    );
    spi_device_interface_config_t dispositivo_config = {
        .clock_speed_hz = SPI_FREQUENCY,
        .mode = 0,
        .spics_io_num = OLED_CS,
        .queue_size = 1
    };
    ESP_ERROR_CHECK(
        spi_bus_add_device(SPI2_HOST,&dispositivo_config, &oled_spi)
    );
    oled_reset();
    /* Secuencia de inicializacion SSD1306 */
    oled_command(0xAE); /* Pantalla apagada */
    oled_command(0xD5);
    oled_command(0x80); /* Reloj */
    oled_command(0xA8);
    oled_command(0x3F); /* Multiplex 64 filas */
    oled_command(0xD3);
    oled_command(0x00); /* Desplazamiento */
    oled_command(0x40); /* Inicio de pantalla */
    oled_command(0x8D);
    oled_command(0x14); /* Bomba de carga */
    oled_command(0x20);
    oled_command(0x02); /* Direccionamiento por paginas */
    oled_command(0xA1); /* Orientacion horizontal */
    oled_command(0xC8); /* Orientacion vertical */
    oled_command(0xDA);
    oled_command(0x12); /* Configuracion de pines */
    oled_command(0x81);
    oled_command(0x7F); /* Contraste */
    oled_command(0xD9);
    oled_command(0xF1); /* Pre-carga */
    oled_command(0xDB);
    oled_command(0x40); /* Nivel VCOMH */
    oled_command(0xA4); /* Mostrar RAM */
    oled_command(0xA6); /* Imagen normal */
    oled_command(0xAF); /* Pantalla encendida */
}
/* Reiniciar fisicamente el controlador */
static void oled_reset(void)
{
    gpio_set_level(OLED_RES, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    gpio_set_level(OLED_RES, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}
/* Enviar un comando: DC = 0 */
static void oled_command(uint8_t command)
{
    spi_transaction_t transaccion = {
        .length = 8,
        .tx_buffer = &command
    };
    gpio_set_level(OLED_DC, 0);
    ESP_ERROR_CHECK(
        spi_device_transmit(oled_spi, &transaccion)
    );
}
/* Enviar un byte de imagen: DC = 1 */
static void oled_data(uint8_t data)
{
    spi_transaction_t transaccion = {
        .length = 8,
        .tx_buffer = &data
    };
    gpio_set_level(OLED_DC, 1);
    ESP_ERROR_CHECK(
        spi_device_transmit(oled_spi, &transaccion)
    );
}
/* Seleccionar pagina y columna inicial */
static void oled_set_position(uint8_t page, uint8_t column)
{
    oled_command(0xB0 | page);
    oled_command(0x00 | (column & 0x0F));
    oled_command(0x10 | (column >> 4));
}
/* Borrar todos los pixeles */
static void oled_clear(void)
{
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        oled_set_position(page, 0);
        for (uint8_t column = 0;
             column < OLED_WIDTH;
             column++) {
            oled_data(0x00);
        }
    }
}
/* Obtener el patron de una letra, numero o simbolo */
static const uint8_t *get_glyph(char character)
{
    static const uint8_t espacio[5] = {0, 0, 0, 0, 0};
    /* Patron del punto decimal */
    static const uint8_t punto[5] = {
        0x00, 0x60, 0x60, 0x00, 0x00
    };
    /* Conservar el punto antes de convertir a mayusculas */
    if (character == '.') {
        return punto;
    }
    character = (char)toupper((unsigned char)character);
    if (character >= 'A' && character <= 'Z') {
        return font[character - 'A'];
    }
    if (character >= '0' && character <= '9') {
        return font[26 + character - '0'];
    }
    return espacio;
}
static void oled_write_text(uint8_t page,uint8_t column,const char *text)
{
    if (page >= OLED_PAGES || column >= OLED_WIDTH) {
        return;
    }
    oled_set_position(page, column);
    while (*text != '\0' && column < OLED_WIDTH) {
        const uint8_t *glyph = get_glyph(*text);
        /* Cinco columnas forman cada caracter */
        for (int i = 0; i < 5 && column < OLED_WIDTH; i++) {
            oled_data(glyph[i]);
            column++;
        }
        /* Separacion entre caracteres */
        if (column < OLED_WIDTH) {
            oled_data(0x00);
            column++;
        }
        text++;
    }
     /* cuando el texto nuevo es mas corto que el anterior */
    while (column < OLED_WIDTH) {
        oled_data(0x00);
        column++;
    }
}
static void update_oled(float temperature, float pressure, float altitude)
{
    char line[24];
    // Pagina 0: titulo
    oled_write_text(0, 0, "ESP32 SPI");
    // Pagina 1: DSE
    oled_write_text(1, 0, "DSE");
    // Pagina 3: temperatura en grados C
    snprintf(line, sizeof(line), "TEMPERATURA=%.1fC",
             (double)temperature);
    oled_write_text(3, 0, line);
    // Pagina 5: presion en mmHg
    snprintf(line, sizeof(line), "PRESION=%.1f MMHG",
             (double)pressure);
    oled_write_text(5, 0, line);
    // Pagina 7: altitud en metros
    snprintf(line, sizeof(line), "ALTITUD=%.1f M",
             (double)altitude);
    oled_write_text(7, 0, line);
}