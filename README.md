Sebastián Rodríguez De Alba | DISEÑO DE SISTEMAS EMBEBIDOS | GPO: 001 | UANL-FIME | Realizado con apoyo del Doc.HECTOR GILBERTO BARRON GONZALEZ
# PantallaOLEDSPI_SensorBMP280
Firmware en ESP-IDF para ESP32 que lee temperatura, presión y altitud de un sensor BMP280 por I²C y las muestra en una pantalla OLED SSD1306 de 128x64 por SPI, además de enviarlas por el puerto serie.
Proyecto en **ESP-IDF** para el **ESP32** que mide temperatura, presión atmosférica y altitud con un sensor **BMP280** (I²C) y muestra los datos en una pantalla **OLED SSD1306 de 128x64** (SPI). Los valores también se envían por el puerto serie.

## Características
- Lectura del BMP280 por I²C con el driver nativo `i2c_master` de ESP-IDF.
- Verificación del ID del sensor (0x58) y lectura de los coeficientes de calibración.
- Compensación de temperatura y presión con las fórmulas de la hoja de datos de Bosch.
- Conversión de presión a mmHg y cálculo de altitud respecto a la presión a nivel del mar (ajustable).
- Control de la OLED por SPI sin librerías externas: inicialización del SSD1306, fuente 5x7 propia y escritura por páginas.
- Salida serie en formato `DATA,temperatura,presion,altitud` para graficar o registrar los datos.

## Hardware
- ESP32
- Sensor BMP280 (dirección I²C `0x76`)
- Pantalla OLED 128x64 SPI (controlador SSD1306)

## Conexiones
| Dispositivo | Señal | GPIO ESP32 |
|-------------|-------|-----------:|
| BMP280 | SDA | 21 |
| BMP280 | SCL | 22 |
| OLED | SCLK (D0) | 18 |
| OLED | MOSI (D1) | 23 |
| OLED | RES | 17 |
| OLED | DC | 16 |
| OLED | CS | 19 |
Alimenta el BMP280 y la OLED a 3.3 V y conecta GND en común.

## Requisitos
- ESP-IDF v5.2 o superior (usa `driver/i2c_master.h`).

## Compilar y cargar
```bash
idf.py set-target esp32
idf.py build
idf.py -p <PUERTO> flash monitor
```

## Funcionamiento
1. Se inicializan el bus I²C y el BMP280 (modo normal, oversampling x1, filtro desactivado).
2. Se inicializa la OLED por SPI y se muestra un mensaje de bienvenida.
3. Cada 2 segundos se leen los datos, se actualizan los tres renglones de la pantalla (temperatura, presión y altitud) y se imprimen por el puerto serie.

Ejemplo de salida serie:
```
DATA,24.35,707.82,1.2
```

## Notas
- La fuente incluida solo contiene letras A-Z, números 0-9 y el punto, por lo que el texto se muestra en mayúsculas.
- Para una altitud más precisa, ajusta `SEA_LEVEL_PRESSURE_PA` según la presión local a nivel del mar.

## Referencias
- Bosch Sensortec, *BMP280 Digital pressure sensor, Data sheet*.
- Espressif Systems, *ESP-IDF Programming Guide* (SPI Master Driver, I2C).
