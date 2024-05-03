| Placas suportadas | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- |
# Descrição do projeto
Neste repositório foram desenvolvidas duas bibliotecas para utilizar do módulo MPU6050 utilizando das medições do giroscópio e do acelerômetro.

Na biblioteca **sensor_imu** temos a definição dos structs que capturam os dados do giroscópio e do acelerômetro além das seguintes funções:

```
imu_init(): Configura a comunicação I2C e o MPU6050 deixando ele pronto para captura dos dados.

get_acceleration_data(AccelerationData *data): Retorna os dados do acelerômetro nos eixos x, y e z.

get_gyroscope_data(GyroscopeData *data): Retorna os dados do giroscópio nos eixos x, y e z.
```

//TODO biblioteca imu_tools

# Diagrama de blocos
![diagrama de blocos](images/blocoMPU6050.drawio.png)  

# Esquemático
![esquemático](images/esquematicoMPU6050.png)

**OBS: Caso sua placa ESP32 utilize outros pinos diferentes do 21 e 22 para o SDA e SCL é importante modificar na função i2c_bus_init dentro da biblioteca sensor_imu.c**

# Máquina de estados


# Como rodar o projeto
Faça o download da extensão abaixo no VSCode
![ESP-IDF Extension](images/extension.png)  

Após baixar com sucesso deverá aparecer o menu abaixo daí é só clicar no ícone "ESP-IDF: Build, Flash and Monitor"  

![icone](images/icone.png)







