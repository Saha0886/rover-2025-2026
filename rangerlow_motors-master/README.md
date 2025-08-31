# RangerLow Motors

Пакет для управления моторами и сервоприводами робота Ranger с использованием ROS 2.

## Описание

Этот пакет предоставляет драйверы для управления периферийными устройствами робота Ranger:
- **Моторы**: управление через UART-соединения с тремя STM-контроллерами (передний, средний, задний)
- **Сервоприводы**: управление четырьмя сервоприводами через I2C с помощью PCA9685

## Структура проекта

```
rangerlow_motors/
├── LICENSE                     
├── README.md                   
├── package.xml                 
├── setup.py                    
├── setup.cfg                   
├── requirements.txt            
├── resource/
│   └── rangerlow_motors        
└── rangerlow_motors/           
    ├── __init__.py            # Инициализация модуля
    ├── motors_node.py         # Нода управления моторами
    ├── servo_node.py          # Нода управления сервоприводами
    ├── uart.py                # UART-драйвер для связи с STM
    └── servo.py               # Обёртка для сервоприводов
```

## Системные требования

- ROS 2 Humble
- Python 3.8+
- Аппаратные компоненты:
  - 3x STM-контроллера на UART портах
  - PCA9685 PWM-драйвер на I2C
  - 4x сервопривода

## Установка


### 1. Установка в рабочие пространство

```bash
cd ros2_ws/src
git clone https://github.com/sipmine/rangerlow_motors
git clone https://github.com/sipmine/rangerros_interfaces.git
```




### 2. Установка Python зависимостей

```bash
pip install -r requirements.txt
```

Основные зависимости:
- `pyserial` - для UART-коммуникации
- `adafruit-circuitpython-pca9685` - для I2C PWM-драйвера
- `adafruit-circuitpython-motor` - для управления сервоприводами

### 3. Сборка пакета

```bash
cd ~/ros_ws/
colcon build --packages-select rangerlow_motors
source install/setup.bash
```

## Конфигурация

### Параметры моторов

По умолчанию используются следующие UART-порты:
- Передний контроллер: `/dev/ttyAMA0`
- Средний контроллер: `/dev/ttyAMA2`
- Задний контроллер: `/dev/ttyAMA4`

Параметры можно изменить при запуске:

```bash
ros2 run rangerlow_motors motors_node --ros-args \
  -p stm_front_port:=/dev/ttyUSB0 \
  -p stm_mid_port:=/dev/ttyUSB1 \
  -p stm_rear_port:=/dev/ttyUSB2
```

### Сервоприводы

Используется I2C-интерфейс с частотой 50 Hz для PCA9685:
- Канал 0: `front_right_servo`
- Канал 1: `front_left_servo`
- Канал 2: `rear_left_servo`
- Канал 3: `rear_right_servo`

## Запуск

### Запуск ноды моторов

```bash
ros2 run rangerlow_motors motors_node
```

### Запуск ноды сервоприводов

```bash
ros2 run rangerlow_motors servo_node
```

### Запуск обеих нод одновременно

```bash
# В отдельных терминалах:
ros2 run rangerlow_motors motors_node &
ros2 run rangerlow_motors servo_node &
```

## ROS 2 интерфейсы

### Топики моторов

**Подписка (команды установки шим сигнала):**
- `/rangerlow_motors/front_driver` - команды переднему контроллеру
- `/rangerlow_motors/mid_driver` - команды среднему контроллеру
- `/rangerlow_motors/rear_driver` - команды заднему контроллеру

**Публикация (состояния энкодеров):**
- `/rangerlow_motors/front_driver_info` - состояние переднего контроллера 
- `/rangerlow_motors/mid_driver_info` - состояние среднего контроллера
- `/rangerlow_motors/rear_driver_info` - состояние заднего контроллера

**Формат сообщения MotorDriver:**
```
uint8 core_id       # ID контроллера
uint8 m1_cmd        # Команда для мотора 1
uint8 m2_cmd        # Команда для мотора 2
float32 m1_data     # Данные для мотора 1
float32 m2_data     # Данные для мотора 2
```

### Топики сервоприводов

**Подписка (команды):**
- `/rangerlow_servo/front_left_servo`
- `/rangerlow_servo/rear_left_servo`
- `/rangerlow_servo/front_right_servo`
- `/rangerlow_servo/rear_right_servo`

**Публикация (состояние):**
- `/rangerlow_servo/front_left_servo_info`
- `/rangerlow_servo/rear_left_servo_info`
- `/rangerlow_servo/front_right_servo_info`
- `/rangerlow_servo/rear_right_servo_info`

**Формат сообщения Servo:**
```
string servo_name   # Имя сервопривода
float32 angle       # Угол в градусах (0-180)
```

## Примеры использования

### Управление мотором

```bash
# Отправка команды переднему мотору
ros2 topic pub /rangerlow_motors/front_driver rangerros_interfaces/msg/MotorDriver \
  '{core_id: 1, m1_cmd: 26, m2_cmd: 27, m1_data: 100.0, m2_data: -50.0}'
```

### Управление сервоприводом

```bash
# Установить угол 90 градусов для переднего левого серво
ros2 topic pub /rangerlow_servo/front_left_servo rangerros_interfaces/msg/Servo \
  '{servo_name: "front_left_servo", angle: 90.0}'
```

### Мониторинг состояния

```bash
# Просмотр состояния моторов
ros2 topic echo /rangerlow_motors/front_driver_info

# Просмотр состояния сервоприводов
ros2 topic echo /rangerlow_servo/front_left_servo_info
```

## Протокол UART

### Формат пакета (8 байт)

| Байт | Назначение | Описание |
|------|------------|----------|
| 0    | START_BYTE | 0xA5 (маркер начала) |
| 1    | CORE_ID    | ID контроллера |
| 2    | CMD        | Код команды |
| 3-6  | DATA       | Данные (float, little-endian) |
| 7    | CHECKSUM   | XOR всех предыдущих байт |

### Команды моторов

- `26` - команда для получение значение энкодера мотора 1
- `27` - команда для получение значение энкодера мотора 2
- `10` - команда управления мотором 1
- `11` - команда управления мотором 2
## Диагностика

### Проверка подключений

```bash
# Проверка UART-портов
ls -la /dev/ttyAMA*

# Проверка I2C
i2cdetect -y 1
```

### Логи

```bash
# Просмотр логов нод
ros2 run rqt_console rqt_console

# Или через командную строку
ros2 topic echo /rosout
```

### Типичные ошибки

1. **"NOT CONN FRONT/MID/REAR"** - не удается подключиться к UART-порту
   - Проверьте существование порта
   - Убедитесь в правах доступа: `sudo chmod 666 /dev/ttyAMA*`

2. **"UNPACK FAILED"** - ошибка разбора пакета
   - Проверьте качество UART-соединения
   - Убедитесь в правильности протокола на STM-стороне

3. **"Invalid checksum"** - нарушена целостность данных
   - Проверьте электрические соединения
   - Рассмотрите добавление помехозащиты

### Лицензия
Apache-2.0

### Частота обновления
- Моторы: 20 Hz (каждые 0.05 сек)
- Сервоприводы: 20 Hz (каждые 0.05 сек)

Эта частота обеспечивает плавное управление при минимальной нагрузке на систему.

