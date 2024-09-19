This project is test as example for coding skills. Task - we have MCU with ADC and UART onboard. We need measure analog signal level with fastest sample rate. If our signal threshold 70% we need a switch LED. UART parameters is 115200 8N1. If UART receiving "e" or "E" symbol we need a turn-on streaming 1 byte ADC samples. If UART receiving "d" or "D" symbol we need a turn-off ADC sample streaming. If UART receiving "r" or "R" symbol we need a, reset threshold LED.

For resolve this test I use STM32F103C8T6 MCU on Bluepill board

PA1 - Analog input

PA9 - UART Tx

PA10 - UART Rx

Небольшое тестовое задание для примера написания кода. Задача - имеется микроконтроллер с АЦП и UART на борту, необходимо проверять аналоговый сигнал максимально возможное количество раз. В случае превышения уровня сигнала свыше 70% от максимального, включать светодиод. Параметры UART 115200 8N1. В случае приёма по UART команды с символом "e" или "E", необходимо включать потоковую трансляцию 8 бит данных с АЦП. По приёму команды с символом "d" или "D" - выключать трансляцию. По приёму команды с символом "r" или "R", необходимо сбрасывать светодиод.

Для решения этой задачи я использую STM32F103C8T6 на плате Bluepill

PA1 - Аналоговый вход

PA9 - Выход UART

PA10 - Вход UART
