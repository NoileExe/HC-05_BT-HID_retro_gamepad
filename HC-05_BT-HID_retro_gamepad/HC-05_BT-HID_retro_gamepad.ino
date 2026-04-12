
/*
 * HC-05_BT-HID_retro_gamepad
 * Firmware for a retro gamepad (SEGA Genesis/MD2/Saturn or SNES) based on Arduino Pro Mini
 * or LGT8F328P, emulating a Bluetooth HID device via HC-05 module flashed with RN-42 firmware
 * (or original RN-42 module).
 * Copyright (C) 2026 Belobragin Anton Igorevich
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
    Все нажатия кнопок на цифровых пинах срабатывают на логический нуль (LOW)
    Аналоговые пины работают в режиме цифровых
    В режиме сна на ATMEG'е с 8МГц менее 10 мкА / В режиме поиска соединения ~18-40 мА / В рабочем режиме 25-35 мА
    В режиме сна на LGT8F328P ~100 мкА / В режиме поиска соединения ~18-40 мА / В рабочем режиме 25-35 мА
    
    Пины D6 и D7 (кнопки вверх и вправо) на ATMEGA328P/ATMEGA328PA почему-то частенько отваливаются, начинаются ложные срабатывания и залипания
    По этой причине эти пины лучше сразу паять на другие выходы, либо попытать удачу с предложенным кодом
    

    Возможности:
      Контроль низкого напряжения каждые 10 секунд
      Контроль бездействия более 5 минут (есть вариант с проверкой соединения. В таком случае если соединение отсутствует - сон через 2 минуты)
      Контроль наличия сопряжения с BT-модулем
      Глубокий сон BT-модуля и ардуинки по достижению таймера бездействия или по нажатию соотв. комбинации
        (SEGA - "ВНИЗ + X + Z + B"; SNES - "ВНИЗ + Y + B + A") в течение 3 сек.
      Защита от зависаний Ардуино
      Защита от зависаний BT-модуля путём перевода джойстика в режим сна и пробуждения

      Бит   Скан-код    Кнопка          Код
     ---------------------------------------
      0     key 304     BUTTON_A        96
      1     key 305     BUTTON_B        97
      2     key 306     BUTTON_C        98
      3     key 307     BUTTON_X        99
      4     key 308     BUTTON_Y        100
      5     key 309     BUTTON_Z        101
      6     key 310     BUTTON_L1       102
      7     key 311     BUTTON_R1       103
      8     key 312     BUTTON_L2       104
      9     key 313     BUTTON_R2       105
      10    key 314     BUTTON_SELECT   109
      11    key 315     BUTTON_START    108
      12    key 316     BUTTON_MODE     110
      13    key 317     BUTTON_THUMBL   106
      14    key 318     BUTTON_THUMBR   107
     ---------------------------------------
*/

#ifdef __LGT8FX8P__
  #include <PMU.h>
  #include <WDT.h>
#else
  #include <avr/sleep.h>
  #include <avr/power.h>
  #include <avr/wdt.h>
  #include <iarduino_VCC.h>
#endif


//GyverLibs
#include <TimerMs.h>
#include <Blinker.h>

//------------------------------------------------------------------

//Выбор режима геймпада - SEGA либо SNES
//#define SEGA_JOY
//#define SNES_JOY


//Используется ли считывание ардуинкой статуса соединения с GPIO2 (25 pin) BT-модуля HC-05
//#define BT_STATUS_ENABLED

//------------------------------------------------------------------

// Выбор регистров и битов ATMEGA для управления питанием
#ifndef __LGT8FX8P__
  #if defined(PRR0)
    #define PRR_NAME PRR0
  #elif defined(PRR)
    #define PRR_NAME PRR
  #endif

  #if defined(PRUSART0)
    #define PRUSART_BIT_NAME PRUSART0
  #elif defined(PRUSART)
    #define PRUSART_BIT_NAME PRUSART
  #endif

  #if defined(PRSPI0)
    #define PRSPI_BIT_NAME PRSPI0
  #elif defined(PRSPI)
    #define PRSPI_BIT_NAME PRSPI
  #endif

  #if defined(PRTWI0)
    #define PRTWI_BIT_NAME PRTWI0
  #elif defined(PRTWI)
    #define PRTWI_BIT_NAME PRTWI
  #endif
#endif


// Проверка, что определён хотя бы один из вариантов геймпада
#if !defined(SEGA_JOY) && !defined(SNES_JOY)
  #error "Define SEGA_JOY or SNES_JOY!"
#endif

#if defined(SEGA_JOY)
  //ПИНЫ SEGA
  #define TX_PIN 0     //присоединить к RXD BT-модуля (логический уровень 3.3 Вольта!)
  #define RX_PIN 1     //присоединить к TXD BT-модуля
  #define START_PIN 2  //Т.к. на данном пине можно произвести выход из сна
  #define A_PIN 5
  #define B_PIN 4
  #define C_PIN 3
  #define UP_PIN 6     //Возможно придётся поменять (напр. A4, если 6 глючит)
  #define RIGHT_PIN 7  //Возможно придётся поменять (напр. A5, если 7 глючит)
  #define DOWN_PIN 8
  #define LEFT_PIN 9
  #define X_PIN A1
  #define Y_PIN A2
  #define Z_PIN A3
  #define L1_PIN 11
  #define R1_PIN 12
  #define BTONOFF_PIN 10  //Включение-выключение модуля BT: HIGH - включено, LOW - выключено - иначе в режиме сна потребляет 5мА
  #define PWRLED_PIN 13   //Сигнал о низком заряде АКБ
#elif defined(SNES_JOY)
  //ПИНЫ SNES
  #define TX_PIN 0     //присоединить к RXD BT-модуля (логический уровень 3.3 Вольта!)
  #define RX_PIN 1     //присоединить к TXD BT-модуля
  #define START_PIN 2  //Т.к. на данном пине можно произвести выход из сна
  #define SELECT_PIN 3
  #define A_PIN 4
  #define B_PIN 5
  #define UP_PIN 6     //Возможно придётся поменять (напр. A4, если 6 глючит)
  #define RIGHT_PIN 7  //Возможно придётся поменять (напр. A5, если 7 глючит)
  #define DOWN_PIN 8
  #define LEFT_PIN 9
  #define X_PIN A3
  #define Y_PIN A2
  #define L1_PIN 11
  #define R1_PIN 12
  #define BTONOFF_PIN 10  //Включение-выключение модуля BT: HIGH - включено, LOW - выключено - потребление всего джойстика <20мкА (иначе в режиме сна потребляет 5мА)
  #define PWRLED_PIN 13   //Сигнал о низком заряде АКБ
#endif

#ifdef BT_STATUS_ENABLED
  //Сигнал о состоянии сопряжения: HIGH - подключено, LOW - нет сопряжения/состояние поиска
  //Приходит непосредственно с выхода светодиода HC-05 о статусе существующего подключения (PIO2 - 25 pin)
  //
  //Пины A6 и A7 на ATMEGA328(P/PA) используются только как аналоговые входы
  // - поэтому универсальное решение через analogRead
  #define BTSTAT_PIN A6
#endif

//------------------------------------------------------------------

//GAMEPAD/JOYSTICK КОДЫ КНОПОК
// 1ST == первый набор кнопок (BTN0 - BTN7)
#define JOY_1ST_A_BTN (1 << 0)
#define JOY_1ST_B_BTN (1 << 1)
#define JOY_1ST_C_BTN (1 << 2)
#define JOY_1ST_X_BTN (1 << 3)
#define JOY_1ST_Y_BTN (1 << 4)
#define JOY_1ST_Z_BTN (1 << 5)
#define JOY_1ST_L1_BTN (1 << 6)
#define JOY_1ST_R1_BTN (1 << 7)
#define JOY_1ST_NOBTN 0x00

// 2ND = второй набор кнопок (BTN0 - BTN7)
#define JOY_2ND_L2_BTN (1 << 0)       // L2/R2 НЕ аналоговые, а цифровые в RN42
#define JOY_2ND_R2_BTN (1 << 1)
#define JOY_2ND_SELECT_BTN (1 << 2)
#define JOY_2ND_START_BTN (1 << 3)
#define JOY_2ND_MODE_BTN (1 << 4)
#define JOY_2ND_THUMBL_BTN (1 << 5)   // L3
#define JOY_2ND_THUMBR_BTN (1 << 6)   // R3
#define JOY_2ND_BTN7 (1 << 7)         // HOME?
#define JOY_2ND_NOBTN 0x00

//Кнопки направлений (D-Pad)
#define JOY_X1_UP (1 << 0)
#define JOY_X1_DOWN (1 << 1)
#define JOY_Y1_LEFT (1 << 2)
#define JOY_Y1_RIGHT (1 << 3)

//------------------------------------------------------------------

//Задержка в мс (16.7 ~60 Гц)
const float delay_time_ms = 17;

//Таймер удержания комбинации кнопок для выключения. 3 СЕКУНДЫ
TimerMs timer_poweroff_combination(3 * 1000ul, 0, 1);

//Таймер проверки текущего напряжения. Каждые 10 СЕКУНД
TimerMs timer_voltage_check(10 * 1000ul, 0, 1);

//Таймер бездействия при наличии соединения для выключения при простое более 5 МИНУТ
TimerMs timer_connected_inactivity(5 * 1000ul * 60ul, 0, 1);

#ifdef BT_STATUS_ENABLED
  //Таймер бездействия при отсутствии соединения для выключения при простое более 2 МИНУТ
  TimerMs timer_disconnected_inactivity(2 * 1000ul * 60ul, 0, 1);
#endif

//Перевод цифрового пина в номер пина с прерыванием (для Arduino Pro Mini D2 == 0, D3 == 1)
const uint8_t interrupt_pin = digitalPinToInterrupt(START_PIN);

//------------------------------------------------------------------

//Напряжение, которое МК считает низким
const float avr_yellow_voltage = 3.3;
const float avr_red_voltage = 3.1;

//Текущее напряжение
float gamepad_voltage = 5.0;

//Сопряжён ли геймпад
#ifdef BT_STATUS_ENABLED
  bool gamepad_connected = false;
#endif

//Предыдущие состояния кнопок
byte old_btnsByte1 = 0;
byte old_btnsByte2 = 0;
byte old_arrowsByte = 0;  //D-Pad

//Текущие состояния кнопок
byte btnsByte1 = 0;
byte btnsByte2 = 0;
byte arrowsByte = 0;  //D-Pad

//Данные крестовины, как стика PlayStation, по всем направлениям для отправки
int8_t x1 = 0;
int8_t y1 = 0;
int8_t x2 = 0;
int8_t y2 = 0;

//Светодиод, мигающий при разрядке
Blinker power_led(PWRLED_PIN);

//------------------------------------------------------------------

//Считывание нажатых в данный момент кнопок
//Обнуляет и считывает все данные используя и меняя глобальные переменные
//Результат будет в btnsByte1, btnsByte2, x1, y1, x2, y2
void readAllButtons();

//Отправка комбинации кнопок. По умолчанию все кнопки отпущены
void sendButtons(byte first_btns = 0, byte second_btns = 0, int8_t first_x = 0, int8_t first_y = 0, int8_t second_x = 0, int8_t second_y = 0);

//Проверка нажатия комбинации
// SEGA - "ВНИЗ + X + Z + B"
// SNES - "ВНИЗ + Y + B + A"
bool isPoweroffCombination(byte first_btns, byte second_btns, byte arrows_btns);

//Обработчик аппаратного прерывания для выхода из режима сна
void isrWakeUp();

//Сон и пробуждение джойстика и BT-модуля
void gamepadSleep();

//Проверка напряжения питания:
//При напряжениях в диапазоне (avr_red_voltage; avr_yellow_voltage] Вольт - медленно мигаем и возвращаемся
//При напряжении <= avr_red_voltage Вольт - не выходим из функции сообщая быстрым морганием и переходим в глубокий сон пока не будет заряжена АКБ
void low_power_check();

//------------------------------------------------------------------

void setup() {
#ifdef __LGT8FX8P__
  //analogReference(INTERNAL4V096);
  analogReference(INTERNAL1V024);
#else
  analogReference(DEFAULT);
#endif

//Отключаем компаратор в ПОПЫТКЕ ПОЧИНИТЬ проблему D7
#ifdef __LGT8FX8P__
  ACSR = _BV(C0D);
#else
  ACSR = _BV(ACD);
#endif

//Отключаем OC0A после инициализации в ПОПЫТКЕ ПОЧИНИТЬ проблему D6 - к сожалению плат на ATMEGA328P/PA у меня для проверки не осталось
TCCR0A &= ~(_BV(COM0A0) | _BV(COM0A1));


#ifdef SEGA_JOY
  //SEGA
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(RIGHT_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);
  pinMode(LEFT_PIN, INPUT_PULLUP);
  pinMode(A_PIN, INPUT_PULLUP);
  pinMode(B_PIN, INPUT_PULLUP);
  pinMode(C_PIN, INPUT_PULLUP);
  pinMode(X_PIN, INPUT_PULLUP);  //Аналоговый пин как цифровой
  pinMode(Y_PIN, INPUT_PULLUP);  //Аналоговый пин как цифровой
  pinMode(Z_PIN, INPUT_PULLUP);  //Аналоговый пин как цифровой
  pinMode(L1_PIN, INPUT_PULLUP);
  pinMode(R1_PIN, INPUT_PULLUP);
#elif defined(SNES_JOY)
  //SNES
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(SELECT_PIN, INPUT_PULLUP);
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(RIGHT_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);
  pinMode(LEFT_PIN, INPUT_PULLUP);
  pinMode(A_PIN, INPUT_PULLUP);
  pinMode(B_PIN, INPUT_PULLUP);
  pinMode(X_PIN, INPUT_PULLUP);  //Аналоговый пин как цифровой
  pinMode(Y_PIN, INPUT_PULLUP);  //Аналоговый пин как цифровой
  pinMode(L1_PIN, INPUT_PULLUP);
  pinMode(R1_PIN, INPUT_PULLUP);
#endif

#if defined BT_STATUS_ENABLED && defined(ADC6D) && defined(ADC7D)
  pinMode(BTSTAT_PIN, INPUT);
  DIDR0 |= _BV(ADC6D);    // Отключить цифровой буфер (нет утечки)
  DIDR0 |= _BV(ADC7D);
#endif

  pinMode(BTONOFF_PIN, OUTPUT);
  digitalWrite(BTONOFF_PIN, HIGH);  //Включаем преобразователь напряжения на модуле BT

//Слежение за зависанием самой ардуинки. Таймаут установлен максимальный ~8с
#ifdef __LGT8FX8P__
  wdt_enable(WTO_8S);
#else
  wdt_enable(WDTO_8S);
#endif

  Serial.begin(9600);
  delay(500);  //Обязательная минимальная задержка после включения перед отправкой команд


#ifdef BT_STATUS_ENABLED
  timer_disconnected_inactivity.start();
#else
  timer_connected_inactivity.start();
#endif
}

//------------------------------------------------------------------

void loop() {
  if (power_led.running())
    power_led.tick();

  delay(delay_time_ms);

  //=======================Предусловия=======================
  //Каждые 10 секунд проверяем напряжение
  if (!timer_voltage_check.active() || timer_voltage_check.tick()) {
    low_power_check();            //Проверка напряжения
    timer_voltage_check.start();  //Сброс таймера
  }
  //=======================Предусловия=======================

  //========================Комбинации=======================
  readAllButtons();  //Чтение нажатых кнопок

#ifdef BT_STATUS_ENABLED
  gamepad_connected = analogRead(BTSTAT_PIN) > 500; //Примерно половина от максимального значения 1023
  bool is_new_combination = gamepad_connected && (btnsByte1 != old_btnsByte1 || btnsByte2 != old_btnsByte2 || arrowsByte != old_arrowsByte);
#else
  bool is_new_combination = btnsByte1 != old_btnsByte1 || btnsByte2 != old_btnsByte2 || arrowsByte != old_arrowsByte;
#endif

//Если таймер простоя достиг своего требуемого значения - переход в спящий режим
#ifdef BT_STATUS_ENABLED
  //Меняем активность таймеров в зависимости от смены статуса соединения
  if (gamepad_connected && timer_disconnected_inactivity.active()) {
    timer_disconnected_inactivity.stop();
    timer_connected_inactivity.start();
  } else if (!gamepad_connected && timer_connected_inactivity.active()) {
    timer_connected_inactivity.stop();
    timer_disconnected_inactivity.start();
  }

  if ((!gamepad_connected && timer_disconnected_inactivity.tick()) || (gamepad_connected && timer_connected_inactivity.tick()))
    gamepadSleep();
  else if (is_new_combination)
    timer_connected_inactivity.start();  //Сброс таймера
#else
  if (timer_connected_inactivity.tick())
    gamepadSleep();
  else if (btnsByte1 != 0 || btnsByte2 != 0 || arrowsByte != 0)
    timer_connected_inactivity.start();  //Сброс таймера
#endif

  //Если удерживается комбинация кнопок для выключения
  // или таймер простоя достиг требуемого значения - прибавляем таймер
  if (isPoweroffCombination(btnsByte1, btnsByte2, arrowsByte)) {
    if (!timer_poweroff_combination.active()) timer_poweroff_combination.start();

    //Если таймер удержания кнопок достиг требуемого значения - переход в спящий режим
    if (timer_poweroff_combination.tick())
      gamepadSleep();

    //Если геймпад сопряжён и предыдущая комбинация нажатых (или не нажатых) кнопок отличается от текущей - отправляем пустую комбинацию кнопок,
    // чтобы устройство, к которому подключен геймпад не получало лишней информации
    if (is_new_combination) {
      sendButtons();

      //Запоминаем значения
      old_btnsByte1 = btnsByte1;
      old_btnsByte2 = btnsByte2;
      old_arrowsByte = arrowsByte;
    }

    wdt_reset();
    return;
  }
  //========================Комбинации=======================

  //===================Работа с таймерами====================
  if (timer_poweroff_combination.active()) timer_poweroff_combination.stop();
  //===================Работа с таймерами====================

  //=====================Отправка данных=====================
  //Если геймпад сопряжён и предыдущая комбинация нажатых (или не нажатых) кнопок отличается от текущей - отправляем новую комбинацию кнопок
  if (is_new_combination) {
    sendButtons(btnsByte1, btnsByte2, x1, y1, x2, y2);

    //Запоминаем значения
    old_btnsByte1 = btnsByte1;
    old_btnsByte2 = btnsByte2;
    old_arrowsByte = arrowsByte;
  }
  //=====================Отправка данных=====================

  wdt_reset();
}

//------------------------------------------------------------------

void isrWakeUp() {
}


void gamepadSleep() {
#ifdef BT_STATUS_ENABLED
  bool send_empty_btns = gamepad_connected;
#else
  bool send_empty_btns = true;
#endif

  if (send_empty_btns) {
    sendButtons();

    //Обнуляем все значения
    btnsByte1 = 0;
    btnsByte2 = 0;
    arrowsByte = 0;
    old_btnsByte1 = 0;
    old_btnsByte2 = 0;
    old_arrowsByte = 0;

    x1 = 0;
    y1 = 0;
    x2 = 0;
    y2 = 0;

    delay(500);
  }

  digitalWrite(BTONOFF_PIN, LOW); //Выключаем преобразователь напряжения на модуле BT
  Serial.end();
  pinMode(RX_PIN, INPUT);         // RX в High-Z (высокий импеданс)
  digitalWrite(TX_PIN, LOW);      // Принудительно 0 на TX
  pinMode(TX_PIN, OUTPUT);        // TX как выход 0Вольт

  if (timer_poweroff_combination.active()) timer_poweroff_combination.stop();
  if (timer_voltage_check.active()) timer_voltage_check.stop();
  if (timer_connected_inactivity.active()) timer_connected_inactivity.stop();
#ifdef BT_STATUS_ENABLED
  if (timer_disconnected_inactivity.active()) timer_disconnected_inactivity.stop();
#endif

  wdt_disable();

  digitalWrite(PWRLED_PIN, LOW);

#ifdef __LGT8FX8P__
  // АЦП + сброс флага
  ADCSRA &= ~_BV(ADEN);
  ADCSRA |= _BV(ADIF);

  ADCSRD = 0x00;  // Полное отключение аналога

  // LVD (Low Voltage Detector)
  VDTCR |= _BV(WCE);
  VDTCR &= ~_BV(VDTEN);

  PRR = _BV(PRADC) | _BV(PRTIM1) | _BV(PRTIM2) | _BV(PRUSART0) | _BV(PRSPI) | _BV(PRTWI);
  // Бит 1 (PRPCI) НЕ трогаем — нужно для пробуждения по кнопке!
  //     PRTIM3    PRWDT    PREFL
  PRR1 = _BV(3) | _BV(5) | _BV(2);

  DIDR0 = 0xFF;  // Цифровые входы на аналоговых пинах на время сна отключены
  DIDR1 = 0xFF;

  EIFR = 0xFF;
  attachInterrupt(interrupt_pin, isrWakeUp, LOW);
  PMU.sleep(PM_POFFS1, SLEEP_FOREVER);

  PRR = 0;
  PRR1 = 0;
  DIDR0 = 0;
  DIDR1 = 0;
  ADCSRA |= _BV(ADEN);  // Если нужен АЦП
  ADCSRD |= _BV(ADEN);  // Разрешаем работу АЦП (бит 0)

  VDTCR |= _BV(WCE);
  VDTCR |= _BV(VDTEN);  // Включаем детектор
#else
  ADMUX = 0;
  ADCSRA &= ~_BV(ADEN);  // Отключаем АЦП

  PRR_NAME = _BV(PRADC) | _BV(PRTIM1) | _BV(PRTIM2) | _BV(PRUSART_BIT_NAME) | _BV(PRSPI_BIT_NAME) | _BV(PRTWI_BIT_NAME);
  #ifdef PRR1
    //     PRTIM3    PRDAC   PRSPI1   PRTWI1
    PRR1 = _BV(0) | _BV(3) | _BV(1) | _BV(2);
  #endif

  DIDR0 = 0xFF;  // Цифровые входы на аналоговых пинах на время сна отключены
  DIDR1 = 0xFF;

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  //Устанавливаем интересующий нас режим

  cli();  // Временно запрещаем обработку прерываний
  sleep_enable();
  
  //Очистка флагов прерываний
  if (interrupt_pin == 2)         EIFR = _BV(INTF0);
  else if (interrupt_pin == 3)    EIFR = _BV(INTF1);

  attachInterrupt(interrupt_pin, isrWakeUp, LOW);   //Выход из режима сна по нажатию кнопки на 2ом пине (START)
  sei();                                            // Разрешаем обработку прерываний
  
  sleep_bod_disable();                              // Отключаем детектор пониженного напряжения питания
  sleep_cpu();                                      // Переводим МК в спящий режим

  // ПРОСНУЛИСЬ
  sleep_disable();

  PRR_NAME = 0;
  #ifdef PRR1
    PRR1 = 0;
  #endif

  DIDR0 = 0;
  DIDR1 = 0;

  ADCSRA |= _BV(ADEN);
  analogReference(DEFAULT);
#endif

  detachInterrupt(interrupt_pin);

#ifdef __LGT8FX8P__
  wdt_enable(WTO_8S);
#else
  wdt_enable(WDTO_8S);
#endif

  //pinMode(BTONOFF_PIN, OUTPUT);
  digitalWrite(BTONOFF_PIN, HIGH);  //Включаем преобразователь напряжения на модуле BT
  Serial.begin(9600);
  delay(500);

#if defined BT_STATUS_ENABLED && defined(ADC6D) && defined(ADC7D)
  timer_disconnected_inactivity.start();  //Сброс таймера
  gamepad_connected = false;
  DIDR0 |= _BV(ADC6D);    // Отключить цифровой буфер (нет утечки)
  DIDR0 |= _BV(ADC7D);
#else
  timer_connected_inactivity.start();  //Сброс таймера
#endif
}


void low_power_check() {
#ifdef __LGT8FX8P__
  gamepad_voltage = (analogRead(VCCM) * 1.024 * 5.0) / 1023.00;
#else
  gamepad_voltage = analogRead_VCC();
#endif

  //Отладка измерения напряжения
  //Serial.println(gamepad_voltage, 3);
  //Serial.print("*****\n");

  //При напряжении (avr_red_voltage; avr_yellow_voltage] - запуск медленного мигания и возврат для индикации работы джойстика
  if (avr_red_voltage < gamepad_voltage && gamepad_voltage < (avr_yellow_voltage - 0.07)) {
    if (!power_led.running())
      power_led.blinkForever(2000, 500);  //Мигать постоянно 2000мс вкл, 500мс выкл
    return;
  }
  else if (power_led.running())
    power_led.stop();

  //При слишком низком напряжении не даём полноценно включиться устройству и быстро мигаем светодиодом
  while (gamepad_voltage <= avr_red_voltage) {
    digitalWrite(BTONOFF_PIN, LOW); //Выключаем преобразователь напряжения на модуле BT
    Serial.end();
    pinMode(RX_PIN, INPUT);         // RX в High-Z (высокий импеданс)
    digitalWrite(TX_PIN, LOW);      // Принудительно 0 на TX
    pinMode(TX_PIN, OUTPUT);        // TX как выход 0Вольт
    
    for (int i = 0; i < 3; ++i) {
      power_led.blink(4, 300, 300);  // мигнуть 4 раза, 300мс вкл, 300мс выкл
      
      while (!power_led.ready()) {
        power_led.tick();
        wdt_reset();
        delay(50);
      }

      delay(1000);
    }

    gamepadSleep();
    power_led.stop();  //на всякий случай сбросить, если осталось активным

#ifdef __LGT8FX8P__
    gamepad_voltage = (analogRead(VCCM) * 1.024 * 5.0) / 1023.00;
#else
    gamepad_voltage = analogRead_VCC();
#endif
  }
}

//=====================================================================================================================================

void readAllButtons() {
  //==============================Обнуляем все данные==============================
  btnsByte1 = 0;
  btnsByte2 = 0;
  arrowsByte = 0;

  x1 = 0;
  y1 = 0;
  x2 = 0;
  y2 = 0;
//==============================Обнуляем все данные==============================

//=============================Считываем все данные==============================
//=========================Кнопки==========================
#ifdef SEGA_JOY
  //SEGA
  btnsByte1 |= (digitalRead(A_PIN) == LOW) ? JOY_1ST_A_BTN : 0;
  btnsByte1 |= (digitalRead(B_PIN) == LOW) ? JOY_1ST_B_BTN : 0;
  btnsByte1 |= (digitalRead(C_PIN) == LOW) ? JOY_1ST_C_BTN : 0;
  btnsByte1 |= (digitalRead(X_PIN) == LOW) ? JOY_1ST_X_BTN : 0;
  btnsByte1 |= (digitalRead(Y_PIN) == LOW) ? JOY_1ST_Y_BTN : 0;
  btnsByte1 |= (digitalRead(Z_PIN) == LOW) ? JOY_1ST_Z_BTN : 0;
  btnsByte1 |= (digitalRead(L1_PIN) == LOW) ? JOY_1ST_L1_BTN : 0;
  btnsByte1 |= (digitalRead(R1_PIN) == LOW) ? JOY_1ST_R1_BTN : 0;

  btnsByte2 |= (false) ? JOY_2ND_L2_BTN : 0;      //Не задействован в Sega Saturn
  btnsByte2 |= (false) ? JOY_2ND_R2_BTN : 0;      //Не задействован в Sega Saturn
  btnsByte2 |= (false) ? JOY_2ND_SELECT_BTN : 0;  //Не задействован в Sega Saturn
  btnsByte2 |= (digitalRead(START_PIN) == LOW) ? JOY_2ND_START_BTN : 0;
  btnsByte2 |= (false) ? JOY_2ND_MODE_BTN : 0;    //Не задействован в Sega Saturn
  btnsByte2 |= (false) ? JOY_2ND_THUMBL_BTN : 0;  //Не задействован в Sega Saturn
  btnsByte2 |= (false) ? JOY_2ND_THUMBR_BTN : 0;  //Не задействован в Sega Saturn
  btnsByte2 |= (false) ? JOY_2ND_BTN7 : 0;        //Не задействован в Sega Saturn
#elif defined(SNES_JOY)
  //SNES
  btnsByte1 |= (digitalRead(A_PIN) == LOW) ? JOY_1ST_A_BTN : 0;
  btnsByte1 |= (digitalRead(B_PIN) == LOW) ? JOY_1ST_B_BTN : 0;
  btnsByte1 |= (false) ? JOY_1ST_C_BTN : 0;  //Не задействован в SNES
  btnsByte1 |= (digitalRead(X_PIN) == LOW) ? JOY_1ST_X_BTN : 0;
  btnsByte1 |= (digitalRead(Y_PIN) == LOW) ? JOY_1ST_Y_BTN : 0;
  btnsByte1 |= (false) ? JOY_1ST_Z_BTN : 0;  //Не задействован в SNES
  btnsByte1 |= (digitalRead(L1_PIN) == LOW) ? JOY_1ST_L1_BTN : 0;
  btnsByte1 |= (digitalRead(R1_PIN) == LOW) ? JOY_1ST_R1_BTN : 0;

  btnsByte2 |= (false) ? JOY_2ND_L2_BTN : 0;  //Не задействован в SNES
  btnsByte2 |= (false) ? JOY_2ND_R2_BTN : 0;  //Не задействован в SNES
  btnsByte2 |= (digitalRead(SELECT_PIN) == LOW) ? JOY_2ND_SELECT_BTN : 0;
  btnsByte2 |= (digitalRead(START_PIN) == LOW) ? JOY_2ND_START_BTN : 0;
  btnsByte2 |= (false) ? JOY_2ND_MODE_BTN : 0;                                    //Не задействован в SNES
  btnsByte2 |= (false) ? JOY_2ND_THUMBL_BTN : 0;                                  //Не задействован в SNES
  btnsByte2 |= (false) ? JOY_2ND_THUMBR_BTN : 0;                                  //Не задействован в SNES
  btnsByte2 |= (false) ? JOY_2ND_BTN7 : 0;                                        //Не задействован в SNES
#endif
  //=========================Кнопки==========================
  //==========================D-PAD==========================
  arrowsByte |= (digitalRead(UP_PIN) == LOW) ? JOY_X1_UP : 0;
  arrowsByte |= (digitalRead(DOWN_PIN) == LOW) ? JOY_X1_DOWN : 0;
  arrowsByte |= (digitalRead(LEFT_PIN) == LOW) ? JOY_Y1_LEFT : 0;
  arrowsByte |= (digitalRead(RIGHT_PIN) == LOW) ? JOY_Y1_RIGHT : 0;

  //UP + RIGHT
  if ((arrowsByte & JOY_X1_UP) && (arrowsByte & JOY_Y1_RIGHT)) {
    x1 = 127;
    y1 = -127;
  }

  //UP + LEFT
  else if ((arrowsByte & JOY_X1_UP) && (arrowsByte & JOY_Y1_LEFT)) {
    x1 = -127;
    y1 = -127;
  }

  //DOWN + RIGHT
  else if ((arrowsByte & JOY_X1_DOWN) && (arrowsByte & JOY_Y1_RIGHT)) {
    x1 = 127;
    y1 = 127;
  }

  //DOWN + LEFT
  else if ((arrowsByte & JOY_X1_DOWN) && (arrowsByte & JOY_Y1_LEFT)) {
    x1 = -127;
    y1 = 127;
  }

  //UP
  if (arrowsByte & JOY_X1_UP)
    y1 = -127;

  //DOWN
  if (arrowsByte & JOY_X1_DOWN)
    y1 = 127;

  //RIGHT
  else if (arrowsByte & JOY_Y1_RIGHT)
    x1 = 127;

  //LEFT
  else if (arrowsByte & JOY_Y1_LEFT)
    x1 = -127;
  //==========================D-PAD==========================
  //=============================Считываем все данные==============================
}

void sendButtons(byte first_btns, byte second_btns, int8_t first_x, int8_t first_y, int8_t second_x, int8_t second_y) {
  Serial.write((byte)0xFD);         //Начало HID-отчета
  Serial.write((byte)0x6);          //Размер отчета в байтах
  Serial.write((byte)first_x);      //Координаты X первого стика
  Serial.write((byte)first_y);      //Координаты Y первого стика
  Serial.write((byte)second_x);     //Координаты X второго стика
  Serial.write((byte)second_y);     //Координаты Y второго стика
  Serial.write((byte)first_btns);   //Первый байт с состоянием кнопок
  Serial.write((byte)second_btns);  //Второй байт с состоянием кнопок
}



bool isPoweroffCombination(byte first_btns, byte second_btns, byte arrows_btns) {
#ifdef SEGA_JOY
  return first_btns == ((byte)0 | JOY_1ST_X_BTN | JOY_1ST_B_BTN | JOY_1ST_Z_BTN)  //X + B + Z
         && btnsByte2 == 0
         && arrowsByte == ((byte)0 | JOY_X1_DOWN);  //Вниз
                                                    /*&& (arrowsByte == ((byte)0 | JOY_X1_DOWN)                               //Вниз
                || arrowsByte == ((byte)0 | JOY_X1_DOWN | JOY_Y1_LEFT)              //Вниз и Влево
                || arrowsByte == ((byte)0 | JOY_X1_DOWN | JOY_Y1_RIGHT);            //Вниз и Вправо*/
#elif defined(SNES_JOY)
  return first_btns == ((byte)0 | JOY_1ST_Y_BTN | JOY_1ST_B_BTN | JOY_1ST_A_BTN)  //Y + B + A
         && btnsByte2 == 0
         && arrowsByte == ((byte)0 | JOY_X1_DOWN);  //Вниз
                                                    /*&& (arrowsByte == ((byte)0 | JOY_X1_DOWN)                               //Вниз
                || arrowsByte == ((byte)0 | JOY_X1_DOWN | JOY_Y1_LEFT)              //Вниз и Влево
                || arrowsByte == ((byte)0 | JOY_X1_DOWN | JOY_Y1_RIGHT);            //Вниз и Вправо*/
#endif
}
