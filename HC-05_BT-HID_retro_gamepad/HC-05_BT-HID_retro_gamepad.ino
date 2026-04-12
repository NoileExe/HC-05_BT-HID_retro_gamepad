
/*
 * HC-05_BT-HID_retro_gamepad
 * Firmware for a retro gamepad (SEGA Genesis/MD2/Saturn or SNES) based on Arduino Pro Mini
 * or LGT8F328P, emulating a Bluetooth HID device via HC-05 module flashed with RN-42 firmware
 * (or original RN-42 module).
 * Copyright (C) 2026 Belobragin Anton Igorevich
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either d:\MICRO-SD copy Ardu\ARDUINO\REPO_versions\HC-05_BT-HID_retro_gamepad\.gitignoreversion 3 of the License, or
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
    В режиме сна ~5 мА / В режиме поиска соединения ~18-40 мА / В рабочем режиме 31-35 мА 
    
    Пины D6 и D7 (кнопки вверх и вправо) на atmega328p почему-то частенько отваливаются, начинаются ложные срабатывания и залипания
    По этой причине эти пины лучше сразу паять на другие выходы
    

    Возможности:
      Контроль низкого напряжения каждые 10 секунд
      Контроль бездействия более 10 минут
      Контроль наличия сопряжения с BT-модулем каждые 10 секунд
      Глубокий сон BT-модуля и ардуинки по достижению таймера бездействия или по нажатию соотв. комбинации (ВНИЗ + X + B + Z) в течение 5 сек.
      Включение режима сопряжения по соотв. комбинации (ВВЕРХ + A + Y + C) в течение 5 сек. (по сути же просто программная перезагрузка BT-модуля)
      Защита от зависаний Ардуино

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
#define SEGA_JOY
//#define SNES_JOY

//------------------------------------------------------------------

//Режимы Bluetooth
#define BT_MODE_COMMAND    	  "$$$"
#define BT_MODE_EXITCOMMAND  	"---\r\n"
#define BT_MODE_AUTOCONNECT  	"SM,6\r\n"
#define BT_MODE_MANUCONNECT  	"SM,4\r\n"
#define BT_MODE_STATUS			  "SO,/#\r\n"

//Ответы от модуля
#define BT_STAT_CMD  			    "CMD\r\n"
#define BT_STAT_END  			    "END\r\n"
#define BT_STAT_ACK  			    "AOK\r\n"
#define BT_STAT_REBOOT  	    "Reboot!\r\n"

//Команды получения информации
#define BT_GET_HID  			    "GH\n"
#define BT_GET_CONNECT_STAT   "GK\r\n"

//Системные команды
#define BT_REBOOT  				    "R,1\r\n"
#define BT_LAST_CONNECT  		  "C\r\n"
#define BT_NEW_CONNECT  		  "W\r\n"
#define BT_QUIET              "Q\r\n"
#define BT_DEEPSLEEP  			  "Z\r\n"

//------------------------------------------------------------------

#if defined(SEGA_JOY)
  //ПИНЫ SEGA
  #define TX_PIN      0     //присоединить к RXD BT-модуля (логический уровень 3.3 Вольта!)
  #define RX_PIN      1     //присоединить к TXD BT-модуля
  #define START_PIN   2     //Т.к. на данном пине можно произвести выход из сна
  #define A_PIN       5
  #define B_PIN       4
  #define C_PIN       3
  #define UP_PIN      6     //Возможно придётся поменять (напр. A4, если 6 глючит)
  #define RIGHT_PIN   7     //Возможно придётся поменять (напр. A5, если 7 глючит)
  #define DOWN_PIN    8
  #define LEFT_PIN    9
  #define X_PIN       A1
  #define Y_PIN       A2
  #define Z_PIN       A3
  #define L1_PIN      11
  #define R1_PIN      12
  #define BTSTAT_PIN  10    //Сигнал о состоянии сопряжения: HIGH - подключено, LOW - нет сопряжения/состояние поиска
  #define PWRLED_PIN  13    //Сигнал о низком заряде АКБ
#elif defined(SNES_JOY)
  //ПИНЫ SNES
  #define TX_PIN      0     //присоединить к RXD BT-модуля (логический уровень 3.3 Вольта!)
  #define RX_PIN      1     //присоединить к TXD BT-модуля
  #define START_PIN   2     //Т.к. на данном пине можно произвести выход из сна
  #define SELECT_PIN  3
  #define A_PIN       4
  #define B_PIN       5
  #define UP_PIN      6     //Возможно придётся поменять (напр. A4, если 6 глючит)
  #define RIGHT_PIN   7     //Возможно придётся поменять (напр. A5, если 7 глючит)
  #define DOWN_PIN    8
  #define LEFT_PIN    9
  #define X_PIN       A3
  #define Y_PIN       A2
  #define L1_PIN      11
  #define R1_PIN      12
  #define BTSTAT_PIN  10    //Сигнал о состоянии сопряжения: HIGH - подключено, LOW - нет сопряжения/состояние поиска
  #define PWRLED_PIN  13    //Сигнал о низком заряде АКБ
#endif

//------------------------------------------------------------------

//GAMEPAD/JOYSTICK КОДЫ КНОПОК
// 1ST == первый набор кнопок (BTN0 - BTN7)
#define JOY_1ST_A_BTN	      (1<<0)
#define JOY_1ST_B_BTN	      (1<<1)
#define JOY_1ST_C_BTN	      (1<<2)
#define JOY_1ST_X_BTN	      (1<<3)
#define JOY_1ST_Y_BTN       (1<<4)
#define JOY_1ST_Z_BTN       (1<<5)
#define JOY_1ST_L1_BTN	    (1<<6)
#define JOY_1ST_R1_BTN	    (1<<7)
#define JOY_1ST_NOBTN 	    0x00

// 2ND = второй набор кнопок (BTN0 - BTN7)
#define JOY_2ND_L2_BTN	    (1<<0)
#define JOY_2ND_R2_BTN	    (1<<1)
#define JOY_2ND_SELECT_BTN	(1<<2)
#define JOY_2ND_START_BTN	  (1<<3)
#define JOY_2ND_MODE_BTN	  (1<<4)
#define JOY_2ND_THUMBL_BTN	(1<<5)
#define JOY_2ND_THUMBR_BTN	(1<<6)
#define JOY_2ND_BTN7	      (1<<7)
#define JOY_2ND_NOBTN	      0x00

//Кнопки направлений (D-Pad)
#define JOY_X1_UP     (1<<0)
#define JOY_X1_DOWN	  (1<<1)
#define JOY_Y1_LEFT   (1<<2)
#define JOY_Y1_RIGHT  (1<<3)

//------------------------------------------------------------------

//Задержка в мс (16.7 ~60 Гц)
const float delay_time_ms = 17;

//Таймер удержания комбинации кнопок для выключения. 5 СЕКУНД
TimerMs timer_poweroff_combination(5 * 1000ul, 0, 1);

//Таймер проверки текущего напряжения. Каждые 10 СЕКУНД
TimerMs timer_voltage_check(10 * 1000ul, 0, 1);

//Таймер бездействия для выключения при простое более 10 минут
TimerMs timer_inactivity(10 * 1000ul * 60ul, 0, 1);

//Перевод цифрового пина в номер пина с прерыванием (для Arduino Pro Mini D2 == 0, D3 == 1)
const uint8_t interrupt_pin = digitalPinToInterrupt(START_PIN);

//------------------------------------------------------------------

//Напряжение, которое МК считает низким
#ifdef __LGT8FX8P__
	const float avr_voltage = 4.096;
#else
	const float avr_voltage = 3.3;
#endif

//Текущее напряжение
float gamepad_voltage = 5.0;

//Сопряжён ли геймпад
bool gamepad_connected = false;

//Предыдущие состояния кнопок
byte old_btnsByte1 = 0;
byte old_btnsByte2 = 0;
byte old_arrowsByte = 0;  //D-Pad

//Текущие состояния кнопок
byte btnsByte1 = 0;
byte btnsByte2 = 0; 
byte arrowsByte = 0;    //D-Pad

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
//При напряжениях в диапазоне (3.2; avr_voltage] Вольт - медленно мигаем и возвращаемся
//При напряжении <= 3.2 Вольт - не выходим из функции сообщая быстрым морганием и переходим в глубокий сон пока не будет заряжена АКБ
void low_power_check();

//------------------------------------------------------------------

void setup()
{
  #ifdef __LGT8FX8P__
    //analogReference(INTERNAL4V096);
    analogReference(INTERNAL1V024);
  #else
    analogReference(DEFAULT);
  #endif
  
  
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
    pinMode(X_PIN, INPUT_PULLUP);         //Аналоговый пин как цифровой
    pinMode(Y_PIN, INPUT_PULLUP);         //Аналоговый пин как цифровой
    pinMode(Z_PIN, INPUT_PULLUP);         //Аналоговый пин как цифровой
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
    pinMode(X_PIN, INPUT_PULLUP);         //Аналоговый пин как цифровой
    pinMode(Y_PIN, INPUT_PULLUP);         //Аналоговый пин как цифровой
    pinMode(L1_PIN, INPUT_PULLUP);      
    pinMode(R1_PIN, INPUT_PULLUP);
  #endif

  pinMode(BTSTAT_PIN, INPUT_PULLUP);
  
  //Слежение за зависанием самой ардуинки. Таймаут установлен максимальный ~8с
  #ifdef __LGT8FX8P__
    wdt_enable(WTO_8S);
  #else
    wdt_enable(WDTO_8S);
  #endif

  Serial.begin(9600);
  delay(500); //Обязательная минимальная задержка после включения перед отправкой команд
  Serial.print(BT_MODE_COMMAND);
  while ( Serial.available() )    (char)Serial.read();
  delay(500);
  Serial.print(BT_REBOOT);
  while ( Serial.available() )    (char)Serial.read();
  delay(500);

  timer_inactivity.start();
}

//------------------------------------------------------------------

void loop()
{
  power_led.tick();
  delay(delay_time_ms);
  
  //=======================Предусловия=======================
  //Каждые 10 секунд проверяем напряжение
  if ( !timer_voltage_check.active()  ||  timer_voltage_check.tick() )
  {
    low_power_check();  //Проверка напряжения
    timer_voltage_check.start();   //Сброс таймера
  }
  //=======================Предусловия=======================

  //========================Комбинации=======================
  gamepad_connected = digitalRead(BTSTAT_PIN) == HIGH;
  readAllButtons();   //Чтение нажатых кнопок
  
  //Если таймер простоя достиг своего требуемого значения - переход в спящий режим
  if ( timer_inactivity.tick() )
    gamepadSleep();

  else if (btnsByte1 != 0  ||  btnsByte2 != 0  ||  arrowsByte != 0)
    timer_inactivity.start();   //Сброс таймера


  //Если удерживается комбинация кнопок для выключения
  // или таймер простоя достиг требуемого значения - прибавляем таймер
  if ( isPoweroffCombination(btnsByte1, btnsByte2, arrowsByte) )
  {
    if ( !timer_poweroff_combination.active() )   timer_poweroff_combination.start();
    
    //Если таймер удержания кнопок достиг требуемого значения - переход в спящий режим
    if ( timer_poweroff_combination.tick() )
      gamepadSleep();

    //Если геймпад сопряжён и предыдущая комбинация нажатых (или не нажатых) кнопок отличается от текущей - отправляем пустую комбинацию кнопок,
    // чтобы устройство, к которому подключен геймпад не получало лишней информации
    if ( gamepad_connected  &&  (btnsByte1 != old_btnsByte1  ||  btnsByte2 != old_btnsByte2  ||  arrowsByte != old_arrowsByte) )
    {
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
  if (  timer_poweroff_combination.active() )   timer_poweroff_combination.stop();
  //===================Работа с таймерами====================
  
  //=====================Отправка данных=====================
  //Если геймпад сопряжён и предыдущая комбинация нажатых (или не нажатых) кнопок отличается от текущей - отправляем новую комбинацию кнопок
  if ( gamepad_connected  &&  (btnsByte1 != old_btnsByte1  ||  btnsByte2 != old_btnsByte2  ||  arrowsByte != old_arrowsByte) )
  {
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

void isrWakeUp()
{
}


void gamepadSleep()
{
  if (gamepad_connected)
  {
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

  Serial.print(BT_MODE_COMMAND);
  while ( Serial.available() )    (char)Serial.read();
  delay(500);
  Serial.print(BT_QUIET);
  while ( Serial.available() )    (char)Serial.read();
  delay(500);
  Serial.print(BT_DEEPSLEEP);
  while ( Serial.available() )    (char)Serial.read();
  delay(500);
  Serial.print(BT_MODE_EXITCOMMAND);
  while ( Serial.available() )    (char)Serial.read();
  Serial.end();

  if ( timer_poweroff_combination.active() )    timer_poweroff_combination.stop();
  if ( timer_voltage_check.active() )           timer_voltage_check.stop();
  if ( timer_inactivity.active() )              timer_inactivity.stop();
  
  wdt_disable();
  
  #ifdef __LGT8FX8P__
    attachInterrupt(interrupt_pin, isrWakeUp, LOW);
    PMU.sleep(PM_POFFS1, SLEEP_FOREVER); 
  #else
    ADCSRA &= ~(1 << ADEN); // Отключаем АЦП
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); //Устанавливаем интересующий нас режим
  
    cli(); // Временно запрещаем обработку прерываний
    sleep_enable();
    // Отключаем детектор пониженного напряжения питания
    MCUCR != (1 << BODS) | (1 << BODSE);
    MCUCR &= ~(1 << BODSE);
    sei(); // Разрешаем обработку прерываний
  
    //Выход из режима сна по нажатию кнопки на 2ом пине (START)
    attachInterrupt(interrupt_pin, isrWakeUp, LOW);

    sleep_cpu(); // Переводим МК в спящий режим
    sleep_disable();
  
    ADCSRA |= 1 << ADEN; // Включаем АЦП
  #endif
  
  detachInterrupt(interrupt_pin);
  #ifdef __LGT8FX8P__
    wdt_enable(WTO_8S);
  #else
    wdt_enable(WDTO_8S);
  #endif
  
  Serial.begin(9600);
  
  //После пробуждения перезапускаем BT
  Serial.write((byte)0x00);
  delay(500);
  Serial.print(BT_MODE_COMMAND);
  while ( Serial.available() )    (char)Serial.read();
  delay(500);
  Serial.print(BT_REBOOT);
  while ( Serial.available() )    (char)Serial.read();
  delay(500);

  gamepad_connected = false;
  timer_inactivity.start();   //Сброс таймера
}


void low_power_check()
{
  #ifdef __LGT8FX8P__
    gamepad_voltage = (analogRead(VCCM) * 5.0) / 1023.00;
  #else
    gamepad_voltage = analogRead_VCC();
  #endif
  
  //Отладка измерения напряжения
  //Serial.println(gamepad_voltage, 3);
  //Serial.print("*****\n");

  //При напряжении (3.2; avr_voltage] - запуск медленного мигания и возврат для индикации работы джойстика
  if (3.2 < gamepad_voltage  &&  gamepad_voltage < (avr_voltage - 0.07))
  {
    power_led.blink(1, 2000, 500);  //Мигнуть 1 раз, 2000мс вкл, 500мс выкл
    return;
  }

  //При слишком низком напряжении не даём полноценно включиться устройству и быстро мигая светодиодом
  while ( gamepad_voltage <= 3.2 )
  {
    for (int i = 0; i < 3; ++i)
    {
      power_led.blink(4, 300, 300);   // мигнуть 4 раза, 300мс вкл, 300мс выкл
      delay(50);
      while ( !power_led.ready() )
      {
        power_led.tick();
        wdt_reset();
        delay(50);
      }

      delay(1000);
    }

    gamepadSleep();
    
	  #ifdef __LGT8FX8P__
      gamepad_voltage = (analogRead(VCCM) * 5.0) / 1023.00;
    #else
      gamepad_voltage = analogRead_VCC();
    #endif
  }
}

//=====================================================================================================================================

void readAllButtons()
{
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
    btnsByte1 |= (digitalRead(A_PIN) == LOW)  ?  JOY_1ST_A_BTN : 0;
    btnsByte1 |= (digitalRead(B_PIN) == LOW)  ?  JOY_1ST_B_BTN : 0;
    btnsByte1 |= (digitalRead(C_PIN) == LOW)  ?  JOY_1ST_C_BTN : 0;
    btnsByte1 |= (digitalRead(X_PIN) == LOW)  ?  JOY_1ST_X_BTN : 0;
    btnsByte1 |= (digitalRead(Y_PIN) == LOW)  ?  JOY_1ST_Y_BTN : 0;
    btnsByte1 |= (digitalRead(Z_PIN) == LOW)  ?  JOY_1ST_Z_BTN : 0;
    btnsByte1 |= (digitalRead(L1_PIN) == LOW) ?  JOY_1ST_L1_BTN : 0;
    btnsByte1 |= (digitalRead(R1_PIN) == LOW) ?  JOY_1ST_R1_BTN : 0;

    btnsByte2 |= ( false )                        ?  JOY_2ND_L2_BTN : 0;      //Не задействован в Sega Saturn
    btnsByte2 |= ( false )                        ?  JOY_2ND_R2_BTN : 0;      //Не задействован в Sega Saturn
    btnsByte2 |= ( false )                        ?  JOY_2ND_SELECT_BTN : 0;  //Не задействован в Sega Saturn
    btnsByte2 |= (digitalRead(START_PIN) == LOW)  ?  JOY_2ND_START_BTN : 0;
    btnsByte2 |= ( false )                        ?  JOY_2ND_MODE_BTN : 0;    //Не задействован в Sega Saturn
    btnsByte2 |= ( false )                        ?  JOY_2ND_THUMBL_BTN : 0;  //Не задействован в Sega Saturn
    btnsByte2 |= ( false )                        ?  JOY_2ND_THUMBR_BTN : 0;  //Не задействован в Sega Saturn
    btnsByte2 |= ( false )                        ?  JOY_2ND_BTN7 : 0;        //Не задействован в Sega Saturn
  #elif defined(SNES_JOY)
    //SNES
    btnsByte1 |= (digitalRead(A_PIN) == LOW)  ?  JOY_1ST_A_BTN : 0;
    btnsByte1 |= (digitalRead(B_PIN) == LOW)  ?  JOY_1ST_B_BTN : 0;
    btnsByte1 |= ( false )                    ?  JOY_1ST_C_BTN : 0;      //Не задействован в SNES
    btnsByte1 |= (digitalRead(X_PIN) == LOW)  ?  JOY_1ST_X_BTN : 0;
    btnsByte1 |= (digitalRead(Y_PIN) == LOW)  ?  JOY_1ST_Y_BTN : 0;
    btnsByte1 |= ( false )                    ?  JOY_1ST_Z_BTN : 0;      //Не задействован в SNES
    btnsByte1 |= (digitalRead(L1_PIN) == LOW) ?  JOY_1ST_L1_BTN : 0;
    btnsByte1 |= (digitalRead(R1_PIN) == LOW) ?  JOY_1ST_R1_BTN : 0;

    btnsByte2 |= ( false )                        ?  JOY_2ND_L2_BTN : 0;      //Не задействован в SNES
    btnsByte2 |= ( false )                        ?  JOY_2ND_R2_BTN : 0;      //Не задействован в SNES
    btnsByte2 |= (digitalRead(SELECT_PIN) == LOW) ?  JOY_2ND_SELECT_BTN : 0;
    btnsByte2 |= (digitalRead(START_PIN) == LOW)  ?  JOY_2ND_START_BTN : 0;
    btnsByte2 |= ( false )                        ?  JOY_2ND_MODE_BTN : 0;    //Не задействован в SNES
    btnsByte2 |= ( false )                        ?  JOY_2ND_THUMBL_BTN : 0;  //Не задействован в SNES
    btnsByte2 |= ( false )                        ?  JOY_2ND_THUMBR_BTN : 0;  //Не задействован в SNES
    btnsByte2 |= ( false )                        ?  JOY_2ND_BTN7 : 0;        //Не задействован в SNES
  #endif
  //=========================Кнопки==========================
  //==========================D-PAD==========================
  arrowsByte |= (digitalRead(UP_PIN) == LOW)    ?  JOY_X1_UP : 0;
  arrowsByte |= (digitalRead(DOWN_PIN) == LOW)  ?  JOY_X1_DOWN : 0;
  arrowsByte |= (digitalRead(LEFT_PIN) == LOW)  ?  JOY_Y1_LEFT : 0;
  arrowsByte |= (digitalRead(RIGHT_PIN) == LOW) ?  JOY_Y1_RIGHT : 0;

  //UP + RIGHT
  if ( (arrowsByte & JOY_X1_UP)  &&  (arrowsByte & JOY_Y1_RIGHT) )
  {
    x1 = 127;
    y1 = -127;
  }

  //UP + LEFT
  else if ( (arrowsByte & JOY_X1_UP)  &&  (arrowsByte & JOY_Y1_LEFT) )
  {
    x1 = -127;
    y1 = -127;
  }

  //DOWN + RIGHT
  else if ( (arrowsByte & JOY_X1_DOWN)  &&  (arrowsByte & JOY_Y1_RIGHT) )
  {
    x1 = 127;
    y1 = 127;
  }

  //DOWN + LEFT
  else if ( (arrowsByte & JOY_X1_DOWN)  &&  (arrowsByte & JOY_Y1_LEFT) )
  {
    x1 = -127;
    y1 = 127;
  }

  //UP
  if ( arrowsByte & JOY_X1_UP )
    y1 = -127;

  //DOWN
  if ( arrowsByte & JOY_X1_DOWN )
    y1 = 127;

  //RIGHT
  else if ( arrowsByte & JOY_Y1_RIGHT )
    x1 = 127;

  //LEFT
  else if ( arrowsByte & JOY_Y1_LEFT )
    x1 = -127;
  //==========================D-PAD==========================
  //=============================Считываем все данные==============================
}

void sendButtons(byte first_btns, byte second_btns, int8_t first_x, int8_t first_y, int8_t second_x, int8_t second_y)
{
  Serial.write((byte)0xFD);         //Начало HID-отчета
  Serial.write((byte)0x6);          //Размер отчета в байтах
  Serial.write((byte)first_x);      //Координаты X первого стика
  Serial.write((byte)first_y);      //Координаты Y первого стика
  Serial.write((byte)second_x);     //Координаты X второго стика
  Serial.write((byte)second_y);     //Координаты Y второго стика
  Serial.write((byte)first_btns);   //Первый байт с состоянием кнопок
  Serial.write((byte)second_btns);  //Второй байт с состоянием кнопок
}



bool isPoweroffCombination(byte first_btns, byte second_btns, byte arrows_btns)
{
  #ifdef SEGA_JOY
    return first_btns == ((byte)0 | JOY_1ST_X_BTN | JOY_1ST_B_BTN | JOY_1ST_Z_BTN)  //X + B + Z
            && btnsByte2 == 0
            && arrowsByte == ((byte)0 | JOY_X1_DOWN);                               //Вниз
  #elif defined(SNES_JOY)
    return first_btns == ((byte)0 | JOY_1ST_Y_BTN | JOY_1ST_B_BTN | JOY_1ST_A_BTN)  //Y + B + A
            && btnsByte2 == 0
            && arrowsByte == ((byte)0 | JOY_X1_DOWN);                               //Вниз
  #endif
}
