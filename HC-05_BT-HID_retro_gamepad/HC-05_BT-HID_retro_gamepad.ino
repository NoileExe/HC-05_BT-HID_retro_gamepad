
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

//=====================================================================================================================================

/*
    Все нажатия кнопок на цифровых пинах срабатывают на логический нуль (LOW)
    Аналоговые пины работают в режиме цифровых
    Потребление:
      В режиме поиска соединения до 40 мА
      В рабочем режиме до 35 мА
      В режиме сна на ATMEG'е с 8МГц менее 10 мкА / на LGT8F328P ~50 мкА
    
    Пины D6 и D7 (кнопки вверх и вправо) на ATMEGA328P/ATMEGA328PA могут работать некорректно - начинаются ложные срабатывания и залипания
    По этой причине эти пины лучше сразу перенастроить на другие, либо попытать удачу с предложенным кодом
    

    Возможности:
      Контроль низкого напряжения каждые 10 секунд и разное поведение и световая индикация при небольшом и критическом разряде
      Контроль бездействия более 5 минут
      Контроль наличия сопряжения с BT-модулем (если соединение отсутствует более 2 минут - уход в сон)
      Глубокий сон BT-модуля и Arduino по достижению таймера бездействия или по нажатию соотв. комбинации:  "ВНИЗ  + A + B"  в течение 3 сек.
      Смена режима (Стандартный/Тубро/Слоумо) по комбинации (в дополнение световая индикация):              "ВВЕРХ + A + B"  в течение 3 сек.
      Защита от зависаний Arduino / LGT8F328P
      Защита от зависаний BT-модуля путём перевода джойстика в режим сна и пробуждения
*/


#include <GTimer.h>    //GyverLibs

#include "RN42_HID_gamepad.h"

#include "gamepadConfig.h"
#include "gamepadPowerManager.h"

//=====================================================================================================================================

// Считывание нажатых в данный момент кнопок и установка в RN42_HID_gamepad
// Возвращает true если нажата хоть одна кнопка
void readAllButtons();


// Колбэк при срабатывании таймера удержания одной из комбинаций
void onCombinationTimeout();

// Колбэк при срабатывании таймера бездействия
void onInactivityTimeout();


// Подготовка устройства ко сну
void preparingForSleep();

// Обработчик аппаратного прерывания для выхода из режима сна
void isrWakeUp();

//=====================================================================================================================================

//Таймер удержания комбинации кнопок для выключения/смены режима. 3 СЕКУНДЫ
GTimerCb<millis> timer_combination(COMBINATION_HOLD_MS, onCombinationTimeout, GTMode::Overflow);

//Таймер бездействия для выключения при простое более 5 МИНУТ
GTimerCb<millis> timer_inactivity(INACTIVITY_CONNECTED_MS, onInactivityTimeout, GTMode::Overflow);

//=====================================================================================================================================

// Управление питанием и сном при низком напряжении или долгом отсутствии сопряжения
PowerManager gamepad_power_manager( getButtonPin(RN42_HID_gamepad::Button::START) );

// Класс взаимодействия с BT-модулем RN-42
RN42_HID_gamepad hid_gamepad;

//=====================================================================================================================================

void setup()
{
  // Инициализация пинов всех кнопок
  for (uint8_t btn = 0; btn < BUTTONS_VARIANTS; btn++)
  {
    uint8_t pin = getButtonPin(static_cast<RN42_HID_gamepad::Button>(btn));
    
    if (pin != UINT8_MAX)
      pinMode(pin, INPUT_PULLUP);
  }

  // Ожидаем пока кнопку пробуждения отпустят
  uint8_t startPin = getButtonPin(RN42_HID_gamepad::Button::START);
  while (digitalRead(startPin) == LOW)
  {
    delay(10);
  }

  gamepad_power_manager.setPreSleepCallback(preparingForSleep);
  gamepad_power_manager.setWakeupCallback(isrWakeUp);
  gamepad_power_manager.begin();  //Serial.begin(9600) - обязательно перед hid_gamepad.begin(), делается внутри PowerManager::begin()

  hid_gamepad.begin(Serial);


#ifdef GAMEPAD_NAME
  digitalWrite(PWRLED_PIN, HIGH);
  gamepad_power_manager.watchdog_enable(false);


  // Установка настроек геймпада, НО только при несовпадении имени BT-модуля
  // Делается в две попытки - если не удалось поменять в первый раз, значит
  // скорее всего скорость на дефолтном уровне и её требуется сменить на время настройки
  for (int i = 0; i <2; i++)
  {
    if (i > 0)
    {
      Serial.end();
      Serial.begin(115200);
    }

    delay(3000);
    const char* gamepadName = hid_gamepad.getDeviceName();

    if (gamepadName
        && *gamepadName != '\0'
        && strcmp(gamepadName, GAMEPAD_NAME) != 0)
    {
      delay(1000);
      //hid_gamepad.setDeviceName(GAMEPAD_NAME);
      hid_gamepad.setDefaultSettings(GAMEPAD_NAME);
      
      if (i > 0)
      {
        Serial.end();
        Serial.begin(9600);
      }
    }

    delay(3000);
    gamepadName = hid_gamepad.getDeviceName();
    if (gamepadName
        && *gamepadName != '\0'
        && strcmp(gamepadName, GAMEPAD_NAME) == 0)
    {
      break;
    }
  }

  gamepad_power_manager.watchdog_enable();
  digitalWrite(PWRLED_PIN, LOW);
#endif

  hid_gamepad.setPowerOffCombination(POWEROFF_COMBO, POWEROFF_COMBO_SIZE);
  hid_gamepad.setChangeModeCombination(CHMODE_COMBO, CHMODE_COMBO_SIZE);
  hid_gamepad.changeMode(RN42_HID_gamepad::Mode::Standard);

  timer_inactivity.start();
}

//------------------------------------------------------------------

void loop()
{
  delay(REPORT_INTERVAL_MS);

  readAllButtons();               // Чтение нажатых кнопок сразу в hid_gamepad

  gamepad_power_manager.tick();   // Предусловия (напряжение батареи, наличие активного сопряжения)

  // Отсчёт таймеров (если активны)
  if (timer_combination.tick() || timer_inactivity.tick())
  {
    return;
  }  

  // Нажата хоть одна кнопка
  if ( hid_gamepad.isAnyButtonPressed() )
  {
    timer_inactivity.start();  //Сброс таймера


    bool isPowerOffCombination = hid_gamepad.isPowerOffCombination();
    bool isChangeModeCombination = hid_gamepad.isChangeModeCombination();

    // Если удерживается комбинация кнопок для выключения/смены режима
    if (isPowerOffCombination || isChangeModeCombination)
    {
      if (!timer_combination.running())
      {
        timer_combination.start();
        hid_gamepad.sendAllEmptyButtons();
      }
      
      return;
    }
  }
  
  
  if (timer_combination.running())
  {
    timer_combination.stop();
    hid_gamepad.sendAllEmptyButtons();
    return;
  }
  

  hid_gamepad.sendButtons();    // Отправка нажатых и не нажатых кнопок
}

//=====================================================================================================================================

void readAllButtons()
{
  // Считываем все данные
  for (uint8_t btn = 0; btn < BUTTONS_VARIANTS; btn++)
  {
    RN42_HID_gamepad::Button currBtn = static_cast<RN42_HID_gamepad::Button>(btn);
    uint8_t pin = getButtonPin(currBtn);
    
    if (pin != UINT8_MAX)
    {
      bool isCurrButtonPressed = digitalRead(pin) == LOW;
      hid_gamepad.setButtonState(currBtn, isCurrButtonPressed);
    }
  }
  
  // Левый и правый стики читаются отдельно как аналоговые значения
  // Обработка отсутствует т.к. у Sega MD2 / Saturn и NES / SNES нет аналоговых стиков
}

//=====================================================================================================================================

void onCombinationTimeout()
{
  timer_combination.stop();   // Сбросим сразу
  

  // Если таймер удержания кнопок достиг требуемого значения и комбинация соответствует - ВЫКЛЮЧАЕМ
  if ( hid_gamepad.isPowerOffCombination() )
  {
    gamepad_power_manager.sleep();  // Вызовет колбэк preparingForSleep()
                                    // При пробуждении вызовет колбэк isrWakeUp()
  }
  
  // Если таймер удержания кнопок достиг требуемого значения и комбинация соответствует - МЕНЯЕМ РЕЖИМ
  else if ( hid_gamepad.isChangeModeCombination() )
  {
    // Количество значений
    static const uint8_t modeCount = static_cast<uint8_t>(RN42_HID_gamepad::Mode::ModeCount);
        
    RN42_HID_gamepad::Mode nextMode = hid_gamepad.getMode();
    nextMode = static_cast<RN42_HID_gamepad::Mode>((nextMode + 1) % modeCount);
    hid_gamepad.changeMode(nextMode);

    // Сообщаем пользователю о смене моргая светодиодом питания
    for (int i = 0; i < 5; i++)
    {
      digitalWrite(PWRLED_PIN, HIGH);
      delay(100);
      digitalWrite(PWRLED_PIN, LOW);
      delay(100);
    }
  }
}

void onInactivityTimeout()
{
  gamepad_power_manager.sleep();  // Вызовет колбэк preparingForSleep()
                                  // При пробуждении вызовет колбэк isrWakeUp()
}

//=====================================================================================================================================

// Подготовка ко сну
void preparingForSleep()
{
  bool isBtConnected = gamepad_power_manager.readConnectionState();

  if (isBtConnected)
    hid_gamepad.disconnect();

  timer_combination.stop();
  timer_inactivity.stop();
}

// Пробуждение
void isrWakeUp()
{
  timer_inactivity.start();  //Сброс таймера
}

//=====================================================================================================================================
