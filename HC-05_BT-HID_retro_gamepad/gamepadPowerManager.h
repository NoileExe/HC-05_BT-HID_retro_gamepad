
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

//------------------------------------------------------------------

#pragma once


#include <Arduino.h>
#include <Blinker.h>	// GyverLibs

//------------------------------------------------------------------

class PowerManager
{
public:
	PowerManager(uint8_t wakeUpPin);
	
	void begin();
	void tick();

	// Актуализация состояния сопряжения BT-модуля
	bool readConnectionState();
	
	// Возвращает текущее напряжение
	float readVoltage();


	void setPreSleepCallback(void (*callback)());	// Действия перед сном
	void setWakeupCallback(void (*callback)());		// Действия при пробуждении

	void watchdog_reset();
	void watchdog_enable(bool enabled = true);

	void sleep();
	void wake();

private:
	// Есть ли соединение или таймаут по его отсутствию
	void checkConnection();
	
	//Проверка напряжения питания и управление LED/сном (при критическом разряде):
	//При напряжениях в диапазоне (VOLTAGE_CRITICAL_LEVEL; VOLTAGE_WARNING_LEVEL] Вольт - медленно мигаем и возвращаемся
	//При напряжении <= VOLTAGE_CRITICAL_LEVEL Вольт - не выходим из функции сообщая быстрым морганием и переходим в глубокий сон пока не будет заряжена АКБ
	void checkBattery();
	
	void indicateWarning();
	void indicateCritical();

	// Статическая функция-обработчик для attachInterrupt
	static void isrStub();

private:
	Blinker powerLed;				//Светодиод, мигающий при разрядке
	
	bool currentState;				// Последнее считанное состояние сопряжения BT-модуля
	uint32_t lastConnectedState;	// Время последнего активного состояния сопряжения (для выключения при простое более 2 МИНУТ)
	
	float currentVoltage;			// Последнее измеренное напряжение батареи (в Вольтах)
	uint32_t lastVoltageCheck;		// Время последнего измерения (в мс, millis())
	
	uint8_t interrupt_pin;

	// Статический указатель на пользовательскую функцию подготовки ко сну
	static void (*preSleepCallback)();
	// Статический указатель на пользовательскую функцию пробуждения
	static void (*wakeupCallback)();
};
