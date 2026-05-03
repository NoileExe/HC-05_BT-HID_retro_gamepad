
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

#include "RN42_CMD.h"

//------------------------------------------------------------------

class RN42_HID_gamepad
{
	struct ButtonsCombination
	{
		uint8_t btn1{0}, btn2{0}, dpad{0};
		int8_t x1{0}, y1{0}, x2{0}, y2{0};
		
		bool isEmpty() const
		{
			return *this == ButtonsCombination();
		}
		
		bool operator==(const ButtonsCombination& other) const
		{
			return btn1 == other.btn1 && btn2 == other.btn2 && 
					dpad == other.dpad && x1 == other.x1 && y1 == other.y1 && 
					x2 == other.x2 && y2 == other.y2;
		}
		
		bool operator!=(const ButtonsCombination& other) const
		{
			return !(*this == other);
		}
	};

public:
	enum Button : uint8_t
	{
		A,
		B,
		C,
		X,
		Y,
		Z,
		L1,
		R1,
		L2,			// L2/R2 НЕ аналоговые, а цифровые в RN42
		R2,
		SELECT,
		START,
		MODE,
		THUMBL,		// L3
		THUMBR,		// R3
		HOME,		// HOME?
		UP,
		DOWN,
		LEFT,
		RIGHT,
		ButtonCount
	};

	enum Mode : uint8_t
	{
		Standard,
		Turbo,		// Множественное нажатие кнопок A/B/C/X/Z/Y/L1/R1/L2/R2/L3/R3
		Slow,		// Быстрое нажатие-отпускание кнопки START для эмуляции слоумо
		ModeCount
	};


public:
	RN42_HID_gamepad();

	// Обязательно требуется вызов Serial.begin()
	bool begin(Stream& uart_stream);
	
	bool end() { uart = nullptr; }

	// Установка состояний осей и кнопок ДО ОТПРАВКИ
	void setAxisState(int8_t x1, int8_t y1, int8_t x2, int8_t y2);
	void setButtonState(Button btn, bool isPressed);

	// Отправка HID-репорта
	void sendButtons();
	
	void sendAllEmptyButtons();
	
	bool isNewCombination() const { return currBtns != prevBtns; };
	bool isButtonPressed(Button btn) const;
	bool isAnyButtonPressed() const { return !currBtns.isEmpty(); };

	// Включить/выключить Tubro-функционал для отдельной кнопки ДЕЙСТВИЯ
	void setTurboButton(Button btn, bool isTurbo);
	bool isTurboButton(Button btn) const;
	bool isTurboSetForAnyButton() const { return !turboBtns.isEmpty(); }

	// Смена режима работы геймпада: Standard, Turbo, Slow
	void changeMode(Mode mode);
	Mode getMode() { return currMode; }
	void setChangeModeCombination(const Button* combo, uint8_t count);		//Только кнопки, без осей
	bool isChangeModeCombination(const Button* combo, uint8_t count) const;
	bool isChangeModeCombination() const;


	// Возможно если пооддерживается пробуждение по UART
	// Иначе только подача сигнала на RESET (аппаратный сброс)
	bool wakeUp(unsigned long timeout_ms = 500);
	
	// Deep Sleep BT-модуля
	void powerOff();
	void setPowerOffCombination(const Button* combo, uint8_t count);		//Только кнопки, без осей
	bool isPowerOffCombination(const Button* combo, uint8_t count) const;
	bool isPowerOffCombination() const;

	// Установка базовых настроек геймпада с отключением SniffMode, включением DeepSleep, изменением имени.
	// ПЕРЕЗАГРУЖАЕТ модуль для применения настроек
	// ВАЖНО:
	//	НА ВХОДЕ
	//		соблюдение скорости вашего BT-модуля на момент входа (возможно 115200)
	//	НА ВЫХОДЕ
	//		Serial baudrate устанавливается на скорость указанную в аргументе br - если скорость изменилась:
	//		для Serial настроенного в uart нужно обязательно вызвать Serial.end(); Serial.begin(НОВАЯ_СКОРОСТЬ);
	void setDefaultSettings(const char* name = "RN42_HID_gamepad", RN42::BaudRate br = RN42::BaudRate::BR_9600);

	// Установка имени BT-устройства. ПЕРЕЗАГРУЖАЕТ модуль для применения настроек
	void setDeviceName(const char* name = "RN42_HID_gamepad");

	// Установка скорости UART. ПЕРЕЗАГРУЖАЕТ модуль для применения настроек
	// ВАЖНО:
	//	НА ВХОДЕ
	//		соблюдение скорости вашего BT-модуля на момент входа (после factoryReset() - 115200)
	//	НА ВЫХОДЕ
	//		Serial baudrate устанавливается на скорость указанную в аргументе br - если скорость изменилась:
	//		для Serial настроенного в uart нужно обязательно вызвать Serial.end(); Serial.begin(НОВАЯ_СКОРОСТЬ);
	void setBaudRate(RN42::BaudRate br = RN42::BaudRate::BR_9600);

	// Отправка пустого отчета (все кнопки отжаты) и разрыв соединения
	void disconnect();

	const char* getDeviceName();	// Получить имя BT-устройства
	bool isConnected();				// Установленно ли соединение, но быстрее по светодиоду на GPIO2

private:
	// Установка состояний кнопок в комбинации (установка бита кнопки в нужном байте - ON/OFF)
	void setButtonInReport(ButtonsCombination& bc, Button btn, bool state);
	
	// Включен ли бит кнопки в комбинации
	bool isButtonInReport(const ButtonsCombination& bc, Button btn) const;
	
	// Создание комбинации из указанного массива
	inline ButtonsCombination createCombo(const Button* combo, uint8_t count) const;
	
	// Чтение ответной команды и возврат её длины и самого ответа out
	int readLine(char* out, size_t maxLen, unsigned long timeout_ms = 500);
	
	// Проверка ответа RN-42 на посланные команды
	// Ответ: индекс строки в expected ИЛИ -1 - не найдено/ошибка/таймаут
	//int readResponse(const char* const expected[], size_t count, unsigned long timeout_ms = 500);

	bool enterCommandMode(unsigned long timeout_ms = 500);
	bool exitCommandMode(unsigned long timeout_ms = 500);

private:
	Stream* uart;

	Mode currMode;
	
	ButtonsCombination currBtns;
	ButtonsCombination prevBtns;
	
	ButtonsCombination turboBtns; // Кнопки, на которых всегда включен режим турбо
	
	ButtonsCombination powerOffCombo;
	ButtonsCombination changeModeCombo;


	static constexpr size_t MAX_STR_LEN = 32; // RN-42 лимит: имя ≤20, версия ≤18
	char nameBuf[MAX_STR_LEN] = {0};
};