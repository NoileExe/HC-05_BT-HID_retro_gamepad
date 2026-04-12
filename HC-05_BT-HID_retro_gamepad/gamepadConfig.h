
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

#include "RN42_HID_gamepad.h"

//------------------------------------------------------------------

// ВЫБОР РЕЖИМА ГЕЙМПАДА - SEGA Genesis/MD2/Saturn либо NES или SNES
//#define SEGA_JOY
//#define NES_JOY
//#define SNES_JOY


// ИМЯ BT-УСТРОЙСТВА (RN-42 поддерживает имена длиной не более 20 символов; можно с пробелами)
//#define GAMEPAD_NAME "My_Gamepad_001"

//------------------------------------------------------------------

// Проверка, что определён хотя бы один из вариантов геймпада
#if !defined(SEGA_JOY) && !defined(SNES_JOY)
  #error "Define SEGA_JOY, NES_JOY or SNES_JOY!"
#endif

//------------------------------------------------------------------

// Комбинации кнопок для системных действий
static constexpr uint8_t POWEROFF_COMBO_SIZE = 4;
static constexpr uint8_t CHMODE_COMBO_SIZE = 4;

#if defined(SEGA_JOY)
	static constexpr RN42_HID_gamepad::Button POWEROFF_COMBO[] = {
		RN42_HID_gamepad::Button::DOWN,
		RN42_HID_gamepad::Button::X,
		RN42_HID_gamepad::Button::B,
		RN42_HID_gamepad::Button::Z
	};

	static constexpr RN42_HID_gamepad::Button CHMODE_COMBO[] = {
		RN42_HID_gamepad::Button::UP,
		RN42_HID_gamepad::Button::A,
		RN42_HID_gamepad::Button::Y,
		RN42_HID_gamepad::Button::C
	};
#elif defined(NES_JOY)
	static constexpr RN42_HID_gamepad::Button POWEROFF_COMBO[] = {
		RN42_HID_gamepad::Button::DOWN,
		RN42_HID_gamepad::Button::B,
		RN42_HID_gamepad::Button::A
	};

	static constexpr RN42_HID_gamepad::Button CHMODE_COMBO[] = {
		RN42_HID_gamepad::Button::UP,
		RN42_HID_gamepad::Button::B,
		RN42_HID_gamepad::Button::A
	};
#elif defined(SNES_JOY)
	static constexpr RN42_HID_gamepad::Button POWEROFF_COMBO[] = {
		RN42_HID_gamepad::Button::DOWN,
		RN42_HID_gamepad::Button::Y,
		RN42_HID_gamepad::Button::B,
		RN42_HID_gamepad::Button::A
	};

	static constexpr RN42_HID_gamepad::Button CHMODE_COMBO[] = {
		RN42_HID_gamepad::Button::UP,
		RN42_HID_gamepad::Button::Y,
		RN42_HID_gamepad::Button::X,
		RN42_HID_gamepad::Button::A
	};
#endif

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

//------------------------------------------------------------------

// === Общие настройки ===
static constexpr float REPORT_INTERVAL_MS = 17.0f;							//Задержка в мс (16.7 ~60 Гц)
static constexpr float VOLTAGE_ACCURACY_LEVEL = 0.07f;						// Порог погрешности измерений
static constexpr float VOLTAGE_WARNING_LEVEL = 3.3f;						// При этом напряжении моргает LED для предупреждения о низком заряде
static constexpr float VOLTAGE_CRITICAL_LEVEL = 3.1f;						// При этом напряжении быстро моргает LED и геймпад отключается
static constexpr uint32_t VOLTAGE_CHECK_MS = 10ul * 1000;					// Проверка напряжения каждые 10 секунд
static constexpr uint32_t INACTIVITY_CONNECTED_MS = 5ul * 1000 * 60;		// Автоотключение при бездействии 5 мин
static constexpr uint32_t INACTIVITY_DISCONNECTED_MS = 2ul * 1000 * 60;		// Автоотключение при отсутствии подключения 2 мин
static constexpr uint32_t COMBINATION_HOLD_MS = 3ul * 1000;					// Активация комбинации кнопок удержанием 3 секунды

//------------------------------------------------------------------

// Конфигурация пинов геймпада

// Сигнал о состоянии сопряжения: HIGH - подключено, LOW - нет сопряжения/состояние поиска
// Приходит непосредственно с выхода светодиода прошитого HC-05 / RN-42 о статусе существующего подключения (PIO2 - 25 pin)
// Стоит учитывать что A6 и A7 на ATMEGA328(P/PA) используются только как аналоговые входы
static constexpr uint8_t BT_STAT_PIN = A6;

static constexpr uint8_t BT_ONOFF_PIN = 10;	// Включение-выключение модуля BT: HIGH - включено, LOW - выключено

static constexpr uint8_t PWRLED_PIN = 13;	// Сигнал о низком заряде АКБ

static constexpr uint8_t TX_PIN = 0;		// Присоединить к RXD BT-модуля (логический уровень 3.3 Вольта!)
static constexpr uint8_t RX_PIN = 1;		// Присоединить к TXD BT-модуля

//------------------------------------------------------------------

static constexpr uint8_t BUTTONS_VARIANTS = static_cast<uint8_t>(RN42_HID_gamepad::Button::ButtonCount);

inline uint8_t getButtonPin(RN42_HID_gamepad::Button btn)
{
	switch(btn)
	{
		case RN42_HID_gamepad::Button::START:	return 2;
		
#if defined(SEGA_JOY)
		case RN42_HID_gamepad::Button::A:		return 5;
		case RN42_HID_gamepad::Button::B:		return 4;
		case RN42_HID_gamepad::Button::C:		return 3;
		
		case RN42_HID_gamepad::Button::X:		return A1;
		case RN42_HID_gamepad::Button::Y:		return A2;
		case RN42_HID_gamepad::Button::Z:		return A3;
#elif defined(NES_JOY) || defined(SNES_JOY)
		case RN42_HID_gamepad::Button::SELECT:	return 3;
		
		case RN42_HID_gamepad::Button::A:		return 4;
		case RN42_HID_gamepad::Button::B:		return 5;
		
		case RN42_HID_gamepad::Button::X:		return A3;
		case RN42_HID_gamepad::Button::Y:		return A2;
#endif
		
		case RN42_HID_gamepad::Button::UP:		return 6;
		case RN42_HID_gamepad::Button::RIGHT:	return 7;
		case RN42_HID_gamepad::Button::DOWN:	return 8;
		case RN42_HID_gamepad::Button::LEFT:	return 9;
		
		case RN42_HID_gamepad::Button::L1:		return 11;
		case RN42_HID_gamepad::Button::R1:		return 12;
		
		default: return UINT8_MAX; // Новая/неподключённая кнопка
	}
}