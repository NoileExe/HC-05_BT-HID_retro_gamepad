
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

//------------------------------------------------------------------

namespace RN42
{
	// === Управление режимом команд ===
	constexpr char CMD_ENTER[]		= "$$$";		// Без терминатора!
	constexpr char CMD_EXIT[]		= "---\r";
	constexpr char RESP_CMD[]		= "CMD\r\n";
	constexpr char RESP_END[]		= "END\r\n";
	constexpr char RESP_AOK[]		= "AOK\r\n";
	constexpr char RESP_ERROR[]		= "ERR\r\n";
	constexpr char RESP_KILL[]		= "KILL\r\n";
	constexpr char RESP_WAKE[]		= "Wake\r\n";
	constexpr char RESP_CONN_ON[]	= "1,0,0\r\n";		// GK: подключено
	constexpr char RESP_CONN_OFF[]	= "0,0,0\r\n";		// GK: отключено
	constexpr char RESP_REBOOT[]	= "Reboot!\r\n";

	// === SET-команды (настройка) ===
	constexpr char SET_FACTORY[]	= "SF,1\r\n";		// Сброс к заводским
	constexpr char SET_PROFILE[]	= "S~,%d\r\n";		// HID-профиль
	constexpr char SET_HIDFLAGS[]	= "SH,%04X\r\n";	// Формат: SH,0210 (gamepad)
	constexpr char SET_BAUD[]		= "SU,%d\r\n";		// SU,96 = 9600 бод
	constexpr char SET_POWER[]		= "SY,%04X\r\n";	// Мощность: 0010 = 0dBm (default), 0000 = -12 dBm
	constexpr char SET_SNIFF[]		= "SW,%04X\r\n";	// SW,8000 = включить поддержку deep sleep
	constexpr char SET_NAME[]		= "SN,%s\r\n";		// SN,MyGamepad
	constexpr char SET_MODE[]		= "SM,%d\r\n";		// SM,4 = DTR (auto-connect)
	constexpr char SET_PIN[]		= "SP,%s\r\n";		// SP,1234
	constexpr char SET_QUIET[]		= "Q\r\n";			// Отключить discoverable/connectable

	// === GET-команды (чтение настроек) ===
	constexpr char GET_HID[]		= "GH\r\n";			// Возврат: "OK,<flags>"
	constexpr char GET_STATUS[]		= "GK\r\n";			// Возврат: "CONNECT" / "DISCONNECT"
	constexpr char GET_NAME[]		= "GN\r\n";			// Возврат: "FireFly-xxxx" или заданное имя
	constexpr char GET_VERSION[]	= "V\r\n";			// Возврат: версия прошивки
	constexpr char GET_BAUD[]		= "GU\r\n";			// Возврат: "<baud_code>"
	constexpr char GET_ADDR[]		= "GB\r\n";			// Возврат: MAC-адрес модуля

	// === Action-команды ===
	constexpr char ACT_REBOOT[]		= "R,1\r\n";		// Перезагрузка
	constexpr char ACT_DISCONNECT[]	= "K,\r\n";			// Разрыв соединения
	constexpr char ACT_SLEEP[]		= "Z\r";			// Глубокий сон (без \n!)
	constexpr char ACT_CONNECT[]	= "C\r\n";			// Подключение к сохранённому адресу
	constexpr char ACT_NEWCONNECT[]	= "W\r\n";			// Возобновите поиск и подключение (после выполнения Q)
	constexpr char ACT_FASTDATA[]	= "F,1\r\n";		// Быстрый выход из командного режима и переход в FastData

	// Коды режимов (для подстановки в форматы)
	enum class Mode : uint8_t
	{
		Slave = 0,
		Master = 1,
		Trigger = 2,
		Auto = 3,
		DTR = 4,
		Any = 5
	};

	enum class Profile : uint8_t
	{
		SPP = 0,
		DUN_DCE = 1,
		DUN_DTE = 2,
		MDM_SPP = 3,
		DUN_SPP = 4,
		APL = 5,
		HID = 6
	};

	// HID-флаги (битовая маска для SH)
	enum class HidFlags : uint16_t
	{
		GAMEPAD = 0x0210,	// Биты 7-4 = 0010 (геймпад), биты 2-0 = 000
		JOYSTICK = 0x0240,	// Биты 7-4 = 0010, биты 2-0 = 001
		KEYBOARD = 0x0100,	// Биты 7-4 = 0001
		MOUSE = 0x0080		// Биты 7-4 = 0000, бит 7 = 1
	};

	// Коды скорости для SU
	enum class BaudRate : uint16_t
	{
		BR_1200		= 12,
		BR_2400		= 24,
		BR_4800		= 48,
		BR_9600		= 96,
		BR_19200	= 19,
		BR_38400	= 38,
		BR_57600	= 57,
		BR_115200	= 11		// Заводское значение
	};

	// Новая схема (после 08.2012): значение = dBm в hex
	enum class PowerNew : uint16_t
	{
		PW_16dBm = 0x0010,		// По-умолчанию, максимальное значение
		PW_12dBm = 0x000C,
		PW_8dBm  = 0x0008,
		PW_4dBm  = 0x0004,
		PW_0dBm  = 0x0000,
		PW_neg4dBm = 0xFFFC
	};
	
	// Старая схема (до 08.2012): произвольные коды
	enum class PowerOld : uint16_t
	{
		PW_12dBm = 0x0004,		// Максимум для старых
		PW_6dBm  = 0x0000,
		PW_2dBm  = 0xFFFC,
		PW_0dBm  = 0xFFF8,
		PW_neg5dBm = 0xFFF4,
		PW_neg10dBm = 0xFFF0,
		PW_neg20dBm = 0xFFE8
	};

	// Режим энергосбережения
	enum class Sniff : uint16_t {
		INTERVAL_20MS		= 0x0020,
		INTERVAL_50MS		= 0x0050,
		INTERVAL_100MS		= 0x00A0,
		INTERVAL_250MS		= 0x0190,
		INTERVAL_500MS		= 0x0320,
		INTERVAL_1S			= 0x0640,
		DEEP_SLEEP_ONLY		= 0x8000,	// Включение DeepSleep (SniffMode полностью отключен)
		DEEP_SLEEP_500MS	= 0x8320 
	};
	
	inline Sniff operator|(Sniff a, Sniff b)
	{
		return static_cast<Sniff>(static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
	}
	
	inline Sniff operator&(Sniff a, Sniff b)
	{
		return static_cast<Sniff>(static_cast<uint16_t>(a) & static_cast<uint16_t>(b));
	}

} // namespace RN42