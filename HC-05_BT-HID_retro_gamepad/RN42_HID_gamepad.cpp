
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

#include "RN42_HID_gamepad.h"
#include "RN42_CMD.h"

//------------------------------------------------------------------

namespace
{
	// Кнопки первого байта
	enum Btn1 : uint8_t
	{
		A =		1<<0,
		B =		1<<1,
		C =		1<<2,
		X =		1<<3,
		Y =		1<<4,
		Z =		1<<5,
		L1 =	1<<6,
		R1 =	1<<7
	};

	// Кнопки второго байта
	enum Btn2 : uint8_t
	{
		L2 =		1<<0,
		R2 =		1<<1,
		SELECT =	1<<2,
		START =		1<<3,
		MODE =		1<<4,
		THUMBL =	1<<5,
		THUMBR =	1<<6,
		HOME =		1<<7
	};

	// D-Pad
	enum DPad : uint8_t
	{
		UP =	1<<0,
		RIGHT =	1<<1,
		DOWN =	1<<2,
		LEFT =	1<<3
	};
}

//------------------------------------------------------------------

RN42_HID_gamepad::RN42_HID_gamepad()
	: currMode(RN42_HID_gamepad::Mode::Standard)
	, currBtns()
	, prevBtns()
	, turboBtns()
	, powerOffCombo()
	, changeModeCombo()
{
}

bool RN42_HID_gamepad::begin(Stream& uart_stream)
{
	uart = &uart_stream;
	delay(500);		//Обязательная минимальная задержка после включения перед отправкой команд
	return true;
}

//------------------------------------------------------------------

// Установка состояний кнопок и осей
void RN42_HID_gamepad::setAxisState(int8_t x1, int8_t y1, int8_t x2, int8_t y2)
{
	currBtns.x1 = x1;
	currBtns.y1 = y1;
	currBtns.x2 = x2;
	currBtns.y2 = y2;
}

void RN42_HID_gamepad::setButtonState(RN42_HID_gamepad::Button btn, bool isPressed)
{
	setButtonInReport(currBtns, btn, isPressed);
}


// Отправка кнопок
void RN42_HID_gamepad::sendButtons()
{
	if (!uart)
		return;

	// =========================== Маски кнопок, на которые действует Turbo ===========================
	// В режиме Turbo — все кнопки действий, иначе — только помеченные через setTurboButton()
	uint8_t turboMask1 = (currMode == Mode::Turbo) ? 0xFF : turboBtns.btn1;
	uint8_t turboMask2 = (currMode == Mode::Turbo) 
							? Btn2::L2 | Btn2::R2 | Btn2::THUMBL | Btn2::THUMBR
							: turboBtns.btn2;

	const bool hasTurboButtons = (turboMask1 || turboMask2);	// Есть ли Turbo-кнопки
	// =========================== Маски кнопок, на которые действует Turbo ===========================
	// ================================================================================================
	// =============================== Состояние кнопок для модификации ===============================
	uint8_t tmp_btn1 = currBtns.btn1;
	uint8_t tmp_btn2 = currBtns.btn2;
	int8_t tmp_x1 = 0, tmp_y1 = 0;
	// =============================== Состояние кнопок для модификации ===============================
	// ================================================================================================
	// =========================== Разрешение конфликта D-Pad / Левый стик ============================
	// В HID-репорте RN-42 нет отдельного байта D-Pad.
	// Проводим векторное сложение в отдельные переменные, чтобы не портить currBtns для сравнений
	if (currBtns.dpad & DPad::UP)		tmp_y1 -= 127;
	if (currBtns.dpad & DPad::RIGHT)	tmp_x1 += 127;
	if (currBtns.dpad & DPad::DOWN)		tmp_y1 += 127;
	if (currBtns.dpad & DPad::LEFT)		tmp_x1 -= 127;

	// Суммируем и ограничиваем диапазон, чтобы не было переполнения int8
	tmp_x1 = constrain((int16_t)currBtns.x1 + tmp_x1, -127, 127);
	tmp_y1 = constrain((int16_t)currBtns.y1 + tmp_y1, -127, 127);
	// =========================== Разрешение конфликта D-Pad / Левый стик ============================
	// ================================================================================================
	// ================================= Смена состояний Turbo-кнопок =================================
	static bool prevTurboPhase = false;
	static uint8_t prevSlowStartState = 0;

	const bool currTurboPhase = (millis() / 33ul) & 1;				// Тайминг пульсации турбокнопок
	const bool turboPhaseChanged = (currTurboPhase != prevTurboPhase);
	
	prevTurboPhase = currTurboPhase;
	
	// Применяем маску в фазе "отпускания"
	if (currTurboPhase && hasTurboButtons)
	{
		tmp_btn1 &= ~turboMask1;
		tmp_btn2 &= ~turboMask2;
	}
	// ================================= Смена состояний Turbo-кнопок =================================
	// ================================================================================================

	bool shouldSend = (currBtns != prevBtns);
	
	switch (currMode)
	{
		case Mode::Standard:
			if (hasTurboButtons)
			{
				bool turboPressed = (currBtns.btn1 & turboMask1) || (currBtns.btn2 & turboMask2);
				if (turboPressed && turboPhaseChanged)
					shouldSend = true;
			}
			break;
		
		case Mode::Turbo:
			// В режиме Turbo: отправка только при смене фазы
			shouldSend |= turboPhaseChanged;
			break;
		
		case Mode::Slow:
			const bool slowPhase = ((millis() / 100ul) & 1);
			const uint8_t currStartState = slowPhase ? Btn2::START : 0;
			
			if (currStartState != prevSlowStartState)
			{
				tmp_btn2 = (tmp_btn2 & ~Btn2::START) | currStartState;
				shouldSend = true;
				prevSlowStartState = currStartState;
			}
			break;
	}


	if (shouldSend)
	{
		uint8_t buf[8] = {
			0xFD,					//Начало HID-отчета
			0x06,					//Размер отчета в байтах
			(uint8_t)tmp_x1,		//Координаты X первого стика (левого)
			(uint8_t)tmp_y1,		//Координаты Y первого стика (левого)
			(uint8_t)currBtns.x2,	//Координаты X второго стика (правого)
			(uint8_t)currBtns.y2,	//Координаты Y второго стика (правого)
			tmp_btn1,				//Первый байт с состоянием кнопок
			tmp_btn2				//Второй байт с состоянием кнопок
		};
		
		uart->write(buf, sizeof(buf));
		
		prevBtns = currBtns;
	}
}

void RN42_HID_gamepad::sendAllEmptyButtons()
{
	if (!uart)
		return;

	RN42_HID_gamepad::ButtonsCombination tmpBtns;
	
	uint8_t buf[8] = {
		0xFD,					//Начало HID-отчета
		0x06,					//Размер отчета в байтах
		(uint8_t)tmpBtns.x1,	//Координаты X первого стика (левого)
		(uint8_t)tmpBtns.y1,	//Координаты Y первого стика (левого)
		(uint8_t)tmpBtns.x2,	//Координаты X второго стика (правого)
		(uint8_t)tmpBtns.y2,	//Координаты Y второго стика (правого)
		tmpBtns.btn1,			//Первый байт с состоянием кнопок
		tmpBtns.btn2			//Второй байт с состоянием кнопок
	};
	
	uart->write(buf, sizeof(buf));
}

void RN42_HID_gamepad::changeMode(Mode mode)
{
	if (currMode != mode)
		currMode = mode;
}

bool RN42_HID_gamepad::isButtonPressed(RN42_HID_gamepad::Button btn) const
{
	return isButtonInReport(currBtns, btn);
}

//------------------------------------------------------------------

void RN42_HID_gamepad::setTurboButton(Button btn, bool isTurbo)
{
	switch (btn)
	{
		case RN42_HID_gamepad::Button::A:
		case RN42_HID_gamepad::Button::B:
		case RN42_HID_gamepad::Button::C:
		case RN42_HID_gamepad::Button::X:
		case RN42_HID_gamepad::Button::Y:
		case RN42_HID_gamepad::Button::Z:
		case RN42_HID_gamepad::Button::L1:
		case RN42_HID_gamepad::Button::R1:
		case RN42_HID_gamepad::Button::L2:
		case RN42_HID_gamepad::Button::R2:
		case RN42_HID_gamepad::Button::THUMBL:
		case RN42_HID_gamepad::Button::THUMBR:
			setButtonInReport(turboBtns, btn, isTurbo);
			break;
	}
}

bool RN42_HID_gamepad::isTurboButton(Button btn) const
{
	return isButtonInReport(turboBtns, btn);
}

//------------------------------------------------------------------

RN42_HID_gamepad::ButtonsCombination RN42_HID_gamepad::createCombo(const Button* combo, uint8_t count) const
{
	RN42_HID_gamepad::ButtonsCombination testCombo;
	
	if (!combo || !count)
		return testCombo;
	
	for (uint8_t i = 0; i < count; i++)
		setButtonInReport(testCombo, combo[i], true);
	
	return testCombo;
}

// Установка
void RN42_HID_gamepad::setPowerOffCombination(const Button* combo, uint8_t count)
{
	auto testCombo = createCombo(combo, count);
	if (testCombo.isEmpty() || testCombo == changeModeCombo)
		return;

	powerOffCombo = testCombo;
}

void RN42_HID_gamepad::setChangeModeCombination(const Button* combo, uint8_t count)
{
	auto testCombo = createCombo(combo, count);
	if (testCombo.isEmpty() || testCombo == powerOffCombo)
		return;

	changeModeCombo = testCombo;
}

//------------------------------------------------------------------

// Проверки
bool RN42_HID_gamepad::isPowerOffCombination(const Button* combo, uint8_t count) const
{
	auto testCombo = createCombo(combo, count);
	return !testCombo.isEmpty() && testCombo == powerOffCombo;
}

bool RN42_HID_gamepad::isPowerOffCombination() const
{
	return powerOffCombo == currBtns  &&  powerOffCombo != ButtonsCombination();
}

bool RN42_HID_gamepad::isChangeModeCombination(const Button* combo, uint8_t count) const
{
	auto testCombo = createCombo(combo, count);
	return !testCombo.isEmpty() && testCombo == changeModeCombo;
}

bool RN42_HID_gamepad::isChangeModeCombination() const
{
	return changeModeCombo == currBtns  &&  changeModeCombo != ButtonsCombination();
}

//------------------------------------------------------------------

// Хелпер
void RN42_HID_gamepad::setButtonInReport(ButtonsCombination& bc, Button btn, bool state)
{
	uint8_t* byte = nullptr;
	uint8_t mask = 0;

	switch (btn) {
		// Первый байт кнопок
		case Button::A:			byte = &bc.btn1;	mask = Btn1::A;			break;
		case Button::B:			byte = &bc.btn1;	mask = Btn1::B;			break;
		case Button::C:			byte = &bc.btn1;	mask = Btn1::C;			break;
		case Button::X:			byte = &bc.btn1;	mask = Btn1::X;			break;
		case Button::Y:			byte = &bc.btn1;	mask = Btn1::Y;			break;
		case Button::Z:			byte = &bc.btn1;	mask = Btn1::Z;			break;
		case Button::L1:		byte = &bc.btn1;	mask = Btn1::L1;		break;
		case Button::R1:		byte = &bc.btn1;	mask = Btn1::R1;		break;
		// Второй байт кнопок
		case Button::L2:		byte = &bc.btn2;	mask = Btn2::L2;		break;
		case Button::R2:		byte = &bc.btn2;	mask = Btn2::R2;		break;
		case Button::SELECT:	byte = &bc.btn2;	mask = Btn2::SELECT;	break;
		case Button::START:		byte = &bc.btn2;	mask = Btn2::START;		break;
		case Button::MODE:		byte = &bc.btn2;	mask = Btn2::MODE;		break;
		case Button::THUMBL:	byte = &bc.btn2;	mask = Btn2::THUMBL;	break;
		case Button::THUMBR:	byte = &bc.btn2;	mask = Btn2::THUMBR;	break;
		case Button::HOME:		byte = &bc.btn2;	mask = Btn2::HOME;		break;
		// D-Pad
		case Button::UP:		byte = &bc.dpad;	mask = DPad::UP;		break;
		case Button::RIGHT:		byte = &bc.dpad;	mask = DPad::RIGHT;		break;
		case Button::DOWN:		byte = &bc.dpad;	mask = DPad::DOWN;		break;
		case Button::LEFT:		byte = &bc.dpad;	mask = DPad::LEFT;		break;
		
		default: return;
	}

	if (state)	*byte |= mask;
	else		*byte &= ~mask;
}

bool RN42_HID_gamepad::isButtonInReport(const ButtonsCombination& bc, Button btn) const
{
	uint8_t byte = 0;
	uint8_t mask = 0;

	switch (btn) {
		// Первый байт кнопок
		case Button::A:			byte = bc.btn1;		mask = Btn1::A;			break;
		case Button::B:			byte = bc.btn1;		mask = Btn1::B;			break;
		case Button::C:			byte = bc.btn1;		mask = Btn1::C;			break;
		case Button::X:			byte = bc.btn1;		mask = Btn1::X;			break;
		case Button::Y:			byte = bc.btn1;		mask = Btn1::Y;			break;
		case Button::Z:			byte = bc.btn1;		mask = Btn1::Z;			break;
		case Button::L1:		byte = bc.btn1;		mask = Btn1::L1;		break;
		case Button::R1:		byte = bc.btn1;		mask = Btn1::R1;		break;
		// Второй байт кнопок
		case Button::L2:		byte = bc.btn2;		mask = Btn2::L2;		break;
		case Button::R2:		byte = bc.btn2;		mask = Btn2::R2;		break;
		case Button::SELECT:	byte = bc.btn2;		mask = Btn2::SELECT;	break;
		case Button::START:		byte = bc.btn2;		mask = Btn2::START;		break;
		case Button::MODE:		byte = bc.btn2;		mask = Btn2::MODE;		break;
		case Button::THUMBL:	byte = bc.btn2;		mask = Btn2::THUMBL;	break;
		case Button::THUMBR:	byte = bc.btn2;		mask = Btn2::THUMBR;	break;
		case Button::HOME:		byte = bc.btn2;		mask = Btn2::HOME;		break;
		// D-Pad
		case Button::UP:		byte = bc.dpad;		mask = DPad::UP;		break;
		case Button::RIGHT:		byte = bc.dpad;		mask = DPad::RIGHT;		break;
		case Button::DOWN:		byte = bc.dpad;		mask = DPad::DOWN;		break;
		case Button::LEFT:		byte = bc.dpad;		mask = DPad::LEFT;		break;
		
		default: return false;
	}

	return (byte & mask);
}

//------------------------------------------------------------------

void RN42_HID_gamepad::powerOff()
{
	if (!uart)
		return;

	disconnect();
	delay(500);

	if (!enterCommandMode())
		return;

	while (uart->available())		uart->read();
	uart->print(RN42::ACT_SLEEP);
	uart->flush();
	
	// Модуль уходит в сон сразу
	// НЕ вызываем exitCommandMode() - т.к. может прервать сон
	
	delay(500);
}

bool RN42_HID_gamepad::wakeUp(unsigned long timeout_ms/* = 500*/)
{
	if (!uart)
		return false;

	// Пробуждающий сигнал: любой байт в UART
	uart->write((uint8_t)0x00);
	uart->flush();
	delay(500);
	
	// Очистить буфер от "потерянного" байта и возможных артефактов
	while (uart->available())		uart->read();
	
	//const char* exp[] = { RN42::RESP_WAKE };
	//bool res = readResponse(exp, 1, timeout_ms) >= 0;
	delay(500);
	
	//return res;
	return true;
}

//------------------------------------------------------------------

void RN42_HID_gamepad::setDefaultSettings(const char* name /*= "RN42_HID_gamepad"*/
											, RN42::BaudRate br /*= RN42::BaudRate::BR_9600*/)
{
	if (!uart || !name || !enterCommandMode())
		return;

	auto sendAOK =
		[this](const char* cmd, unsigned long timeout = 500) -> bool
		{
			while (uart->available())		uart->read();
			uart->print(cmd);
			uart->flush();
			
			//const char* exp[] = { RN42::RESP_AOK, RN42::RESP_ERROR };
			//bool res = (readResponse(exp, 2, timeout) == 0);
			delay(500);
			
			//return res;
			return true;
		};

	// Установка заводских настроек
	if (!sendAOK(RN42::SET_FACTORY))
	{
		exitCommandMode(200);
		return;
	}
	delay(1000);

	// Установка профиля HID
	char buf[64];
	snprintf(buf, sizeof(buf), RN42::SET_PROFILE, static_cast<uint16_t>(RN42::Profile::HID));
	if (!sendAOK(buf))
	{
		exitCommandMode(200);
		return;
	}
	delay(1000);

	// Настройка типа устройства: геймпад
	snprintf(buf, sizeof(buf), RN42::SET_HIDFLAGS, static_cast<uint16_t>(RN42::HidFlags::GAMEPAD));
	if (!sendAOK(buf))
	{
		exitCommandMode(200);
		return;
	}

	// Deep Sleep без Sniff (устраняет задержки нажатий)
	snprintf(buf, sizeof(buf), RN42::SET_SNIFF, static_cast<uint16_t>(RN42::Sniff::DEEP_SLEEP_ONLY));
	if (!sendAOK(buf))
	{
		exitCommandMode(200);
		return;
	}

	// Имя устройства
	snprintf(buf, sizeof(buf), RN42::SET_NAME, name);
	if (!sendAOK(buf))
	{
		exitCommandMode(200);
		return;
	}

	// Смена скорости
	snprintf(buf, sizeof(buf), RN42::SET_BAUD, static_cast<uint16_t>(br));
	if (!sendAOK(buf))
	{
		exitCommandMode(200);
		return;
	}
	delay(500);

	// Перезагружаем для применения настроек
	while (uart->available())		uart->read();
	uart->print(RN42::ACT_REBOOT);
	uart->flush();
	
	//const char* exp[] = { RN42::RESP_REBOOT };
	//readResponse(exp, 1, 1000);
	delay(3000);

	currMode = RN42_HID_gamepad::Mode::Standard;
	powerOffCombo = {};
	changeModeCombo = {};
}

void RN42_HID_gamepad::setDeviceName(const char* name /*= "RN42_HID_gamepad"*/)
{
	if (!uart || !name || !enterCommandMode())
		return;

	char buf[64];
	snprintf(buf, sizeof(buf), RN42::SET_NAME, name);
	
	while (uart->available())		uart->read();
	uart->print(buf);
	uart->flush();

	//const char* expSet[] = { RN42::RESP_AOK, RN42::RESP_ERROR };
	//bool res = (readResponse(expSet, 2) == 0);
	delay(500);

	// Перезагружаем для применения настроек
	while (uart->available())		uart->read();
	uart->print(RN42::ACT_REBOOT);
	uart->flush();
	
	//const char* expReboot[] = { RN42::RESP_REBOOT };
	//readResponse(expReboot, 1, 1000);
	delay(3000);
}

void RN42_HID_gamepad::setBaudRate(RN42::BaudRate br /*= RN42::BaudRate::BR_9600*/)
{
	if (!uart || !enterCommandMode())
		return;

	char buf[64];
	snprintf(buf, sizeof(buf), RN42::SET_BAUD, static_cast<uint16_t>(br));
	
	while (uart->available())		uart->read();
	uart->print(buf);
	uart->flush();

	//const char* expSet[] = { RN42::RESP_AOK, RN42::RESP_ERROR };
	//bool res = (readResponse(expSet, 2) == 0);
	delay(500);


	// Перезагружаем для применения настроек
	while (uart->available())		uart->read();
	uart->print(RN42::ACT_REBOOT);
	uart->flush();
	
	//const char* expReboot[] = { RN42::RESP_REBOOT };
	//readResponse(expReboot, 1, 1000);
	delay(3000);
}

//------------------------------------------------------------------

void RN42_HID_gamepad::disconnect()
{
	sendAllEmptyButtons();

	// Отправка команды на завершение сеанса коннекта
	if (uart && enterCommandMode())
	{
		while (uart->available())		uart->read();
		uart->print(RN42::ACT_DISCONNECT);
		uart->flush();
		
		//const char* exp[] = { RN42::RESP_KILL, RN42::RESP_ERROR };
		//bool disconnected = (readResponse(exp, 2) == 0);
		delay(500);
		
		exitCommandMode(200);
	}

	delay(500);
}

//------------------------------------------------------------------

const char* RN42_HID_gamepad::getDeviceName()
{
	if (!uart || !enterCommandMode())
		return nullptr;

	while (uart->available())		uart->read();
	uart->print(RN42::GET_NAME);
	uart->flush();

	// Читаем ответ (имя устройства) до \r\n
	char buf[32];
	int len = readLine(buf, sizeof(buf), 500);
	delay(500);

	exitCommandMode(200);

	if (len <= 0)
		return nullptr; // Таймаут или пустая строка

	// Копируем в буфер класса (с защитой от переполнения)
	strncpy(nameBuf, buf, MAX_STR_LEN - 1);
	nameBuf[MAX_STR_LEN - 1] = '\0';

	return nameBuf;
}

// Установленно ли соединение
bool RN42_HID_gamepad::isConnected()
{
	if (!uart || !enterCommandMode())
		return false;

	while (uart->available())		uart->read();
	uart->print(RN42::GET_STATUS);
	uart->flush();

	// Читаем ответ (статус сопряжения) до \r\n
	char buf[32];
	int len = readLine(buf, sizeof(buf), 500);
	size_t cmpLen = strlen(RN42::RESP_CONN_ON) - 2;		// Длина строки минус "\r\n"
	
	bool connected = (len == cmpLen && strncmp(buf, RN42::RESP_CONN_ON, cmpLen) == 0);

	//const char* exp[] = {RN42::RESP_CONN_ON, RN42::RESP_CONN_OFF};
	//bool connected = (readResponse(exp, 2) == 0);
	delay(500);

	exitCommandMode(200);
	return connected;
}

//------------------------------------------------------------------

int RN42_HID_gamepad::readLine(char* out, size_t maxLen, unsigned long timeout_ms/* = 500*/)
{
	if (!uart || !out || maxLen == 0)
		return -1;

	size_t idx = 0;
	unsigned long start = millis();

	while (millis() - start < timeout_ms)
	{
		if (!uart->available()) 
			continue;
		
		char c = uart->read();
		if (c == '\r' || c == '\n')
		{
			out[idx] = '\0';
			return static_cast<int>(idx); // успех: возвращаем длину
		}
		
		if (idx < maxLen - 1)
			out[idx++] = c;
	}

	out[idx] = '\0';
	return -1; // таймаут
}

/*int RN42_HID_gamepad::readResponse(const char* const expected[], size_t count, unsigned long timeout_ms)
{
	if (!uart || count == 0)
		return -1;

	char buf[32];
	if (readLine(buf, sizeof(buf), timeout_ms) < 0)
		return -1;

	for (size_t i = 0; i < count; ++i)
	{
		// Отрезаем \r\n у констант перед сравнением
		size_t len = strlen(expected[i]);
		size_t cmpLen = (len >= 2 && expected[i][len-2] == '\r' && expected[i][len-1] == '\n') ?
						len - 2 : len;
		
		if (strlen(buf) == cmpLen && strncmp(buf, expected[i], cmpLen) == 0)
			return static_cast<int>(i);
	}

	return -1; // не совпало ни с одним
}*/

bool RN42_HID_gamepad::enterCommandMode(unsigned long timeout_ms/* = 500*/)
{
	if (!uart)		return false;

	while (uart->available())		uart->read();
	uart->print(RN42::CMD_ENTER);
	uart->flush();

	//const char* exp[] = {RN42::RESP_CMD};
	//bool res = readResponse(exp, 2, timeout_ms) >= 0;
	delay(500);

	//return res;
	return true;
}

bool RN42_HID_gamepad::exitCommandMode(unsigned long timeout_ms/* = 500*/)
{
	if (!uart)		return false;

	while (uart->available())		uart->read();
	uart->print(RN42::CMD_EXIT);
	uart->flush();

	//const char* exp[] = {RN42::RESP_END, RN42::RESP_AOK};
	//bool res = readResponse(exp, 2, timeout_ms) >= 0;
	delay(500);

	//return res;
	return true;
}
