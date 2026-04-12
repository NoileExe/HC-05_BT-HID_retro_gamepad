
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

#ifdef __LGT8FX8P__
	#include <PMU.h>
	#include <WDT.h>
#else
	#include <avr/sleep.h>
	#include <avr/power.h>
	#include <avr/wdt.h>
	#include <iarduino_VCC.h>
#endif

#include "gamepadPowerManager.h"
#include "gamepadConfig.h"

//------------------------------------------------------------------

// Инициализация статических членов
void (*PowerManager::preSleepCallback)() = nullptr;
void (*PowerManager::wakeupCallback)() = nullptr;

// Статическая функция-прослойка
void PowerManager::isrStub()
{
	if (wakeupCallback)
		wakeupCallback();
}

void PowerManager::setPreSleepCallback(void (*callback)())
{
	preSleepCallback = callback;
}

void PowerManager::setWakeupCallback(void (*callback)())
{
	wakeupCallback = callback;
}

//------------------------------------------------------------------

PowerManager::PowerManager(uint8_t wakeUpPin)
	: currentState(false)
	, lastConnectedState(0)
	, currentVoltage(5.0)
	, lastVoltageCheck(0)
	, powerLed(PWRLED_PIN)
	, interrupt_pin(wakeUpPin)
{
}

void PowerManager::begin()
{
#ifdef __LGT8FX8P__
	analogReference(INTERNAL1V024);
#else
	analogReference(DEFAULT);
#endif

	pinMode(BT_ONOFF_PIN, OUTPUT);
	digitalWrite(BT_ONOFF_PIN, HIGH);	//Включаем преобразователь напряжения на модуле BT


	pinMode(BT_STAT_PIN, INPUT);

#if defined(ADC6D) && defined(ADC7D)
	if (BT_STAT_PIN == A6 || BT_STAT_PIN == A7)
	{
		// Отключить цифровой буфер (нет утечки - LED не светится во сне)
		DIDR0 |= _BV(ADC6D);
		DIDR0 |= _BV(ADC7D);
	}
#endif


	// Отключаем OC0A после инициализации в ПОПЫТКЕ ПОЧИНИТЬ проблему D6
	// К сожалению, плат на ATMEGA328P/PA у меня для проверки не осталось
	TCCR0A &= ~(_BV(COM0A0) | _BV(COM0A1));

	//Слежение за зависанием самой ардуинки. Таймаут установлен максимальный ~8с
#ifdef __LGT8FX8P__
	ACSR = _BV(C0D);	//Отключаем компаратор в ПОПЫТКЕ ПОЧИНИТЬ проблему D7 и для снижения потребления
	wdt_enable(WTO_8S);
#else
	ACSR = _BV(ACD);	//Отключаем компаратор в ПОПЫТКЕ ПОЧИНИТЬ проблему D7 и для снижения потребления
	wdt_enable(WDTO_8S);
#endif

	Serial.begin(9600);

	lastConnectedState = millis();
	lastVoltageCheck = millis() - VOLTAGE_CHECK_MS;
}




void PowerManager::watchdog_reset()
{
	wdt_reset();
}

void PowerManager::watchdog_enable(bool enabled /*= true*/)
{
	if (enabled)
	{
#ifdef __LGT8FX8P__
		wdt_enable(WTO_8S);
#else
		wdt_enable(WDTO_8S);
#endif
	}
	else
	{
		wdt_disable();
	}
}


void PowerManager::tick()
{
	readConnectionState();
	readVoltage();
	
	checkConnection();
	checkBattery();
	
	wdt_reset();
}

//------------------------------------------------------------------

bool PowerManager::readConnectionState()
{
	currentState = analogRead(BT_STAT_PIN) > 500;	//Примерно половина от максимального значения 1023
	return currentState;
}

void PowerManager::checkConnection()
{
	uint32_t now = millis();

	if (currentState)
		lastConnectedState = now;
	else if (INACTIVITY_DISCONNECTED_MS < now - lastConnectedState)
	{
		sleep();
		wake();
	}
}

//------------------------------------------------------------------

float PowerManager::readVoltage()
{
#ifdef __LGT8FX8P__
	currentVoltage = (analogRead(VCCM) * 1.024 * 5.0) / 1023.00;
#else
	currentVoltage = analogRead_VCC();
#endif

	//Отладка измерения напряжения
	//Serial.println(currentVoltage, 3);
	//Serial.print("*****\n");

	return currentVoltage;
}

void PowerManager::checkBattery()
{
	uint32_t now = millis();
	if (now - lastVoltageCheck >= VOLTAGE_CHECK_MS)
	{
		lastVoltageCheck = now;
		
		// Критический разряд: быстро мигаем и уходим в глубокий сон до зарядки
		// Не даем пользоваться устройством пока батарея не будет заряжена
		while (currentVoltage <= VOLTAGE_CRITICAL_LEVEL)
		{
			digitalWrite(BT_ONOFF_PIN, LOW);	//Выключаем преобразователь напряжения на модуле BT
			Serial.end();
			pinMode(RX_PIN, INPUT);				// RX в High-Z (высокий импеданс)
			digitalWrite(TX_PIN, LOW);			// Принудительно 0 на TX
			pinMode(TX_PIN, OUTPUT);			// TX как выход 0Вольт
			
			indicateCritical();
			sleep();
			wake();
			
			// Читаем при пробуждении
			readVoltage();
		}
		
		
		if (currentVoltage < (VOLTAGE_WARNING_LEVEL - VOLTAGE_ACCURACY_LEVEL))
			indicateWarning();
		
		// Напряжение в норме – гасим светодиод
		else if (powerLed.running())
			powerLed.stop();
	}
	
	// Даже если не проверяем напряжение, поддерживаем мигание светодиода
	else if (powerLed.running())
		powerLed.tick();
}

void PowerManager::indicateWarning()
{
	// Предупреждение: медленное мигание (2000 мс вкл / 500 мс выкл)
	if (!powerLed.running())
		powerLed.blinkForever(2000, 500);
	
	powerLed.tick();
}

void PowerManager::indicateCritical()
{
	powerLed.stop();
	
	for (int i = 0; i < 3; ++i)
	{
		powerLed.blink(4, 300, 300);	// мигнуть 4 раза, 300мс вкл, 300мс выкл
		
		while (!powerLed.ready())
		{
			powerLed.tick();
			wdt_reset();
			delay(50);
		}
		
		delay(1000);
	}
}

//------------------------------------------------------------------

void PowerManager::sleep()
{
	// Вызываем пользовательскую подготовку перед сном
	if (preSleepCallback)
	{
		preSleepCallback();
	}

	digitalWrite(BT_ONOFF_PIN, LOW); //Выключаем преобразователь напряжения на модуле BT
	Serial.end();
	pinMode(RX_PIN, INPUT);			// RX в High-Z (высокий импеданс)
	digitalWrite(TX_PIN, LOW);		// Принудительно 0 на TX
	pinMode(TX_PIN, OUTPUT);		// TX как выход 0Вольт

	digitalWrite(PWRLED_PIN, LOW);

	wdt_disable();

#ifdef __LGT8FX8P__
	// АЦП + сброс флага
	ADCSRA &= ~_BV(ADEN);
	ADCSRA |= _BV(ADIF);

	ADCSRD = 0x00;	// Полное отключение аналога

	// LVD (Low Voltage Detector)
	VDTCR |= _BV(WCE);
	VDTCR &= ~_BV(VDTEN);

	PRR = _BV(PRADC) /*| _BV(PRTIM0)*/ | _BV(PRTIM1) | _BV(PRTIM2) | _BV(PRUSART0) | _BV(PRSPI) | _BV(PRTWI);
	// Бит 1 (PRPCI) НЕ трогаем — нужно для пробуждения по кнопке!
	//     PRTIM3    PRWDT    PREFL
	PRR1 = _BV(3) | _BV(5) | _BV(2);

	DIDR0 = 0xFF;	// Цифровые входы на аналоговых пинах на время сна отключены
	DIDR1 = 0xFF;

	EIFR = 0xFF;
	attachInterrupt(interrupt_pin, isrStub, LOW);
	PMU.sleep(PM_POFFS1, SLEEP_FOREVER);	// PM_POFFS1 - переводим МК в спящий режим
											// (DPS2 экономичнее, но его нет в enum pmu_t и при пробуждении он перезагрузит МК,
											// т.е. при запуске В setup() придется выполнять всё то что происходит в wake()
#else
	ADMUX = 0;
	ADCSRA &= ~_BV(ADEN);	// Отключаем АЦП

	PRR_NAME = _BV(PRADC) /*| _BV(PRTIM0)*/ | _BV(PRTIM1) | _BV(PRTIM2) | _BV(PRUSART_BIT_NAME) | _BV(PRSPI_BIT_NAME) | _BV(PRTWI_BIT_NAME);
	#ifdef PRR1
		//     PRTIM3    PRDAC   PRSPI1   PRTWI1
		PRR1 = _BV(0) | _BV(3) | _BV(1) | _BV(2);
	#endif

	DIDR0 = 0xFF;							// Цифровые входы на аналоговых пинах на время сна отключены
	DIDR1 = 0xFF;

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);	//Устанавливаем интересующий нас режим

	cli();									// Временно запрещаем обработку прерываний
	sleep_enable();
	
	//Очистка флагов прерываний
	if (interrupt_pin == 2)			EIFR = _BV(INTF0);
	else if (interrupt_pin == 3)	EIFR = _BV(INTF1);

	attachInterrupt(interrupt_pin, isrStub, LOW);	//Выход из режима сна по нажатию кнопки на 2ом пине (START)
	sei();											// Разрешаем обработку прерываний
	
	sleep_bod_disable();							// Отключаем детектор пониженного напряжения питания
	sleep_cpu();									// Переводим МК в спящий режим
#endif
}

void PowerManager::wake()
{
#ifdef __LGT8FX8P__
	PRR = 0;
	PRR1 = 0;
	DIDR0 = 0;
	DIDR1 = 0;
	ADCSRA |= _BV(ADEN);	// Если нужен АЦП
	ADCSRD |= _BV(ADEN);	// Разрешаем работу АЦП (бит 0)

	VDTCR |= _BV(WCE);
	VDTCR |= _BV(VDTEN);	// Включаем детектор

	analogReference(INTERNAL1V024);
	
	wdt_enable(WTO_8S);
#else
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
	
	wdt_enable(WDTO_8S);
#endif

	detachInterrupt(interrupt_pin);

	digitalWrite(BT_ONOFF_PIN, HIGH);	//Включаем преобразователь напряжения на модуле BT
	Serial.begin(9600);
	delay(500);

	currentState = false;
	lastConnectedState = millis();

#if defined(ADC6D) && defined(ADC7D)
	if (BT_STAT_PIN == A6 || BT_STAT_PIN == A7)
	{
		// Отключить цифровой буфер (нет утечки - LED не светится во сне)
		DIDR0 |= _BV(ADC6D);
		DIDR0 |= _BV(ADC7D);
	}
#endif

	// Ожидаем пока кнопку отпустят
	while (digitalRead(interrupt_pin) == LOW)
	{
		delay(10);
	}
}
