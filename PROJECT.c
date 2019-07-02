#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <util/delay.h>
#include <math.h>
#include "lcd.h"
#include "twi.h"

#define CLI() cli()
#define SEI() sei()

#define EX_LCD_DATA        (*(volatile unsigned char *)0x8000) 
#define EX_LCD_CONTROL        (*(volatile unsigned char *)0x8001) 
#define EX_SS_DATA        (*(volatile unsigned char *)0x8002) 
#define EX_SS_SEL        (*(volatile unsigned char *)0x8003) 
#define EX_DM_SEL        (*(volatile unsigned int  *)0x8004) 
#define EX_DM_DATA        (*(volatile unsigned int  *)0x8006) 
#define EX_LED                (*(volatile unsigned char *)0x8008) 
#define EX_STEPPING        (*(volatile unsigned char *)0x8009) 
#define EX_DCMOTOR        (*(volatile unsigned char *)0x800A) 
#define EX_SERVO        (*(volatile unsigned char *)0x800B) 

int hexh[24] = { 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x20,0x21,0x22,0x23 };
int hexms[60] = { 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x30

,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59 };

int hexy[100] = { 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x30

,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69

,0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x90,0x91,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99 };

int hexm[12] = { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x10,0x11,0x12 };
int hexd[31] = { 0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x30,0x31 };
int hexdate[7] = { 0x00,0x01,0x02,0x03,0x04,0x05,0x06 };
int hex[7] = { 0x00,0x01,0x02,0x03 };

unsigned char segment_data[10] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x27, 0x7f, 0x6f }; // 0,1,2,3,4,5,6,7,8,9

float count = 0;
int i = 0;
unsigned char keydata;

char tmin = 0, tsec = 0, tmsec = 0;
int hour = 0x00, min = 0x00, sec = 0x00, record_cnt = 0;
char altmin = 0, altsec = 0, al = 0;
char alhour = 0, almin = 0, alsec = 0, al1 = 0;
char year = 0, mon = 0, date = 0, day = 0;
int cmin = 0, csec = 0, cmsec = 0;
char temp_tmin = 0, temp_tsec = 0, temp_tmsec = 0;

volatile unsigned char led = 0;
volatile long T1HIGHCNT, T1LOWCNT, TCNT;
unsigned int freq;

// idx ( 0 ~ 3 ) : ID
// idx ( 4 ~ 7 ) : PW

int User_Info[8];

void timer1_init(void)
{ 
	TCCR1B = 0x00; //stop
	TCNT1H = 0x00; //setup
	TCNT1L = 0x00;
	TCCR1A = 0x00;
	TCCR1B = 0x02; //Normal mode, prescale = 8 start Timer
}

void port_init(void)
{
	PORTA = 0x00;
	DDRA = 0xff; //주소출력
	PORTB = 0x00;
	DDRB = 0x00;
	PORTC = 0x00; //m103 output only
	DDRC = 0x03; //주소 출력
	PORTD = 0x00;
	DDRD = 0x00;
	PORTE = 0x00;
	DDRE = 0x00;
	PORTF = 0x00;
	DDRF = 0x00;
	PORTG = 0x00;
	DDRG = 0x03; // Write, ale 신호
}
void init_devices(void)
{ 
	//stop errant interrupts until set up
	CLI(); //disable all interrupts
	port_init();
	lcdInit();
	lcdClear();
	timer1_init();
	MCUCR = 0x80;
	TIMSK = 0x04; //timer1 interrupt sources
	SEI(); //re-enable interrupts
}

ISR(TIMER1_OVF_vect)
{ 
	TCNT1 = TCNT;
	PORTG = PORTG ^ 0x10;
}
void sound(int freq)
{
	//T1HIGHCNT = (0xFFFF-floor(1000000/freq))/0x100;
	//T1LOWCNT = 0xFF00 - (0xFFFF-floor(1000000/freq) );
	TCNT = (0xFFFF-floor(1000000/(freq/2)));
}

int main(void)
{
	init_devices();
	init_ds1307();
	
	static char k;
	int i = 0;
	unsigned char keydata;

	MCUCR = 0x80;
	PORTD = 0x00; // PORTD 초기값 설정
	DDRD = 0xff; // PORTD 모두 출력으로 설정
	EX_SS_SEL = 0x0f;

	lcdInit();
	_delay_ms(500);

	// ID : 0123 를 EEPROM에 저장
	eeprom_write_byte(0x0000, 0);
	eeprom_write_byte(0x0001, 1);
	eeprom_write_byte(0x0002, 2);
	eeprom_write_byte(0x0003, 3);

	// PW : 4567 를 EEPROM에 저장
	eeprom_write_byte(0x0004, 4);
	eeprom_write_byte(0x0005, 5);
	eeprom_write_byte(0x0006, 6);
	eeprom_write_byte(0x0007, 7);

	// 시간
	writebyte(0x00, 0x00);
	writebyte(0x01, 0x30);
	writebyte(0x02, 0x15);

	writebyte(0x03, 0x00);
	writebyte(0x04, 0x10);
	writebyte(0x05, 0x06);
	writebyte(0x06, 0x19);

	writebyte(0x08, 0x02);

	// Log_in Function
	while (1)
	{
		lcd_puts(1, "ID : ");
		lcd_puts(2, "PW : ");

		int test = 0;

		keydata = PINB & 0xFF;

		if (i == 8)
		{
			for (int j = 0; j < 8; j++)
			{
	
				k = eeprom_read_byte(j);


				if (k != User_Info[j])
					test = 1;
			}

			if (test == 0) {
				lcd_puts(1, "           ");
				lcd_puts(2, "           ");
				lcd_puts(1, " OK");

				// 로그인 성공시 (도미솔도) 스피커 소리
				sound(523); _delay_ms(300);
				sound(659); _delay_ms(300);
				sound(784); _delay_ms(300);
				sound(523 * 2); _delay_ms(300);
				sound(0); _delay_ms(300);
				i = 0;
				break;
			} 
			else 
			{

				for (int j = 0; j < 8; j++)	
					User_Info[j] = 0;

				i = 0;

				lcd_puts(1, "           ");
				lcd_puts(2, "           ");
				lcd_puts(1, " No");
				_delay_ms(2000);

				continue;
			}
		}
		else if (i < 4)
		{
			switch (keydata)
			{
			case 0x01:
				User_Info[i] = 0;
				lcd_gotoxy(6 + i, 1);
				lcd_putch('0');
				_delay_ms(500);
				i++;
				break;
			case 0x02:
				User_Info[i] = 1;
				lcd_gotoxy(6 + i, 1);
				lcd_putch('1');
				_delay_ms(500);
				i++;
				break;
			case 0x04:
				User_Info[i] = 2;
				lcd_gotoxy(6 + i, 1);
				lcd_putch('2');
				_delay_ms(500);
				i++;
				break;
			case 0x08:
				User_Info[i] = 3;
				lcd_gotoxy(6 + i, 1);
				lcd_putch('3');
				_delay_ms(500);
				i++;
				break;
			case 0x10:
				User_Info[i] = 4;
				lcd_gotoxy(6 + i, 1);
				lcd_putch('4');
				_delay_ms(500);
				i++;
				break;
			case 0x20:
				User_Info[i] = 5;
				lcd_gotoxy(6 + i, 1);
				lcd_putch('5');
				_delay_ms(500);
				i++;
				break;
			case 0x40:
				User_Info[i] = 6;
				lcd_gotoxy(6 + i, 1);
				lcd_putch('6');
				_delay_ms(500);
				i++;
				break;
			case 0x80:
				User_Info[i] = 7;
				lcd_gotoxy(6 + i, 1);
				lcd_putch('7');
				_delay_ms(500);
				i++;
				break;
			default:
				break;
			}
		}
		else if (i < 8)
		{
			switch (keydata)
			{
			case 0x01:
				User_Info[i] = 0;
				lcd_gotoxy(2 + i, 2);
				lcd_putch('0');
				_delay_ms(500);
				i++;
				break;
			case 0x02:
				User_Info[i] = 1;
				lcd_gotoxy(2 + i, 2);
				lcd_putch('1');
				_delay_ms(500);
				i++;
				break;
			case 0x04:
				User_Info[i] = 2;
				lcd_gotoxy(2 + i, 2);
				lcd_putch('2');
				_delay_ms(500);
				i++;
				break;
			case 0x08:
				User_Info[i] = 3;
				lcd_gotoxy(2 + i, 2);
				lcd_putch('3');
				_delay_ms(500);
				i++;
				break;
			case 0x10:
				User_Info[i] = 4;
				lcd_gotoxy(2 + i, 2);
				lcd_putch('4');
				_delay_ms(500);
				i++;
				break;
			case 0x20:
				User_Info[i] = 5;
				lcd_gotoxy(2 + i, 2);
				lcd_putch('5');
				_delay_ms(500);
				i++;
				break;
			case 0x40:
				User_Info[i] = 6;
				lcd_gotoxy(2 + i, 2);
				lcd_putch('6');
				_delay_ms(500);
				i++;
				break;
			case 0x80:
				User_Info[i] = 7;
				lcd_gotoxy(2 + i, 2);
				lcd_putch('7');
				_delay_ms(500);
				i++;
				break;
			default:
				break;
			}
		}
		break;
	}

	
	// 기능 선택 Function
	while (1)
	{
		lcd_puts(1, "0    1     2    ");
		lcd_puts(2, "STWC|CLOCK|TIMER");

		keydata = PINB & 0xFF;

		switch (keydata)
		{
		case 0x01: // 스탑 워치

			lcdClear();
			_delay_ms(10);


			while (1) {
			
				if(keydata == 0x10)
					break;
			
				lcd_puts(1, "TIME  ");

				lcd_gotoxy(7, 1);
				lcd_putn2(tmin);

				lcd_gotoxy(9, 1);
				lcd_putch(':');

				lcd_gotoxy(10, 1);
				lcd_putn2(tsec);

				lcd_gotoxy(12, 1);
				lcd_putch(':');

				lcd_gotoxy(13, 1);
				lcd_putn2(tmsec);

				lcd_gotoxy(1,2);
				putString("START ");
				
				count++;

				keydata = PINB & 0xFF;

				EX_SS_SEL = ~(0x01);
				EX_SS_DATA = segment_data[altmin / 10];

				_delay_ms(1);

				EX_SS_SEL = ~(0x02);
				EX_SS_DATA = segment_data[altmin % 10];

				_delay_ms(1);

				EX_SS_SEL = ~(0x04);
				EX_SS_DATA = segment_data[altsec / 10];

				_delay_ms(1);

				EX_SS_SEL = ~(0x08);
				EX_SS_DATA = segment_data[altsec % 10];

				if (keydata == 0x01)
				{
					while (1)
					{
						lcd_puts(2, "STOP            ");
						keydata = PINB & 0xFF;

						if (keydata == 0x02)
							break;
					}
				}
				else if (keydata == 0x04)
				{
					while (1)
					{
						lcd_puts(2, "MODIFY          ");
						keydata = PINB & 0xFF;

						switch (keydata)
						{
						case 0x40:
							tmin++;
							i++;
							break;
						case 0x20:
							tsec++;
							i++;
							break;
						}
						if (i == 1)
						{
							i--;
							break;
						}
					}
				}
				else if (keydata == 0x08)
				{
					al++;

					while (1)
					{
						lcd_puts(2, "ALRAM           ");
						keydata = PINB & 0xFF;

						switch (keydata)
						{
						case 0x40:
							altmin++;
							i++;
							break;
						case 0x20:
							altsec++;
							i++;
							break;
						}
						if (i == 1)
						{
							i--;
							break;
						}
					}
				}
				else if (keydata == 0x80)
				{
					temp_tmin = tmin;
					temp_tsec = tsec;
					temp_tmsec = tmsec;
				
					eeprom_write_byte(hex[3 * record_cnt], temp_tmin); // eeprom에 타임 스탬프 값을 저장
					eeprom_write_byte(hex[3 * record_cnt + 1], temp_tsec);
					eeprom_write_byte(hex[3 * record_cnt + 2], temp_tmsec);
					
					
					if (record_cnt < 4){
					
						lcd_gotoxy(7, 2);
						lcd_putn2(temp_tmin);

						lcd_gotoxy(9, 2);
						lcd_putch(':');

						lcd_gotoxy(10, 2);
						lcd_putn2(temp_tsec);

						lcd_gotoxy(12, 2);
						lcd_putch(':');

						lcd_gotoxy(13, 2);
						lcd_putn2(temp_tmsec);
					
						record_cnt++;
					}

					if (record_cnt == 1)
						EX_LED = 0x01;
					else if (record_cnt == 2)
						EX_LED = 0x03;
					else if (record_cnt == 3)
						EX_LED = 0x07;
					else if (record_cnt == 4)
						EX_LED = 0x0f;

					_delay_ms(200);
				}

				if (al != 0 && altmin == tmin && altsec == tsec && tmsec == 0)
				{
					sound(523); _delay_ms(300);
					sound(659); _delay_ms(300);
					sound(784); _delay_ms(300);
					sound(523 * 2); _delay_ms(300);
					sound(0);	_delay_ms(600);
					sound(523); _delay_ms(300);
					sound(659); _delay_ms(300);
					sound(784); _delay_ms(300);
					sound(523 * 2); _delay_ms(300);
					sound(0);	_delay_ms(600);
				}

				if (count == 1)
				{
					count = 0;
					tmsec++;

					if (tmsec == 100)
					{
						tsec++;
						tmsec = 0;
					}

					if (tsec == 60)
					{
						tmin++;
						tsec = 0;
					}

					if (tmin == 60)

						tmin = 0;

					lcd_puts(1, "TIME  ");

					lcd_gotoxy(7, 1);
					lcd_putn2(tmin);

					lcd_gotoxy(9, 1);
					lcd_putch(':');

					lcd_gotoxy(10, 1);
					lcd_putn2(tsec);

					lcd_gotoxy(12, 1);
					lcd_putch(':');

					lcd_gotoxy(13, 1);
					lcd_putn2(tmsec);
				}
			}
			break;

		case 0x02: // 일반 시계 + 듀얼 시계
			while (1)
			{
				lcdClear();
				
				while (1)
				{
					lcd_puts(1, "0 : SEOUL       ");
					lcd_puts(2, "1 : NY | 3: back");

					_delay_ms(200);

					if (keydata == 0x08)
						break;

					keydata = PINB & 0xFF;
					switch (keydata)
					{
					case 0x01:

						lcdClear();
						_delay_ms(10);

						while (1)
						{
							keydata = PINB & 0xFF;
							
							lcd_puts(1, "Seoul");

							if (keydata == 0x80)
								break;

							switch (keydata)
							{
							case 0x40: // 시간 수정

								if (hour == 23)
									hour = -1;
									
								writebyte(0x02, hexh[hour + 1]);
								break;
							case 0x20: // 분 수정
								if (hour == 59)
									hour = 0;
								writebyte(0x01, hexms[min + 1]);
								break;
							case 0x10: // 초 수정
								if (hour == 59)
									hour = 0;
								writebyte(0x00, hexms[sec + 1]);
								break;
							case 0x08: // 연도 수정
								if (year == 99)
									year = 0;
								writebyte(0x06, hexy[year + 1]);
								break;
							case 0x04: // 월 수정
								if (mon == 12)
									mon = 0;
								writebyte(0x05, hexm[mon]);
								break;
							case 0x02: // 일 수정
								if (day == 31)
									day = 0;
								writebyte(0x04, hexd[day]);
								break;
							case 0x01: // 요일 수정
								if (date == 7)
									date = 0;
								writebyte(0x03, hexdate[date + 1]);
								break;
							}

							hour = readbyte(0x02);
							hour = ((hour >> 4) & 0x07) * 10 + (hour & 0x0f);

							lcd_gotoxy(9, 1);
							lcd_putn2(hour);

							lcd_gotoxy(11, 1);
							lcd_putch(':');

							min = readbyte(0x01);
							min = ((min >> 4) & 0x07) * 10 + (min & 0x0f);

							lcd_gotoxy(12, 1);
							lcd_putn2(min);

							lcd_gotoxy(14, 1);
							lcd_putch(':');

							sec = readbyte(0x00);
							sec = ((sec >> 4) & 0x07) * 10 + (sec & 0x0f);

							lcd_gotoxy(15, 1);
							lcd_putn2(sec);

							//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
							// 연도를 LCD에 출력

							year = readbyte(0x06);
							year = ((year >> 4) & 0x07) * 10 + (year & 0x0f);

							lcd_gotoxy(7, 2);
							lcd_putn2(20);

							lcd_gotoxy(9, 2);
							lcd_putn2(year);

							lcd_gotoxy(11, 2);
							lcd_putch('/');

							// 월을 출력
							mon = readbyte(0x05);
							mon = ((mon >> 4) & 0x07) * 10 + (mon & 0x0f);

							lcd_gotoxy(12, 2);
							lcd_putn2(mon);

							lcd_gotoxy(14, 2);
							lcd_putch('/');

							// 일을 출력

							day = readbyte(0x04);
							day = ((day >> 4) & 0x07) * 10 + (day & 0x0f);

							lcd_gotoxy(15, 2);
							lcd_putn2(day);

							// 요일을 출력

							date = readbyte(0x03);
							date = ((date >> 4) & 0x07) * 10 + (date & 0x0f);

							lcd_gotoxy(1, 2);

							switch (date)
							{
							case 0:
								putString("MON");
								break;
							case 1:
								putString("TUE");
								break;
							case 2:
								putString("WEN");
								break;
							case 3:
								putString("THU");
								break;
							case 4:
								putString("FRI");
								break;
							case 5:
								putString("SAT");
								break;
							case 6:
								putString("SUN");
								break;
							}

							_delay_ms(1000);
						}
						break;

					case 0x02:
						lcdClear();
						_delay_ms(10);

						while (1)
						{
							keydata = PINB & 0xFF;

							lcd_puts(1, "NewYork");

							if (keydata == 0x80)
								break;

							switch (keydata)
							{
							case 0x40: // 시간 수정

								if (hour == 23)
									hour = -1;

								writebyte(0x08, hexh[hour + 1]);

								break;
							case 0x20: // 분 수정

								if (hour == 59)
									hour = 0;

								writebyte(0x01, hexms[min + 1]);

								break;
							case 0x10: // 초 수정

								if (hour == 59)
									hour = 0;

								writebyte(0x00, hexms[sec + 1]);

								break;
							case 0x08: // 연도 수정

								if (year == 99)
									year = 0;

								writebyte(0x06, hexy[year + 1]);

								break;
							case 0x04: // 월 수정

								if (mon == 12)
									mon = 0;

								writebyte(0x05, hexm[mon]);

								break;
							case 0x02: // 일 수정

								if (day == 31)
									day = 0;

								writebyte(0x04, hexd[day]);

								break;
							case 0x01: // 요일 수정

								if (date == 7)
									date = 0;

								writebyte(0x03, hexdate[date + 1]);
								break;
							}

							hour = readbyte(0x08);
							hour = ((hour >> 4) & 0x07) * 10 + (hour & 0x0f);

							lcd_gotoxy(9, 1);
							lcd_putn2(hour);

							lcd_gotoxy(11, 1);
							lcd_putch(':');

							min = readbyte(0x01);
							min = ((min >> 4) & 0x07) * 10 + (min & 0x0f);

							lcd_gotoxy(12, 1);
							lcd_putn2(min);

							lcd_gotoxy(14, 1);
							lcd_putch(':');

							sec = readbyte(0x00);
							sec = ((sec >> 4) & 0x07) * 10 + (sec & 0x0f);

							lcd_gotoxy(15, 1);
							lcd_putn2(sec);
							//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

							// 연도를 LCD에 출력
							year = readbyte(0x06);
							year = ((year >> 4) & 0x07) * 10 + (year & 0x0f);

							lcd_gotoxy(7, 2);
							lcd_putn2(20);

							lcd_gotoxy(9, 2);
							lcd_putn2(year);

							lcd_gotoxy(11, 2);
							lcd_putch('/');

							// 월을 출력
							mon = readbyte(0x05);
							mon = ((mon >> 4) & 0x07) * 10 + (mon & 0x0f);

							lcd_gotoxy(12, 2);
							lcd_putn2(mon);

							lcd_gotoxy(14, 2);
							lcd_putch('/');

							// 일을 출력
							day = readbyte(0x04);
							day = ((day >> 4) & 0x07) * 10 + (day & 0x0f);

							lcd_gotoxy(15, 2);
							lcd_putn2(day);

							// 요일을 출력
							date = readbyte(0x03);
							date = ((date >> 4) & 0x07) * 10 + (date & 0x0f);

							lcd_gotoxy(1, 2);
							
							switch (date)
							{
							case 0:
								putString("MON");
								break;
							case 1:
								putString("TUE");
								break;
							case 2:
								putString("WEN");
								break;
							case 3:
								putString("THU");
								break;
							case 4:
								putString("FRI");
								break;
							case 5:
								putString("SAT");
								break;
							case 6:
								putString("SUN");
								break;
							}
							_delay_ms(1000);
						}
						break;
					default:
						break;
					}
				}
				break;
			}
			break;

		case 0x04: // 타이머/카운트

			lcdClear();
			_delay_ms(10);

			lcd_puts(1, "TIMER");
			lcd_puts(2, "0 sec | 1 min ");
			
			lcd_gotoxy(9, 1);
			lcd_putn2(cmin);
			
			lcd_gotoxy(11, 1);
			lcd_putch(':');
			
			lcd_gotoxy(12, 1);
			lcd_putn2(csec);

			while (1)
			{	
				keydata = PINB & 0xFF;
				
				if(keydata == 0x80)
					break;
					
				switch (keydata)
				{
				case 0x01:
				
					csec++;

					if (csec == 60)
					{
						csec = 0;
					}

					lcd_gotoxy(12, 1);
					lcd_putn2(csec);

					_delay_ms(200);
					break;
				case 0x02:
					cmin++;

					if (cmin == 60)
					{
						cmin = 0;
					}

					lcd_gotoxy(9, 1);
					lcd_putn2(cmin);
					
					_delay_ms(200);
					break;
				case 0x08:
					while (1)
					{
						cmsec++;
						
						if(cmsec == 40)
						{
							csec--;
							cmsec = 0;
						}
						
						if (cmin == 0 && csec == 0)
						{	
							EX_SS_SEL =~ (0x08);
							EX_SS_DATA = segment_data[0];
						
							sound(523); _delay_ms(300);
							sound(659); _delay_ms(300);
							sound(784); _delay_ms(300);
							sound(523 * 2); _delay_ms(300);
							sound(0);	_delay_ms(600);
							sound(523); _delay_ms(300);
							sound(659); _delay_ms(300);
							sound(784); _delay_ms(300);
							sound(523 * 2); _delay_ms(300);
							sound(0);	_delay_ms(600);
							
							lcd_gotoxy(12, 1);
							lcd_putn2(0);
							
							lcd_gotoxy(9, 1);
							lcd_putn2(0);
							
							break;
						}

						EX_SS_SEL =~ (0x01);
						EX_SS_DATA = segment_data[cmin / 10];

						_delay_ms(5);

						EX_SS_SEL =~ (0x012);
						EX_SS_DATA = segment_data[cmin % 10];

						_delay_ms(5);

						EX_SS_SEL =~ (0x04);
						EX_SS_DATA = segment_data[csec / 10];

						_delay_ms(5);

						EX_SS_SEL =~ (0x08);
						EX_SS_DATA = segment_data[csec % 10];
						
						_delay_ms(5);

						if (csec == 0)
						{
							csec = 59;
							cmin--;
						}
						
						_delay_ms(5);
					}
					break;
				case 0x80:
					break;
				default:
					break;
				}
			}
			break;
		}
	}
}

