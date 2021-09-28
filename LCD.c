/**
*	@file 		LCD.c
*	@brief 		LCD screen API
*	@author		luj, Klaus Kiefer, Peter Wegmann
*	@version	1.1
*	@date		2020/03/25
*
*
*/


#include "LCD.h"
#include "assert.h"


// Variables for Init_Screen
uint8_t InitScreenMaxNoOfLines = 6;
uint8_t InitScreenNextLine = 1;


/**
 * @brief Displays a character from the ASCII array 7pxl wide and 8pxl high. 
 * 
 * @param character Character(The character that shall be placed in ASCII code.)
 * 
 * @return void
 */
void LCD_Character(char character)
{
	LCD_Write(LCD_DATA, 0x00);
	for (uint8_t index = 0; index < 5; index++)
	{
		switch (character) // for additional characters who aren't in ASCII
		{
			case '³':
				LCD_Write(LCD_DATA, additional_char[0][index]);
				break;
			case '°':
				LCD_Write(LCD_DATA, additional_char[1][index]);
				break;
			default:
				LCD_Write(LCD_DATA, ASCII[character - 0x20][index]);
				break;
		}
	}
	LCD_Write(LCD_DATA, 0x00);
}


/**
 * @brief Clears the LCD screen.
 * 
 * 
 * @return void
 */
void LCD_Clear(void)
{
	for (int index = 0; index < ((LCD_X * LCD_Y) >> 3); index++)
	{
		LCD_Write(LCD_DATA, 0x00);
	}
}

/**
 * @brief   Clears the LCD on a given row from a column to a column.
 * 
 * @param from_x from_x(From column position on the display.)
 * @param to_x to_x(To column position on the display.)
 * @param y y(Row position on the display.)
 * 
 * @return void
 */
void LCD_Clear_row_from_column_to_column(uint8_t from_x, uint8_t to_x, uint8_t y)
{
	int8_t i;
	
	LCD_gotoXY(from_x, y);
	// the addition is for the multiplication with 7
	for (i = 0; i < ((to_x + to_x + to_x + to_x + to_x + to_x + to_x) - (from_x + from_x + from_x + from_x + from_x + from_x + from_x)); i++)
	{
		LCD_Write(LCD_DATA, 0x00);
	}
}


/**
 * @brief Clears the LCD on a given row from a given column.
 * 
 * @param x x(Column position on the display.)
 * @param y y(Row position on the display.)
 * 
 * @return void
 */
void LCD_Clear_row_from_column(uint8_t x, uint8_t y)
{
	uint8_t i;
	
	LCD_gotoXY(x, y);
	/////////////////////////for the multiplication with 7
	for (i = 0; i < LCD_X - (x+x+x+x+x+x+x); i++)
	{
		LCD_Write(LCD_DATA, 0x00);
	}
}


/**
 * @brief Initialization of the LCD ports and the configurations. 
 * 
 * 
 * @return void
 */
void init_LCD(void)
{
	
	LCD_DDR |= (1<<PIN_DC) | (1<<PIN_RESET) | (1<<PIN_SCE) | (1<<PIN_SCLK) | (1<<PIN_SDIN); // sets the pins for the LCD as outputs
	
	LCD_PORT &= ~(1<<PIN_RESET);
	_delay_ms(10);
	LCD_PORT |= (1<<PIN_RESET);
	_delay_ms(10);
	
	
	LCD_Write(LCD_CMD, LCD_C_FS | LCD_C_FS_ADDITIONAL_CMD | LCD_C_FS_ACTIVE_MODE | LCD_C_FS_INCREASE_HORIZONTAL_LINE);  // LCD Additional Commands.
	LCD_Write(LCD_CMD, LCD_C_SVOP | 66);  // Set LCD Vop (Contrast). -> Vlcd = 7V (internal, 66) 
	LCD_Write(LCD_CMD, LCD_C_TC | LCD_C_TC_COEFFICIENT_0);  // Set Temp coefficent. 
	LCD_Write(LCD_CMD, LCD_C_BS | LCD_C_BS_N_4);  // LCD bias mode 1:48. 
	LCD_Write(LCD_CMD, LCD_C_FS | LCD_C_FS_ACTIVE_MODE | LCD_C_FS_BASIC_CMD | LCD_C_FS_INCREASE_HORIZONTAL_LINE);
	LCD_Write(LCD_CMD, LCD_C_DC | LCD_C_DC_NORMAL_MODE);
	
	LCD_Clear();
	LCD_gotoXY(0, 0);
	
}


/**
 * @brief  Displays a String at a certain position.
 *
 * 
 * @param characters Pointer to byte array of the string.
 * @param x X position on the screen.
 * @param y Y position on the screen.
 * 
 * @return void
 */
void LCD_String(char *characters, uint8_t x, uint8_t y)
{
	LCD_gotoXY(x, y);
	
	while (*characters)
	{
		LCD_Character(*characters++);
	}
}


/**
 * @brief Displays a value with a unit on a certain position. The value is a an sign sensitive number with a fixed dot point position.
 *	That's because floating point number are not easy and fast to calculate with a MC. But with this function a floating point number can easily displayed.
 * 
 * @param number    The signed integer value that shall be displayed.
 * @param dot_point Represents the position of the dot point right aligned. If the is no dot point the value must be 0.	Example: dot_point = 2 -> 1.24	(OR: dot_point = 4, number = 3 -> 0.0003)
 * @param x         Column on the display.
 * @param y         Row on the Display.
 * @param unit      A string for the unit of the value. If there is no unit type in NULL.
 * 
 * @return void
 */
void LCD_Value(int64_t number, uint8_t dot_point, uint8_t x, uint8_t y, const char *unit)
{
	uint8_t number_characters = 0;
	uint8_t digits = 1;
	uint8_t i;
	int64_t compare_number;
	uint8_t string_length = 0;
	
	//array for the splitted number
	char digit_values[max_characters_one_line];
	
	if (number > 0)
	{
		compare_number = 10;
		
		for (i = 0; i < max_characters_one_line - 1; i++)
		{
			if (number >= compare_number)
			{
				digits++;
				compare_number *= 10;
			}
		}
	}
	
	if (number < 0)
	{
		compare_number = -10;
		
		for (i = 0; i < max_characters_one_line - 1; i++)
		{
			if (number <= compare_number)
			{
				digits++;
				compare_number = compare_number * 10;
			}
		}
	}
	
	//to get the numbers direct from the ASCII table
	for (i = 0; i < digits; i++)
	{
		digit_values[i] = 0x30;
	}
	
	if (unit != NULL)
	{
		while (*unit != '\0')
		{
			string_length++;
			unit++;
		}
		
		for (i = 0; i < string_length; i++)
		{
			unit--;
		}
	}
	
	if (dot_point)
	{
		number_characters++;
	}
	
	number_characters = number_characters + digits + string_length;
	
	while ((number_characters) > (max_characters_one_line - x)) 
	{
		if (dot_point)
		{
			if (--dot_point == 0)
			{
				number_characters--;
			}
			number_characters--;
			digits--;
			number /= 10;
		}
		else
		{
			if ((max_characters_one_line - x) >= 7)
			{
				LCD_String("too big", x, y);
				return;
			}
			else
			{
				return;
			}
		}
	}
	
	
	if (number < 0)
	{
		LCD_String("-", x, y);
	}
	else
	{
		LCD_gotoXY(x, y);
	}
	
	
	//divide the number in the values of the digits
	if (number > 0)
	{
		while (number >= 1000000000)
		{
			number -= 1000000000;
			digit_values[9]++;
		}
		while (number >= 100000000)
		{
			number -= 100000000;
			digit_values[8]++;
		}
		while (number >= 10000000)
		{
			number -= 10000000;
			digit_values[7]++;
		}
		while (number >= 1000000)
		{
			number -= 1000000;
			digit_values[6]++;
		}
		while (number >= 100000)
		{
			number -= 100000;
			digit_values[5]++;
		}
		while (number >= 10000)
		{
			number -= 10000;
			digit_values[4]++;
		}
		while (number >= 1000)
		{
			number -= 1000;
			digit_values[3]++;
		}
		while (number >= 100)
		{
			number -= 100;
			digit_values[2]++;
		}
		while (number >= 10)
		{
			number -= 10;
			digit_values[1]++;
		}
		while(number)
		{
			number--;
			digit_values[0]++;
		}
	}
	
	else if (number < 0)
	{
		while (number <= -1000000000)
		{
			number += 1000000000;
			digit_values[9]++;
		}
		while (number <= -100000000)
		{
			number += 100000000;
			digit_values[8]++;
		}
		while (number <= -10000000)
		{
			number += 10000000;
			digit_values[7]++;
		}
		while (number <= -1000000)
		{
			number += 1000000;
			digit_values[6]++;
		}
		while (number <= -100000)
		{
			number += 100000;
			digit_values[5]++;
		}
		while (number <= -10000)
		{
			number += 10000;
			digit_values[4]++;
		}
		while (number <= -1000)
		{
			number += 1000;
			digit_values[3]++;
		}
		while (number <= -100)
		{
			number += 100;
			digit_values[2]++;
		}
		while (number <= -10)
		{
			number += 10;
			digit_values[1]++;
		}		
		while(number)
		{
			number++;
			digit_values[0]++;
		}
	}
	else
	{
		digits = 1;
		digit_values[0] = 0x30;
	}
	
	if (digits <= dot_point) // Appends the number with zeros left aligned, if the dot point should be beyond the number on the left side.
	{
		for (i = digits; dot_point >= i; i++)
		{
			digit_values[digits++] = 0x30;
		}
	}
	
	
	for (i = 0; i < digits; i++)
	{
		LCD_Character(digit_values[digits - i - 1]);
		if (dot_point && (digits - i - 1) == (dot_point))
		{
			LCD_Character(0x2E); //ASCII code for a dot point
		}
	}
	
	if (unit != NULL)
	{
		while (*unit != '\0')
		{
			LCD_Character(*unit++);
		}
	}
}


/**
 * @brief  Sends either data to be displayed or a command to the LCD.
 * 
 * @param dc Indicator for sending data(1) or a command(0).
 * @param data The 8bit data or command to be sent.
 * 
 * @return void
 */
void LCD_Write(uint8_t dc, uint8_t data)
{
	if (dc) // for the Data-Command Pin
	{
		LCD_PORT |= (1<<PIN_DC);
	}
	else
	{
		LCD_PORT &= ~(1<<PIN_DC);
	}
	
	LCD_PORT &= ~(1<<PIN_SCE); // enables the receiving of data or commands
	
	LCD_ShiftOut(PIN_SDIN, PIN_SCLK, data); 
	
	LCD_PORT |= (1<<PIN_SCE); // disables the receiving of data or commands 
}


/**
 * @brief Moves the cursor in block form (7pxl x 8pxl) on the display.
 * 
 * @param x X position on the display.
 * @param y Y position on the display.
 * 
 * @return void
 */
void LCD_gotoXY(uint8_t x, uint8_t y)
{
	LCD_Write(LCD_CMD, LCD_C_SET_X | x*7);  // Column.
	LCD_Write(LCD_CMD, LCD_C_SET_Y | y);  // Row.
}


/**
 * @brief  Shifts the data via a serial port out.
 * 
 * @param data_pin On this the pin is data is transmitted.
 * @param clock_pin The frequency pin of the transmission.
 * @param data The 8bit data that shall be transmitted.
 * 
 * @return void
 */
void LCD_ShiftOut(uint8_t data_pin, uint8_t clock_pin, uint8_t data)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		if ((data>>(7-i)) & 0x01)
		{
			LCD_PORT |= (1<<data_pin);
		}
		else
		{
			LCD_PORT &= ~(1<<data_pin);
		}
		
		LCD_PORT |= (1<<clock_pin);
		LCD_PORT &= ~(1<<clock_pin);
	}
}


//=========================================================================
// Init screen with messages appearing one under the other
//=========================================================================


/**
 * @brief Print one line of InitScreen. wraps around if lines exceed display height
 * 
 * @param Text The text that is printed (max len = 12)
 * @param FirstLine if set to one the screen is cleared and the text is printed inthe first line
 * 
 * @return void
 */
void LCD_InitScreen_AddLine(char* Text, const char FirstLine)
{
	if (FirstLine)
	{
		InitScreenNextLine = 1;
	}
	if (InitScreenNextLine == 1) LCD_Clear();  // Clear Screen, before first line is written
	
	LCD_String(Text, 0, InitScreenNextLine-1);
	++InitScreenNextLine;
	if (InitScreenNextLine > InitScreenMaxNoOfLines)
	{
		InitScreenNextLine = 1;
		_delay_ms(1000);		  // wait until next page is displayed
	}
	_delay_ms(300);

}

/**
 * @brief Used to display system info/status in the bottom right corner of the display
 * 
 * @param line The taxt that is printed (max len =  6)
 * @param update If set to one the line is cleared befor printing
 * 
 * @return void
 */
void LCD_paint_info_line(char *line, _Bool update)
{
	if (!update) LCD_String("      ", 6, 5);  // clears line (not necessary if in update mode)
	LCD_String(line, 6, 5);
}



