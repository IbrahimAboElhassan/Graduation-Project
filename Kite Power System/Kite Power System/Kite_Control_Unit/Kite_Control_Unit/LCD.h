

#ifndef LCD_H_
#define LCD_H_


/*Macros*/
#include "LCD_CONFIG.h"
/*Function Prototypes*/
void LCD_Init(void);
void LCD_Write_Command(Uint8t command);
void LCD_Write_Data(Uint8t data);
void LCD_Clear(void);
void LCD_Write_String(Uint8t * str);
void LCD_Write_Number(Uint32t number);
//
void LCD_Write_Float(float number, Uint8t decimal_places);
//

#endif /* LCD_H_ */