
#include "main.h"

extern "C" int mymain(void);

#include <stdio.h>
#include <lcd.h>


int mymain(void)
{
  LCD_INIT();

  char name[] = "DAS, Ujaan";
  LCD_DrawString(0, 0, name);

  while (1)
  {


  }

  return 0;
}
