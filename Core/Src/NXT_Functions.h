
#ifndef SRC_NXT_FUNCTIONS_H_
#define SRC_NXT_FUNCTIONS_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

void NXT_SendNum(char *obj, int32_t num);
void NXT_SendFloat(char *obj, float num, int dp);
void NXT_SendPB(char *obj, uint16_t num);
void NXT_SendTXT(char *obj, char val[]);
void NXT_SendCmd(char *obj, char *param, uint16_t num, char *str, bool isStr);

#endif /* SRC_NXT_FUNCTIONS_H_ */
