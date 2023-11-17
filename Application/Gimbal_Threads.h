#pragma once

#include "main.h"

#include "controller.h"
#define CAN_TRANSMITION_FREQ 500

// #define PITCH_IN_RESPONSE_TEST_MODE 1U
// #define YAW_IN_RESPONSE_TEST_MODE 1U

void Give_Values(void *arg);
void CAN_Transmition_Thread(void *arg);

void YawEcdReportTask(void *pvParameters);
