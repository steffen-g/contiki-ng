/*
 * battery.c
 *
 *  Created on: Oct 14, 2019
 */

#include "contiki.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "battery-sensor.h"
/*---------------------------------------------------------------------------*/
#include "board-conf.h"
#include <Board.h>

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include <ti/drivers/ADC.h>

static ADC_Params params;
static ADC_Handle adcHandle;
static uint16_t result;
static int vbat;
static struct ctimer convert_timer;

static void
notify_ready(void *unused)
{
	vbat = ADC_convertToMicroVolts(adcHandle, result);
	sensors_changed(&battery_sensor);
}


static int value(int type)
{
	return (vbat*4)/10000;
}

static int configure(int type, int enable)
{
	  switch(type) {
	  case SENSORS_HW_INIT:
		  ADC_init();
		  ADC_Params_init(&params);
		  params.isProtected = true;
		  adcHandle = ADC_open(3, &params);
		  /*
		  int value = ADC_convertToMicroVolts(adcHandle, result);
		  value = value +1;
		  ADC_close(adcHandle);*/
	    break;
	  case SENSORS_ACTIVE:
	    if(enable) {
	    	ADC_convert(adcHandle, &result);
	    	ctimer_set(&convert_timer, 3, notify_ready, NULL);
	    }
	    break;
	  default:
	    break;
	  }
	  return 1;
}

static int status(int type)
{
	return 1;
}

SENSORS_SENSOR(battery_sensor, "Bat", value, configure, status);
