/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <stdio.h>

#define MAX30001 DT_LABEL(DT_NODELABEL(adc_max30001))

static bool lead_status;
static struct k_sem sem;

static void max30001_ecg_sample_handler(const struct device *dev,
					const struct sensor_trigger *trigger)
{
	struct sensor_value values;
	int ret, counter = 0;

	do {
		counter++;

		ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_VOLTAGE);
		if (ret < 0) {
			printk("Error Detected: %d\n", ret);
			return;
		}

		sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &values);
	} while (ret == 1);

	printk("sample[%u]: %d\n", (counter - 1), values.val1);
}

static void max30001_lead_status_handler(const struct device *dev,
				       const struct sensor_trigger *trigger)
{
	struct sensor_value values;

	sensor_channel_get(dev, SENSOR_CHAN_PROX, &values);

	lead_status = (bool)values.val1;
	/* Notify main loop, lead status changed. */
	k_sem_give(&sem);
}

void main(void)
{
	const struct device *dev;
	struct sensor_trigger ecg_trig = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_VOLTAGE
	};
	struct sensor_trigger lead_trig = {
		.type = SENSOR_TRIG_NEAR_FAR,
		.chan = SENSOR_CHAN_PROX
	};

	dev = device_get_binding(MAX30001);
	if (!device_is_ready(dev)) {
		printk("max30001 not ready\n");
		return;
	}

	lead_status = false;
	k_sem_init(&sem, 1, 1);

	while (true) {
		k_sem_take(&sem, K_FOREVER);

		printk("Lead status: %d\n", lead_status);

		/* Lead status changed, disable ecg sampling and lead detectioin. */
		sensor_trigger_set(dev, &ecg_trig, NULL);

		if (lead_status) {

			/* Start ECG sampling. */
			if (sensor_trigger_set(dev, &ecg_trig, max30001_ecg_sample_handler) < 0) {
				printk("Failed to set ecg sampling handler.\n");
				return;
			}

			/* Enable lead-off detection. */
			if (sensor_trigger_set(dev, &lead_trig, max30001_lead_status_handler) < 0) {
				printk("Failed to set lead-off event handler.\n");
				return;
			}
		} else {

			/* Enable lead-on detection. */
			if (sensor_trigger_set(dev, &lead_trig, max30001_lead_status_handler) < 0) {
				printk("Failed to set lead-on event handler.\n");
				return;
			}
		}
	}
}
