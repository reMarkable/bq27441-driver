#ifndef _BQ27441_BATTERY_H
#define _BQ27441_BATTERY_H

#include <linux/power/bq27xxx_battery.h>
int bq27441_init(struct bq27xxx_device_info *di);
int bq27441_exit(struct bq27xxx_device_info *di);

#endif /* _BQ27441_BATTERY_H */
