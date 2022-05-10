/** 
 * reference: https://github.com/vanvincent/googleroom
 */

#include <avr/io.h>
#include <stdlib.h>

#include "i2c.h"
#include "rtc.h"

#define RTC_DEVICE_ADDRESS 0xD0

void rtc_init(void)
{
    /* nothing */
}

uint8_t rtc_get_time(struct rtc_time *time_buf)
{
    uint8_t abuf[1];
    uint8_t buf[7];

    abuf[0] = 0x00;

    int res = i2c_io(RTC_DEVICE_ADDRESS, abuf, sizeof(abuf), NULL, 0, buf, sizeof(buf));
    if (res) {
        return res;
    }

    time_buf->seconds  = buf[0] & 0x0f;
    time_buf->seconds += ((buf[0] & 0x70) >> 4) * 10;

    time_buf->minutes  = buf[1] & 0x0f;
    time_buf->minutes += ((buf[1] & 0x70) >> 4) * 10;

    time_buf->hours    = buf[2] & 0x0f;
    time_buf->hours   += ((buf[2] & 0x30) >> 4) * 10;

    time_buf->day      = buf[3] & 0x07;

    time_buf->date     = buf[4] & 0x0f;
    time_buf->date    += ((buf[4] & 0x30) >> 4) * 10;

    time_buf->month    = buf[5] & 0x0f;
    time_buf->month   += ((buf[5] & 0x10) >> 4) * 10;

    time_buf->year     = buf[6] & 0x0f;
    time_buf->year    += ((buf[6] & 0xf0) >> 4) * 10;

    return 0;
}

uint8_t rtc_run(void)
{
    uint8_t buf[] = {
            0x00, // seconds
            0x25, // minutes
            0x51, // hours
            0x05, // day
            0x22, // date
            0x04, // month
            0x22  // year
    };
    uint8_t abuf[] = {
            0x00
    };

    return i2c_io(RTC_DEVICE_ADDRESS, abuf, sizeof(abuf), buf, sizeof(buf), NULL, 0);
}
