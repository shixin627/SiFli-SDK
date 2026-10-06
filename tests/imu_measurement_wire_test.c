#include "../example/get-started/dualcore/src/modules/bloc/imu_measurement_wire.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    uint8_t packet[84] = {0};
    float values[16] = {9.80665f, -1.0f, -2.0f, -3.0f, -4.0f, -5.0f,
        -6.0f, -7.0f, -8.0f, -9.0f, -10.0f, -11.0f, -12.0f, -13.0f, -14.0f, -15.0f};
    assert(sizeof(float) == 4);
    imu_wire_header(packet, 1, 123);
    imu_wire_sample(packet + 8, 9, UINT32_MAX, 100, values);
    assert(memcmp(packet, "IM\1\1\173\0\0\0", 8) == 0);
    assert(imu_wire_read_u32(packet + 8) == 9);
    assert(imu_wire_read_u32(packet + 12) == UINT32_MAX);
    assert(packet[16] == 100 && packet[17] == 0 && packet[18] == 1 && packet[19] == 0);
    assert(imu_wire_read_u32(packet + 24) == 0xbf800000u); /* -1 float LE */
    assert(imu_wire_read_u32(packet + 80) == 0xc1700000u); /* quaternion z=-15 */
    float decoded; uint32_t bits = imu_wire_read_u32(packet + 20);
    memcpy(&decoded, &bits, 4); assert(decoded == values[0]);
    imu_wire_header(packet, 0, 0x12345678u);
    assert(imu_wire_read_u32(packet + 4) == 0x12345678u);
    puts("imu_measurement_wire_test: PASS");
    return 0;
}
