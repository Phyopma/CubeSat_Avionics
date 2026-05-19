#include "power_runtime.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_snapshot_publish_and_read(void)
{
    power_snapshot_t snapshot_a;
    power_snapshot_t snapshot_b;
    power_snapshot_t out;

    Power_Init();
    (void)memset(&snapshot_a, 0, sizeof(snapshot_a));
    (void)memset(&snapshot_b, 0, sizeof(snapshot_b));

    snapshot_a.publish_timestamp_ms = 10U;
    snapshot_a.battery.valid_mask = POWER_BATTERY_VALID_CURRENT;
    snapshot_a.battery.current_mA = -125;

    snapshot_b.publish_timestamp_ms = 20U;
    snapshot_b.battery.valid_mask = POWER_BATTERY_VALID_VOLTAGE;
    snapshot_b.battery.voltage_mV = 7400U;
    snapshot_b.digital_temp.valid_mask = POWER_DIGITAL_TEMP_VALID_TEMPERATURE | POWER_DIGITAL_TEMP_VALID_STATUS;
    snapshot_b.digital_temp.temperature_cC = 2350;
    snapshot_b.digital_temp.status = 0x80U;

    Power_PublishSnapshot(&snapshot_a);
    Power_ReadSnapshot(&out);
    assert(out.publish_sequence == 1U);
    assert(out.publish_timestamp_ms == 10U);
    assert(out.battery.current_mA == -125);

    Power_PublishSnapshot(&snapshot_b);
    Power_ReadSnapshot(&out);
    assert(out.publish_sequence == 2U);
    assert(out.publish_timestamp_ms == 20U);
    assert(out.battery.voltage_mV == 7400U);
    assert(out.digital_temp.temperature_cC == 2350);
    assert(out.digital_temp.status == 0x80U);
}

int main(void)
{
    test_snapshot_publish_and_read();
    puts("power_runtime_test: PASS");
    return 0;
}
