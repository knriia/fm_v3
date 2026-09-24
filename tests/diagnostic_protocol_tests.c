#include "diagnostic_dto.h"
#include "dto.h"
#include "FreeRTOS.h"
#include "freertos_hooks.h"
#include "lwip/memp.h"
#include "lwip/netif.h"
#include "lwip/priv/memp_priv.h"
#include "lwip/stats.h"
#include "lwip_diagnostics.h"
#include "lwip_udp_diagnostics.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
#include "system_diagnostics.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define EXPECTED_DIAGNOSTIC_PAYLOAD_SIZE 1696U
#define EXPECTED_DIAGNOSTIC_FRAME_SIZE 1704U

struct netif gnetif;
struct stats lwip_stats;
const struct memp_desc *memp_pools[MEMP_MAX];
uint8_t _end;
uint8_t _Min_Heap_Size;
uint8_t _Min_Stack_Size;
uint8_t __lwip_heap_end__;

DBGMCU_TypeDef test_dbgmcu;
RCC_TypeDef test_rcc;
SCB_Type test_scb;
MPU_Type test_mpu;
DWT_Type test_dwt;
SysTick_Type test_systick;
NVIC_Type test_nvic;
uint32_t SystemCoreClock;

static struct stats_memp test_memp_stats[MEMP_MAX];
static struct memp_desc test_memp_descriptors[MEMP_MAX];
static uint32_t test_failures;
static uint32_t test_free_heap;
static uint32_t test_minimum_free_heap;
static UBaseType_t test_task_count;
static UBaseType_t test_snapshot_count;
static uint32_t test_total_runtime_ticks;
static uint32_t test_idle_runtime_ticks;
static uint32_t test_scheduler_state;
static uint32_t test_tick_count;
static uint32_t test_stack_overflow_count;
static uint32_t test_malloc_failed_count;
static uint32_t test_ipsr;
static uint32_t test_control;
static uint32_t test_basepri;
static uint32_t test_primask;
static uint32_t test_faultmask;
static uint32_t test_msp;
static uint32_t test_psp;
static TaskHandle_t test_idle_task_handle = (TaskHandle_t)(uintptr_t)1U;

uint32_t xPortGetFreeHeapSize(void) { return test_free_heap; }

uint32_t xPortGetMinimumEverFreeHeapSize(void) { return test_minimum_free_heap; }

UBaseType_t uxTaskGetNumberOfTasks(void) { return test_task_count; }

UBaseType_t uxTaskGetSystemState(TaskStatus_t *task_statuses, UBaseType_t array_size, uint32_t *total_runtime_ticks) {
    (void)task_statuses;
    (void)array_size;
    *total_runtime_ticks = test_total_runtime_ticks;
    return test_snapshot_count;
}

uint32_t xTaskGetSchedulerState(void) { return test_scheduler_state; }

TickType_t xTaskGetTickCount(void) { return test_tick_count; }

TaskHandle_t xTaskGetIdleTaskHandle(void) { return test_idle_task_handle; }

void vTaskGetInfo(TaskHandle_t task_handle, TaskStatus_t *task_status, int get_free_stack, int state) {
    (void)task_handle;
    (void)get_free_stack;
    (void)state;
    task_status->ulRunTimeCounter = test_idle_runtime_ticks;
}

void freertos_hooks_get_diagnostics(uint32_t *stack_overflow_count, uint32_t *malloc_failed_count) {
    *stack_overflow_count = test_stack_overflow_count;
    *malloc_failed_count = test_malloc_failed_count;
}

uint32_t HAL_GetUIDw0(void) { return 0x10000001U; }

uint32_t HAL_GetUIDw1(void) { return 0x10000002U; }

uint32_t HAL_GetUIDw2(void) { return 0x10000003U; }

uint32_t HAL_RCC_GetSysClockFreq(void) { return 550000000U; }

uint32_t HAL_RCC_GetHCLKFreq(void) { return 275000000U; }

uint32_t HAL_RCC_GetPCLK1Freq(void) { return 137500000U; }

uint32_t HAL_RCC_GetPCLK2Freq(void) { return 137500000U; }

uint32_t HAL_GetTick(void) { return 5011U; }

uint32_t __get_IPSR(void) { return test_ipsr; }

uint32_t __get_CONTROL(void) { return test_control; }

uint32_t __get_BASEPRI(void) { return test_basepri; }

uint32_t __get_PRIMASK(void) { return test_primask; }

uint32_t __get_FAULTMASK(void) { return test_faultmask; }

uint32_t __get_MSP(void) { return test_msp; }

uint32_t __get_PSP(void) { return test_psp; }

static void expect_true(int condition, const char *message) {
    if (condition == 0) {
        ++test_failures;
        (void)fprintf(stderr, "FAIL: %s\n", message);
    }
}

static void expect_u32(uint32_t actual, uint32_t expected, const char *message) {
    if (actual != expected) {
        ++test_failures;
        (void)fprintf(
            stderr,
            "FAIL: %s (expected %lu, got %lu)\n",
            message,
            (unsigned long)expected,
            (unsigned long)actual
        );
    }
}

static void fill_protocol_stats(struct stats_proto *stats, uint32_t base) {
    stats->xmit = base + 0U;
    stats->recv = base + 1U;
    stats->fw = base + 2U;
    stats->drop = base + 3U;
    stats->chkerr = base + 4U;
    stats->lenerr = base + 5U;
    stats->memerr = base + 6U;
    stats->rterr = base + 7U;
    stats->proterr = base + 8U;
    stats->opterr = base + 9U;
    stats->err = base + 10U;
    stats->cachehit = base + 11U;
}

static void
expect_protocol_stats(const LwipProtocolDiagnostics *diagnostics, const struct stats_proto *stats, const char *name) {
    expect_true(memcmp(diagnostics, stats, sizeof(*diagnostics)) == 0, name);
}

static void fill_mib2_stats(struct stats_mib2 *stats, uint32_t base) {
    uint32_t values[48];

    for (uint32_t index = 0U; index < 48U; ++index) {
        values[index] = base + index;
    }

    (void)memcpy(stats, values, sizeof(values));
}

static void prepare_lwip_fixture(void) {
    (void)memset(&gnetif, 0, sizeof(gnetif));
    (void)memset(&lwip_stats, 0, sizeof(lwip_stats));

    gnetif.flags = TEST_NETIF_FLAG_UP | TEST_NETIF_FLAG_LINK_UP;
    gnetif.mtu = 1500U;
    gnetif.link_type = 6U;
    gnetif.link_speed = 100000000U;
    gnetif.ts = 0x12345678U;
    gnetif.ip_addr.addr = 0x0101A8C0U;
    gnetif.netmask.addr = 0x00FFFFFFU;
    gnetif.gw.addr = 0x0101A8C0U;

    gnetif.mib2_counters.ifinoctets = 101U;
    gnetif.mib2_counters.ifinucastpkts = 102U;
    gnetif.mib2_counters.ifinnucastpkts = 103U;
    gnetif.mib2_counters.ifindiscards = 104U;
    gnetif.mib2_counters.ifinerrors = 105U;
    gnetif.mib2_counters.ifinunknownprotos = 106U;
    gnetif.mib2_counters.ifoutoctets = 107U;
    gnetif.mib2_counters.ifoutucastpkts = 108U;
    gnetif.mib2_counters.ifoutnucastpkts = 109U;
    gnetif.mib2_counters.ifoutdiscards = 110U;
    gnetif.mib2_counters.ifouterrors = 111U;

    lwip_stats.mem.err = 201U;
    lwip_stats.mem.avail = 202U;
    lwip_stats.mem.used = 203U;
    lwip_stats.mem.max = 204U;
    lwip_stats.mem.illegal = 205U;

    for (uint32_t index = 0U; index < MEMP_MAX; ++index) {
        test_memp_stats[index].used = 300U + index;
        test_memp_stats[index].max = 400U + index;
        test_memp_stats[index].err = 500U + index;
        test_memp_stats[index].illegal = 600U + index;
        test_memp_descriptors[index].num = (uint16_t)(700U + index);
        test_memp_descriptors[index].size = (uint16_t)(800U + index);
        lwip_stats.memp[index] = &test_memp_stats[index];
        memp_pools[index] = &test_memp_descriptors[index];
    }

    lwip_stats.sys.sem.used = 901U;
    lwip_stats.sys.sem.max = 902U;
    lwip_stats.sys.sem.err = 903U;
    lwip_stats.sys.mutex.used = 904U;
    lwip_stats.sys.mutex.max = 905U;
    lwip_stats.sys.mutex.err = 906U;
    lwip_stats.sys.mbox.used = 907U;
    lwip_stats.sys.mbox.max = 908U;
    lwip_stats.sys.mbox.err = 909U;

    fill_protocol_stats(&lwip_stats.link, 1000U);
    fill_protocol_stats(&lwip_stats.etharp, 1100U);
    fill_protocol_stats(&lwip_stats.ip, 1200U);
    fill_protocol_stats(&lwip_stats.icmp, 1300U);
    fill_protocol_stats(&lwip_stats.tcp, 1400U);
    fill_mib2_stats(&lwip_stats.mib2, 1500U);
}

static void test_network_frame_header_contract(void) {
    NetworkFrameHeaderDTO_t header = {
        .magic = NETWORK_PROTOCOL_MAGIC,
        .version = NETWORK_PROTOCOL_VERSION,
        .message_type = NETWORK_MESSAGE_TYPE_DIAGNOSTIC_SNAPSHOT,
        .header_length = sizeof(NetworkFrameHeaderDTO_t),
        .payload_length = EXPECTED_DIAGNOSTIC_PAYLOAD_SIZE,
    };
    uint8_t serialized_header[sizeof(header)] = {0};

    (void)memcpy(serialized_header, &header, sizeof(header));

    expect_u32(sizeof(NetworkFrameHeaderDTO_t), 8U, "network frame header size");
    expect_u32(sizeof(DiagnosticPayloadDTO_t), EXPECTED_DIAGNOSTIC_PAYLOAD_SIZE, "diagnostic payload size");
    expect_u32(sizeof(DiagnosticFrameDTO_t), EXPECTED_DIAGNOSTIC_FRAME_SIZE, "diagnostic frame size");
    expect_u32(header.magic, 0x4447U, "network protocol magic");
    expect_u32(header.version, 1U, "network protocol version");
    expect_u32(header.message_type, 1U, "diagnostic snapshot message type");
    expect_u32(header.header_length, 8U, "header_length");
    expect_u32(header.payload_length, EXPECTED_DIAGNOSTIC_PAYLOAD_SIZE, "payload_length");
    expect_u32(sizeof(header) + header.payload_length, EXPECTED_DIAGNOSTIC_FRAME_SIZE, "complete frame size");

    uint16_t little_endian_probe = 1U;
    const uint8_t *probe_bytes = (const uint8_t *)&little_endian_probe;
    expect_true(probe_bytes[0] == 1U, "host test must run on a little-endian target");
    expect_u32(serialized_header[0], 0x47U, "magic low byte");
    expect_u32(serialized_header[1], 0x44U, "magic high byte");
    expect_u32(serialized_header[4], 0x08U, "header_length low byte");
    expect_u32(serialized_header[5], 0x00U, "header_length high byte");
}

static void test_udp_packet_counter(void) {
    expect_u32(lwip_udp_diagnostics_get_packet_count(), 0U, "UDP packet counter initial value");

    lwip_udp_diagnostics_record_packet();
    expect_u32(lwip_udp_diagnostics_get_packet_count(), 1U, "UDP packet counter after first packet");

    lwip_udp_diagnostics_record_packet();
    expect_u32(lwip_udp_diagnostics_get_packet_count(), 2U, "UDP packet counter after second packet");
}

static void test_lwip_diagnostics_collection(void) {
    LwipDiagnostics diagnostics;
    uint32_t expected_memp_used = 0U;
    uint32_t expected_memp_max = 0U;
    uint32_t expected_memp_errors = 0U;
    uint32_t expected_memp_illegal = 0U;

    prepare_lwip_fixture();
    (void)memset(&diagnostics, 0, sizeof(diagnostics));
    lwip_diagnostics_collect(NULL);
    lwip_diagnostics_collect(&diagnostics);

    expect_u32(diagnostics.netif_up, 1U, "netif_up");
    expect_u32(diagnostics.link_up, 1U, "link_up");
    expect_u32(diagnostics.flags, gnetif.flags, "netif flags");
    expect_u32(diagnostics.mtu, gnetif.mtu, "MTU");
    expect_u32(diagnostics.ip_address, gnetif.ip_addr.addr, "IP address");
    expect_u32(diagnostics.netmask, gnetif.netmask.addr, "netmask");
    expect_u32(diagnostics.gateway, gnetif.gw.addr, "gateway");
    expect_u32(diagnostics.netif_link_type, gnetif.link_type, "link type");
    expect_u32(diagnostics.netif_link_speed, gnetif.link_speed, "link speed");
    expect_u32(diagnostics.netif_timestamp, gnetif.ts, "netif timestamp");

    expect_u32(diagnostics.mem_err, 201U, "LwIP memory errors");
    expect_u32(diagnostics.mem_available, 202U, "LwIP memory available");
    expect_u32(diagnostics.mem_used, 203U, "LwIP memory used");
    expect_u32(diagnostics.mem_max_used, 204U, "LwIP maximum memory used");
    expect_u32(diagnostics.mem_illegal, 205U, "LwIP illegal memory operations");

    for (uint32_t index = 0U; index < MEMP_MAX; ++index) {
        expected_memp_used += test_memp_stats[index].used;
        expected_memp_max += test_memp_stats[index].max;
        expected_memp_errors += test_memp_stats[index].err;
        expected_memp_illegal += test_memp_stats[index].illegal;
        expect_u32(diagnostics.memp[index].used, test_memp_stats[index].used, "memp used");
        expect_u32(diagnostics.memp[index].max, test_memp_stats[index].max, "memp max");
        expect_u32(diagnostics.memp[index].err, test_memp_stats[index].err, "memp errors");
        expect_u32(diagnostics.memp[index].illegal, test_memp_stats[index].illegal, "memp illegal");
        expect_u32(diagnostics.memp[index].capacity, test_memp_descriptors[index].num, "memp capacity");
        expect_u32(diagnostics.memp[index].element_size, test_memp_descriptors[index].size, "memp element size");
    }

    expect_u32(diagnostics.memp_used, expected_memp_used, "total memp used");
    expect_u32(diagnostics.memp_max_used, expected_memp_max, "total memp maximum used");
    expect_u32(diagnostics.memp_errors, expected_memp_errors, "total memp errors");
    expect_u32(diagnostics.memp_illegal, expected_memp_illegal, "total memp illegal operations");

    expect_u32(diagnostics.semaphores.used, 901U, "semaphore usage");
    expect_u32(diagnostics.semaphores.max, 902U, "semaphore maximum usage");
    expect_u32(diagnostics.semaphores.err, 903U, "semaphore errors");
    expect_u32(diagnostics.mutexes.used, 904U, "mutex usage");
    expect_u32(diagnostics.mutexes.max, 905U, "mutex maximum usage");
    expect_u32(diagnostics.mutexes.err, 906U, "mutex errors");
    expect_u32(diagnostics.mailboxes.used, 907U, "mailbox usage");
    expect_u32(diagnostics.mailboxes.max, 908U, "mailbox maximum usage");
    expect_u32(diagnostics.mailboxes.err, 909U, "mailbox errors");

    expect_protocol_stats(&diagnostics.link, &lwip_stats.link, "link protocol statistics");
    expect_protocol_stats(&diagnostics.etharp, &lwip_stats.etharp, "ARP protocol statistics");
    expect_protocol_stats(&diagnostics.ip, &lwip_stats.ip, "IP protocol statistics");
    expect_protocol_stats(&diagnostics.icmp, &lwip_stats.icmp, "ICMP protocol statistics");
    expect_protocol_stats(&diagnostics.tcp, &lwip_stats.tcp, "TCP protocol statistics");
    expect_true(memcmp(&diagnostics.mib2, &lwip_stats.mib2, sizeof(diagnostics.mib2)) == 0, "MIB2 statistics");

    expect_u32(diagnostics.tcpip_thread_stack_size, 1024U, "TCP/IP thread stack size");
    expect_u32(diagnostics.tcpip_thread_priority, 24U, "TCP/IP thread priority");
    expect_u32(diagnostics.tcpip_mbox_size, 6U, "TCP/IP mailbox size");
    expect_u32(diagnostics.tcp_mss, 1460U, "TCP MSS");
    expect_u32(diagnostics.tcp_snd_buf, 5840U, "TCP send buffer");
    expect_u32(diagnostics.tcp_wnd, 5840U, "TCP receive window");
    expect_u32(diagnostics.tcp_snd_queuelen, 16U, "TCP send queue length");
    expect_u32(diagnostics.pbuf_pool_size, 16U, "pbuf pool size");
    expect_u32(diagnostics.pbuf_pool_bufsize, 1536U, "pbuf pool buffer size");
    expect_u32(diagnostics.udp_packets_count, 2U, "UDP packet count in diagnostics");

    expect_u32(diagnostics.if_in_octets, 101U, "interface input octets");
    expect_u32(diagnostics.if_in_unicast_packets, 102U, "interface input unicast packets");
    expect_u32(diagnostics.if_in_non_unicast_packets, 103U, "interface input non-unicast packets");
    expect_u32(diagnostics.if_in_discards, 104U, "interface input discards");
    expect_u32(diagnostics.if_in_errors, 105U, "interface input errors");
    expect_u32(diagnostics.if_in_unknown_protocols, 106U, "interface input unknown protocols");
    expect_u32(diagnostics.if_out_octets, 107U, "interface output octets");
    expect_u32(diagnostics.if_out_unicast_packets, 108U, "interface output unicast packets");
    expect_u32(diagnostics.if_out_non_unicast_packets, 109U, "interface output non-unicast packets");
    expect_u32(diagnostics.if_out_discards, 110U, "interface output discards");
    expect_u32(diagnostics.if_out_errors, 111U, "interface output errors");

    lwip_stats.memp[0] = NULL;
    lwip_diagnostics_collect(&diagnostics);
    expect_u32(diagnostics.memp[0].used, 0U, "empty memp pool used");
    expect_u32(diagnostics.memp[0].max, 0U, "empty memp pool maximum used");
    expect_u32(diagnostics.memp[0].err, 0U, "empty memp pool errors");
    expect_u32(diagnostics.memp[0].illegal, 0U, "empty memp pool illegal operations");
    expect_u32(diagnostics.memp[0].capacity, test_memp_descriptors[0].num, "empty memp pool capacity");
    expect_u32(diagnostics.memp[0].element_size, test_memp_descriptors[0].size, "empty memp pool element size");
}

static void expect_memory_region_invariant(const MemoryRegionDiagnostics *region, const char *name) {
    expect_u32(region->used_bytes + region->reserved_bytes + region->free_bytes, region->total_bytes, name);
}

static void test_system_diagnostics_collection(void) {
    SystemDiagnostics diagnostics;

    (void)memset(&test_dbgmcu, 0, sizeof(test_dbgmcu));
    (void)memset(&test_rcc, 0, sizeof(test_rcc));
    (void)memset(&test_scb, 0, sizeof(test_scb));
    (void)memset(&test_mpu, 0, sizeof(test_mpu));
    (void)memset(&test_dwt, 0, sizeof(test_dwt));
    (void)memset(&test_systick, 0, sizeof(test_systick));
    (void)memset(&test_nvic, 0, sizeof(test_nvic));

    test_free_heap = 19464U;
    test_minimum_free_heap = 18000U;
    test_task_count = 7U;
    test_snapshot_count = 7U;
    test_total_runtime_ticks = 1000U;
    test_idle_runtime_ticks = 700U;
    test_scheduler_state = 2U;
    test_tick_count = 5007U;
    test_stack_overflow_count = 3U;
    test_malloc_failed_count = 4U;
    test_ipsr = 5U;
    test_control = 6U;
    test_basepri = 7U;
    test_primask = 8U;
    test_faultmask = 9U;
    test_msp = 10U;
    test_psp = 11U;
    SystemCoreClock = 550000000U;

    test_dbgmcu.IDCODE = 0x12345678U;
    test_rcc.RSR = 12U;
    test_scb.CFSR = 13U;
    test_scb.HFSR = 14U;
    test_scb.DFSR = 15U;
    test_scb.MMFAR = 16U;
    test_scb.BFAR = 17U;
    test_scb.AFSR = 18U;
    test_scb.SHCSR = 19U;
    test_scb.CCR = 20U;
    test_scb.ICSR = 21U;
    test_scb.VTOR = 22U;
    test_mpu.CTRL = 23U;
    test_dwt.CTRL = 24U;
    test_dwt.CYCCNT = 25U;
    test_systick.CTRL = 26U;
    test_systick.LOAD = 27U;
    test_systick.VAL = 28U;
    test_nvic.ISER[0] = 29U;
    test_nvic.ISPR[0] = 30U;
    test_nvic.IABR[0] = 31U;

    (void)memset(&diagnostics, 0, sizeof(diagnostics));
    system_diagnostics_collect(NULL);
    system_diagnostics_collect(&diagnostics);

    expect_u32(diagnostics.free_heap, test_free_heap, "system free heap");
    expect_u32(diagnostics.minimum_free_heap, test_minimum_free_heap, "system minimum free heap");
    expect_u32(diagnostics.uid0, 0x10000001U, "MCU UID0");
    expect_u32(diagnostics.uid1, 0x10000002U, "MCU UID1");
    expect_u32(diagnostics.uid2, 0x10000003U, "MCU UID2");
    expect_u32(diagnostics.device_id, 0x678U, "MCU device ID");
    expect_u32(diagnostics.revision_id, 0x1234U, "MCU revision ID");
    expect_u32(diagnostics.reset_flags, 12U, "reset flags");
    expect_u32(diagnostics.sys_clock_hz, 550000000U, "system clock");
    expect_u32(diagnostics.hclk_hz, 275000000U, "HCLK");
    expect_u32(diagnostics.pclk1_hz, 137500000U, "PCLK1");
    expect_u32(diagnostics.pclk2_hz, 137500000U, "PCLK2");
    expect_u32(diagnostics.systick_ms, 5011U, "SysTick milliseconds");
    expect_u32(diagnostics.scb_cfsr, 13U, "SCB CFSR");
    expect_u32(diagnostics.scb_hfsr, 14U, "SCB HFSR");
    expect_u32(diagnostics.scb_dfsr, 15U, "SCB DFSR");
    expect_u32(diagnostics.scb_mmfar, 16U, "SCB MMFAR");
    expect_u32(diagnostics.scb_bfar, 17U, "SCB BFAR");
    expect_u32(diagnostics.scb_afsr, 18U, "SCB AFSR");
    expect_u32(diagnostics.scb_shcsr, 19U, "SCB SHCSR");
    expect_u32(diagnostics.scb_ccr, 20U, "SCB CCR");
    expect_u32(diagnostics.scb_icsr, 21U, "SCB ICSR");
    expect_u32(diagnostics.scb_vtor, 22U, "SCB VTOR");
    expect_u32(diagnostics.mpu_ctrl, 23U, "MPU CTRL");
    expect_u32(diagnostics.dwt_ctrl, 24U, "DWT CTRL");
    expect_u32(diagnostics.dwt_cyccnt, 25U, "DWT CYCCNT");
    expect_u32(diagnostics.ipsr, test_ipsr, "IPSR");
    expect_u32(diagnostics.control, test_control, "CONTROL");
    expect_u32(diagnostics.basepri, test_basepri, "BASEPRI");
    expect_u32(diagnostics.primask, test_primask, "PRIMASK");
    expect_u32(diagnostics.faultmask, test_faultmask, "FAULTMASK");
    expect_u32(diagnostics.msp, test_msp, "MSP");
    expect_u32(diagnostics.psp, test_psp, "PSP");
    expect_u32(diagnostics.systick_ctrl, 26U, "SysTick CTRL");
    expect_u32(diagnostics.systick_load, 27U, "SysTick LOAD");
    expect_u32(diagnostics.systick_val, 28U, "SysTick VAL");
    expect_u32(diagnostics.nvic_iser0, 29U, "NVIC ISER0");
    expect_u32(diagnostics.nvic_ispr0, 30U, "NVIC ISPR0");
    expect_u32(diagnostics.nvic_iabr0, 31U, "NVIC IABR0");

    expect_u32(diagnostics.memory.itcm.base_address, 0x00000000U, "ITCM base");
    expect_u32(diagnostics.memory.itcm.total_bytes, 0x00010000U, "ITCM size");
    expect_u32(diagnostics.memory.itcm.used_bytes, 0U, "ITCM used");
    expect_u32(diagnostics.memory.itcm.reserved_bytes, 0U, "ITCM reserved");
    expect_u32(diagnostics.memory.itcm.free_bytes, 0x00010000U, "ITCM free");
    expect_u32(diagnostics.memory.dtcm.base_address, 0x20000000U, "DTCM base");
    expect_u32(diagnostics.memory.dtcm.total_bytes, 0x00020000U, "DTCM size");
    expect_u32(diagnostics.memory.d3.base_address, 0x38000000U, "D3 base");
    expect_u32(diagnostics.memory.d3.total_bytes, 0x00004000U, "D3 size");
    expect_memory_region_invariant(&diagnostics.memory.d1, "D1 memory conservation");
    expect_memory_region_invariant(&diagnostics.memory.d2, "D2 memory conservation");
    expect_memory_region_invariant(&diagnostics.memory.d3, "D3 memory conservation");

    expect_u32(diagnostics.freertos.total_heap, 32768U, "FreeRTOS total heap");
    expect_u32(diagnostics.freertos.used_heap, 13304U, "FreeRTOS used heap");
    expect_u32(diagnostics.freertos.free_heap, test_free_heap, "FreeRTOS free heap");
    expect_u32(diagnostics.freertos.minimum_free_heap, test_minimum_free_heap, "FreeRTOS minimum heap");
    expect_u32(diagnostics.freertos.task_count, test_task_count, "FreeRTOS task count");
    expect_u32(diagnostics.freertos.task_snapshot_count, test_snapshot_count, "FreeRTOS snapshot count");
    expect_u32(diagnostics.freertos.task_snapshot_capacity, 32U, "FreeRTOS snapshot capacity");
    expect_u32(diagnostics.freertos.task_snapshot_truncated, 0U, "FreeRTOS snapshot truncation");
    expect_u32(diagnostics.freertos.scheduler_state, test_scheduler_state, "FreeRTOS scheduler state");
    expect_u32(diagnostics.freertos.tick_count, test_tick_count, "FreeRTOS tick count");
    expect_u32(diagnostics.freertos.runtime_counter_hz, SystemCoreClock, "FreeRTOS runtime counter frequency");
    expect_u32(diagnostics.freertos.total_runtime_ticks, test_total_runtime_ticks, "FreeRTOS total runtime");
    expect_u32(diagnostics.freertos.idle_runtime_ticks, test_idle_runtime_ticks, "FreeRTOS idle runtime");
    expect_u32(diagnostics.freertos.cpu_usage_percent, 0U, "initial CPU usage");
    expect_u32(diagnostics.freertos.stack_overflow_count, test_stack_overflow_count, "stack overflow count");
    expect_u32(diagnostics.freertos.malloc_failed_count, test_malloc_failed_count, "malloc failed count");

    test_task_count = 40U;
    test_snapshot_count = 32U;
    test_total_runtime_ticks = 2000U;
    test_idle_runtime_ticks = 1200U;
    system_diagnostics_collect(&diagnostics);
    expect_u32(diagnostics.freertos.task_snapshot_truncated, 1U, "truncated FreeRTOS snapshot");
    expect_u32(diagnostics.freertos.cpu_usage_percent, 50U, "CPU usage between samples");

    test_idle_task_handle = NULL;
    test_total_runtime_ticks = 3000U;
    test_idle_runtime_ticks = 0U;
    system_diagnostics_collect(&diagnostics);
    expect_u32(diagnostics.freertos.idle_runtime_ticks, 0U, "missing idle task runtime");
    expect_u32(diagnostics.freertos.cpu_usage_percent, 0U, "CPU usage without idle task");
}

int main(void) {
    test_network_frame_header_contract();
    test_udp_packet_counter();
    test_lwip_diagnostics_collection();
    test_system_diagnostics_collection();

    if (test_failures != 0U) {
        (void)fprintf(stderr, "Diagnostic protocol tests failed: %u\n", test_failures);
        return 1;
    }

    (void)printf("Diagnostic protocol tests passed.\n");
    return 0;
}
