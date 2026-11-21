/**
 * @file test_claim.c
 * @brief The claim model against the fake chip of test_request.c: the
 *        EXCLUSIVE holder that claims again, the firmware update's
 *        reservation, the count of tasks that wait. Split out of
 *        test_request.c on 2026-11-21 (700-line rule); the chip model stays
 *        there and is reached through rq_engine_up(), rq_wire_clear() and
 *        rq_wire().
 */
#include "obd_chip.h"

#include <string.h>

#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "obd_chip_private.h"

#define T_AT pdMS_TO_TICKS(500)
#define T_SHORT pdMS_TO_TICKS(60)

void rq_engine_up(void);
void rq_wire_clear(void);
const char *rq_wire(void);

/* The start-up check's order (2026-11-11): the bring-up task takes EXCLUSIVE
 * before it opens the bring-up gate, and the firmware check it then runs
 * claims again, as every caller of the update engine does. The holder is let
 * in; anybody else is refused while it holds; one release ends the hold. */
static volatile esp_err_t s_other_claim;

static void other_task_claims_exclusive(void *arg)
{
    (void)arg;
    s_other_claim = obd_chip_claim(OBD_CHIP_CLAIM_EXCLUSIVE, 0);
    vTaskDelete(NULL);
}

void test_claim_exclusive_holder_claims_again(void)
{
    rq_engine_up();
    TEST_ASSERT_EQUAL(ESP_OK, obd_chip_claim(OBD_CHIP_CLAIM_EXCLUSIVE, 0));
    TEST_ASSERT_EQUAL(ESP_OK, obd_chip_claim(OBD_CHIP_CLAIM_EXCLUSIVE, 0));
    TEST_ASSERT_EQUAL(3, obd_core_claim_state());

    /* another task is not the holder: refused, not let in */
    s_other_claim = ESP_FAIL;
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(other_task_claims_exclusive, "other",
                                          4096, NULL, 5, NULL));

    for (int i = 0; i < 100 && s_other_claim == ESP_FAIL; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, s_other_claim);

    /* a request meanwhile is refused too (the first requester of a boot
       waits its 2 s for the check; here the wait is the short one) */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      obd_chip_claim(OBD_CHIP_CLAIM_COMMAND, T_SHORT));
    TEST_ASSERT_EQUAL_STRING("", rq_wire());

    /* claimed twice, released once: the chip is free and remembers no
       holder (the task that held it is not let in over another claim) */
    TEST_ASSERT_EQUAL(ESP_OK, obd_chip_release());
    TEST_ASSERT_EQUAL(0, obd_core_claim_state());
    TEST_ASSERT_EQUAL(ESP_OK, obd_chip_claim(OBD_CHIP_CLAIM_MONITOR, 0));
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT,
                      obd_chip_claim(OBD_CHIP_CLAIM_EXCLUSIVE, 0));
    TEST_ASSERT_EQUAL(ESP_OK, obd_chip_release());
}

/* The firmware update's reservation (2026-11-21, issue #1075): the update's
 * task keeps the chip for itself from before its claim. The holder of the
 * moment ends its transaction (its nested request is still granted), then
 * everybody but the update's task is refused at once, a bridge's send
 * included, with nothing on the wire; the update's own claim is granted;
 * given back, the chip is everybody's again. The other task is the update. */
static volatile int       s_resv_step;
static volatile esp_err_t s_resv_claim;

static void resv_wait(int step)
{
    /* one tick at least: 5 ms is no tick at all on the host's 100 Hz */
    for (int i = 0; i < 300 && s_resv_step < step; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void other_task_reserves(void *arg)
{
    (void)arg;
    obd_core_reserve(true);
    s_resv_step = 1;
    resv_wait(2);
    s_resv_claim = obd_chip_claim(OBD_CHIP_CLAIM_EXCLUSIVE, T_AT);
    s_resv_step = 3;
    resv_wait(4);
    (void)obd_chip_release();
    obd_core_reserve(false);
    s_resv_step = 5;
    vTaskDelete(NULL);
}

void test_claim_reserved_for_the_update(void)
{
    char resp[64];

    rq_engine_up();
    TEST_ASSERT_FALSE(obd_core_reserved());
    TEST_ASSERT_EQUAL(ESP_OK, obd_chip_txn_begin(T_AT));

    s_resv_step = 0;
    s_resv_claim = ESP_FAIL;
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(other_task_reserves, "update", 4096,
                                          NULL, 5, NULL));
    resv_wait(1);
    TEST_ASSERT_EQUAL(1, s_resv_step);
    TEST_ASSERT_TRUE(obd_core_reserved());

    /* the holder ends what it began */
    TEST_ASSERT_EQUAL(ESP_OK,
                      obd_chip_request("ATI", resp, sizeof(resp), T_AT));
    TEST_ASSERT_EQUAL_STRING("ATI\r", rq_wire());
    obd_chip_txn_end();
    TEST_ASSERT_EQUAL(0, obd_core_claim_state());

    /* and gets nothing new: refused at once, not at the end of its wait */
    rq_wire_clear();

    TickType_t t0 = xTaskGetTickCount();

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      obd_chip_claim(OBD_CHIP_CLAIM_COMMAND, T_AT));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      obd_chip_claim(OBD_CHIP_CLAIM_MONITOR, T_AT));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      obd_chip_request("0100", resp, sizeof(resp), T_AT));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, obd_chip_txn_begin(T_AT));
    TEST_ASSERT_TRUE(xTaskGetTickCount() - t0 < pdMS_TO_TICKS(200));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      obd_chip_send((const uint8_t *)"ATI\r", 4));
    TEST_ASSERT_EQUAL_STRING("", rq_wire());

    /* the update's own claim is granted */
    s_resv_step = 2;
    resv_wait(3);
    TEST_ASSERT_EQUAL(ESP_OK, s_resv_claim);
    TEST_ASSERT_EQUAL(3, obd_core_claim_state());

    /* released and given back: the chip is everybody's again */
    s_resv_step = 4;
    resv_wait(5);
    TEST_ASSERT_EQUAL(5, s_resv_step);
    TEST_ASSERT_FALSE(obd_core_reserved());
    TEST_ASSERT_EQUAL(ESP_OK,
                      obd_chip_request("0100", resp, sizeof(resp), T_AT));
    TEST_ASSERT_EQUAL(0, obd_core_claim_state());
}

/* A task that waits for the chip is counted while it waits (2026-11-21): a
 * requester that claims without a pause reads the count and leaves a gap. */
static volatile int       s_wait_step;
static volatile esp_err_t s_wait_claim;

static void other_task_waits(void *arg)
{
    (void)arg;
    s_wait_step = 1;
    s_wait_claim = obd_chip_claim(OBD_CHIP_CLAIM_COMMAND, pdMS_TO_TICKS(2000));
    s_wait_step = 2;
    (void)obd_chip_release();
    s_wait_step = 3;
    vTaskDelete(NULL);
}

void test_claim_waiters_counted(void)
{
    rq_engine_up();
    TEST_ASSERT_EQUAL(0, obd_chip_waiters());
    TEST_ASSERT_EQUAL(ESP_OK, obd_chip_txn_begin(T_AT));

    s_wait_step = 0;
    s_wait_claim = ESP_FAIL;
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(other_task_waits, "waiter", 4096,
                                          NULL, 5, NULL));

    /* it found the chip taken: counted from its first look on */
    for (int i = 0; i < 100 && obd_chip_waiters() == 0; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TEST_ASSERT_EQUAL(1, obd_chip_waiters());
    TEST_ASSERT_EQUAL(1, s_wait_step);

    /* the holder lets go: the waiter gets the chip at its next look and is
       no waiter any more */
    obd_chip_txn_end();

    for (int i = 0; i < 100 && s_wait_step < 3; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TEST_ASSERT_EQUAL(3, s_wait_step);
    TEST_ASSERT_EQUAL(ESP_OK, s_wait_claim);
    TEST_ASSERT_EQUAL(0, obd_chip_waiters());

    /* a claim granted at its first look was never counted */
    TEST_ASSERT_EQUAL(ESP_OK, obd_chip_claim(OBD_CHIP_CLAIM_COMMAND, T_AT));
    TEST_ASSERT_EQUAL(0, obd_chip_waiters());
    (void)obd_chip_release();
    TEST_ASSERT_EQUAL(0, obd_core_claim_state());
}
