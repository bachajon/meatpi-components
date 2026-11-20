/**
 * @file test_suites.h
 * @brief The cases of the test files beside test_main.c, each run from its
 *        app_main in this order.
 */
#pragma once

void run_voltage_tests(void);   /* test_voltage.c: its four cases */
void run_iso_cfg_tests(void);   /* test_iso_cfg.c: its five cases */
void run_session_tests(void);   /* test_session.c: its six cases */
void run_iso_ae_tests(void);    /* test_iso_ae.c: its six cases */
