/**
 * @file test_main.c
 * @brief Host tests for http_client_manager's pure helpers: URL
 *        validation, base64 (RFC 4648 vectors), Bearer/Basic building,
 *        header splitting, the redirect rule, the class of an address a
 *        request went to, and why a request failed (the reason and the code
 *        for support). Expected: 9 Tests 0 Failures 0 Ignored.
 */
#include <string.h>

#include "unity.h"

#include "http_client_manager.h"
#include "http_client_manager_private.h"

void setUp(void)
{
}

void tearDown(void)
{
}

static void test_url_validation(void)
{
    TEST_ASSERT_TRUE(hc_url_valid("http://10.42.0.1:8080/x"));
    TEST_ASSERT_TRUE(hc_url_valid("https://api.example.com/v1"));
    TEST_ASSERT_FALSE(hc_url_valid("ftp://x"));
    TEST_ASSERT_FALSE(hc_url_valid("http://"));
    TEST_ASSERT_FALSE(hc_url_valid(NULL));
    TEST_ASSERT_TRUE(hc_url_is_tls("https://x.y"));
    TEST_ASSERT_FALSE(hc_url_is_tls("http://x.y"));
}

static void test_base64_rfc_vectors(void)
{
    char out[32];

    TEST_ASSERT_EQUAL_size_t(4,
        hc_base64((const uint8_t *)"f", 1, out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("Zg==", out);
    hc_base64((const uint8_t *)"fo", 2, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("Zm8=", out);
    hc_base64((const uint8_t *)"foo", 3, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("Zm9v", out);
    hc_base64((const uint8_t *)"foobar", 6, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("Zm9vYmFy", out);
    /* capacity gate */
    TEST_ASSERT_EQUAL_size_t(0,
        hc_base64((const uint8_t *)"foobar", 6, out, 8));
}

static void test_bearer(void)
{
    char out[64];

    TEST_ASSERT_TRUE(hc_auth_bearer("tok123", out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("Bearer tok123", out);
    TEST_ASSERT_FALSE(hc_auth_bearer("", out, sizeof(out)));
    TEST_ASSERT_FALSE(hc_auth_bearer("way-too-long", out, 8));
}

static void test_basic_rfc7617_vector(void)
{
    char out[64];

    TEST_ASSERT_TRUE(hc_auth_basic("Aladdin", "open sesame", out,
                                   sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("Basic QWxhZGRpbjpvcGVuIHNlc2FtZQ==", out);
    TEST_ASSERT_FALSE(hc_auth_basic(NULL, "x", out, sizeof(out)));
}

static void test_header_split(void)
{
    char key[64];
    const char *val;

    val = hc_header_split("X-Custom: hello world", key, sizeof(key));
    TEST_ASSERT_NOT_NULL(val);
    TEST_ASSERT_EQUAL_STRING("X-Custom", key);
    TEST_ASSERT_EQUAL_STRING("hello world", val);
    TEST_ASSERT_NULL(hc_header_split("no-colon-here", key, sizeof(key)));
    TEST_ASSERT_NULL(hc_header_split(": empty-key", key, sizeof(key)));
    TEST_ASSERT_NULL(hc_header_split("Key:", key, sizeof(key)));
}

static void test_redirect_rule(void)
{
    /* the GitHub release asset: 302 to the signed CDN address */
    TEST_ASSERT_TRUE(hc_redirect_follow(302, true, 0, HC_MAX_REDIRECTS));
    TEST_ASSERT_TRUE(hc_redirect_follow(301, true, 0, HC_MAX_REDIRECTS));
    TEST_ASSERT_TRUE(hc_redirect_follow(303, true, 1, HC_MAX_REDIRECTS));
    TEST_ASSERT_TRUE(hc_redirect_follow(307, true, 2, HC_MAX_REDIRECTS));
    TEST_ASSERT_TRUE(hc_redirect_follow(308, true, 2, HC_MAX_REDIRECTS));
    /* three hops, then the 3xx is the answer */
    TEST_ASSERT_FALSE(hc_redirect_follow(302, true, 3, HC_MAX_REDIRECTS));
    /* not a redirect */
    TEST_ASSERT_FALSE(hc_redirect_follow(200, true, 0, HC_MAX_REDIRECTS));
    TEST_ASSERT_FALSE(hc_redirect_follow(304, true, 0, HC_MAX_REDIRECTS));
    TEST_ASSERT_FALSE(hc_redirect_follow(404, true, 0, HC_MAX_REDIRECTS));
    /* a POST keeps its 3xx */
    TEST_ASSERT_FALSE(hc_redirect_follow(302, false, 0, HC_MAX_REDIRECTS));
}

static bool ip4_private(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    const uint8_t addr[4] = { a, b, c, d };

    return hc_ip4_private(addr);
}

/* where a request went (2026-11-14): ha_webhooks takes settings from a
   push's reply over plain http only from an address that is not routed on
   the internet */
static void test_private_addresses(void)
{
    /* the home LAN ranges, edge to edge */
    TEST_ASSERT_TRUE(ip4_private(10, 0, 0, 1));
    TEST_ASSERT_TRUE(ip4_private(10, 255, 255, 254));
    TEST_ASSERT_TRUE(ip4_private(172, 16, 0, 1));
    TEST_ASSERT_TRUE(ip4_private(172, 31, 255, 254));
    TEST_ASSERT_FALSE(ip4_private(172, 15, 0, 1));
    TEST_ASSERT_FALSE(ip4_private(172, 32, 0, 1));
    TEST_ASSERT_TRUE(ip4_private(192, 168, 1, 10));
    TEST_ASSERT_FALSE(ip4_private(192, 169, 1, 10));
    TEST_ASSERT_FALSE(ip4_private(193, 168, 1, 10));
    /* carrier NAT and the VPN tunnels' range, 100.64/10 */
    TEST_ASSERT_TRUE(ip4_private(100, 64, 0, 1));
    TEST_ASSERT_TRUE(ip4_private(100, 127, 255, 254));
    TEST_ASSERT_FALSE(ip4_private(100, 63, 0, 1));
    TEST_ASSERT_FALSE(ip4_private(100, 128, 0, 1));
    /* link-local */
    TEST_ASSERT_TRUE(ip4_private(169, 254, 10, 10));
    /* the internet, the documentation ranges, loopback, nothing */
    TEST_ASSERT_FALSE(ip4_private(8, 8, 8, 8));
    TEST_ASSERT_FALSE(ip4_private(203, 0, 113, 1));
    TEST_ASSERT_FALSE(ip4_private(127, 0, 0, 1));
    TEST_ASSERT_FALSE(ip4_private(0, 0, 0, 0));

    /* IPv6: unique-local and link-local, an IPv4 peer on a v6 socket by
       the IPv4 rule, a global address not */
    const uint8_t ula[16] = { 0xfd, 0x12, 0x34 };
    const uint8_t ula_c[16] = { 0xfc, 0x00 };
    const uint8_t ll[16] = { 0xfe, 0x80, 0, 0, 0, 0, 0, 0, 1 };
    const uint8_t site[16] = { 0xfe, 0xc0 };
    const uint8_t global[16] = { 0x20, 0x01, 0x0d, 0xb8 };
    const uint8_t mapped_lan[16] =
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff, 192, 168, 1, 10 };
    const uint8_t mapped_pub[16] =
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff, 8, 8, 8, 8 };
    const uint8_t loop[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

    TEST_ASSERT_TRUE(hc_ip6_private(ula));
    TEST_ASSERT_TRUE(hc_ip6_private(ula_c));
    TEST_ASSERT_TRUE(hc_ip6_private(ll));
    TEST_ASSERT_FALSE(hc_ip6_private(site));
    TEST_ASSERT_FALSE(hc_ip6_private(global));
    TEST_ASSERT_TRUE(hc_ip6_private(mapped_lan));
    TEST_ASSERT_FALSE(hc_ip6_private(mapped_pub));
    TEST_ASSERT_FALSE(hc_ip6_private(loop));
}

#define CERT_NAME_FLAG    0x0004u /* MBEDTLS_X509_BADCERT_CN_MISMATCH */
#define CERT_UNTRUST_FLAG 0x0008u /* MBEDTLS_X509_BADCERT_NOT_TRUSTED */

static http_client_fail_reason_t why(int tls_err, int tls_code,
                                     uint32_t flags, bool opened)
{
    const http_client_fail_t f =
    {
        .tls_err = tls_err,
        .tls_code = tls_code,
        .cert_flags = flags,
        .opened = opened,
    };

    return http_client_manager_fail_reason(ESP_FAIL, &f);
}

/* why a request failed (2026-11-22): every connection that does not open is
   ESP_ERR_HTTP_CONNECT; what esp-tls kept beside it tells the cases apart.
   The values are the ones the bench unit gave for each far end. */
static void test_fail_reason(void)
{
    const uint32_t none = HC_CERT_NO_RESULT;

    /* a request that went through has no reason, whatever is left over */
    const http_client_fail_t junk = { .tls_code = HC_MBED_CONN_EOF };

    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_NONE,
                      http_client_manager_fail_reason(ESP_OK, &junk));
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_OTHER,
                      http_client_manager_fail_reason(ESP_FAIL, NULL));

    /* before the handshake: the name, the connection */
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_DNS,
                      why(HC_TLS_ERR_RESOLVE, 0, 0, false));
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_NO_ANSWER,
                      why(HC_TLS_ERR_CONNECT, 0, 0, false));
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_NO_ANSWER,
                      why(HC_TLS_ERR_TIMEOUT, 0, 0, false));

    /* the field report: the far end closed during the handshake */
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_TLS_CLOSED,
                      why(HC_TLS_ERR_HANDSHAKE, HC_MBED_CONN_EOF, none, false));
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_TLS_CLOSED,
                      why(HC_TLS_ERR_HANDSHAKE, HC_MBED_NET_RESET, none, false));
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_TLS_REFUSED,
                      why(HC_TLS_ERR_HANDSHAKE, HC_MBED_FATAL_ALERT, none,
                          false));
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_NOT_TLS,
                      why(HC_TLS_ERR_HANDSHAKE, HC_MBED_INVALID_RECORD, none,
                          false));
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_NO_ANSWER,
                      why(HC_TLS_ERR_HANDSHAKE, HC_MBED_SSL_TIMEOUT, none,
                          false));
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_TLS_OTHER,
                      why(HC_TLS_ERR_HANDSHAKE, 0x7100, none, false));
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_TLS_OTHER,
                      why(HC_TLS_ERR_HANDSHAKE, 0, 0, false));

    /* the certificate: the bundle that finds no root ends with -0x3000;
       every other "no" of the check is -0x2700, and the flags that would
       say which (esp-tls gives none after a failed handshake: 0, as the
       bench unit had it) do not change the reason when they are there */
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_CERT_UNTRUSTED,
                      why(HC_TLS_ERR_HANDSHAKE, HC_MBED_X509_FATAL, 0, false));
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_CERT_UNTRUSTED,
                      why(HC_TLS_ERR_HANDSHAKE, HC_MBED_X509_FATAL, none,
                          false));
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_CERT,
                      why(HC_TLS_ERR_HANDSHAKE, HC_MBED_X509_VERIFY, 0, false));
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_CERT,
                      why(HC_TLS_ERR_HANDSHAKE, HC_MBED_X509_VERIFY, none,
                          false));
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_CERT,
                      why(HC_TLS_ERR_HANDSHAKE, HC_MBED_X509_VERIFY,
                          CERT_NAME_FLAG, false));
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_CERT,
                      why(HC_TLS_ERR_HANDSHAKE, HC_MBED_X509_VERIFY,
                          CERT_UNTRUST_FLAG, false));

    /* the connection was open: the request left, the answer did not come */
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_NO_REPLY, why(0, 0, 0, true));
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_NO_REPLY,
                      why(HC_TLS_ERR_HANDSHAKE, HC_MBED_CONN_EOF, none, true));
    /* nothing kept */
    TEST_ASSERT_EQUAL(HTTP_CLIENT_FAIL_OTHER, why(0, 0, 0, false));

    /* every reason has its own word, none empty but "none" */
    TEST_ASSERT_EQUAL_STRING(
        "", http_client_manager_fail_reason_str(HTTP_CLIENT_FAIL_NONE));

    for (int a = HTTP_CLIENT_FAIL_DNS; a <= HTTP_CLIENT_FAIL_OTHER; a++)
    {
        const char *wa = http_client_manager_fail_reason_str(a);

        TEST_ASSERT_TRUE(wa[0] != '\0');

        for (int b = a + 1; b <= HTTP_CLIENT_FAIL_OTHER; b++)
        {
            TEST_ASSERT_TRUE(
                strcmp(wa, http_client_manager_fail_reason_str(b)) != 0);
        }
    }

    TEST_ASSERT_EQUAL_STRING(
        "tls_closed",
        http_client_manager_fail_reason_str(HTTP_CLIENT_FAIL_TLS_CLOSED));
}

static void test_fail_code(void)
{
    char out[HTTP_CLIENT_FAIL_CODE_LEN];
    http_client_fail_t f = { .tls_err = HC_TLS_ERR_HANDSHAKE,
                             .tls_code = HC_MBED_CONN_EOF,
                             .cert_flags = HC_CERT_NO_RESULT };

    http_client_manager_fail_code(&f, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("tls -0x7280", out);

    /* the flags only where the certificate check gave some */
    f.tls_code = HC_MBED_X509_VERIFY;
    http_client_manager_fail_code(&f, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("tls -0x2700", out);
    f.cert_flags = CERT_UNTRUST_FLAG;
    http_client_manager_fail_code(&f, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("tls -0x2700 flags 0x8", out);
    f.cert_flags = CERT_UNTRUST_FLAG | CERT_NAME_FLAG;
    http_client_manager_fail_code(&f, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("tls -0x2700 flags 0xC", out);
    f.tls_code = HC_MBED_NET_RESET;
    http_client_manager_fail_code(&f, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("tls -0x0050", out);

    /* no TLS code: esp-tls's own, with the socket's errno when there is one */
    f = (http_client_fail_t){ .tls_err = HC_TLS_ERR_TIMEOUT };
    http_client_manager_fail_code(&f, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("esp-tls 0x8006", out);
    f = (http_client_fail_t){ .tls_err = HC_TLS_ERR_CONNECT, .sock_errno = 113 };
    http_client_manager_fail_code(&f, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("esp-tls 0x8004 errno 113", out);
    f = (http_client_fail_t){ .sock_errno = 11, .opened = true };
    http_client_manager_fail_code(&f, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("errno 11", out);

    /* nothing kept, no room, no struct */
    f = (http_client_fail_t){ 0 };
    strcpy(out, "x");
    http_client_manager_fail_code(&f, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("", out);
    strcpy(out, "x");
    http_client_manager_fail_code(NULL, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("", out);
    f = (http_client_fail_t){ .tls_code = HC_MBED_CONN_EOF };
    http_client_manager_fail_code(&f, out, 6);
    TEST_ASSERT_EQUAL_STRING("tls -", out);
    http_client_manager_fail_code(&f, out, 0); /* nothing written */
    TEST_ASSERT_EQUAL_STRING("tls -", out);
}

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_fail_reason);
    RUN_TEST(test_fail_code);
    RUN_TEST(test_url_validation);
    RUN_TEST(test_base64_rfc_vectors);
    RUN_TEST(test_bearer);
    RUN_TEST(test_basic_rfc7617_vector);
    RUN_TEST(test_header_split);
    RUN_TEST(test_redirect_rule);
    RUN_TEST(test_private_addresses);
    UNITY_END();
}
