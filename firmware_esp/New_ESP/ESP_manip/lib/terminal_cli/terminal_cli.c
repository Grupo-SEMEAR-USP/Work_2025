#include "terminal_cli.h"
#include "utils.h"
#include "stepper_control.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

#define CLI_UART_PORT      UART_NUM_0
#define CLI_BAUD           115200
#define CLI_RX_BUF_SIZE    256
#define CLI_TX_BUF_SIZE    0
#define CLI_PROMPT         "> "
#define CRLF               "\r\n"

static void cli_printf(const char* fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    uart_write_bytes(CLI_UART_PORT, buf, n);
}

static void cli_print_help(void)
{
    cli_printf(CRLF
        "Comandos:" CRLF
        "  help                      - mostra esta ajuda" CRLF
        "  status                    - mostra valores atuais" CRLF
        "  arm   <delta_passos>      - adiciona delta ao STEPPER_ARM" CRLF
        "  base  <delta_passos>      - adiciona delta ao STEPPER_ROTATORY_BASE" CRLF
        "  wrist <graus 0..180>      - define SERVO_WRIST" CRLF
        "  grip  <graus 0..180>      - define SERVO_GRIPPER" CRLF
        "  set <arm> <base> <w> <g>  - atalho (arm/base = delta; w/g = setpoints)" CRLF
        CRLF);
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void process_line(char *line)
{
    char *p = line;
    while (*p == ' ' || *p == '\t') p++;
    size_t len = strlen(p);
    while (len && (p[len-1] == ' ' || p[len-1] == '\t')) p[--len] = '\0';
    if (!*p) return;

    float a, b, c, d;

    if (!strncmp(p, "help", 4)) { cli_print_help(); return; }

    if (!strncmp(p, "status", 6)) {
        cli_printf(CRLF
            "Status:" CRLF
            "  STEPPER_ARM            = %.3f (delta)" CRLF
            "  STEPPER_ROTATORY_BASE  = %.3f (delta)" CRLF
            "  SERVO_WRIST            = %.1f deg" CRLF
            "  SERVO_GRIPPER          = %.1f deg" CRLF
            "  Steppers idle?         = %s" CRLF
            CRLF,
            (double)STEPPER_ARM,
            (double)STEPPER_ROTATORY_BASE,
            (double)SERVO_WRIST,
            (double)SERVO_GRIPPER,
            stepper_is_idle() ? "yes" : "no");
        return;
    }

    if (sscanf(p, "arm %f", &a) == 1) {
        STEPPER_ARM += a;
        cli_printf("OK: arm += %.3f" CRLF, (double)a);
        return;
    }

    if (sscanf(p, "base %f", &a) == 1) {
        STEPPER_ROTATORY_BASE += a;
        cli_printf("OK: base += %.3f" CRLF, (double)a);
        return;
    }

    if (sscanf(p, "wrist %f", &a) == 1) {
        SERVO_WRIST = clampf(a, 0.f, 180.f);
        cli_printf("OK: wrist = %.1f" CRLF, (double)SERVO_WRIST);
        return;
    }

    if (sscanf(p, "grip %f", &a) == 1) {
        SERVO_GRIPPER = clampf(a, 0.f, 180.f);
        cli_printf("OK: grip = %.1f" CRLF, (double)SERVO_GRIPPER);
        return;
    }

    if (sscanf(p, "set %f %f %f %f", &a, &b, &c, &d) == 4) {
        STEPPER_ARM           += a;
        STEPPER_ROTATORY_BASE += b;
        SERVO_WRIST            = clampf(c, 0.f, 180.f);
        SERVO_GRIPPER          = clampf(d, 0.f, 180.f);
        cli_printf("OK: arm+=%.3f base+=%.3f wrist=%.1f grip=%.1f" CRLF,
                   (double)a, (double)b, (double)SERVO_WRIST, (double)SERVO_GRIPPER);
        return;
    }

    cli_printf("Comando invalido. Digite 'help'." CRLF);
}

static void cli_task(void *pv)
{
    // Configura UART0
    const uart_config_t cfg = {
        .baud_rate  = CLI_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(CLI_UART_PORT, CLI_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(CLI_UART_PORT, &cfg);
    uart_set_pin(CLI_UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    char line[CLI_RX_BUF_SIZE];
    int pos = 0;

    cli_printf(CRLF "CLI pronto. Digite 'help' para ajuda." CRLF);
    cli_printf(CLI_PROMPT);

    while (1) {
        uint8_t ch;
        int n = uart_read_bytes(CLI_UART_PORT, &ch, 1, pdMS_TO_TICKS(100));
        if (n <= 0) continue;

        if (ch == '\r' || ch == '\n') {
            // final da linha
            line[pos] = '\0';
            if (pos > 0) process_line(line);
            pos = 0;
            cli_printf(CLI_PROMPT);
        } else if (ch == 0x7F || ch == 0x08) {
            // backspace
            if (pos > 0) pos--;
        } else {
            if (pos < (int)sizeof(line) - 1) {
                line[pos++] = (char)ch;
            }
        }
    }
}

void start_cli(void)
{
    xTaskCreatePinnedToCore(cli_task, "cli_task", 4096, NULL, 3, NULL, 0);
}
