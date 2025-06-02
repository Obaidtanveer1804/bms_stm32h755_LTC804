/**
 * @file mini_cli.h
 * @brief Header file for minimalistic CLI implementation for STM32
 */

#ifndef MINI_CLI_H
#define MINI_CLI_H

#include <stdint.h>
#include <stdarg.h>
#include <main.h>


/* Command handler function type */
typedef void (*CLI_CommandHandler_t)(int argc, char *argv[]);

/* Command structure */
typedef struct {
    const char *command;                /* Command string */
    CLI_CommandHandler_t handler;       /* Command handler function */
    const char *help;                   /* Help text */
} CLI_Command_t;

/* Public function prototypes */
void CLI_Init(void);
void CLI_ProcessChar(uint8_t c);
void CLI_Print(const char *str);
void CLI_Printf(const char *format, ...);

/* Command handlers */
void cli_cmd_help(int argc, char *argv[]);
void cli_cmd_led(int argc, char *argv[]);
void cli_cmd_info(int argc, char *argv[]);
void cli_cmd_reset(int argc, char *argv[]);
void cli_cmd_echo(int argc, char *argv[]);

#endif /* MINI_CLI_H */
