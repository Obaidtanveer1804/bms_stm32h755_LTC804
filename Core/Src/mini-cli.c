/**
 * @file mini_cli.c
 * @brief Minimalistic CLI implementation for STM32
 */

#include "mini-cli.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Private defines */
#define CLI_PROMPT              "> "
#define CLI_MAX_CMD_LENGTH      64
#define CLI_MAX_ARGS            8
#define CLI_HISTORY_SIZE        5
#define CLI_WELCOME_MSG         "\r\n\r\nSTM32 Mini-CLI v1.0\r\nType 'help' for available commands\r\n"
extern UART_HandleTypeDef huart3;
extern uint8_t unBalancing;
extern uint8_t unCell_1 ;
/* Private variables */
static char cmd_buffer[CLI_MAX_CMD_LENGTH];
static uint16_t cmd_pos = 0;
static uint8_t rx_char;
extern float fChargeprotectionLimit;
extern float fDisChargeprotectionLimit;
extern BMS_COMMAND_RECEPTION canparam;


static char cmd_history[CLI_HISTORY_SIZE][CLI_MAX_CMD_LENGTH];
static uint8_t history_count = 0;
static int8_t history_index = -1;
void cli_cmd_printcellsdata(int argc, char *argv[]);
void cli_cmd_balancing(int argc, char *argv[]);
void cli_cmd_isbalanced(int argc, char *argv[]);
void cli_cmd_sizeofstructs(int argc, char *argv[]);
void cli_cmd_print_bms_params(int argc, char *argv[]);
void cli_cmd_print_bms_calc(int argc, char *argv[]);
void cli_cmd_set_ovpValue(int argc, char *argv[]);
void cli_cmd_set_uvpValue(int argc, char *argv[]);

/* Command table */
static const CLI_Command_t cmd_list[] = {
    {"help",    cli_cmd_help,    "Display available commands"},
    {"led",     cli_cmd_led,     "Control onboard LED: on, off, toggle"},
    {"info",    cli_cmd_info,    "Display system information"},
    {"reset",   cli_cmd_reset,   "Reset the microcontroller"},
    {"echo",    cli_cmd_echo,    "Echo the provided arguments"},
	{"celldata",    cli_cmd_printcellsdata,    "Print Cell Vo,ltages"},
	{"balance",    cli_cmd_balancing,    "Start/Stop Balancing"},
	{"isbalanced",    cli_cmd_isbalanced,    "check if the cell has been balanced or not"},
	{"sizeofstructs",    cli_cmd_sizeofstructs,    "Print Sizes of structs etc"},
	{"bmsparams",    cli_cmd_print_bms_params,    "Print bms Parameters"},
	{"bmscalc",    cli_cmd_print_bms_calc,    "Print bms Calculated Parameters"},
	{"setovp",   cli_cmd_set_ovpValue,    "Set Charge Protection Voltage"},
	{"setuvp",   cli_cmd_set_uvpValue,    "Set Discharge Protection Voltage"},

	{NULL, NULL, NULL} /* Terminator */
};

/* Private function prototypes */
static void CLI_ProcessCommand(void);
static void CLI_ExecuteCommand(char *cmd_line);
static void CLI_PrintPrompt(void);
static void CLI_AddToHistory(const char *cmd);
static void CLI_GetHistory(int8_t dir);

/**
 * @brief Initialize the CLI
 */
void CLI_Init(void)
{
    /* Clear the buffer */
    memset(cmd_buffer, 0, sizeof(cmd_buffer));
    cmd_pos = 0;

    /* Start receiving characters from UART */
    HAL_UART_Receive_IT(&huart3, &rx_char, 1);

    /* Print welcome message */
    CLI_Print(CLI_WELCOME_MSG);
    CLI_PrintPrompt();
}

/**
 * @brief Print a string to the CLI
 * @param str String to print
 */
void CLI_Print(const char *str)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

/**
 * @brief Print a formatted string to the CLI
 * @param format Printf-style format string
 * @param ... Printf-style arguments
 */
void CLI_Printf(const char *format, ...)
{
    char buffer[128];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    CLI_Print(buffer);
}

/**
 * @brief Process a received character
 * @param c Character to process
 */
void CLI_ProcessChar(uint8_t c)
{
    /* Resume UART reception for next char */
    HAL_UART_Receive_IT(&huart3, &rx_char, 1);

    switch (c)
    {
        case '\r':  /* Enter key */
            CLI_Print("\r\n");

            /* Process the command if buffer is not empty */
            if (cmd_pos > 0)
            {
                cmd_buffer[cmd_pos] = '\0';
                CLI_ProcessCommand();

                /* Add to history */
                CLI_AddToHistory(cmd_buffer);

                /* Reset buffer and history navigation */
                memset(cmd_buffer, 0, sizeof(cmd_buffer));
                cmd_pos = 0;
                history_index = -1;
            }

            CLI_PrintPrompt();
            break;

        case '\b':  /* Backspace */
        case 127:   /* Delete */
            if (cmd_pos > 0)
            {
                cmd_pos--;
                cmd_buffer[cmd_pos] = '\0';
                CLI_Print("\b \b");  /* Erase character on terminal */
            }
            break;

        case 27:    /* Escape sequence (arrow keys) */
            /* Skip next two chars for arrow key sequences */
            /* This is a simplified approach; more robust handling would be needed */
            /* for complete VT100 terminal support */
            HAL_UART_Receive(&huart3, &rx_char, 1, HAL_MAX_DELAY);  /* Skip [ */
            if (rx_char == '[')
            {
                HAL_UART_Receive(&huart3, &rx_char, 1, HAL_MAX_DELAY);
                switch (rx_char)
                {
                    case 'A':  /* Up arrow */
                        CLI_GetHistory(-1);
                        break;
                    case 'B':  /* Down arrow */
                        CLI_GetHistory(1);
                        break;
                    default:
                        break;
                }
            }
            break;

        default:
            /* Add to buffer if there's room and it's a printable character */
            if (cmd_pos < (CLI_MAX_CMD_LENGTH - 1) && c >= 32 && c <= 126)
            {
                cmd_buffer[cmd_pos++] = c;
                cmd_buffer[cmd_pos] = '\0';
                HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);
            }
            break;
    }
}

/**
 * @brief UART Rx Callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        CLI_ProcessChar(rx_char);
    }
}

/**
 * @brief Process the command in the buffer
 */
static void CLI_ProcessCommand(void)
{
    CLI_ExecuteCommand(cmd_buffer);
}

/**
 * @brief Execute a command
 * @param cmd_line Command line to execute
 */
static void CLI_ExecuteCommand(char *cmd_line)
{
    char *argv[CLI_MAX_ARGS];
    int argc = 0;
    char *token;
    char *rest = cmd_line;

    /* Parse command line into arguments */
    while ((token = strtok_r(rest, " ", &rest)) != NULL && argc < CLI_MAX_ARGS)
    {
        argv[argc++] = token;
    }

    /* Exit if no command */
    if (argc == 0)
    {
        return;
    }

    /* Find and execute command */
    for (int i = 0; cmd_list[i].command != NULL; i++)
    {
        if (strcmp(argv[0], cmd_list[i].command) == 0)
        {
            cmd_list[i].handler(argc, argv);
            return;
        }
    }

    /* Command not found */
    CLI_Printf("Unknown command: %s\r\n", argv[0]);
}

/**
 * @brief Print command prompt
 */
static void CLI_PrintPrompt(void)
{
    CLI_Print(CLI_PROMPT);
}

/**
 * @brief Add command to history
 * @param cmd Command to add
 */
static void CLI_AddToHistory(const char *cmd)
{
    /* Check if command is different from the last one */
    if (history_count > 0 && strcmp(cmd, cmd_history[0]) == 0)
    {
        return;
    }

    /* Shift history entries */
    for (int i = CLI_HISTORY_SIZE - 1; i > 0; i--)
    {
        strcpy(cmd_history[i], cmd_history[i-1]);
    }

    /* Add new command */
    strcpy(cmd_history[0], cmd);

    if (history_count < CLI_HISTORY_SIZE)
    {
        history_count++;
    }
}

/**
 * @brief Navigate command history
 * @param dir Direction (-1 for up/older, 1 for down/newer)
 */
static void CLI_GetHistory(int8_t dir)
{
    if (history_count == 0)
    {
        return;
    }

    if (dir < 0 && history_index < (history_count - 1))
    {
        /* Move back in history (older commands) */
        history_index++;
    }
    else if (dir > 0 && history_index >= 0)
    {
        /* Move forward in history (newer commands) */
        history_index--;
    }
    else
    {
        return;
    }

    /* Clear current command line */
    while (cmd_pos > 0)
    {
        CLI_Print("\b \b");
        cmd_pos--;
    }

    if (history_index >= 0)
    {
        /* Copy history entry to command buffer */
        strcpy(cmd_buffer, cmd_history[history_index]);
        cmd_pos = strlen(cmd_buffer);
        CLI_Print(cmd_buffer);
    }
    else
    {
        /* Clear command buffer if at beginning of history */
        memset(cmd_buffer, 0, sizeof(cmd_buffer));
        cmd_pos = 0;
    }
}

/* Command handlers */

/**
 * @brief Help command handler
 */
void cli_cmd_help(int argc, char *argv[])
{
    CLI_Print("Available commands:\r\n");

    for (int i = 0; cmd_list[i].command != NULL; i++)
    {
        CLI_Printf("  %-10s - %s\r\n", cmd_list[i].command, cmd_list[i].help);
    }
}

/**
 * @brief LED control command handler
 */
void cli_cmd_led(int argc, char *argv[])
{
    if (argc < 2)
    {
        CLI_Print("Usage: led <on|off|toggle>\r\n");
        return;
    }

    if (strcmp(argv[1], "on") == 0)
    {
       // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        CLI_Print("LED turned ON\r\n");
    }
    else if (strcmp(argv[1], "off") == 0)
    {
      //  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        CLI_Print("LED turned OFF\r\n");
    }
    else if (strcmp(argv[1], "toggle") == 0)
    {
       // HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        CLI_Print("LED toggled\r\n");
    }
    else
    {
        CLI_Print("Invalid option. Use: on, off, or toggle\r\n");
    }
}

/**
 * @brief System info command handler
 */
void cli_cmd_info(int argc, char *argv[])
{
    uint32_t tick = HAL_GetTick();
    uint32_t seconds = tick / 1000;
    uint32_t minutes = seconds / 60;
    uint32_t hours = minutes / 60;

    seconds %= 60;
    minutes %= 60;

    CLI_Print("System Information:\r\n");
    CLI_Printf("  MCU:        STM32\r\n");
    CLI_Printf("  Uptime:     %02lu:%02lu:%02lu\r\n", hours, minutes, seconds);
    CLI_Printf("  Sys Clock:  %lu MHz\r\n", HAL_RCC_GetSysClockFreq() / 1000000);
    vCalculateTotalVoltage();
}

/**
 * @brief Reset command handler
 */
void cli_cmd_reset(int argc, char *argv[])
{
    CLI_Print("Resetting system...\r\n");
    HAL_Delay(500);  /* Small delay for the message to be sent */
    NVIC_SystemReset();
}

/**
 * @brief Print Cell voltages and populated can data to be sent
 */
void cli_cmd_printcellsdata(int argc, char *argv[])
{
    CLI_Print("Printing Cells data...\r\n");
    vCalculateTotalVoltage();
    vPrintCellVoltages();
    vPrintCandataGroups();
}

/**
 * @brief Print BMS parameters
 */
void cli_cmd_print_bms_params(int argc, char *argv[])
{
    CLI_Print("Printing bms_params...\r\n");
    vPrintCandataParams();
}
/**
 * @brief Print BMS calculated parameters
 */
void cli_cmd_print_bms_calc(int argc, char *argv[])
{
    CLI_Print("Printing bms_calc...\r\n");
    vPrintBmsCalcData();
}
/**
 * @brief Echo command handler
 */
void cli_cmd_echo(int argc, char *argv[])
{
    for (int i = 1; i < argc; i++)
    {
        CLI_Print(argv[i]);
        if (i < argc - 1)
        {
            CLI_Print(" ");
        }
    }
    CLI_Print("\r\n");
}
void cli_cmd_balancing(int argc, char *argv[])
{
	   if (argc < 1)
	    {
	        CLI_Print("Usage: balance <on|off>\r\n");
	        return;
	    }
	   if (strcmp(argv[1], "on") == 0)
	       {
	           CLI_Print("BALANCING ON\r\n");
	           unCell_1 = 1;
	           unBalancing = 1;
	       }
	       else if (strcmp(argv[1], "off") == 0)
	       {
	           CLI_Print("BALANCING OFF\r\n");
	           unCell_1 = 0;
	           unBalancing = 0;
	       }
}
void cli_cmd_isbalanced(int argc, char *argv[])
{
    if (argc < 2)
    {
        CLI_Print("Usage: isbalanced <cell_number>\r\n");
        return;
    }

    uint8_t cell_number = (uint8_t)atoi(argv[1]);

    if (cell_number >= 192)
    {
        CLI_Print("Error: Cell number must be between 0 and 63\r\n");
        return;
    }

    if (bIs_cell_balanced(cell_number))
    {
        CLI_Print("Cell is BALANCED\r\n");
    }
    else
    {
        CLI_Print("Cell is NOT balanced\r\n");
    }
}

void cli_cmd_sizeofstructs(int argc, char *argv[])
{
	vPrint_struct_sizes();
}

void cli_cmd_set_uvpValue(int argc, char *argv[])
{
    if (argc != 2) {
        CLI_Print("Error: Expected exactly one argument (e.g., setuvp 3.65)\r\n");
        return;
    }

    char *endptr;
    float new_uvp = strtof(argv[1], &endptr);
    if (*endptr != '\0') {
        CLI_Print("Error: Invalid float value (e.g., use 3.65)\r\n");
        return;
    }
    if (new_uvp < 2.0f || new_uvp > 4.0f) {
        CLI_Print("Error: UVP value must be between 2.0V and 4.0V\r\n");
        return;
    }

    // Store locally if you need it
    fDisChargeprotectionLimit = new_uvp;

    // Convert to integer representation
    uint16_t raw = (uint16_t)(new_uvp * 100.0f + 0.5f);

    // Index 1 is OverDischargeVoltageProtection (0x01)
    UpdateParameter(1, raw);

    char buf[64];
    snprintf(buf, sizeof(buf),
             "UVP threshold set to %.2f V (raw = 0x%04X)\r\n",
             new_uvp, raw);
    CLI_Print(buf);
}

void cli_cmd_set_ovpValue(int argc, char *argv[])
{
    if (argc != 2) {
        CLI_Print("Error: Expected exactly one argument (e.g., setovp 3.65)\r\n");
        return;
    }

    char *endptr;
    float new_ovp = strtof(argv[1], &endptr);
    if (*endptr != '\0') {
        CLI_Print("Error: Invalid float (e.g., use 3.65)\r\n");
        return;
    }
    if (new_ovp < 2.0f || new_ovp > 5.0f) {
        CLI_Print("Error: OVP must be between 2.0V and 5.0V\r\n");
        return;
    }

    fChargeprotectionLimit = new_ovp;
    uint16_t raw = (uint16_t)(new_ovp * 100.0f + 0.5f);

    // Index 5 is OverChargeVoltageProtectionValue (0x05)
    UpdateParameter(5, raw);

    char buf[64];
    snprintf(buf, sizeof(buf),
             "OVP threshold set to %.2f V (raw = 0x%04X)\r\n",
             new_ovp, raw);
    CLI_Print(buf);
}

//void cli_cmd_set_ovpValue(int argc, char *argv[])
//{
//    if (argc != 2) {
//        CLI_Print("Error: Expected exactly one argument (e.g., setovp 3.65)\r\n");
//        return;
//    }
//    char *endptr;
//    float new_ovp_value = strtof(argv[1], &endptr);
//    if (*endptr != '\0') {
//        CLI_Print("Error: Invalid float value (e.g., use 3.65)\r\n");
//        return;
//    }
//    if (new_ovp_value < 3.0f || new_ovp_value > 4.0f) {
//        CLI_Print("Error: OVP value must be between 3.0V and 4.0V\r\n");
//        return;
//    }
//    fChargeprotectionLimit = new_ovp_value;
//    char buffer[50];
//    snprintf(buffer, sizeof(buffer), "OVP threshold set to %.4f V\r\n", fChargeprotectionLimit);
//    CLI_Print(buffer);
//}
//
//void cli_cmd_set_uvpValue(int argc, char *argv[])
//{
//    if (argc != 2) {
//        CLI_Print("Error: Expected exactly one argument (e.g., setuvp 3.65)\r\n");
//        return;
//    }
//
//    char *endptr;
//    float new_uvp_value = strtof(argv[1], &endptr);
//    if (*endptr != '\0') {
//        CLI_Print("Error: Invalid float value (e.g., use 3.65)\r\n");
//        return;
//    }
//    if (new_uvp_value < 2.0f || new_uvp_value > 4.0f) {
//        CLI_Print("Error: UVP value must be between 2.0V and 4.0V\r\n");
//        return;
//    }
//    fDisChargeprotectionLimit = new_uvp_value;
//    char buffer[50];
//    snprintf(buffer, sizeof(buffer), "UVP threshold set to %.4f V\r\n", fDisChargeprotectionLimit);
//    CLI_Print(buffer);
//}
