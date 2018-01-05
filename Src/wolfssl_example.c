#include "wolfssl_example.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* UART definitions */
extern UART_HandleTypeDef huart4;
extern SPI_HandleTypeDef hspi1;


#ifdef WOLFSSL_TPM2
extern int TPM2_Demo(void);
#endif


/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
typedef struct func_args {
    int    argc;
    char** argv;
    int    return_code;
} func_args;

const char menu1[] = "\r\n"
    "\tt. WolfCrypt Test\r\n"
    "\tb. WolfCrypt Benchmark\r\n"
	"\tm. WolfCrypt TPM 2.0 Test\r\n";

/*****************************************************************************
 * Private functions
 ****************************************************************************/


/*****************************************************************************
 * Public functions
 ****************************************************************************/
void wolfCryptDemo(void const * argument)
{
    uint8_t buffer[2];
    func_args args;

    while (1) {
        printf("\r\n\t\t\t\tMENU\r\n");
        printf(menu1);
        printf("Please select one of the above options: ");

        HAL_UART_Receive(&huart4, buffer, sizeof(buffer), 5000);

        switch (buffer[0]) {

        case 't':
            memset(&args, 0, sizeof(args));
            printf("\nCrypt Test\n");
            wolfcrypt_test(&args);
            printf("Crypt Test: Return code %d\n", args.return_code);
            break;

        case 'b':
            memset(&args, 0, sizeof(args));
            printf("\nBenchmark Test\n");
            benchmark_test(&args);
            printf("Benchmark Test: Return code %d\n", args.return_code);
            break;

        case 'm':
			printf("\nTPM 2.0 Test\n");
#ifdef WOLFSSL_TPM2
			args.return_code = TPM2_Demo();
#endif
			printf("TPM 2.0 Test: Return code %d\n", args.return_code);
			break;

        // All other cases go here
        default: printf("\r\nSelection out of range\r\n"); break;
        }
    }
}

extern RTC_HandleTypeDef hrtc;
double current_time()
{
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	uint32_t subsec;

	/* must get time and date here due to STM32 HW bug */
	HAL_RTC_GetTime(&hrtc, &time, FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &date, FORMAT_BIN);
	subsec = (255 - time.SubSeconds) * 1000 / 255;

	(void)date;

	/* return seconds.milliseconds */
	return ((double)time.Hours * 24) +
		   ((double)time.Minutes * 60) +
			(double)time.Seconds +
		   ((double)subsec/1000);
}

