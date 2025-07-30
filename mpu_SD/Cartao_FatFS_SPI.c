#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "ssd1306.h"         //inicia o ssd1306
#include "font.h"            //Fonte de palavras a serem escritas no ssd1306
#include "hardware/pwm.h"
#include "hardware/rtc.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "rtc.h"
#include "sd_card.h"


    // MPU6050 I2C address
#define I2C_PORT_MPU i2c0               // i2c0 pinos 0 e 1, i2c1 pinos 2 e 3
#define I2C_SDA_MPU 0                   // 0 ou 2
#define I2C_SCL_MPU 1                  // 1 ou 3
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define ENDERECO_I2C 0x3C
#define buzzer1 10
#define buzzer2 21
#define AZUL 12
#define VERDE 11
#define VERMELHO 13
#define BOTAO_A 5
#define PWM_WRAP 4095
#define PWM_CLK_DIV 30.52f
static bool logger_enabled;
static const uint32_t period = 1000;
static absolute_time_t next_log_time;
absolute_time_t ultimo_bip;
absolute_time_t last_interrupt_time = 0;
int ciclos_buzzer = 0;
static char filename[20] = "MPU_data1.csv";
// O endereço padrao deste IMU é o 0x68
static int addr = 0x68;
ssd1306_t ssd;
bool estado_buzzer = false;
bool buzzer_ativo = false;
bool botaoa = false;
// Inicializa um pino GPIO para operar como PWM.
void pwm_init_gpio(uint gpio, uint wrap, float clkdiv) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, wrap);
    pwm_config_set_clkdiv(&config, clkdiv);
    pwm_init(slice_num, &config, true);
}

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(I2C_PORT_MPU, addr, buf, 2, false);
    sleep_ms(100); // Allow device to reset and stabilize

    // Clear sleep mode (0x6B register, 0x00 value)
    buf[1] = 0x00;  // Clear sleep mode by writing 0x00 to the 0x6B register
    i2c_write_blocking(I2C_PORT_MPU, addr, buf, 2, false); 
    sleep_ms(10); // Allow stabilization after waking up
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_PORT_MPU, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(I2C_PORT_MPU, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(I2C_PORT_MPU, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT_MPU, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(I2C_PORT_MPU, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT_MPU, addr, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}
static sd_card_t *sd_get_by_name(const char *const name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return sd_get_by_num(i);
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}
static FATFS *sd_get_fs_by_name(const char *name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return &sd_get_by_num(i)->fatfs;
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}

static void run_setrtc()
{
    const char *dateStr = strtok(NULL, " ");
    if (!dateStr)
    {
        printf("Missing argument\n");
        return;
    }
    int date = atoi(dateStr);

    const char *monthStr = strtok(NULL, " ");
    if (!monthStr)
    {
        printf("Missing argument\n");
        return;
    }
    int month = atoi(monthStr);

    const char *yearStr = strtok(NULL, " ");
    if (!yearStr)
    {
        printf("Missing argument\n");
        return;
    }
    int year = atoi(yearStr) + 2000;

    const char *hourStr = strtok(NULL, " ");
    if (!hourStr)
    {
        printf("Missing argument\n");
        return;
    }
    int hour = atoi(hourStr);

    const char *minStr = strtok(NULL, " ");
    if (!minStr)
    {
        printf("Missing argument\n");
        return;
    }
    int min = atoi(minStr);

    const char *secStr = strtok(NULL, " ");
    if (!secStr)
    {
        printf("Missing argument\n");
        return;
    }
    int sec = atoi(secStr);

    datetime_t t = {
        .year = (int16_t)year,
        .month = (int8_t)month,
        .day = (int8_t)date,
        .dotw = 0, // 0 is Sunday
        .hour = (int8_t)hour,
        .min = (int8_t)min,
        .sec = (int8_t)sec};
    rtc_set_datetime(&t);
}

static void run_format()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    /* Format the drive with default parameters */
    FRESULT fr = f_mkfs(arg1, 0, 0, FF_MAX_SS * 2);
    if (FR_OK != fr)
        printf("f_mkfs error: %s (%d)\n", FRESULT_str(fr), fr);
}
static void run_mount()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_mount(p_fs, arg1, 1);
    if (FR_OK != fr)
    {
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = true;
    printf("Processo de montagem do SD ( %s ) concluído\n", pSD->pcName);
}
static void run_unmount()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_unmount(arg1);
    if (FR_OK != fr)
    {
        printf("f_unmount error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = false;
    pSD->m_Status |= STA_NOINIT; // in case medium is removed
    printf("SD ( %s ) desmontado\n", pSD->pcName);
}
static void run_getfree()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    DWORD fre_clust, fre_sect, tot_sect;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_getfree(arg1, &fre_clust, &p_fs);
    if (FR_OK != fr)
    {
        printf("f_getfree error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    tot_sect = (p_fs->n_fatent - 2) * p_fs->csize;
    fre_sect = fre_clust * p_fs->csize;
    printf("%10lu KiB total drive space.\n%10lu KiB available.\n", tot_sect / 2, fre_sect / 2);
}
static void run_ls()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = "";
    char cwdbuf[FF_LFN_BUF] = {0};
    FRESULT fr;
    char const *p_dir;
    if (arg1[0])
    {
        p_dir = arg1;
    }
    else
    {
        fr = f_getcwd(cwdbuf, sizeof cwdbuf);
        if (FR_OK != fr)
        {
            printf("f_getcwd error: %s (%d)\n", FRESULT_str(fr), fr);
            return;
        }
        p_dir = cwdbuf;
    }
    printf("Directory Listing: %s\n", p_dir);
    DIR dj;
    FILINFO fno;
    memset(&dj, 0, sizeof dj);
    memset(&fno, 0, sizeof fno);
    fr = f_findfirst(&dj, &fno, p_dir, "*");
    if (FR_OK != fr)
    {
        printf("f_findfirst error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    while (fr == FR_OK && fno.fname[0])
    {
        const char *pcWritableFile = "writable file",
                   *pcReadOnlyFile = "read only file",
                   *pcDirectory = "directory";
        const char *pcAttrib;
        if (fno.fattrib & AM_DIR)
            pcAttrib = pcDirectory;
        else if (fno.fattrib & AM_RDO)
            pcAttrib = pcReadOnlyFile;
        else
            pcAttrib = pcWritableFile;
        printf("%s [%s] [size=%llu]\n", fno.fname, pcAttrib, fno.fsize);

        fr = f_findnext(&dj, &fno);
    }
    f_closedir(&dj);
}
static void run_cat()
{
    char *arg1 = strtok(NULL, " ");
    if (!arg1)
    {
        printf("Missing argument\n");
        return;
    }
    FIL fil;
    FRESULT fr = f_open(&fil, arg1, FA_READ);
    if (FR_OK != fr)
    {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    char buf[256];
    while (f_gets(buf, sizeof buf, &fil))
    {
        printf("%s", buf);
    }
    fr = f_close(&fil);
    if (FR_OK != fr)
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
}

// Função para capturar dados do MPU e salvar no arquivo *.txt
void capture_mpu_data_and_save()
{
    printf("\nCapturando dados do MPU6050. Aguarde finalização...\n");
    FIL file;
    FRESULT res = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK)
    {
        printf("\n[ERRO] Não foi possível abrir o arquivo para escrita. Monte o Cartao.\n");
        return;
    }

    // Escreve o cabeçalho
    char header[] = "numero_amostra,accel_x,accel_y,accel_z,giro_x,giro_y,giro_z\n";
    UINT bw;
    res = f_write(&file, header, strlen(header), &bw);
    if (res != FR_OK)
    {
        printf("[ERRO] Não foi possível escrever o cabeçalho no arquivo.\n");
        f_close(&file);
        return;
    }

    int16_t acceleration[3], gyro[3], temp;
    for (int i = 0; i < 150; i++)
    {
        mpu6050_read_raw(acceleration, gyro, &temp);
        char buffer[100];
        char msg[32];
        sprintf(msg, "Amostra: %d/150", i+1);
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "Gravando...", 0, 0);
        ssd1306_draw_string(&ssd, msg, 0, 16);
        ssd1306_send_data(&ssd);
        sprintf(buffer, "%d             ,%d   ,%d  ,%d  ,%d    ,%d     ,%d\n", i + 1, acceleration[0], acceleration[1], acceleration[2], gyro[0], gyro[1], gyro[2]);
        res = f_write(&file, buffer, strlen(buffer), &bw);
        if (res != FR_OK)
        {
            printf("[ERRO] Não foi possível escrever no arquivo. Monte o Cartao.\n");
            f_close(&file);
            return;
        }
        sleep_ms(100);

    }
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Dados Salvos!", 0, 0);
    ssd1306_send_data(&ssd);
    f_close(&file);
    printf("\nDados do MPU6050 salvos no arquivo %s.\n\n", filename);
}

// Função para ler o conteúdo de um arquivo e exibir no terminal
void read_file(const char *filename)
{
    FIL file;
    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK)
    {
        printf("[ERRO] Não foi possível abrir o arquivo para leitura. Verifique se o Cartão está montado ou se o arquivo existe.\n");

        return;
    }
    char buffer[128];
    UINT br;
    printf("Conteúdo do arquivo %s:\n", filename);
    while (f_read(&file, buffer, sizeof(buffer) - 1, &br) == FR_OK && br > 0)
    {
        buffer[br] = '\0';
        printf("%s", buffer);
    }
    f_close(&file);
    printf("\nLeitura do arquivo %s concluída.\n\n", filename);
}

// Trecho para modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6
// Função de callback para interrupções de GPIO (botões).
void gpio_callback(uint gpio, uint32_t events) {
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(last_interrupt_time, now) < 250000) return;
    last_interrupt_time = now;

    if(gpio == BOTAO_A) {

        botaoa = !botaoa; // Alterna o estado do botão A
    }
    if(gpio == botaoB) {

         reset_usb_boot(0, 0);
    }
}

static void run_help()
{
    printf("\nComandos disponíveis:\n\n");
    printf("Digite 'a' para montar o cartão SD\n");
    printf("Digite 'b' para desmontar o cartão SD\n");
    printf("Digite 'c' para listar arquivos\n");
    printf("Digite 'd' para mostrar conteúdo do arquivo\n");
    printf("Digite 'e' para obter espaço livre no cartão SD\n");
    printf("Digite 'f' para capturar dados do MPU e salvar no arquivo\n");
    printf("Digite 'g' para formatar o cartão SD\n");
    printf("Digite 'h' para exibir os comandos disponíveis\n");
    printf("\nEscolha o comando:  ");
}

typedef void (*p_fn_t)();
typedef struct
{
    char const *const command;
    p_fn_t const function;
    char const *const help;
} cmd_def_t;

static cmd_def_t cmds[] = {
    {"setrtc", run_setrtc, "setrtc <DD> <MM> <YY> <hh> <mm> <ss>: Set Real Time Clock"},
    {"format", run_format, "format [<drive#:>]: Formata o cartão SD"},
    {"mount", run_mount, "mount [<drive#:>]: Monta o cartão SD"},
    {"unmount", run_unmount, "unmount <drive#:>: Desmonta o cartão SD"},
    {"getfree", run_getfree, "getfree [<drive#:>]: Espaço livre"},
    {"ls", run_ls, "ls: Lista arquivos"},
    {"cat", run_cat, "cat <filename>: Mostra conteúdo do arquivo"},
    {"help", run_help, "help: Mostra comandos disponíveis"}};

static void process_stdio(int cRxedChar)
{
    static char cmd[256];
    static size_t ix;

    if (!isprint(cRxedChar) && !isspace(cRxedChar) && '\r' != cRxedChar &&
        '\b' != cRxedChar && cRxedChar != (char)127)
        return;
    printf("%c", cRxedChar); // echo
    stdio_flush();
    if (cRxedChar == '\r')
    {
        printf("%c", '\n');
        stdio_flush();

        if (!strnlen(cmd, sizeof cmd))
        {
            printf("> ");
            stdio_flush();
            return;
        }
        char *cmdn = strtok(cmd, " ");
        if (cmdn)
        {
            size_t i;
            for (i = 0; i < count_of(cmds); ++i)
            {
                if (0 == strcmp(cmds[i].command, cmdn))
                {
                    (*cmds[i].function)();
                    break;
                }
            }
            if (count_of(cmds) == i)
                printf("Command \"%s\" not found\n", cmdn);
        }
        ix = 0;
        memset(cmd, 0, sizeof cmd);
        printf("\n> ");
        stdio_flush();
    }
    else
    {
        if (cRxedChar == '\b' || cRxedChar == (char)127)
        {
            if (ix > 0)
            {
                ix--;
                cmd[ix] = '\0';
            }
        }
        else
        {
            if (ix < sizeof cmd - 1)
            {
                cmd[ix] = cRxedChar;
                ix++;
            }
        }
    }
}
void inicia_pinos(){
    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    stdio_init_all();
    sleep_ms(5000);
    
    i2c_init(I2C_PORT_MPU, 400 * 1000);
    gpio_set_function(I2C_SDA_MPU, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_MPU, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_MPU);
    gpio_pull_up(I2C_SCL_MPU);
    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO_I2C, I2C_PORT_DISP);

    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Inicializando...", 10, 10);
    ssd1306_send_data(&ssd);
    gpio_init(VERDE);
    gpio_set_dir(VERDE, GPIO_OUT);
    gpio_init(AZUL);
    gpio_set_dir(AZUL, GPIO_OUT);
    gpio_init(VERMELHO);
    gpio_set_dir(VERMELHO, GPIO_OUT);
    gpio_put(VERDE, true); // LED verde aceso   
    gpio_put(VERMELHO, true); 
    gpio_put(AZUL, false); 
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A);
    // Inicializa PWM para os buzzers.
    pwm_init_gpio(buzzer1, PWM_WRAP, PWM_CLK_DIV);
    pwm_set_gpio_level(buzzer1, 0);
    pwm_init_gpio(buzzer2, PWM_WRAP, PWM_CLK_DIV);
    pwm_set_gpio_level(buzzer2, 0);
}
void oled_status() {
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Aguardando...", 0, 0);
    ssd1306_send_data(&ssd);
}
int main()
{
    time_init();
    printf("FatFS SPI example\n");
    printf("\033[2J\033[H"); // Limpa tela
    printf("\n> ");
    stdio_flush();
    //    printf("A tela foi limpa...\n");
    //    printf("Depois do Flush\n");
    run_help();
    printf("Hello, MPU6050! Reading raw data from registers...\n");
    inicia_pinos();
  
    //printf("Antes do bi_decl...\n");
    bi_decl(bi_2pins_with_func(I2C_SDA_MPU, I2C_SCL_MPU, GPIO_FUNC_I2C));
    //printf("Antes do reset MPU...\n");
    mpu6050_reset();
 
    int16_t acceleration[3], gyro[3], temp;
    while (true)
    {
        mpu6050_read_raw(acceleration, gyro, &temp);
 

         //printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
         //printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
         //printf("Temp. = %f\n", (temp / 340.0) + 36.53);
        
         // Exibe os dados no display OLED
        oled_status();
        int cRxedChar = getchar_timeout_us(0);
        if (PICO_ERROR_TIMEOUT != cRxedChar)
            process_stdio(cRxedChar);

        if(cRxedChar == PICO_ERROR_TIMEOUT){ 
            gpio_put(VERDE, true);
            gpio_put(VERMELHO, true); 
            gpio_put(AZUL, false);
        } 
        if (cRxedChar == 'a') // Monta o SD card se pressionar 'a'
        {   
            gpio_put(VERDE, true);
            gpio_put(VERMELHO, true); 
            gpio_put(AZUL, false); 
            printf("\nMontando o SD...\n");
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "Montando SD", 0, 0);
            ssd1306_send_data(&ssd);
            run_mount();
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "Montagem", 0, 0);
            ssd1306_draw_string(&ssd, "completa", 16, 0);
            ssd1306_send_data(&ssd);
            sleep_ms(1000);
            oled_status();
            gpio_put(VERDE, true);
            gpio_put(VERMELHO, false); 
            gpio_put(AZUL, false); 
            printf("\nEscolha o comando (h = help):  ");
            
        }
        if (cRxedChar == 'b') // Desmonta o SD card se pressionar 'b'
        {
            printf("\nDesmontando o SD. Aguarde...\n");
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "Desmontando SD...", 0, 0);
            ssd1306_send_data(&ssd);
            run_unmount();
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "Desmontagem completa", 0, 0);
            ssd1306_send_data(&ssd);
            sleep_ms(1000);
            oled_status();
            gpio_put(VERDE, true);
            gpio_put(VERMELHO, true); 
            gpio_put(AZUL, false);
            printf("\nEscolha o comando (h = help):  ");

        }
        if (cRxedChar == 'c') // Lista diretórios e os arquivos se pressionar 'c'
        {
            printf("\nListagem de arquivos no cartão SD.\n");
            run_ls();
            printf("\nListagem concluída.\n");
            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'd') // Exibe o conteúdo do arquivo se pressionar 'd'
        {
            read_file(filename);
            printf("Escolha o comando (h = help):  ");
        }
        if (cRxedChar == 'e') // Obtém o espaço livre no SD card se pressionar 'e'
        {
            printf("\nObtendo espaço livre no SD.\n\n");
            run_getfree();
            printf("\nEspaço livre obtido.\n");
            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'f' || botaoa == true) // Captura dados do MPU e salva no arquivo se pressionar 'f'
        {   
            gpio_put(VERDE, false);
            gpio_put(VERMELHO, true); 
            gpio_put(AZUL, false); 
            pwm_set_gpio_level(buzzer2, 250);
            pwm_set_gpio_level(buzzer1, 250);
            sleep_ms(200);
            pwm_set_gpio_level(buzzer2, 0);
            pwm_set_gpio_level(buzzer1, 0);
            capture_mpu_data_and_save();
            pwm_set_gpio_level(buzzer2, 250);
            pwm_set_gpio_level(buzzer1, 250);
            sleep_ms(200);
            pwm_set_gpio_level(buzzer2, 0);
            pwm_set_gpio_level(buzzer1, 0);
            sleep_ms(200);
            pwm_set_gpio_level(buzzer2, 250);
            pwm_set_gpio_level(buzzer1, 250);
            sleep_ms(200);
            pwm_set_gpio_level(buzzer2, 0);
            pwm_set_gpio_level(buzzer1, 0);
            gpio_put(VERDE, true);
            gpio_put(VERMELHO, false); 
            gpio_put(AZUL, false); 
            printf("\nEscolha o comando (h = help):  ");
            botaoa = false; // Reseta o estado do botão A
        }
        if (cRxedChar == 'g') // Formata o SD card se pressionar 'g'
        {
            printf("\nProcesso de formatação do SD iniciado. Aguarde...\n");
            gpio_put(VERDE, false);
            gpio_put(VERMELHO, true); 
            gpio_put(AZUL, true);
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "Formatando SD", 0, 0);
            ssd1306_send_data(&ssd);
            run_format();
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "SD Formatado", 0, 0);
            ssd1306_send_data(&ssd);
            sleep_ms(1000);
            oled_status();
            gpio_put(VERDE, true);
            gpio_put(VERMELHO, false); 
            gpio_put(AZUL, false);
            printf("\nFormatação concluída.\n\n");
            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'h') // Exibe os comandos disponíveis se pressionar 'h'
        {
            run_help();
        }
        sleep_ms(500);
    }
    return 0;
}