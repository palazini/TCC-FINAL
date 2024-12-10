/*
   ______  _____  _____     ____    ____       _       ___  ____             _________    ______    ______  
 .' ___  ||_   _||_   _|   |_   \  /   _|     / \     |_  ||_  _|           |  _   _  | .' ___  | .' ___  | 
/ .'   \_|  | |    | |       |   \/   |      / _ \      | |_/ /     ______  |_/ | | \_|/ .'   \_|/ .'   \_| 
| |   ____  | |    | |   _   | |\  /| |     / ___ \     |  __'.    |______|     | |    | |       | |        
\ `.___]  |_| |_  _| |__/ | _| |_\/_| |_  _/ /   \ \_  _| |  \ \_              _| |_   \ `.___.'\\ `.___.'\ 
 `._____.'|_____||________||_____||_____||____| |____||____||____|            |_____|   `.____ .' `.____ .' 
*/

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
//#include "esp_eap_client.h" 
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_smartconfig.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "mqtt_client.h"
#include "driver/adc.h"
#include "ioplaca.h"



//Área das macros
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define DHT_PIN 23
#define MAX_TIMINGS 85

#define W_DEVICE_ID "66f69cefbebd00ed95780bca" //Use o DeviceID no Wegnology  
#define W_ACCESS_KEY "0470c049-c6d3-44eb-a4b1-5759e907f917" //use a chave de acesso e a senha
#define W_PASSWORD "0ce0c57bc76fd3dc10a85c2fdf46653672837365cb1894e6ec1242bed7a680a7"
#define W_TOPICO_PUBLICAR "wnology/66f69cefbebd00ed95780bca/state" //esse número no meio do tópico deve ser mudado pelo ID do seu device Wegnology
#define W_TOPICO_SUBSCREVER "wnology/674f3a6c63185eb4e4c9f053/state" // aqui também

//ADXL335
#define ADXL335_X_PIN ADC1_CHANNEL_0  // GPIO 36
#define ADXL335_Y_PIN ADC1_CHANNEL_3  // GPIO 39
#define ADXL335_Z_PIN ADC1_CHANNEL_4  // GPIO 32

#define AGGREGATION_PERIOD_MS 2000  // Período de agregação em milissegundos
#define NUM_SAMPLES 100

#define THRESHOLD 0.01  // Tolerância para considerar uma passagem
#define SAMPLE_RATE 100  // Taxa de amostragem em Hz (100 ms)

//ZMPT
#define ZMPT101B_PIN  ADC1_CHANNEL_5 // GPIO33 33

#define SENSOR_VOLTAGE_REF 3.35  // Tensão de referência do sensor em V para 250V de entrada
#define INPUT_VOLTAGE_REF 250.0    // Tensão de entrada que corresponde a SENSOR_VOLTAGE_REF

#define SENSOR_OFFSET 1.65  // Offset do sensor (1.65V)


//ACS
#define ACS712_PIN ADC1_CHANNEL_7  // GPIO 35
#define V_REF 3.3                   // Tensão de referência do ESP32
#define SENSIBILIDADE 0.111        // Sensibilidade calculada (0,111 V/A)
#define OFFSET 2.0                 // Offset do sensor em volts



//Declaração de variáveis
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t s_wifi_event_group;

// The event group allows multiple bits for each event,but we only care about one event - are we connected to the AP with an IP?
static const int CONNECTED_BIT = BIT0;  
static const int ESPTOUCH_DONE_BIT = BIT1;
static const char *TAG_SC = "smartconfig_example";
//

//DHT
int temperature = 0;
int humidity = 0;

float temp_celsius;
float hum;

static float last_temp_celsius; // Armazena a última temperatura lida
static float last_humidity;    // Armazena a última umidade lida

//ADXL335
float x_g;
float y_g;
float z_g;

float x_offset;
float y_offset;
float z_offset;

int previous_x = 0, previous_y = 0, previous_z = 0; // Leituras anteriores
int x_crossings = 0, y_crossings = 0, z_crossings = 0; // Contagem de passagens
unsigned long last_time = 0; // Armazena o tempo da última contagem

float x_frequency; 
float y_frequency;
float z_frequency;

//ZMPT
//volatile float pico_tensao_global = 0.0;
float adc_atual;
float soma_adc;
int cont_adc;

float tensao_rms;

//ACS
float corrente_rms = 0.0;
float valor_final;
float ps_corrente_rms;

//Placa e Softstarter
static uint8_t entradas = 0; //variáveis de controle de entradas e saídas
static uint8_t saidas = 0; //variáveis de controle de entradas e saídas
const char *strSOFT = "SOFT\":";

//MQTT
static const char *TAG = "MQTT_EXAMPLE";
char *Inform;
const char *string_temp = "{\"data\": {\"temperature\": ";
const char *string_hum = "{\"data\": {\"humidity\": ";
const char *string_adxlX = "{\"data\": {\"vibr_x\": ";
const char *string_adxlY = "{\"data\": {\"vibr_y\": ";
const char *string_adxlZ = "{\"data\": {\"vibr_z\": ";
const char *string_tensao = "{\"data\": {\"tensao\": ";
const char *string_crr = "{\"data\": {\"corrente\": ";
const char *string_motorstatus = "{\"data\": {\"motorstatus\": ";
char mensa[40];
esp_mqtt_client_handle_t cliente; 


//Declaração de funções auxiliares
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Funções smart config
static void smartconfig_example_task(void * parm);
static void event_handler();
static void initialise_wifi();
static void smartconfig_example_task();

//Funções DHT
void DHT22_set_pin_output();
void DHT22_set_pin_input();
void DHT22_send_start_signal();
int DHT22_wait_for_response();
int DHT22_read_bit();
int DHT22_read_byte();
void DHT22_read_data();

static SemaphoreHandle_t xMutexDHT22 = NULL;  // Declaração global do mutex

void DHT22_init() 
{
    xMutexDHT22 = xSemaphoreCreateMutex();  // Cria o mutex
    if (xMutexDHT22 == NULL) {
        ESP_LOGE("DHT22", "Falha ao criar o mutex do DHT22");
    }
}

//MQTT - Wegnology
static void log_error_if_nonzero();
static void mqtt_event_handler();
static void mqtt_app_start();

//ADXL335
void detect_zero_crossing(float x_acceleration, float y_acceleration, float z_acceleration);
void read_adxl335();
void calculate_frequency();
void init_adc();
void set_offset();
float convert_to_g();

//ZMPT
float read_voltage() 
{
    // Lê o valor bruto do ADC
    int valor_adc = adc1_get_raw(ZMPT101B_PIN) /*- 2048*/;

    return valor_adc;
}


 
//SOFTSTARTER e IoT
void mensagem(esp_mqtt_client_handle_t cliente) 
{
        if(strstr(Inform, "ativa"))
        {
            ESP_LOGI(TAG, "Recebeu inform");
            if(strstr(Inform, "true"))
            {
                saidas = saidas|0b00000001;

                //sprintf(&mensa[0],"%s %f }}",string_motorstatus, 1);

                //esp_mqtt_client_publish(cliente, W_TOPICO_PUBLICAR, &mensa[0], 0, 0, 0);

                ESP_LOGI(TAG, "Ligou");
            }
            else
            {
                saidas = saidas&0b11111110;

                //sprintf(&mensa[0],"%s %f }}",string_motorstatus, 0);

                //esp_mqtt_client_publish(cliente, W_TOPICO_PUBLICAR, &mensa[0], 0, 0, 0);
                
                ESP_LOGI(TAG, "Desligou");
            }
        }
}
//Inicio do código principal 
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void adxl335_task(void *pvParameters) 
{
    set_offset();
    vTaskDelay(pdMS_TO_TICKS(15000));
        
    while (1) 
    {
        read_adxl335();  // Lê a aceleração com média
        calculate_frequency();
        vTaskDelay(pdMS_TO_TICKS(10));  // Aguarda 10 ms (100 leituras por segundo)
    }
}

void zmpt_task(void *pvParameter) 
{
    const TickType_t delay = pdMS_TO_TICKS(10);  // Intervalo de 10 ms para cada ciclo
    const int leituras_por_ciclo = 1600;            // Quantidade de leituras por ciclo para atingir 500 leituras por segundo

    while (1) 
    {
        for (int i = 0; i < leituras_por_ciclo; i++) 
        {
            adc_atual = read_voltage()-1940;

            soma_adc += adc_atual * adc_atual;
            cont_adc++;

            esp_rom_delay_us(10); // Delay de 100 microssegundos
        }

        float simplifca_conta = sqrt(soma_adc / cont_adc)-60;
        
        if(simplifca_conta > 2)
        {
            tensao_rms = simplifca_conta;
        }
        else
        {
            tensao_rms = 0;
        }

        soma_adc = 0;
        cont_adc = 0;

        //printf("valor rms: %f\n", tensao_rms);

        vTaskDelay(delay);  // Aguarda 10 ms antes do próximo ciclo
    }
}

void calcular_corrente_task(void *pvParameters) 
{
    const TickType_t delay = pdMS_TO_TICKS(10);
    const int n_de_leituras = 1800;

    while (1) 
    {
        float soma_quadrados = 0;
        int cont_adc2 = 0;

        for (int i = 0; i < n_de_leituras; i++) {
            int leitura_adc = adc1_get_raw(ACS712_PIN);        // Leitura bruta do ADC
            float adc_sem_offset = leitura_adc - 2109;          // Remove o offset de 1,7V
            soma_quadrados += adc_sem_offset * adc_sem_offset;
            cont_adc2++;
            esp_rom_delay_us(10); // Delay de 100 microssegundos
        }

        float co_tensao_rms = sqrt(soma_quadrados / cont_adc2); // Calcula tensão RMS

        valor_final = (co_tensao_rms * V_REF) / 4095.0;    // Converte ADC para tensão

        float valor_aux = ((valor_final / 0.12) - 1);                    // Calcula corrente RMS com sensibilidade média

        ps_corrente_rms = (valor_aux - 1) / 5 + valor_aux;

        vTaskDelay(delay);                                     // Atraso antes do próximo ciclo
    }
}

void dht22_task(void *pvParameters)
{
    last_temp_celsius = 0.0; // Armazena a última temperatura lida
    last_humidity = 0.0;    // Armazena a última umidade lida

    while (1) {
        if (xSemaphoreTake(xMutexDHT22, portMAX_DELAY)) {
            ESP_LOGI("DHT22_task", "Mutex adquirido para leitura do DHT22");

            int temperature = 0, humidity = 0;
            DHT22_read_data(&temperature, &humidity);

            if (temperature != 0 && humidity != 0) { // Verifica se a leitura foi bem-sucedida
                float temp_celsius = (temperature & 0x8000) ? 
                    -((temperature & 0x7FFF) / 10.0) : (temperature / 10.0);
                float hum = humidity / 10.0;

                // Atualiza os valores apenas se a leitura for válida
                last_temp_celsius = temp_celsius;
                last_humidity = hum;

                ESP_LOGI("DHT22_task", "Nova leitura - Temperatura: %.1f C, Umidade: %.1f%%", temp_celsius, hum);
            } else {
                // Mantém os valores antigos e loga a falha
                ESP_LOGW("DHT22_task", "Leitura falhou, mantendo última leitura");
            }

            ESP_LOGI("DHT22_task", "Últimos valores - Temperatura: %.1f C, Umidade: %.1f%%", 
                     last_temp_celsius, last_humidity);

            xSemaphoreGive(xMutexDHT22);
        } else {
            ESP_LOGE("DHT22_task", "Falha ao adquirir o mutex");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay de 1 segundo
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    
    ioinit();
    io_le_escreve(0);

    initialise_wifi();

    init_adc();  // Inicializa o ADC

    vTaskDelay(pdMS_TO_TICKS(15000));

    mqtt_app_start();

    DHT22_init();
    
    xTaskCreate(adxl335_task, "adxl335_task", 2048, NULL, 4, NULL);

    xTaskCreate(zmpt_task, "Monitor de Pico de Tensão", 2048, NULL, 4, NULL);
    
    xTaskCreate(calcular_corrente_task, "Calcular Corrente", 2048, NULL, 4, NULL);

    xTaskCreate(dht22_task, "", 2048, NULL, 6, NULL);
    
    while(1)
    {
        io_le_escreve(saidas);
        
        //Temperatura e umidade
        ESP_LOGI("DHT22", "Temperatura: %.1f C, Umidade: %.1f%%", last_temp_celsius, last_humidity);

        sprintf(&mensa[0],"%s %f }}",string_temp,last_temp_celsius);

        esp_mqtt_client_publish(cliente, W_TOPICO_PUBLICAR, &mensa[0], 0, 0, 0);

        sprintf(&mensa[0],"%s %f }}",string_hum,last_humidity);

        esp_mqtt_client_publish(cliente, W_TOPICO_PUBLICAR, &mensa[0], 0, 0, 0);

        vTaskDelay(pdMS_TO_TICKS(1000));  // Aguarde 1 segundo 

        //Vibração
        sprintf(&mensa[0],"%s %f }}",string_adxlX, x_frequency);

        esp_mqtt_client_publish(cliente, W_TOPICO_PUBLICAR, &mensa[0], 0, 0, 0);

        sprintf(&mensa[0],"%s %f }}",string_adxlY, y_frequency);

        esp_mqtt_client_publish(cliente, W_TOPICO_PUBLICAR, &mensa[0], 0, 0, 0);

        sprintf(&mensa[0],"%s %f }}",string_adxlZ, z_frequency);

        esp_mqtt_client_publish(cliente, W_TOPICO_PUBLICAR, &mensa[0], 0, 0, 0);

        vTaskDelay(pdMS_TO_TICKS(1000));  // Aguarde 3 segundos para a próxima leitura

        //Tensão
        float rms_w;

        if(tensao_rms != 0)
        {
            rms_w = ((220 - tensao_rms) / 4.4 + tensao_rms);
        }
        else
        {
            rms_w = 0;
        }

        sprintf(&mensa[0],"%s %f }}",string_tensao, rms_w);

        esp_mqtt_client_publish(cliente, W_TOPICO_PUBLICAR, &mensa[0], 0, 0, 0);

        vTaskDelay(pdMS_TO_TICKS(500));

        //Corrente
        if(ps_corrente_rms > 0.2)
        {
            corrente_rms = ps_corrente_rms * 1.4 + 0.4;
        }
        else
        {
            corrente_rms = 0;
        }

        float corrente_real = corrente_rms;

        printf("Corrente: %fA\n", corrente_real);

        sprintf(&mensa[0],"%s %f }}",string_crr, corrente_real);

        esp_mqtt_client_publish(cliente, W_TOPICO_PUBLICAR, &mensa[0], 0, 0, 0);

        vTaskDelay(pdMS_TO_TICKS(2000));  // Aguarde 3 segundos para a próxima leitura

    }
}

//Ignorar abaixo:
//eventos relacionados a conexão wifi
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG_SC, "Scan done");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        ESP_LOGI(TAG_SC, "Found channel");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        ESP_LOGI(TAG_SC, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        uint8_t ssid[33] = { 0 };
        uint8_t password[65] = { 0 };
        uint8_t rvd_data[33] = { 0 };

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));

#ifdef CONFIG_SET_MAC_ADDRESS_OF_TARGET_AP
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) {
            ESP_LOGI(TAG_SC, "Set MAC address of target AP: "MACSTR" ", MAC2STR(evt->bssid));
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }
#endif

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG_SC, "SSID:%s", ssid);
        ESP_LOGI(TAG_SC, "PASSWORD:%s", password);
        if (evt->type == SC_TYPE_ESPTOUCH_V2) {
            ESP_ERROR_CHECK( esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)) );
            ESP_LOGI(TAG_SC, "RVD_DATA:");
            for (int i=0; i<33; i++) {
                printf("%02x ", rvd_data[i]);
            }
            printf("\n");
        }

        ESP_ERROR_CHECK( esp_wifi_disconnect() );
        ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
        esp_wifi_connect();
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
    }
}

static void initialise_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void smartconfig_example_task(void * parm)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_smartconfig_start(&cfg) );
    while (1) {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        if(uxBits & CONNECTED_BIT) {
            ESP_LOGI(TAG_SC, "WiFi Connected to ap");
        }
        if(uxBits & ESPTOUCH_DONE_BIT) {
            ESP_LOGI(TAG_SC, "smartconfig over");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
    }
}

//inicialização e configuração do sesnor dht
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void DHT22_set_pin_output() 
{
    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);
}

void DHT22_set_pin_input() 
{
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
}

void DHT22_send_start_signal() 
{
    DHT22_set_pin_output();
    gpio_set_level(DHT_PIN, 0);
    esp_rom_delay_us(1000);  // 1ms
    gpio_set_level(DHT_PIN, 1);
    esp_rom_delay_us(30);  // 30us
    DHT22_set_pin_input();
}

int DHT22_wait_for_response() 
{
    int timeout = 0;
    while (gpio_get_level(DHT_PIN) == 1) {
        esp_rom_delay_us(1);
        if (++timeout > 80) {
            return 0;  // Timeout
        }
    }

    timeout = 0;
    while (gpio_get_level(DHT_PIN) == 0) {
        esp_rom_delay_us(1);
        if (++timeout > 80) {
            return 0;  // Timeout
        }
    }

    timeout = 0;
    while (gpio_get_level(DHT_PIN) == 1) {
        esp_rom_delay_us(1);
        if (++timeout > 80) {
            return 0;  // Timeout
        }
    }

    return 1;  // Resposta válida
}

int DHT22_read_bit() {
    int timeout = 0;
    // Aguarde o início do bit (pino baixo)
    while (gpio_get_level(DHT_PIN) == 0) {
        esp_rom_delay_us(1);  // Substitua ets_delay_us() por esp_rom_delay_us()
        if (++timeout > 80) return -1;  // Timeout
    }

    // O bit 1 dura mais tempo no nível alto do que o bit 0
    esp_rom_delay_us(30);  // 30us para determinar se é 0 ou 1
    int bit = gpio_get_level(DHT_PIN);

    // Aguarde o fim do bit
    timeout = 0;
    while (gpio_get_level(DHT_PIN) == 1) {
        esp_rom_delay_us(1);  // Substitua ets_delay_us() por esp_rom_delay_us()
        if (++timeout > 80) return -1;  // Timeout
    }

    return bit;
}

int DHT22_read_byte()
{
    int result = 0;
    for (int i = 0; i < 8; i++) {
        int bit = DHT22_read_bit();
        if (bit < 0) return -1;  // Erro de leitura
        result = (result << 1) | bit;
    }
    return result;
}

void DHT22_read_data(int *temperature, int *humidity) 
{
    uint8_t data[5] = {0};

    DHT22_send_start_signal();
    if (DHT22_wait_for_response()) {
        // Ler os 5 bytes de dados (2 para umidade, 2 para temperatura, 1 checksum)
        for (int i = 0; i < 5; i++) {
            int byte = DHT22_read_byte();
            if (byte < 0) {
                ESP_LOGE("DHT22", "Erro de leitura");
                return;
            }
            data[i] = byte;
        }

        // Verificar checksum
        if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
            *humidity = (data[0] << 8) | data[1];
            *temperature = (data[2] << 8) | data[3];
        } else {
            ESP_LOGE("DHT22", "Checksum inválido");
        }
    } else {
        ESP_LOGE("DHT22", "Sem resposta do sensor");
    }
}

//MQTT
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%ld", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:        
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, W_TOPICO_PUBLICAR, 
            &mensa[0], 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        ESP_LOGI(TAG, "%s", &mensa[0]);
       
        msg_id = esp_mqtt_client_subscribe(client, W_TOPICO_SUBSCREVER, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        Inform = event->data;
        mensagem(cliente);
        //qualificador(client); //aqui está a mágica
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://broker.app.wnology.io",// se tu tiver outro broker pode colocar aqui, mas pro wnology é esse mesmo
        .credentials.set_null_client_id = false,  //coloquei e nem sei se precisa
        .credentials.client_id = W_DEVICE_ID,
        .credentials.username = W_ACCESS_KEY,
        .credentials.authentication.password = W_PASSWORD,
        
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    cliente = client;
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

//ADXL335
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void init_adc() 
{
    // Função para inicializar o ADC
    adc1_config_width(ADC_WIDTH_BIT_12);  // Resolução de 12 bits

    // Atenuação para todos os canais (11dB é comum para a faixa completa de voltagem)
    adc1_config_channel_atten(ADXL335_X_PIN, ADC_ATTEN_DB_11);  // Canal 0 (GPIO36)
    adc1_config_channel_atten(ADXL335_Y_PIN, ADC_ATTEN_DB_11);  // Canal 3 (GPIO39)
    adc1_config_channel_atten(ADXL335_Z_PIN, ADC_ATTEN_DB_11);  // Canal 6 (GPIO34)

    adc1_config_channel_atten(ZMPT101B_PIN, ADC_ATTEN_DB_11);   // Canal 4 (GPIO32)

    adc1_config_channel_atten(ACS712_PIN, ADC_ATTEN_DB_11); //
}

void set_offset() 
{
    long x_sum = 0;
    long y_sum = 0;
    long z_sum = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) 
    {
        x_sum += adc1_get_raw(ADXL335_X_PIN);
        y_sum += adc1_get_raw(ADXL335_Y_PIN);
        z_sum += adc1_get_raw(ADXL335_Z_PIN);
        
        vTaskDelay(pdMS_TO_TICKS(10));  // Espera 10 ms entre as leituras
    }

    x_offset = x_sum / NUM_SAMPLES;
    y_offset = y_sum / NUM_SAMPLES;
    z_offset = z_sum / NUM_SAMPLES;
}

float convert_to_g(int raw_value) 
{
    // Função para converter o valor ADC em g (aceleração)
    // Converte o valor ADC para voltagem (considerando referência de 3.3V e resolução de 12 bits)
    float voltage = (float)raw_value * 3.3 / 4095.0;
    // Considera o offset de 1.65V e sensibilidade de 330mV/g
    return (voltage) / 0.33;
}

void read_adxl335() {
    // Leitura dos valores ADC dos eixos X, Y e Z
    int x_val = adc1_get_raw(ADXL335_X_PIN) - x_offset;
    int y_val = adc1_get_raw(ADXL335_Y_PIN) - y_offset;
    int z_val = adc1_get_raw(ADXL335_Z_PIN) - z_offset;
    // Converte os valores ADC para aceleração (g)
    float x_g = convert_to_g(x_val);
    float y_g = convert_to_g(y_val);
    float z_g = convert_to_g(z_val);

    // Detecta passagens pelo zero
    detect_zero_crossing(x_g, y_g, z_g);

    // Calcula a frequência a cada segundo
    calculate_frequency();

    // Exibe as leituras brutas no terminal para depuração
    //printf("Aceleração: X = %.2f g, Y = %.2f g, Z = %.2f g\n", x_g, y_g, z_g);
}

void detect_zero_crossing(float x_acceleration, float y_acceleration, float z_acceleration) 
{
    // Função para detectar passagens pelo zero com tolerância
    // Verifica passagem pelo zero para o eixo X
    if (previous_x > THRESHOLD && x_acceleration <= -THRESHOLD) {
        x_crossings++;
    } else if (previous_x < -THRESHOLD && x_acceleration >= THRESHOLD) {
        x_crossings++;
    }

    // Verifica passagem pelo zero para o eixo Y
    if (previous_y > THRESHOLD && y_acceleration <= -THRESHOLD) {
        y_crossings++;
    } else if (previous_y < -THRESHOLD && y_acceleration >= THRESHOLD) {
        y_crossings++;
    }

    // Verifica passagem pelo zero para o eixo Z
    if (previous_z > THRESHOLD && z_acceleration <= -THRESHOLD) {
        z_crossings++;
    } else if (previous_z < -THRESHOLD && z_acceleration >= THRESHOLD) {
        z_crossings++;
    }

    // Atualiza as leituras anteriores
    previous_x = x_acceleration;
    previous_y = y_acceleration;
    previous_z = z_acceleration;
}

void calculate_frequency()
{
    // Função para calcular a frequência com base nas passagens
    unsigned long current_time = esp_timer_get_time() / 1000; // Obtém o tempo atual em ms

    // Se 1 segundo se passou, calcula a frequência
    if (current_time - last_time >= 1000) {
        x_frequency = (float)x_crossings; // Frequência em Hz para X
        y_frequency = (float)y_crossings; // Frequência em Hz para Y
        z_frequency = (float)z_crossings; // Frequência em Hz para Z

        printf("Frequência: X = %.2f Hz, Y = %.2f Hz, Z = %.2f Hz\n", x_frequency, y_frequency, z_frequency);

        // Reseta as contagens e o tempo
        x_crossings = 0;
        y_crossings = 0;
        z_crossings = 0;
        last_time = current_time; // Atualiza o tempo da última contagem
    }
}