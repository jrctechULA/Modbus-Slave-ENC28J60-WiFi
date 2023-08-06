//____________________________________________________________________________________________________
// Include section:
//____________________________________________________________________________________________________
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_eth_enc28j60.h"
#include "driver/spi_master.h"
#include "mbcontroller.h"
#include "JRC_WiFi.h"

//____________________________________________________________________________________________________
// Macro definitions:
//____________________________________________________________________________________________________

#define CONFIG_FREERTOS_HZ 100
#define ENC28J60_DUPLEX_FULL 1
#define ENC28J60_MISO_GPIO 13
#define ENC28J60_MOSI_GPIO 11
#define ENC28J60_SCLK_GPIO 12
#define ENC28J60_CS_GPIO   10
#define ENC28J60_INT_GPIO  14

#define ENC28J60_SPI_CLOCK_MHZ 16
#define ENC28J60_SPI_HOST 2

#define MB_REG_INPUT_START_AREA0    (0)
#define MB_REG_HOLDING_START_AREA0  (0)
#define MB_REG_HOLD_CNT             (50)
#define MB_REG_INPUT_CNT            (50)

//____________________________________________________________________________________________________
// Global declarations:
//____________________________________________________________________________________________________

static const char *TAG = "Modbus Slave Eth";

mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure
uint16_t holding_reg_area[MB_REG_HOLD_CNT] = {0}; // storage area for holding registers
uint16_t input_reg_area[MB_REG_INPUT_CNT] = {0}; // storage area for input registers

// Statically allocate and initialize the spinlock
static portMUX_TYPE mb_spinlock = portMUX_INITIALIZER_UNLOCKED;

esp_netif_t *eth_netif = NULL;
esp_netif_t* esp_netif_ptr = NULL;   // WiFi

//____________________________________________________________________________________________________
// Function prototypes:
//____________________________________________________________________________________________________
void ethernetInit(void);

esp_err_t modbus_slave_init(void);

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data);

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data);


//____________________________________________________________________________________________________
// Main program:
//____________________________________________________________________________________________________
void app_main(void)
{
    JRC_WiFi_Begin();
    
    ethernetInit();

    modbus_slave_init();

    while (1){

        portENTER_CRITICAL(&mb_spinlock);
        for (int i =0; i<MB_REG_HOLD_CNT; i++){
            holding_reg_area[i]++;
            input_reg_area[i]+=5;
        }
        portEXIT_CRITICAL(&mb_spinlock);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


//____________________________________________________________________________________________________
// Function implementations:
//____________________________________________________________________________________________________

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
}

void ethernetInit() {
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    // Initialize TCP/IP network interface (should be called only once in application)
    //ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    //ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netif = esp_netif_new(&netif_cfg);

    spi_bus_config_t buscfg = {
        .miso_io_num = ENC28J60_MISO_GPIO,
        .mosi_io_num = ENC28J60_MOSI_GPIO,
        .sclk_io_num = ENC28J60_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(ENC28J60_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    /* ENC28J60 ethernet driver is based on spi driver */
    spi_device_interface_config_t spi_devcfg = {
        .mode = 0,
        .clock_speed_hz = ENC28J60_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = ENC28J60_CS_GPIO,
        .queue_size = 20,
        .cs_ena_posttrans = enc28j60_cal_spi_cs_hold_time(ENC28J60_SPI_CLOCK_MHZ),
    };

    eth_enc28j60_config_t enc28j60_config = ETH_ENC28J60_DEFAULT_CONFIG(ENC28J60_SPI_HOST, &spi_devcfg);
    enc28j60_config.int_gpio_num = ENC28J60_INT_GPIO;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_enc28j60(&enc28j60_config, &mac_config);

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.autonego_timeout_ms = 0; // ENC28J60 doesn't support auto-negotiation
    phy_config.reset_gpio_num = -1; // ENC28J60 doesn't have a pin to reset internal PHY
    esp_eth_phy_t *phy = esp_eth_phy_new_enc28j60(&phy_config);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    /* ENC28J60 doesn't burn any factory MAC address, we need to set it manually.
       02:00:00 is a Locally Administered OUI range so should not be used except when testing on a LAN under your control.
    */
    mac->set_addr(mac, (uint8_t[]) {
        0x02, 0x00, 0x00, 0x12, 0x34, 0x56
    });

    // ENC28J60 Errata #1 check
    if (emac_enc28j60_get_chip_info(mac) < ENC28J60_REV_B5 && ENC28J60_SPI_CLOCK_MHZ < 8) {
        ESP_LOGE(TAG, "SPI frequency must be at least 8 MHz for chip revision less than 5");
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    /* attach Ethernet driver to TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    /* It is recommended to use ENC28J60 in Full Duplex mode since multiple errata exist to the Half Duplex mode */
#if ENC28J60_DUPLEX_FULL
    eth_duplex_t duplex = ETH_DUPLEX_FULL;
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_DUPLEX_MODE, &duplex));
#endif

    /* start Ethernet driver state machine */
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}

esp_err_t modbus_slave_init(void){

    // Stage 1. Modbus Port Initialization:

    void* slave_handler = NULL; // Pointer to allocate interface structure
    // Initialization of Modbus slave for TCP
    esp_err_t err = mbc_slave_init_tcp(&slave_handler);
    if (slave_handler == NULL || err != ESP_OK) {
        // Error handling is performed here
        ESP_LOGE(TAG, "mb controller initialization fail.");
    }

    //Stage 2. Configuring Slave Data Access:

    reg_area.type = MB_PARAM_HOLDING;                               // Set type of register area
    reg_area.start_offset = MB_REG_HOLDING_START_AREA0;             // Offset of register area in Modbus protocol
    reg_area.address = (void*)&holding_reg_area[0];                 // Set pointer to storage instance
    reg_area.size = sizeof(holding_reg_area) << 1;                  // Set the size of register storage area in bytes
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    reg_area.type = MB_PARAM_INPUT;
    reg_area.start_offset = MB_REG_INPUT_START_AREA0;
    reg_area.address = (void*)&input_reg_area[0];
    reg_area.size = sizeof(input_reg_area) << 1;
    ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

    //Stage 3. Slave Communication Options:

    mb_communication_info_t comm_info = {
        .ip_port = 502,                            // Modbus TCP port number (default = 502)
        .ip_addr_type = MB_IPV4,                   // version of IP protocol
        .ip_mode = MB_MODE_TCP,                    // Port communication mode
        .ip_addr = NULL,                           // This field keeps the client IP address to bind, NULL - bind to any client
        .ip_netif_ptr = eth_netif                // eth_netif - pointer to the corresponding network interface
    };

    // Setup communication parameters and start stack
    ESP_ERROR_CHECK(mbc_slave_setup((void*)&comm_info));

    //Stage 4. Slave Communication Start:

    ESP_ERROR_CHECK(mbc_slave_start());

    return ESP_OK;
}

