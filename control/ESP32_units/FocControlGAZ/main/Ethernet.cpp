#include "esp_log.h"
#include "esp_eth_driver.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "driver/spi_master.h"
#include "Ethernet.h"
#include "nvstorage.h"

static const char *TAG = "Ethernet";

esp_netif_t* Ethernet::eth_netif;

esp_err_t Ethernet::spi_bus_init(uint8_t pinMiso, uint8_t pinMosi, uint8_t pinSck, spi_host_device_t spiBus) {
    esp_err_t ret = ESP_OK;

    spi_bus_config_t buscfg = {
        .mosi_io_num = pinMosi,
        .miso_io_num = pinMiso,
        .sclk_io_num = pinSck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    // Install GPIO ISR handler to be able to service SPI Eth modules interrupts
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "GPIO ISR handler has been already installed");
            ret = ESP_OK; // ISR handler has been already installed so no issues
        } else {
            ESP_LOGE(TAG, "GPIO ISR handler install failed");
            goto err;
        }
    }

    // Init SPI bus
    ESP_GOTO_ON_ERROR(spi_bus_initialize(spiBus, &buscfg, SPI_DMA_CH_AUTO), err, TAG, "SPI host #%d init failed", 1);

err:
    return ret;
}

esp_err_t Ethernet::eth_init_spi(spi_eth_module_config_t *spi_eth_module_config, spi_host_device_t spiBus) {
    esp_err_t ret = ESP_OK;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

    phy_config.phy_addr = spi_eth_module_config->phy_addr;
    phy_config.reset_gpio_num = spi_eth_module_config->phy_reset_gpio;

    spi_device_interface_config_t spi_devcfg = {
        .mode = 0,
        .clock_speed_hz = 36 * 1000 * 1000,
        .spics_io_num = spi_eth_module_config->spi_cs_gpio,
        .queue_size = 20
    };

    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(spiBus, &spi_devcfg);
    w5500_config.int_gpio_num = spi_eth_module_config->int_gpio;
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    // Init Ethernet driver to default and install it
    eth_handle = NULL;
    esp_eth_config_t eth_config_spi = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_GOTO_ON_FALSE(esp_eth_driver_install(&eth_config_spi, &eth_handle) == ESP_OK, NULL, err, TAG, "SPI Ethernet driver install failed");

    // The SPI Ethernet module might not have a burned factory MAC address, we can set it manually.
    if (spi_eth_module_config->mac_addr != NULL) {
        ESP_GOTO_ON_FALSE(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, spi_eth_module_config->mac_addr) == ESP_OK,
                                        NULL, err, TAG, "SPI Ethernet MAC address config failed");
    }

    return ret;
err:
    if (eth_handle != NULL) {
        esp_eth_driver_uninstall(eth_handle);
    }
    if (mac != NULL) {
        mac->del(mac);
    }
    if (phy != NULL) {
        phy->del(phy);
    }
    return ret;
}

esp_err_t Ethernet::init(uint8_t pinMiso, uint8_t pinMosi, uint8_t pinSck, uint8_t pinCs, uint8_t pinInt, spi_host_device_t spiBus) {
    ESP_LOGI(TAG, "Initializing ethernet");

    esp_err_t ret = ESP_OK;
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netif = esp_netif_new(&cfg);
    eth_cnt = 0;

    ESP_GOTO_ON_ERROR(spi_bus_init(pinMiso, pinMosi, pinSck, spiBus), err, TAG, "SPI bus init failed");

    spi_eth_module_config_t spi_eth_module_config;

    spi_eth_module_config.spi_cs_gpio = pinCs;
    spi_eth_module_config.int_gpio = pinInt;
    spi_eth_module_config.phy_reset_gpio = -1;
    spi_eth_module_config.phy_addr = 1; 
    
    uint8_t base_mac_addr[ETH_ADDR_LEN];
    ESP_GOTO_ON_ERROR(esp_efuse_mac_get_default(base_mac_addr), err, TAG, "get EFUSE MAC failed");
    uint8_t local_mac_1[ETH_ADDR_LEN];
    esp_derive_local_mac(local_mac_1, base_mac_addr);
    spi_eth_module_config.mac_addr = local_mac_1;

    eth_init_spi(&spi_eth_module_config, spiBus);
    ESP_GOTO_ON_FALSE(eth_handle, ESP_FAIL, err, TAG, "SPI Ethernet init failed");
    eth_cnt = 1;

    
    ESP_GOTO_ON_ERROR(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)), err, TAG, "Netif error");    

    return ret;
err:
    return ret;
}

void Ethernet::start() {
    if (eth_cnt) {
        esp_eth_start(eth_handle);
    }
}

Ethernet::Ethernet() {

}

void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;
    esp_err_t err;
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
        if(systemParams.dhcp) {
            ESP_LOGI(TAG, "eth_event_handler dhcp ip");
            if(ESP_OK != (err = esp_netif_dhcpc_start(Ethernet::eth_netif))){
                ESP_LOGE(TAG, "eth_event_handler Start dhcp Err: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGI(TAG, "eth_event_handler static ip");
            if(ESP_OK != (err = esp_netif_dhcpc_stop(Ethernet::eth_netif))) {
                ESP_LOGE(TAG, "eth_event_handler Stop dhcp Err: %s", esp_err_to_name(err));
            }
            esp_netif_ip_info_t ip_info;
            esp_netif_set_ip4_addr(&ip_info.ip, systemParams.ip[0], systemParams.ip[1], systemParams.ip[2], systemParams.ip[3]);
            esp_netif_set_ip4_addr(&ip_info.netmask, systemParams.mask[0], systemParams.mask[1], systemParams.mask[2], systemParams.mask[3]);
            esp_netif_set_ip4_addr(&ip_info.gw, systemParams.gateway[0], systemParams.gateway[1], systemParams.gateway[2], systemParams.gateway[3]);
            if(ESP_OK != (err = esp_netif_set_ip_info(Ethernet::eth_netif, &ip_info))) {
                ESP_LOGE(TAG, "eth_event_handler set ip Err: %s", esp_err_to_name(err));
            }
        }
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    if (systemParams.dhcp) {
        systemParams.ip[0] = ip_info->ip.addr & 0xFF;
        systemParams.ip[1] = (ip_info->ip.addr >> 8) & 0xFF;
        systemParams.ip[2] = (ip_info->ip.addr >> 16) & 0xFF;
        systemParams.ip[3] = (ip_info->ip.addr >> 24) & 0xFF;

        systemParams.gateway[0] = ip_info->gw.addr & 0xFF;
        systemParams.gateway[1] = (ip_info->gw.addr >> 8) & 0xFF;
        systemParams.gateway[2] = (ip_info->gw.addr >> 16) & 0xFF;
        systemParams.gateway[3] = (ip_info->gw.addr >> 24) & 0xFF;

        systemParams.mask[0] = ip_info->netmask.addr & 0xFF;
        systemParams.mask[1] = (ip_info->netmask.addr >> 8) & 0xFF;
        systemParams.mask[2] = (ip_info->netmask.addr >> 16) & 0xFF;
        systemParams.mask[3] = (ip_info->netmask.addr >> 24) & 0xFF;
    }

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
}