#include "Network.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include <esp_log.h>

#define TAG "Ethernet"

#define CONFIG_ETH_PHY_ADDR     (-1)
#define CONFIG_ETH_MDC_GPIO     GPIO_RMII_MDC
#define CONFIG_ETH_MDIO_GPIO    GPIO_RMII_MDIO

//#define ETH_STATIC_IP 1
#define ETH_DHCP_IP 1

esp_netif_t* Ethernet::g_eth_netif;

void Ethernet::eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    esp_err_t err;
    uint8_t mac_addr[6] = {0};
    esp_netif_ip_info_t ip_info;
    esp_netif_dns_info_t dns_info;
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;
    esp_netif_t *eth_netif = g_eth_netif;

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
        if( ESP_OK != (err = esp_netif_dhcps_stop(eth_netif)) )
        {
            ESP_LOGE(TAG, "eth_event_handler Stop dhcp server Err: %s", esp_err_to_name(err));
        }

#if (1 == ETH_STATIC_IP)
        ESP_LOGI(TAG, "eth_event_handler static ip");

        if( ESP_OK != (err = esp_netif_dhcpc_stop(eth_netif)) )
        {
            ESP_LOGE(TAG, "eth_event_handler Stop dhcp Err: %s", esp_err_to_name(err));
        }

        ip_info.ip.addr     = esp_ip4addr_aton("192.168.1.100");
        ip_info.netmask.addr= esp_ip4addr_aton("255.255.255.0");
        ip_info.gw.addr     = esp_ip4addr_aton("192.168.1.1");
        dns_info.ip.u_addr.ip4.addr    = esp_ip4addr_aton("192.168.1.1");
        if( ESP_OK != (err = esp_netif_set_ip_info(eth_netif, &ip_info)) )
        {
            ESP_LOGE(TAG, "eth_event_handler set ip Err: %s", esp_err_to_name(err));
        }
        if( ESP_OK != (err = esp_netif_set_dns_info(eth_netif, ESP_NETIF_DNS_MAIN, &dns_info)) )
        {
            ESP_LOGE(TAG, "eth_event_handler set dns Err: %s", esp_err_to_name(err));
        }
#endif
#if (1 == ETH_DHCP_IP)
        ESP_LOGI(TAG, "eth_event_handler dhcp ip");
        if( ESP_OK != (err = esp_netif_dhcpc_start(eth_netif)) )
        {
            ESP_LOGE(TAG, "eth_event_handler Start dhcp Err: %s", esp_err_to_name(err));
        }
#endif
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

void Ethernet::got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
}

void Ethernet::init(ddt_t ddt) {
    ESP_LOGI(TAG, "Ethernet init begin");
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    g_eth_netif = eth_netif;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = CONFIG_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = ddt.gpio_phyrst;

    esp32_emac_config.clock_config.rmii.clock_gpio = EMAC_CLK_OUT_180_GPIO;
    esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_OUT;
    esp32_emac_config.smi_mdc_gpio_num = CONFIG_ETH_MDC_GPIO;
    esp32_emac_config.smi_mdio_gpio_num = CONFIG_ETH_MDIO_GPIO;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_jl1101(&phy_config);

    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));

    /* attach Ethernet driver to TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    /* start Ethernet driver state machine */
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    ESP_LOGI(TAG, "Ethernet init finished");
}