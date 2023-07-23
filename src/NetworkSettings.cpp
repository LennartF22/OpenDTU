// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022 Thomas Basler and others
 */
#include "NetworkSettings.h"
#include "Configuration.h"
#include "MessageOutput.h"
#include "PinMapping.h"
#include "Utils.h"
#include "defaults.h"
#include <ETH.h>

#include <driver/spi_master.h>

NetworkSettingsClass::NetworkSettingsClass()
    : apIp(192, 168, 4, 1)
    , apNetmask(255, 255, 255, 0)
{
    dnsServer.reset(new DNSServer());
}

void NetworkSettingsClass::init()
{
    using std::placeholders::_1;

    WiFi.onEvent(std::bind(&NetworkSettingsClass::NetworkEvent, this, _1));
    setupMode();
}

void NetworkSettingsClass::NetworkEvent(WiFiEvent_t event)
{
    switch (event) {
    case ARDUINO_EVENT_ETH_START:
        MessageOutput.println("ETH start");
        if (_networkMode == network_mode::Ethernet) {
            raiseEvent(network_event::NETWORK_START);
        }
        break;
    case ARDUINO_EVENT_ETH_STOP:
        MessageOutput.println("ETH stop");
        if (_networkMode == network_mode::Ethernet) {
            raiseEvent(network_event::NETWORK_STOP);
        }
        break;
    case ARDUINO_EVENT_ETH_CONNECTED:
        MessageOutput.println("ETH connected");
        _ethConnected = true;
        raiseEvent(network_event::NETWORK_CONNECTED);
        break;
    case ARDUINO_EVENT_ETH_GOT_IP:
        MessageOutput.printf("ETH got IP: %s\r\n", ETH.localIP().toString().c_str());
        if (_networkMode == network_mode::Ethernet) {
            raiseEvent(network_event::NETWORK_GOT_IP);
        }
        break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
        MessageOutput.println("ETH disconnected");
        _ethConnected = false;
        if (_networkMode == network_mode::Ethernet) {
            raiseEvent(network_event::NETWORK_DISCONNECTED);
        }
        break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        MessageOutput.println("WiFi connected");
        if (_networkMode == network_mode::WiFi) {
            raiseEvent(network_event::NETWORK_CONNECTED);
        }
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        MessageOutput.println("WiFi disconnected");
        if (_networkMode == network_mode::WiFi) {
            MessageOutput.println("Try reconnecting");
            WiFi.reconnect();
            raiseEvent(network_event::NETWORK_DISCONNECTED);
        }
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        MessageOutput.printf("WiFi got ip: %s\r\n", WiFi.localIP().toString().c_str());
        if (_networkMode == network_mode::WiFi) {
            raiseEvent(network_event::NETWORK_GOT_IP);
        }
        break;
    default:
        break;
    }
}

extern void tcpipInit();
extern void add_esp_interface_netif(esp_interface_t interface, esp_netif_t* esp_netif); /* from WiFiGeneric */

extern "C" void periph_module_disable(periph_module_t periph);

#include "driver/uart.h"

void NetworkSettingsClass::setupSpiEth()
{
    uint8_t base_mac[6];
    esp_base_mac_addr_get(base_mac);
    MessageOutput.printf("%02x:%02x:%02x:%02x:%02x:%02x\n", base_mac[0], base_mac[1], base_mac[2], base_mac[3], base_mac[4], base_mac[5]);
    //MessageOutput.println("################## 1 #################");

    gpio_reset_pin(static_cast<gpio_num_t>(12));
    gpio_set_direction(static_cast<gpio_num_t>(12), GPIO_MODE_OUTPUT);
    gpio_set_level(static_cast<gpio_num_t>(12), 0);
    
    //periph_module_disable(PERIPH_UART0_MODULE);
    ESP_ERROR_CHECK(uart_driver_delete(UART_NUM_0));
    gpio_reset_pin(static_cast<gpio_num_t>(43));
    gpio_reset_pin(static_cast<gpio_num_t>(44));

    gpio_set_direction(static_cast<gpio_num_t>(43), GPIO_MODE_OUTPUT);
    gpio_set_level(static_cast<gpio_num_t>(43), 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(static_cast<gpio_num_t>(43), 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);

    gpio_reset_pin(static_cast<gpio_num_t>(39));
    //gpio_set_drive_capability(static_cast<gpio_num_t>(39), GPIO_DRIVE_CAP_3);
    gpio_reset_pin(static_cast<gpio_num_t>(40));
    //gpio_set_drive_capability(static_cast<gpio_num_t>(40), GPIO_DRIVE_CAP_3);
    gpio_reset_pin(static_cast<gpio_num_t>(42));
    //gpio_set_drive_capability(static_cast<gpio_num_t>(42), GPIO_DRIVE_CAP_3);
    gpio_reset_pin(static_cast<gpio_num_t>(41));
    
    //MessageOutput.println("################## 2 #################");

    //ESP_ERROR_CHECK(gpio_install_isr_service(0)); // TODO: Kompatibel? -> offensichtlich nicht ahhhhhhhhhhhhhhhhhhh
    attachInterrupt(digitalPinToInterrupt(11), nullptr, DEFAULT);
    detachInterrupt(digitalPinToInterrupt(11));
    gpio_reset_pin(static_cast<gpio_num_t>(11));

    //MessageOutput.println("################## 3 #################");

    tcpipInit();

    //MessageOutput.println("################## 4 #################");

    //ESP_ERROR_CHECK(tcpip_adapter_set_default_eth_handlers()); // ???????????????????????????????????????????????????????????

    //MessageOutput.println("################## 5 #################");

    spi_bus_config_t buscfg = {
        .mosi_io_num = 40,
        .miso_io_num = 41,
        .sclk_io_num = 39,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 0, // uses default value internally
        .flags = 0,
        .intr_flags = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO)); // TODO: DMA_CH SPI_DMA_CH_AUTO

    spi_device_handle_t spi;

    spi_device_interface_config_t devcfg = {
        .command_bits = 16, // actually address phase
        .address_bits = 8, // actually command phase
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0, // UNBEDINGT 0 LASSEN
        .cs_ena_posttrans = 0, // UNBEDINGT 0 LASSEN
        .clock_speed_hz = 5000000, // TODO
        .input_delay_ns = 0,
        .spics_io_num = 42,
        .flags = 0,
        .queue_size = 20, // TODO
        .pre_cb = NULL,
        .post_cb = NULL
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &spi));
    
    //gpio_set_level(static_cast<gpio_num_t>(12), 1);

    /*spi_transaction_t test = {
        .flags = SPI_TRANS_USE_TXDATA,
        .cmd = 0b1100110011001100,
        .addr = 0b10101010,
        .length = 8,
        .rxlength = 0,
        .user = NULL,
        .tx_buffer = NULL,
        .rx_buffer = NULL
    };
    test.tx_data[0] = 0b11110000;
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &test));*/

    //MessageOutput.println("################## 6 #################");

    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(spi);
    w5500_config.int_gpio_num = 44;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);

    //MessageOutput.println("################## 7 #################");

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = 1; // ???
    phy_config.reset_gpio_num = -1;
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    //MessageOutput.println("################## 8 #################");

    // ######

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    //MessageOutput.println("################## 9 #################");

    uint8_t mac_addr[6] = {
        0x02, 0x00, 0x00, 0x12, 0x34, 0x57
    };
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr));

    //MessageOutput.println("################## 10 #################");

    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_config);

    //MessageOutput.println("################## 11 #################");

    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    //MessageOutput.println("################## 12 #################");

    /* attach to WiFiGeneric to receive events */
    add_esp_interface_netif(ESP_IF_ETH, eth_netif);

    //MessageOutput.println("################## 13 #################");

    esp_err_t err = esp_eth_start(eth_handle);

    gpio_set_level(static_cast<gpio_num_t>(12), 1);

    //MessageOutput.println("################## 14 #################");

    ESP_ERROR_CHECK(err);

    //MessageOutput.println("################## 15 #################");

    delay(100);
}

bool NetworkSettingsClass::onEvent(NetworkEventCb cbEvent, network_event event)
{
    if (!cbEvent) {
        return pdFALSE;
    }
    NetworkEventCbList_t newEventHandler;
    newEventHandler.cb = cbEvent;
    newEventHandler.event = event;
    _cbEventList.push_back(newEventHandler);
    return true;
}

void NetworkSettingsClass::raiseEvent(network_event event)
{
    for (uint32_t i = 0; i < _cbEventList.size(); i++) {
        NetworkEventCbList_t entry = _cbEventList[i];
        if (entry.cb) {
            if (entry.event == event || entry.event == network_event::NETWORK_EVENT_MAX) {
                entry.cb(event);
            }
        }
    }
}

void NetworkSettingsClass::setupMode()
{
    if (adminEnabled) {
        WiFi.mode(WIFI_AP_STA);
        String ssidString = getApName();
        WiFi.softAPConfig(apIp, apIp, apNetmask);
        WiFi.softAP((const char*)ssidString.c_str(), Configuration.get().Security_Password);
        dnsServer->setErrorReplyCode(DNSReplyCode::NoError);
        dnsServer->start(DNS_PORT, "*", WiFi.softAPIP());
        dnsServerStatus = true;
    } else {
        dnsServer->stop();
        dnsServerStatus = false;
        if (_networkMode == network_mode::WiFi) {
            WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
            WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
            WiFi.mode(WIFI_STA);
        } else {
            WiFi.mode(WIFI_MODE_NULL);
        }
    }

    /*if (PinMapping.isValidEthConfig()) {
        PinMapping_t& pin = PinMapping.get();
        ETH.begin(pin.eth_phy_addr, pin.eth_power, pin.eth_mdc, pin.eth_mdio, pin.eth_type, pin.eth_clk_mode);
    }*/
    static bool eth_setup = false;
    if (!eth_setup) {
        eth_setup = true;
        setupSpiEth();
    }
}

void NetworkSettingsClass::enableAdminMode()
{
    adminEnabled = true;
    adminTimeoutCounter = 0;
    setupMode();
}

String NetworkSettingsClass::getApName()
{
    return String(ACCESS_POINT_NAME + String(Utils::getChipId()));
}

void NetworkSettingsClass::loop()
{
    if (_ethConnected) {
        if (_networkMode != network_mode::Ethernet) {
            // Do stuff when switching to Ethernet mode
            MessageOutput.println("Switch to Ethernet mode");
            _networkMode = network_mode::Ethernet;
            WiFi.mode(WIFI_MODE_NULL);
            setStaticIp();
            setHostname();
        }
    } else
        if (_networkMode != network_mode::WiFi) {
        // Do stuff when switching to Ethernet mode
        MessageOutput.println("Switch to WiFi mode");
        _networkMode = network_mode::WiFi;
        enableAdminMode();
        applyConfig();
    }

    if (millis() - lastTimerCall > 1000) {
        adminTimeoutCounter++;
        connectTimeoutTimer++;
        connectRedoTimer++;
        lastTimerCall = millis();
    }
    if (adminEnabled) {
        // Don't disable the admin mode when network is not available
        if (!isConnected()) {
            adminTimeoutCounter = 0;
        }
        // If WiFi is connected to AP for more than ADMIN_TIMEOUT
        // seconds, disable the internal Access Point
        if (adminTimeoutCounter > ADMIN_TIMEOUT) {
            adminEnabled = false;
            MessageOutput.println("Admin mode disabled");
            setupMode();
        }
        // It's nearly not possible to use the internal AP if the
        // WiFi is searching for an AP. So disable searching afer
        // WIFI_RECONNECT_TIMEOUT and repeat after WIFI_RECONNECT_REDO_TIMEOUT
        if (isConnected()) {
            connectTimeoutTimer = 0;
            connectRedoTimer = 0;
        } else {
            if (connectTimeoutTimer > WIFI_RECONNECT_TIMEOUT && !forceDisconnection) {
                MessageOutput.print("Disable search for AP... ");
                WiFi.mode(WIFI_AP);
                MessageOutput.println("done");
                connectRedoTimer = 0;
                forceDisconnection = true;
            }
            if (connectRedoTimer > WIFI_RECONNECT_REDO_TIMEOUT && forceDisconnection) {
                MessageOutput.print("Enable search for AP... ");
                WiFi.mode(WIFI_AP_STA);
                MessageOutput.println("done");
                applyConfig();
                connectTimeoutTimer = 0;
                forceDisconnection = false;
            }
        }
    }
    if (dnsServerStatus) {
        dnsServer->processNextRequest();
    }
}

void NetworkSettingsClass::applyConfig()
{
    setHostname();
    if (!strcmp(Configuration.get().WiFi_Ssid, "")) {
        return;
    }
    MessageOutput.print("Configuring WiFi STA using ");
    if (strcmp(WiFi.SSID().c_str(), Configuration.get().WiFi_Ssid) || strcmp(WiFi.psk().c_str(), Configuration.get().WiFi_Password)) {
        MessageOutput.print("new credentials... ");
        WiFi.begin(
            Configuration.get().WiFi_Ssid,
            Configuration.get().WiFi_Password);
    } else {
        MessageOutput.print("existing credentials... ");
        WiFi.begin();
    }
    MessageOutput.println("done");
    setStaticIp();
}

void NetworkSettingsClass::setHostname()
{
    MessageOutput.print("Setting Hostname... ");
    if (_networkMode == network_mode::WiFi) {
        if (WiFi.hostname(getHostname())) {
            MessageOutput.println("done");
        } else {
            MessageOutput.println("failed");
        }

        // Evil bad hack to get the hostname set up correctly
        WiFi.mode(WIFI_MODE_APSTA);
        WiFi.mode(WIFI_MODE_STA);
        setupMode();
    }
    else if (_networkMode == network_mode::Ethernet) {
        if (ETH.setHostname(getHostname().c_str())) {
            MessageOutput.println("done");
        } else {
            MessageOutput.println("failed");
        }
    }
}

void NetworkSettingsClass::setStaticIp()
{
    if (_networkMode == network_mode::WiFi) {
        if (Configuration.get().WiFi_Dhcp) {
            MessageOutput.print("Configuring WiFi STA DHCP IP... ");
            WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
            MessageOutput.println("done");
        } else {
            MessageOutput.print("Configuring WiFi STA static IP... ");
            WiFi.config(
                IPAddress(Configuration.get().WiFi_Ip),
                IPAddress(Configuration.get().WiFi_Gateway),
                IPAddress(Configuration.get().WiFi_Netmask),
                IPAddress(Configuration.get().WiFi_Dns1),
                IPAddress(Configuration.get().WiFi_Dns2));
            MessageOutput.println("done");
        }
    }
    else if (_networkMode == network_mode::Ethernet) {
        if (Configuration.get().WiFi_Dhcp) {
            MessageOutput.print("Configuring Ethernet DHCP IP... ");
            ETH.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
            MessageOutput.println("done");
        } else {
            MessageOutput.print("Configuring Ethernet static IP... ");
            ETH.config(
                IPAddress(Configuration.get().WiFi_Ip),
                IPAddress(Configuration.get().WiFi_Gateway),
                IPAddress(Configuration.get().WiFi_Netmask),
                IPAddress(Configuration.get().WiFi_Dns1),
                IPAddress(Configuration.get().WiFi_Dns2));
            MessageOutput.println("done");
        }
    }
}

IPAddress NetworkSettingsClass::localIP()
{
    switch (_networkMode) {
    case network_mode::Ethernet:
        return ETH.localIP();
        break;
    case network_mode::WiFi:
        return WiFi.localIP();
        break;
    default:
        return INADDR_NONE;
    }
}

IPAddress NetworkSettingsClass::subnetMask()
{
    switch (_networkMode) {
    case network_mode::Ethernet:
        return ETH.subnetMask();
        break;
    case network_mode::WiFi:
        return WiFi.subnetMask();
        break;
    default:
        return IPAddress(255, 255, 255, 0);
    }
}

IPAddress NetworkSettingsClass::gatewayIP()
{
    switch (_networkMode) {
    case network_mode::Ethernet:
        return ETH.gatewayIP();
        break;
    case network_mode::WiFi:
        return WiFi.gatewayIP();
        break;
    default:
        return INADDR_NONE;
    }
}

IPAddress NetworkSettingsClass::dnsIP(uint8_t dns_no)
{
    switch (_networkMode) {
    case network_mode::Ethernet:
        return ETH.dnsIP(dns_no);
        break;
    case network_mode::WiFi:
        return WiFi.dnsIP(dns_no);
        break;
    default:
        return INADDR_NONE;
    }
}

String NetworkSettingsClass::macAddress()
{
    switch (_networkMode) {
    case network_mode::Ethernet:
        return ETH.macAddress();
        break;
    case network_mode::WiFi:
        return WiFi.macAddress();
        break;
    default:
        return "";
    }
}

String NetworkSettingsClass::getHostname()
{
    const CONFIG_T& config = Configuration.get();
    char preparedHostname[WIFI_MAX_HOSTNAME_STRLEN + 1];
    char resultHostname[WIFI_MAX_HOSTNAME_STRLEN + 1];
    uint8_t pos = 0;

    uint32_t chipId = Utils::getChipId();
    snprintf(preparedHostname, WIFI_MAX_HOSTNAME_STRLEN + 1, config.WiFi_Hostname, chipId);

    const char* pC = preparedHostname;
    while (*pC && pos < WIFI_MAX_HOSTNAME_STRLEN) { // while !null and not over length
        if (isalnum(*pC)) { // if the current char is alpha-numeric append it to the hostname
            resultHostname[pos] = *pC;
            pos++;
        } else if (*pC == ' ' || *pC == '_' || *pC == '-' || *pC == '+' || *pC == '!' || *pC == '?' || *pC == '*') {
            resultHostname[pos] = '-';
            pos++;
        }
        // else do nothing - no leading hyphens and do not include hyphens for all other characters.
        pC++;
    }

    resultHostname[pos] = '\0'; // terminate string

    // last character must not be hyphen
    while (pos > 0 && resultHostname[pos - 1] == '-') {
        resultHostname[pos - 1] = '\0';
        pos--;
    }

    // Fallback if no other rule applied
    if (strlen(resultHostname) == 0) {
        snprintf(resultHostname, WIFI_MAX_HOSTNAME_STRLEN + 1, APP_HOSTNAME, chipId);
    }

    return resultHostname;
}

bool NetworkSettingsClass::isConnected()
{
    return WiFi.localIP()[0] != 0 || ETH.localIP()[0] != 0;
}

network_mode NetworkSettingsClass::NetworkMode()
{
    return _networkMode;
}

NetworkSettingsClass NetworkSettings;