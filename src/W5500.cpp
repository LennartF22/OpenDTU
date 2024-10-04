// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Thomas Basler and others
 */

#include "W5500.h"

#include <SpiManager.h>
#include <NetworkManager.h>
#include <driver/spi_master.h>
#include <esp_eth_mac.h>
#include <esp_mac.h>
#include <esp_interface.h>
#include <esp_netif_types.h>

#define TAG "W5500"

struct custom_spi_context
{
    spi_device_handle_t spi;
    SemaphoreHandle_t lock;
};

static void* custom_spi_init(const void* config)
{
    custom_spi_context* spi_context = new custom_spi_context {
        .spi = reinterpret_cast<spi_device_handle_t>(const_cast<void*>(config)),
        .lock = xSemaphoreCreateMutex(), // TODO: handle error
    };
    return spi_context;
}

static esp_err_t custom_spi_deinit(void* context)
{
    custom_spi_context* spi_context = reinterpret_cast<custom_spi_context*>(context);

    // TODO: do proper cleanup
    vSemaphoreDelete(spi_context->lock);

    delete spi_context;
    return ESP_OK;
}

static esp_err_t custom_spi_read(void* context, uint32_t cmd, uint32_t addr, void* data, uint32_t length)
{
    custom_spi_context* spi_context = reinterpret_cast<custom_spi_context*>(context);

    spi_transaction_t trans {
        .flags = 0,
        .cmd = static_cast<uint16_t>(cmd),
        .addr = addr,
        .length = 8 * length,
        .rxlength = 8 * length,
        .user = nullptr,
        .tx_buffer = nullptr,
        .rx_buffer = data,
    };

    esp_err_t ret = ESP_OK;

    // TODO: error check, timeout
    xSemaphoreTake(spi_context->lock, portMAX_DELAY);
    if (spi_device_polling_transmit(spi_context->spi, &trans) != ESP_OK) {
        ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
        ret = ESP_FAIL;
    }
    xSemaphoreGive(spi_context->lock);

    return ret;
}

static esp_err_t custom_spi_write(void* context, uint32_t cmd, uint32_t addr, const void* data, uint32_t length)
{
    custom_spi_context* spi_context = reinterpret_cast<custom_spi_context*>(context);

    spi_transaction_t trans {
        .flags = 0,
        .cmd = static_cast<uint16_t>(cmd),
        .addr = addr,
        .length = 8 * length,
        .rxlength = 8 * length,
        .user = nullptr,
        .tx_buffer = data,
        .rx_buffer = nullptr,
    };

    esp_err_t ret = ESP_OK;

    // TODO: error check, timeout
    xSemaphoreTake(spi_context->lock, portMAX_DELAY);
    if (spi_device_polling_transmit(spi_context->spi, &trans) != ESP_OK) {
        ESP_LOGE(TAG, "%s(%d): spi transmit failed", __FUNCTION__, __LINE__);
        ret = ESP_FAIL;
    }
    xSemaphoreGive(spi_context->lock);

    return ret;
}

W5500::W5500(spi_device_handle_t spi, gpio_num_t pin_int)
    : eth_handle(nullptr)
{
    // Arduino function to start networking stack if not already started
    Network.begin();

    eth_w5500_config_t w5500_config {
        .int_gpio_num = pin_int,
        .poll_period_ms = 0,
        .spi_host_id = SPI_HOST_MAX,
        .spi_devcfg = nullptr,
        .custom_spi_driver {
            .config = spi,
            .init = custom_spi_init,
            .deinit = custom_spi_deinit,
            .read = custom_spi_read,
            .write = custom_spi_write,
        },
    };

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    mac_config.rx_task_stack_size = 4096;
    esp_eth_mac_t* mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.reset_gpio_num = -1;
    esp_eth_phy_t* phy = esp_eth_phy_new_w5500(&phy_config);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    // Configure MAC address
    uint8_t mac_addr[ETH_ADDR_LEN];
    ESP_ERROR_CHECK(esp_read_mac(mac_addr, ESP_MAC_ETH));
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr));

    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    _esp_netif = esp_netif_new(&netif_config);

    ESP_ERROR_CHECK(esp_netif_attach(_esp_netif, esp_eth_new_netif_glue(eth_handle)));

    // Asynchronous behavior?
    esp_event_handler_instance_t event_handler;
    esp_event_handler_instance_register(ETH_EVENT, ESP_EVENT_ANY_ID, event_cb, this, &event_handler);

    initNetif(ESP_NETIF_ID_ETH0);

    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}

W5500::~W5500()
{
    // TODO(LennartF22): support cleanup at some point?
}

std::unique_ptr<W5500> W5500::setup(int8_t pin_mosi, int8_t pin_miso, int8_t pin_sclk, int8_t pin_cs, int8_t pin_int, int8_t pin_rst)
{
    gpio_reset_pin(static_cast<gpio_num_t>(pin_rst));
    gpio_set_level(static_cast<gpio_num_t>(pin_rst), 0);
    gpio_set_direction(static_cast<gpio_num_t>(pin_rst), GPIO_MODE_OUTPUT);

    gpio_reset_pin(static_cast<gpio_num_t>(pin_cs));
    gpio_reset_pin(static_cast<gpio_num_t>(pin_int));

    auto bus_config = std::make_shared<SpiBusConfig>(
        static_cast<gpio_num_t>(pin_mosi),
        static_cast<gpio_num_t>(pin_miso),
        static_cast<gpio_num_t>(pin_sclk));

    spi_device_interface_config_t device_config {
        .command_bits = 16, // actually address phase
        .address_bits = 8, // actually command phase
        .dummy_bits = 0,
        .mode = 0,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0, // only 0 supported
        .cs_ena_posttrans = 0, // only 0 supported
        .clock_speed_hz = 20000000, // stable with OpenDTU Fusion shield
        .input_delay_ns = 0,
        .spics_io_num = pin_cs,
        .flags = 0,
        .queue_size = 20,
        .pre_cb = nullptr,
        .post_cb = nullptr,
    };

    spi_device_handle_t spi = SpiManagerInst.alloc_device("", bus_config, device_config);
    if (!spi)
        return nullptr;

    // Reset sequence
    delayMicroseconds(500);
    gpio_set_level(static_cast<gpio_num_t>(pin_rst), 1);
    delayMicroseconds(1000);

    if (!connection_check_spi(spi))
        return nullptr;
    if (!connection_check_interrupt(static_cast<gpio_num_t>(pin_int)))
        return nullptr;

    // Use Arduino functions to temporarily attach interrupt to enable the GPIO ISR service
    // (if we used ESP-IDF functions, a warning would be printed the first time anyone uses attachInterrupt)
    attachInterrupt(pin_int, nullptr, FALLING);
    detachInterrupt(pin_int);

    // Return to default state once again after connection check and temporary interrupt registration
    gpio_reset_pin(static_cast<gpio_num_t>(pin_int));

    return std::unique_ptr<W5500>(new W5500(spi, static_cast<gpio_num_t>(pin_int)));
}

size_t W5500::printDriverInfo(Print& out) const
{
    size_t bytes = 0;
    bytes += out.print("TODO");
    return bytes;
}

bool W5500::connection_check_spi(spi_device_handle_t spi)
{
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_RXDATA,
        .cmd = 0x0039, // actually address (VERSIONR)
        .addr = (0b00000 << 3) | (0 << 2) | (0b00 < 0), // actually command (common register, read, VDM)
        .length = 8,
        .rxlength = 8,
        .user = nullptr,
        .tx_buffer = nullptr,
        .rx_data = {},
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &trans));

    // Version number (VERSIONR) is always 0x04
    return *reinterpret_cast<uint8_t*>(&trans.rx_data) == 0x04;
}

bool W5500::connection_check_interrupt(gpio_num_t pin_int)
{
    gpio_set_direction(pin_int, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin_int, GPIO_PULLDOWN_ONLY);
    int level = gpio_get_level(pin_int);

    // Interrupt line must be high
    return level == 1;
}

void W5500::event_cb(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == ETH_EVENT) {
        esp_eth_handle_t eth_handle = *reinterpret_cast<esp_eth_handle_t*>(event_data);
        reinterpret_cast<W5500*>(arg)->handle_event(event_id, event_data);
    }
}

void W5500::handle_event(int32_t event_id, void* event_data) {
    // Taken from Arduino "ETH.cpp"

    arduino_event_t arduino_event;
    arduino_event.event_id = ARDUINO_EVENT_MAX;

    if (event_id == ETHERNET_EVENT_CONNECTED) {
        log_v("%s Connected", desc());
        arduino_event.event_id = ARDUINO_EVENT_ETH_CONNECTED;
        arduino_event.event_info.eth_connected = eth_handle;
        setStatusBits(ESP_NETIF_CONNECTED_BIT);
    } else if (event_id == ETHERNET_EVENT_DISCONNECTED) {
        log_v("%s Disconnected", desc());
        arduino_event.event_id = ARDUINO_EVENT_ETH_DISCONNECTED;
        clearStatusBits(ESP_NETIF_CONNECTED_BIT | ESP_NETIF_HAS_IP_BIT | ESP_NETIF_HAS_LOCAL_IP6_BIT | ESP_NETIF_HAS_GLOBAL_IP6_BIT);
    } else if (event_id == ETHERNET_EVENT_START) {
        log_v("%s Started", desc());
        arduino_event.event_id = ARDUINO_EVENT_ETH_START;
        setStatusBits(ESP_NETIF_STARTED_BIT);
    } else if (event_id == ETHERNET_EVENT_STOP) {
        log_v("%s Stopped", desc());
        arduino_event.event_id = ARDUINO_EVENT_ETH_STOP;
        clearStatusBits(ESP_NETIF_STARTED_BIT | ESP_NETIF_CONNECTED_BIT | ESP_NETIF_HAS_IP_BIT | ESP_NETIF_HAS_LOCAL_IP6_BIT | ESP_NETIF_HAS_GLOBAL_IP6_BIT | ESP_NETIF_HAS_STATIC_IP_BIT);
    }

    if (arduino_event.event_id < ARDUINO_EVENT_MAX) {
        Network.postEvent(&arduino_event);
    }
}
