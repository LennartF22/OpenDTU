// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2023-2024 Thomas Basler and others
 */
#include "InverterSettings.h"
#include "Configuration.h"
#include "MessageOutput.h"
#include "PinMapping.h"
#include "SunPosition.h"
#include <Hoymiles.h>
#include <SpiManager.h>

InverterSettingsClass InverterSettings;

InverterSettingsClass::InverterSettingsClass()
    : _settingsTask(INVERTER_UPDATE_SETTINGS_INTERVAL, TASK_FOREVER, std::bind(&InverterSettingsClass::settingsLoop, this))
    , _hoyTask(TASK_IMMEDIATE, TASK_FOREVER, std::bind(&InverterSettingsClass::hoyLoop, this))
{
}

void InverterSettingsClass::init(Scheduler& scheduler)
{
    const CONFIG_T& config = Configuration.get();
    const PinMapping_t& pin = PinMapping.get();

    // Initialize inverter communication
    MessageOutput.print("Initialize Hoymiles interface... ");

    Hoymiles.setMessageOutput(&MessageOutput);
    Hoymiles.init();

    if (PinMapping.isValidNrf24Config() || PinMapping.isValidCmt2300Config()) {
        if (PinMapping.isValidNrf24Config()) {
            auto bus_config = std::make_shared<SpiBusConfig>(
                static_cast<gpio_num_t>(pin.nrf24_mosi),
                static_cast<gpio_num_t>(pin.nrf24_miso),
                static_cast<gpio_num_t>(pin.nrf24_clk)
            );

            gpio_reset_pin(static_cast<gpio_num_t>(pin.nrf24_cs));
            spi_device_interface_config_t device_config {
                .command_bits = 0,
                .address_bits = 0,
                .dummy_bits = 0,
                .mode = 0, // SPI mode 0
                .duty_cycle_pos = 0,
                .cs_ena_pretrans = 0,
                .cs_ena_posttrans = 0,
                .clock_speed_hz = RF24_SPI_SPEED, // 10 MHz by default
                .input_delay_ns = 0,
                .spics_io_num = static_cast<gpio_num_t>(pin.nrf24_cs),
                .flags = 0,
                .queue_size = 1,
                .pre_cb = nullptr,
                .post_cb = nullptr,
            };

            spi_device_handle_t dev = SpiManagerInst.alloc_device("RF", bus_config, device_config);
            if (!dev)
                ESP_ERROR_CHECK(ESP_FAIL);

            ESPSPIClass *spiClass = new ESPSPIClass();
            spiClass->begin(dev);
            Hoymiles.initNRF(spiClass, pin.nrf24_en, pin.nrf24_irq);
        }

        if (PinMapping.isValidCmt2300Config()) {
            Hoymiles.initCMT(pin.cmt_sdio, pin.cmt_clk, pin.cmt_cs, pin.cmt_fcs, pin.cmt_gpio2, pin.cmt_gpio3);
            MessageOutput.println("  Setting country mode... ");
            Hoymiles.getRadioCmt()->setCountryMode(static_cast<CountryModeId_t>(config.Dtu.Cmt.CountryMode));
            MessageOutput.println("  Setting CMT target frequency... ");
            Hoymiles.getRadioCmt()->setInverterTargetFrequency(config.Dtu.Cmt.Frequency);
        }

        MessageOutput.println("  Setting radio PA level... ");
        Hoymiles.getRadioNrf()->setPALevel((rf24_pa_dbm_e)config.Dtu.Nrf.PaLevel);
        Hoymiles.getRadioCmt()->setPALevel(config.Dtu.Cmt.PaLevel);

        MessageOutput.println("  Setting DTU serial... ");
        Hoymiles.getRadioNrf()->setDtuSerial(config.Dtu.Serial);
        Hoymiles.getRadioCmt()->setDtuSerial(config.Dtu.Serial);

        MessageOutput.println("  Setting poll interval... ");
        Hoymiles.setPollInterval(config.Dtu.PollInterval);

        for (uint8_t i = 0; i < INV_MAX_COUNT; i++) {
            if (config.Inverter[i].Serial > 0) {
                MessageOutput.print("  Adding inverter: ");
                MessageOutput.print(config.Inverter[i].Serial, HEX);
                MessageOutput.print(" - ");
                MessageOutput.print(config.Inverter[i].Name);
                auto inv = Hoymiles.addInverter(
                    config.Inverter[i].Name,
                    config.Inverter[i].Serial);

                if (inv != nullptr) {
                    inv->setReachableThreshold(config.Inverter[i].ReachableThreshold);
                    inv->setZeroValuesIfUnreachable(config.Inverter[i].ZeroRuntimeDataIfUnrechable);
                    inv->setZeroYieldDayOnMidnight(config.Inverter[i].ZeroYieldDayOnMidnight);
                    inv->setClearEventlogOnMidnight(config.Inverter[i].ClearEventlogOnMidnight);
                    inv->Statistics()->setYieldDayCorrection(config.Inverter[i].YieldDayCorrection);
                    for (uint8_t c = 0; c < INV_MAX_CHAN_COUNT; c++) {
                        inv->Statistics()->setStringMaxPower(c, config.Inverter[i].channel[c].MaxChannelPower);
                        inv->Statistics()->setChannelFieldOffset(TYPE_DC, static_cast<ChannelNum_t>(c), FLD_YT, config.Inverter[i].channel[c].YieldTotalOffset);
                    }
                }
                MessageOutput.println(" done");
            }
        }
        MessageOutput.println("done");
    } else {
        MessageOutput.println("Invalid pin config");
    }

    scheduler.addTask(_hoyTask);
    _hoyTask.enable();

    scheduler.addTask(_settingsTask);
    _settingsTask.enable();
}

void InverterSettingsClass::settingsLoop()
{
    const CONFIG_T& config = Configuration.get();
    const bool isDayPeriod = SunPosition.isDayPeriod();

    for (uint8_t i = 0; i < INV_MAX_COUNT; i++) {
        auto const& inv_cfg = config.Inverter[i];
        if (inv_cfg.Serial == 0) {
            continue;
        }
        auto inv = Hoymiles.getInverterBySerial(inv_cfg.Serial);
        if (inv == nullptr) {
            continue;
        }

        inv->setEnablePolling(inv_cfg.Poll_Enable && (isDayPeriod || inv_cfg.Poll_Enable_Night));
        inv->setEnableCommands(inv_cfg.Command_Enable && (isDayPeriod || inv_cfg.Command_Enable_Night));
    }
}

void InverterSettingsClass::hoyLoop()
{
    Hoymiles.loop();
}
