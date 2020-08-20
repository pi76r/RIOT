/*
   ______                              _
   / _____)             _              | |
   ( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
   _____) ) ____| | | || |_| ____( (___| | | |
   (______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

   Description: Driver for SX1280 devices

   License: Revised BSD License, see LICENSE.TXT file include in the project

   Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy
 */

/*
 * Copyright (C) 2020 Université Grenoble Alpes
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup    drivers_sx1280
 *
 * @file	   sx1280_getset.c
 * @brief      Implementation of get and set functions for SX1280
 *
 * @author     Julia CORREA REIS
 * @author     Pierre Millot
 */

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "sx1280.h"
#include "sx1280_internal.h"
#include "sx1280_registers.h"

#define ENABLE_DEBUG SX1280_DEBUG
#include "debug.h"

SX1280_States_t sx1280_get_state(const sx1280_t *dev)
{
    return dev->settings.state;
}

void sx1280_set_state(sx1280_t *dev, SX1280_States_t state)
{
    dev->settings.state = state;
}

SX1280_Status_t sx1280_get_status(sx1280_t *dev)
{
    return (SX1280_Status_t){.Value =
                                 sx1280_read_command(dev, SX1280_GET_STATUS) };
}

void sx1280_set_sleep(sx1280_t *dev, sx1280_sleep_params_t sleepConfig)
{
    return;

    DEBUG("[sx1280] Set sleep\n");

    uint8_t sleep = (sleepConfig.WakeUpRTC << 3) |
                    (sleepConfig.InstructionRamRetention << 2) |
                    (sleepConfig.DataBufferRetention << 1) |
                    (sleepConfig.DataRamRetention);

    dev->settings.mode = SX1280_MODE_SLEEP;
    sx1280_write_command(dev, SX1280_SET_SLEEP, sleep);
    sx1280_set_state(dev, SX1280_RF_IDLE);
    xtimer_remove(&dev->_internal.tx_timeout_timer);
    xtimer_remove(&dev->_internal.rx_timeout_timer);
}

uint16_t sx1280_get_firmware_version(sx1280_t *dev)
{
    return (sx1280_reg_read(dev, REG_LR_FIRMWARE_VERSION_MSB) << 8) |
           sx1280_reg_read(dev, REG_LR_FIRMWARE_VERSION_MSB + 1);
}

void sx1280_set_regulator_mode(sx1280_t *dev, SX1280_RegulatorModes_t mode)
{
    dev->settings.regmode = mode;
    sx1280_write_command(dev, SX1280_SET_REGULATORMODE, mode);
}

void sx1280_set_standby(sx1280_t *dev, SX1280_StandbyModes_t standbyConfig)
{
    sx1280_write_command(dev, SX1280_SET_STANDBY, standbyConfig);
    dev->settings.mode =
        SX1280_STDBY_RC ? SX1280_MODE_STDBY_RC : SX1280_MODE_STDBY_XOSC;
    sx1280_set_state(dev, SX1280_RF_IDLE);
}

void sx1280_set_tx_continuous_wave(sx1280_t *dev)
{
    sx1280_write_command(dev, SX1280_SET_TXCONTINUOUSWAVE, 0);
}

void sx1280_set_tx_continuous_preamble(sx1280_t *dev)
{
    sx1280_write_command(dev, SX1280_SET_TXCONTINUOUSPREAMBLE, 0);
}

void sx1280_set_packet_type(sx1280_t *dev, SX1280_PacketTypes_t packetType)
{
    dev->settings.packet = packetType;
    sx1280_write_command(dev, SX1280_SET_PACKETTYPE, packetType);
}

SX1280_PacketTypes_t sx1280_get_packet_type(sx1280_t *dev, bool localCopy)
{
    if (localCopy) {
        return dev->settings.packet;
    }
    return sx1280_read_command(dev, SX1280_GET_PACKETTYPE);
}

void sx1280_set_modulation_params(sx1280_t *dev,
                                  sx1280_modulation_params_t *modParams)
{
    uint8_t buf[3];

    /* Check if required configuration corresponds to the stored packet type
       If not, silently update radio packet type*/
    if (sx1280_get_packet_type(dev, true) != modParams->PacketType) {
        sx1280_set_packet_type(dev, modParams->PacketType);
    }

    dev->modulation = *modParams;

    switch (modParams->PacketType) {
        case SX1280_PACKET_TYPE_GFSK:
            buf[0] = modParams->Params.Gfsk.BitrateBandwidth;
            buf[1] = modParams->Params.Gfsk.ModulationIndex;
            buf[2] = modParams->Params.Gfsk.ModulationShaping;
            break;
        case SX1280_PACKET_TYPE_LORA:
        case SX1280_PACKET_TYPE_RANGING:
            buf[0] = modParams->Params.LoRa.SpreadingFactor;
            buf[1] = modParams->Params.LoRa.Bandwidth;
            buf[2] = modParams->Params.LoRa.CodingRate;
            break;
        case SX1280_PACKET_TYPE_FLRC:
            buf[0] = modParams->Params.Flrc.BitrateBandwidth;
            buf[1] = modParams->Params.Flrc.CodingRate;
            buf[2] = modParams->Params.Flrc.ModulationShaping;
            break;
        case SX1280_PACKET_TYPE_BLE:
            buf[0] = modParams->Params.Ble.BitrateBandwidth;
            buf[1] = modParams->Params.Ble.ModulationIndex;
            buf[2] = modParams->Params.Ble.ModulationShaping;
            break;
        case SX1280_PACKET_TYPE_NONE:
            buf[0] = 0;
            buf[1] = 0;
            buf[2] = 0;
            break;
    }
    sx1280_write_command_buffer(dev, SX1280_SET_MODULATIONPARAMS, buf, 3);

    /*
       if (modParams->PacketType == SX1280_PACKET_TYPE_LORA) {
        uint8_t cmd = 0;
        switch (modParams->Params.LoRa.SpreadingFactor) {
            case SX1280_LORA_SF5:
            case SX1280_LORA_SF6:
                cmd = 0x1E;
                break;
            case SX1280_LORA_SF7:
            case SX1280_LORA_SF8:
                cmd = 0x37;
                break;
            case SX1280_LORA_SF9:
            case SX1280_LORA_SF10:
            case SX1280_LORA_SF11:
            case SX1280_LORA_SF12:
                cmd = 0x32;
                break;
            default:
                return;
        }
        sx1280_reg_write(dev, 0x925, cmd);
       }*/
}

void sx1280_set_packet_params(sx1280_t *dev,
                              sx1280_packet_params_t *packetParams)
{
    uint8_t buf[7];

    /* Check if required configuration corresponds to the stored packet type
       If not, silently update radio packet type*/
    if (sx1280_get_packet_type(dev, true) != packetParams->PacketType) {
        sx1280_set_packet_type(dev, packetParams->PacketType);
    }

    dev->packet = *packetParams;

    switch (packetParams->PacketType) {
        case SX1280_PACKET_TYPE_GFSK:
            buf[0] = packetParams->Params.Gfsk.PreambleLength;
            buf[1] = packetParams->Params.Gfsk.SyncWordLength;
            buf[2] = packetParams->Params.Gfsk.SyncWordMatch;
            buf[3] = packetParams->Params.Gfsk.HeaderType;
            buf[4] = packetParams->Params.Gfsk.PayloadLength;
            buf[5] = packetParams->Params.Gfsk.CrcLength;
            buf[6] = packetParams->Params.Gfsk.Whitening;
            break;
        case SX1280_PACKET_TYPE_LORA:
        case SX1280_PACKET_TYPE_RANGING:
            buf[0] = packetParams->Params.LoRa.PreambleLength;
            buf[1] = packetParams->Params.LoRa.HeaderType;
            buf[2] = packetParams->Params.LoRa.PayloadLength;
            buf[3] = packetParams->Params.LoRa.CrcMode;
            buf[4] = packetParams->Params.LoRa.InvertIQ;
            buf[5] = 0;
            buf[6] = 0;
            break;
        case SX1280_PACKET_TYPE_FLRC:
            buf[0] = packetParams->Params.Flrc.PreambleLength;
            buf[1] = packetParams->Params.Flrc.SyncWordLength;
            buf[2] = packetParams->Params.Flrc.SyncWordMatch;
            buf[3] = packetParams->Params.Flrc.HeaderType;
            buf[4] = packetParams->Params.Flrc.PayloadLength;
            buf[5] = packetParams->Params.Flrc.CrcLength;
            buf[6] = packetParams->Params.Flrc.Whitening;
            break;
        case SX1280_PACKET_TYPE_BLE:
            buf[0] = packetParams->Params.Ble.ConnectionState;
            buf[1] = packetParams->Params.Ble.CrcField;
            buf[2] = packetParams->Params.Ble.BlePacketType;
            buf[3] = packetParams->Params.Ble.Whitening;
            buf[4] = 0;
            buf[5] = 0;
            buf[6] = 0;
            break;
        case SX1280_PACKET_TYPE_NONE:
            buf[0] = 0;
            buf[1] = 0;
            buf[2] = 0;
            buf[3] = 0;
            buf[4] = 0;
            buf[5] = 0;
            buf[6] = 0;
            break;
    }
    sx1280_write_command_buffer(dev, SX1280_SET_PACKETPARAMS, buf, 7);
}

void sx1280_set_rf_frequency(sx1280_t *dev, uint32_t frequency)
{
    uint8_t buf[3];
    uint32_t freq = 0;

    dev->settings.lora.frequency = frequency;

    freq = (uint32_t)((double)frequency / (double)FREQ_STEP);
    buf[0] = (freq >> 16) & 0xFF;
    buf[1] = (freq >> 8) & 0xFF;
    buf[2] = freq & 0xFF;
    sx1280_write_command_buffer(dev, SX1280_SET_RFFREQUENCY, buf, 3);
}

uint32_t sx1280_get_rf_frequency(sx1280_t *dev)
{
    return dev->settings.lora.frequency;
}

void sx1280_set_buffer_base_address(sx1280_t *dev, uint8_t txBaseAddress,
                                    uint8_t rxBaseAddress)
{
    uint8_t buf[2];

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    sx1280_write_command_buffer(dev, SX1280_SET_BUFFERBASEADDRESS, buf, 2);
}

void sx1280_set_tx_params(sx1280_t *dev, int8_t power,
                          SX1280_RampTimes_t rampTime)
{
    uint8_t buf[2];

    /* The power value to send on SPI/UART is in the range [0..31] and the
       physical output power is in the range [-18..13]dBm*/
    buf[0] = power + 18;
    buf[1] = rampTime;
    sx1280_write_command_buffer(dev, SX1280_SET_TXPARAMS, buf, 2);

    dev->settings.lora.power = buf[0];
}

void sx1280_set_cad_params(sx1280_t *dev, SX1280_LoRaCadSymbols_t cadSymbolNum)
{
    sx1280_write_command(dev, SX1280_SET_CADPARAMS, cadSymbolNum);
    dev->settings.mode = SX1280_MODE_CAD;
}

uint8_t sx1280_set_syncword(sx1280_t *dev, uint8_t syncWordIdx,
                            uint8_t *syncword)
{
    uint16_t addr;
    uint8_t syncwordSize = 0;

    switch (sx1280_get_packet_type(dev, true)) {
        case SX1280_PACKET_TYPE_GFSK:
            syncwordSize = 5;
            switch (syncWordIdx) {
                case 1:
                    addr = REG_LR_SYNCWORDBASEADDRESS1;
                    break;
                case 2:
                    addr = REG_LR_SYNCWORDBASEADDRESS2;
                    break;
                case 3:
                    addr = REG_LR_SYNCWORDBASEADDRESS3;
                    break;
                default:
                    return 1;
            }
            break;
        case SX1280_PACKET_TYPE_FLRC:
            syncwordSize = 4;
            switch (syncWordIdx) {
                case 1:
                    addr = REG_LR_SYNCWORDBASEADDRESS1 + 1;
                    break;
                case 2:
                    addr = REG_LR_SYNCWORDBASEADDRESS2 + 1;
                    break;
                case 3:
                    addr = REG_LR_SYNCWORDBASEADDRESS3 + 1;
                    break;
                default:
                    return 1;
            }
            break;
        case SX1280_PACKET_TYPE_BLE:
            syncwordSize = 4;
            switch (syncWordIdx) {
                case 1:
                    addr = REG_LR_SYNCWORDBASEADDRESS1 + 1;
                    break;
                default:
                    return 1;
            }
            break;
        default:
            return 1;
    }
    sx1280_write_reg_buffer(dev, addr, syncword, syncwordSize);
    return 0;
}

SX1280_LoRaBandwidths_t sx1280_get_bandwidth(const sx1280_t *dev)
{
    return dev->settings.lora.bandwidth;
}

SX1280_OperatingModes_t sx1280_get_operation_mode(sx1280_t *dev)
{
    return dev->settings.mode;
}

int8_t sx1280_get_rssi_inst(sx1280_t *dev)
{
    return (int8_t)(-sx1280_read_command(dev, SX1280_GET_RSSIINST) / 2);
}

void sx1280_set_fs(sx1280_t *dev)
{
    sx1280_write_command(dev, SX1280_SET_FS, 0);
    dev->settings.mode = SX1280_MODE_FS;
}

void sx1280_set_rx_dutycycle(sx1280_t *dev, SX1280_TickSizes_t step,
                             uint16_t NbStepRx,
                             uint16_t RxNbStepSleep)
{
    uint8_t buf[5];

    buf[0] = step;
    buf[1] = (NbStepRx >> 8) & 0x00FF;
    buf[2] = NbStepRx & 0x00FF;
    buf[3] = (RxNbStepSleep >> 8) & 0x00FF;
    buf[4] = RxNbStepSleep & 0x00FF;
    sx1280_write_command_buffer(dev, SX1280_SET_RXDUTYCYCLE, buf, 5);
    dev->settings.mode = SX1280_MODE_RX;
}

void sx1280_set_save_context(sx1280_t *dev)
{
    sx1280_write_command(dev, SX1280_SET_SAVECONTEXT, 0);
}

void sx1280_set_auto_tx(sx1280_t *dev, uint16_t time)
{
    uint16_t compensatedTime = time - (uint16_t)AUTO_RX_TX_OFFSET;
    uint8_t buf[2];

    buf[0] = (compensatedTime >> 8) & 0x00FF;
    buf[1] = compensatedTime & 0x00FF;
    sx1280_write_command_buffer(dev, SX1280_SET_AUTOTX, buf, 2);
}

void sx1280_set_auto_fs(sx1280_t *dev, uint8_t enable)
{
    sx1280_write_command(dev, SX1280_SET_AUTOFS, enable);
}

void sx1280_set_long_preamble(sx1280_t *dev, uint8_t enable)
{
    sx1280_write_command(dev, SX1280_SET_LONGPREAMBLE, enable);
}

void sx1280_set_dio_irq_params(sx1280_t *dev, uint16_t irqMask,
                               uint16_t dio1Mask,
                               uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t buf[8];

    buf[0] = (irqMask >> 8) & 0xFF;
    buf[1] = irqMask & 0xFF;
    buf[2] = (dio1Mask >> 8) & 0xFF;
    buf[3] = dio1Mask & 0xFF;
    buf[4] = (dio2Mask >> 8) & 0xFF;
    buf[5] = dio2Mask & 0xFF;
    buf[6] = (dio3Mask >> 8) & 0xFF;
    buf[7] = dio3Mask & 0xFF;
    sx1280_write_command_buffer(dev, SX1280_SET_DIOIRQPARAMS, buf, 8);
}

void sx1280_set_cad(sx1280_t *dev)
{
    sx1280_write_command(dev, SX1280_SET_CAD, 0);
    dev->settings.mode = SX1280_MODE_CAD;
}

void sx1280_set_rx_single_mode(sx1280_t *dev, bool single)
{
    dev->settings.lora.flags.single = single;
    dev->settings.lora.flags.continuous = !single;
}

void sx1280_set_rx(sx1280_t *dev, sx1280_tick_time_t timeout)
{
    uint8_t buf[3];

    buf[0] = timeout.Step;
    buf[1] = (timeout.NbSteps >> 8) & 0xFF;
    buf[2] = timeout.NbSteps & 0xFF;

    dev->settings.lora.flags.continuous = timeout.NbSteps == 0xFFFF;
    dev->settings.lora.flags.single = timeout.NbSteps == 0;

    sx1280_clear_irq_status(dev, SX1280_IRQ_ALL);

    if (sx1280_get_packet_type(dev, true) == SX1280_PACKET_TYPE_RANGING) {
        sx1280_set_ranging_role(dev, SX1280_RANGING_ROLE_SLAVE);
    }
    dev->settings.mode = SX1280_MODE_RX;
    sx1280_set_state(dev, SX1280_RF_RX_RUNNING);
    if (!dev->settings.lora.flags.single &&
        !dev->settings.lora.flags.continuous) {
        xtimer_set(&(dev->_internal.rx_timeout_timer),
                   dev->settings.lora.rx_timeout * 1000); /* us */
    }

    uint16_t mask = SX1280_IRQ_RX_DONE | SX1280_IRQ_RX_TX_TIMEOUT |
                    SX1280_IRQ_CRC_ERROR |
                    SX1280_IRQ_SYNCWORD_VALID;
    sx1280_set_dio_irq_params(dev, mask, mask, SX1280_IRQ_NONE,
                              SX1280_IRQ_NONE);

    sx1280_write_command_buffer(dev, SX1280_SET_RX, buf, 3);
}

void sx1280_set_ranging_role(sx1280_t *dev, SX1280_RangingRoles_t role)
{
    sx1280_write_command(dev, SX1280_SET_RANGING_ROLE, role);
}

uint16_t sx1280_get_irq_status(sx1280_t *dev)
{
    uint8_t irqStatus[2];

    sx1280_read_command_buffer(dev, SX1280_GET_IRQSTATUS, irqStatus, 2);
    return (irqStatus[0] << 8) | irqStatus[1];
}

void sx1280_set_payload(sx1280_t *dev, uint8_t *payload, uint8_t size)
{
    sx1280_write_buffer(dev, 0, payload, size);
}

void sx1280_set_tx(sx1280_t *dev, sx1280_tick_time_t timeout)
{
    uint8_t buf[3];

    buf[0] = timeout.Step;
    buf[1] = (timeout.NbSteps >> 8) & 0x00FF;
    buf[2] = timeout.NbSteps & 0x00FF;

    sx1280_clear_irq_status(dev, SX1280_IRQ_ALL);

    if (sx1280_get_packet_type(dev, true) == SX1280_PACKET_TYPE_RANGING) {
        sx1280_set_ranging_role(dev, SX1280_RANGING_ROLE_MASTER);
    }
    dev->settings.mode = SX1280_MODE_TX;

    sx1280_set_state(dev, SX1280_RF_TX_RUNNING);

    /* Start TX timeout timer */
    if (dev->settings.lora.tx_timeout != 0) {
        xtimer_set(&(dev->_internal.tx_timeout_timer),
                   dev->settings.lora.tx_timeout * 1000); /* us */
    }

    sx1280_write_command_buffer(dev, SX1280_SET_TX, buf, 3);
}

void sx1280_get_packet_status(sx1280_t *dev,
                              sx1280_packet_status_t *packetStatus)
{
    uint8_t status[5];

    sx1280_read_command_buffer(dev, SX1280_GET_PACKETSTATUS, status, 5);

    packetStatus->packetType = sx1280_get_packet_type(dev, true);
    switch (packetStatus->packetType) {
        case SX1280_PACKET_TYPE_GFSK:
            packetStatus->Params.Gfsk.RssiSync = -(status[1] / 2);

            packetStatus->Params.Gfsk.ErrorStatus.SyncError = (status[2] >> 6) &
                                                              0x01;
            packetStatus->Params.Gfsk.ErrorStatus.LengthError =
                (status[2] >> 5) & 0x01;
            packetStatus->Params.Gfsk.ErrorStatus.CrcError = (status[2] >> 4) &
                                                             0x01;
            packetStatus->Params.Gfsk.ErrorStatus.AbortError =
                (status[2] >> 3) & 0x01;
            packetStatus->Params.Gfsk.ErrorStatus.HeaderReceived =
                (status[2] >> 2) & 0x01;
            packetStatus->Params.Gfsk.ErrorStatus.PacketReceived =
                (status[2] >> 1) & 0x01;
            packetStatus->Params.Gfsk.ErrorStatus.PacketControlerBusy =
                status[2] & 0x01;

            packetStatus->Params.Gfsk.TxRxStatus.RxNoAck = (status[3] >> 5) &
                                                           0x01;
            packetStatus->Params.Gfsk.TxRxStatus.PacketSent = status[3] & 0x01;

            packetStatus->Params.Gfsk.SyncAddrStatus = status[4] & 0x07;
            break;

        case SX1280_PACKET_TYPE_LORA:
        case SX1280_PACKET_TYPE_RANGING:
            packetStatus->Params.LoRa.RssiPkt = -(status[0] / 2);
            if (status[1] < 128) {
                packetStatus->Params.LoRa.SnrPkt = status[1] / 4;
            }
            else {
                packetStatus->Params.LoRa.SnrPkt = ((status[1] - 256) / 4);
            }
            break;
        case SX1280_PACKET_TYPE_FLRC:
            packetStatus->Params.Flrc.RssiSync = -(status[1] / 2);

            packetStatus->Params.Flrc.ErrorStatus.SyncError = (status[2] >> 6) &
                                                              0x01;
            packetStatus->Params.Flrc.ErrorStatus.LengthError =
                (status[2] >> 5) & 0x01;
            packetStatus->Params.Flrc.ErrorStatus.CrcError = (status[2] >> 4) &
                                                             0x01;
            packetStatus->Params.Flrc.ErrorStatus.AbortError =
                (status[2] >> 3) & 0x01;
            packetStatus->Params.Flrc.ErrorStatus.HeaderReceived =
                (status[2] >> 2) & 0x01;
            packetStatus->Params.Flrc.ErrorStatus.PacketReceived =
                (status[2] >> 1) & 0x01;
            packetStatus->Params.Flrc.ErrorStatus.PacketControlerBusy =
                status[2] & 0x01;

            packetStatus->Params.Flrc.TxRxStatus.RxPid = (status[3] >> 6) &
                                                         0x03;
            packetStatus->Params.Flrc.TxRxStatus.RxNoAck = (status[3] >> 5) &
                                                           0x01;
            packetStatus->Params.Flrc.TxRxStatus.RxPidErr = (status[3] >> 4) &
                                                            0x01;
            packetStatus->Params.Flrc.TxRxStatus.PacketSent = status[3] & 0x01;

            packetStatus->Params.Flrc.SyncAddrStatus = status[4] & 0x07;
            break;

        case SX1280_PACKET_TYPE_BLE:
            packetStatus->Params.Ble.RssiSync = -(status[1] / 2);

            packetStatus->Params.Ble.ErrorStatus.SyncError = (status[2] >> 6) &
                                                             0x01;
            packetStatus->Params.Ble.ErrorStatus.LengthError =
                (status[2] >> 5) & 0x01;
            packetStatus->Params.Ble.ErrorStatus.CrcError = (status[2] >> 4) &
                                                            0x01;
            packetStatus->Params.Ble.ErrorStatus.AbortError = (status[2] >> 3) &
                                                              0x01;
            packetStatus->Params.Ble.ErrorStatus.HeaderReceived =
                (status[2] >> 2) & 0x01;
            packetStatus->Params.Ble.ErrorStatus.PacketReceived =
                (status[2] >> 1) & 0x01;
            packetStatus->Params.Ble.ErrorStatus.PacketControlerBusy =
                status[2] & 0x01;

            packetStatus->Params.Ble.TxRxStatus.PacketSent = status[3] & 0x01;

            packetStatus->Params.Ble.SyncAddrStatus = status[4] & 0x07;
            break;

        case SX1280_PACKET_TYPE_NONE:
            /* In that specific case, we set everything in the packetStatus to zeros
               and reset the packet type accordingly */
            memset(packetStatus, 0, sizeof(sx1280_packet_status_t));
            packetStatus->packetType = SX1280_PACKET_TYPE_NONE;
            break;
    }
}

SX1280_LoRaSpreadingFactors_t sx1280_get_spreading_factor(const sx1280_t *dev)
{
    return dev->modulation.Params.LoRa.SpreadingFactor;
}

SX1280_LoRaCodingRates_t sx1280_get_coding_rate(const sx1280_t *dev)
{
    return dev->modulation.Params.LoRa.CodingRate;
}

SX1280_LoRaCrcModes_t sx1280_get_crc(const sx1280_t *dev)
{
    return dev->packet.Params.LoRa.CrcMode;
}

SX1280_LoRaIQModes_t sx1280_get_iq_invert(const sx1280_t *dev)
{
    return dev->packet.Params.LoRa.InvertIQ;
}

bool sx1280_get_rx_single(const sx1280_t *dev)
{
    return dev->settings.lora.flags.single;
}

uint8_t sx1280_get_tx_power(const sx1280_t *dev)
{
    return dev->settings.lora.power;
}

void sx1280_get_rx_buffer_status(sx1280_t *dev, uint8_t *payloadLength,
                                 uint8_t *rxStartBufferPointer)
{
    uint8_t status[2];

    sx1280_read_command_buffer(dev, SX1280_GET_RXBUFFERSTATUS, status, 2);

    /* In case of LORA fixed header, the rxPayloadLength is obtained by reading
       the register REG_LR_PAYLOADLENGTH */
    if ((sx1280_get_packet_type(dev, true) == SX1280_PACKET_TYPE_LORA) &&
        (sx1280_reg_read(dev, REG_LR_PACKETPARAMS) >> 7 == 1)) {
        *payloadLength = sx1280_reg_read(dev, REG_LR_PAYLOADLENGTH);
    }
    else if (sx1280_get_packet_type(dev, true) == SX1280_PACKET_TYPE_BLE) {
        /* In the case of BLE, the size returned in status[0] do not include the 2-byte length PDU
           header so it is added there */
        *payloadLength = status[0] + 2;
    }
    else {
        *payloadLength = status[0];
    }

    *rxStartBufferPointer = status[1];
}

uint8_t sx1280_get_payload(sx1280_t *dev, uint8_t *buffer, uint8_t *size,
                           uint8_t maxSize)
{
    uint8_t offset;

    sx1280_get_rx_buffer_status(dev, size, &offset);
    if (*size > maxSize) {
        return 1;
    }
    sx1280_read_buffer(dev, offset, buffer, *size);
    return 0;
}

void sx1280_set_sync_word_error_tolerance(sx1280_t *dev, uint8_t errorBits)
{
    errorBits =
        (sx1280_reg_read(dev,
                         REG_LR_SYNCWORDTOLERANCE) & 0xF0) | (errorBits & 0x0F);
    sx1280_reg_write(dev, REG_LR_SYNCWORDTOLERANCE, errorBits);
}

uint8_t sx1280_set_crc_seed(sx1280_t *dev, uint8_t *seed)
{
    uint8_t updated = 0;

    switch (sx1280_get_packet_type(dev, true)) {
        case SX1280_PACKET_TYPE_GFSK:
        case SX1280_PACKET_TYPE_FLRC:
            sx1280_write_reg_buffer(dev, REG_LR_CRCSEEDBASEADDR, seed, 2);
            updated = 1;
            break;
        case SX1280_PACKET_TYPE_BLE:
            sx1280_reg_write(dev, 0x9c7, seed[2]);
            sx1280_reg_write(dev, 0x9c8, seed[1]);
            sx1280_reg_write(dev, 0x9c9, seed[0]);
            updated = 1;
            break;
        default:
            break;
    }
    return updated;
}

void sx1280_set_ble_access_address(sx1280_t *dev, uint32_t accessAddress)
{
    sx1280_reg_write(dev, REG_LR_BLE_ACCESS_ADDRESS,
                     (accessAddress >> 24) & 0x000000FF);
    sx1280_reg_write(dev, REG_LR_BLE_ACCESS_ADDRESS + 1,
                     (accessAddress >> 16) & 0x000000FF);
    sx1280_reg_write(dev, REG_LR_BLE_ACCESS_ADDRESS + 2,
                     (accessAddress >> 8) & 0x000000FF);
    sx1280_reg_write(dev, REG_LR_BLE_ACCESS_ADDRESS + 3,
                     accessAddress & 0x000000FF);
}

void sx1280_set_ble_advertizer_access_address(sx1280_t *dev)
{
    sx1280_set_ble_access_address(dev, BLE_ADVERTIZER_ACCESS_ADDRESS);
}

void sx1280_set_crc_polynomial(sx1280_t *dev, uint16_t seed)
{
    uint8_t val[2];

    val[0] = (seed >> 8) & 0xFF;
    val[1] = seed & 0xFF;

    switch (sx1280_get_packet_type(dev, true)) {
        case SX1280_PACKET_TYPE_GFSK:
        case SX1280_PACKET_TYPE_FLRC:
            sx1280_write_reg_buffer(dev, REG_LR_CRCPOLYBASEADDR, val, 2);
            break;
        default:
            break;
    }
}

void sx1280_set_whitening_seed(sx1280_t *dev, uint8_t seed)
{
    switch (sx1280_get_packet_type(dev, true)) {
        case SX1280_PACKET_TYPE_GFSK:
        case SX1280_PACKET_TYPE_FLRC:
        case SX1280_PACKET_TYPE_BLE:
            sx1280_reg_write(dev, REG_LR_WHITSEEDBASEADDR, seed);
            break;
        default:
            break;
    }
}

void sx1280_set_manual_gain_value(sx1280_t *dev, uint8_t gain)
{
    sx1280_reg_write(dev, REG_MANUAL_GAIN_VALUE,
                     (sx1280_reg_read(dev,
                                      REG_MANUAL_GAIN_VALUE) &
                      MASK_MANUAL_GAIN_VALUE) | gain);
}

void sx1280_set_device_ranging_address(sx1280_t *dev, uint32_t address)
{
    uint8_t addrArray[] =
    { address >> 24, address >> 16, address >> 8, address };

    switch (sx1280_get_packet_type(dev, true)) {
        case SX1280_PACKET_TYPE_RANGING:
            sx1280_write_reg_buffer(dev, REG_LR_DEVICERANGINGADDR, addrArray,
                                    4);
            break;
        default:
            break;
    }
}

void sx1280_set_ranging_request_address(sx1280_t *dev, uint32_t address)
{
    uint8_t addrArray[] =
    { address >> 24, address >> 16, address >> 8, address };

    switch (sx1280_get_packet_type(dev, true)) {
        case SX1280_PACKET_TYPE_RANGING:
            sx1280_write_reg_buffer(dev, REG_LR_REQUESTRANGINGADDR, addrArray,
                                    4);
            break;
        default:
            break;
    }
}

uint8_t sx1280_get_ranging_power_delta_threshold_indicator(sx1280_t *dev)
{
    sx1280_set_standby(dev, SX1280_STDBY_XOSC);
    sx1280_reg_write(dev, 0x97F, sx1280_reg_read(dev, 0x97F) | 0x02); /* enable LoRa modem clock */
    sx1280_reg_write(dev, REG_LR_RANGINGRESULTCONFIG,
                     (sx1280_reg_read(dev,
                                      REG_LR_RANGINGRESULTCONFIG) &
                      MASK_RANGINGMUXSEL) |
                     ((SX1280_RANGING_RESULT_RAW & 0x03) << 4));     /* Select raw results */
    return sx1280_reg_read(dev, REG_RANGING_RSSI);
}

void sx1280_set_ranging_calibration(sx1280_t *dev, uint16_t cal)
{
    switch (sx1280_get_packet_type(dev, true)) {
        case SX1280_PACKET_TYPE_RANGING:
            sx1280_reg_write(dev, REG_LR_RANGINGRERXTXDELAYCAL,
                             (cal >> 8) & 0xFF);
            sx1280_reg_write(dev, REG_LR_RANGINGRERXTXDELAYCAL + 1, cal & 0xFF);
            break;

        default:
            break;
    }
}

void sx1280_set_lna_gain_setting(sx1280_t *dev,
                                 const SX1280_LnaSettings_t lnaSetting)
{
    switch (lnaSetting) {
        case SX1280_LNA_HIGH_SENSITIVITY_MODE:
            sx1280_reg_write(dev, REG_LNA_REGIME,
                             sx1280_reg_read(dev,
                                             REG_LNA_REGIME) | MASK_LNA_REGIME);
            break;
        case SX1280_LNA_LOW_POWER_MODE:
            sx1280_reg_write(dev, REG_LNA_REGIME,
                             sx1280_reg_read(dev,
                                             REG_LNA_REGIME) &
                             ~MASK_LNA_REGIME);
            break;
    }
}

void sx1280_set_ranging_id_length(sx1280_t *dev,
                                  SX1280_RangingIdCheckLengths_t length)
{
    switch (sx1280_get_packet_type(dev, true)) {
        case SX1280_PACKET_TYPE_RANGING:
            sx1280_reg_write(dev, REG_LR_RANGINGIDCHECKLENGTH,
                             ((length & 0x03) << 6) |
                             (sx1280_reg_read(dev,
                                              REG_LR_RANGINGIDCHECKLENGTH) &
                              0x3F));
            break;
        default:
            break;
    }
}

double sx1280_get_ranging_result(sx1280_t *dev,
                                 SX1280_RangingResultTypes_t resultType)
{
    uint32_t valLsb = 0;
    double val = 0.0;

    switch (sx1280_get_packet_type(dev, true)) {
        case SX1280_PACKET_TYPE_RANGING:
            sx1280_set_standby(dev, SX1280_STDBY_XOSC);
            sx1280_reg_write(dev, 0x97F,
                             sx1280_reg_read(dev, 0x97F) | 0x02); /* enable LORA modem clock */
            sx1280_reg_write(
                dev, REG_LR_RANGINGRESULTCONFIG,
                (sx1280_reg_read(dev,
                                 REG_LR_RANGINGRESULTCONFIG) &
                 MASK_RANGINGMUXSEL) |
                ((((uint8_t)resultType) & 0x03) << 4));
            valLsb =
                ((sx1280_reg_read(dev, REG_LR_RANGINGRESULTBASEADDR) << 16) |
                 (sx1280_reg_read(dev,
                                  REG_LR_RANGINGRESULTBASEADDR + 1) << 8) |
                 (sx1280_reg_read(dev, REG_LR_RANGINGRESULTBASEADDR + 2)));
            sx1280_set_standby(dev, SX1280_STDBY_RC);

            /* Convertion from LSB to distance. For explanation on the formula, refer to Datasheet
               of SX1280 */
            switch (resultType) {
                case SX1280_RANGING_RESULT_RAW:
                    /* Convert the ranging LSB to distance in meter */
                    val = (double)sx1280_complement2(valLsb, 24) /
                          (double)dev->settings.lora.bandwidth * 36621.09375;
                    break;

                case SX1280_RANGING_RESULT_AVERAGED:
                case SX1280_RANGING_RESULT_DEBIASED:
                case SX1280_RANGING_RESULT_FILTERED:
                    val = (double)valLsb * 20.0 / 100.0;
                    break;

                default:
                    val = 0.0;
            }
            break;

        default:
            break;
    }
    return val;
}

double sx1280_get_frequency_error(sx1280_t *dev)
{
    uint8_t efeRaw[3] = { 0 };
    uint32_t efe = 0;
    double efeHz = 0.0;

    switch (sx1280_get_packet_type(dev, true)) {
        case SX1280_PACKET_TYPE_LORA:
        case SX1280_PACKET_TYPE_RANGING:
            efeRaw[0] = sx1280_reg_read(dev,
                                        REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB);
            efeRaw[1] = sx1280_reg_read(dev,
                                        REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB +
                                        1);
            efeRaw[2] = sx1280_reg_read(dev,
                                        REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB +
                                        2);
            efe = (efeRaw[0] << 16) | (efeRaw[1] << 8) | efeRaw[2];
            efe &= REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK;

            efeHz = 1.55 * (double)sx1280_complement2(efe, 20) /
                    (1600.0 / (double)dev->settings.lora.bandwidth * 1000.0);
            break;

        case SX1280_PACKET_TYPE_NONE:
        case SX1280_PACKET_TYPE_BLE:
        case SX1280_PACKET_TYPE_FLRC:
        case SX1280_PACKET_TYPE_GFSK:
            break;
    }

    return efeHz;
}

void sx1280_save_context(sx1280_t *dev)
{
    sx1280_write_command(dev, SX1280_SET_SAVECONTEXT, 0);
}
