#include "pixy.h"
#include "peripherals.h"
#include "fsl_lpi2c_edma.h"
#include <stdbool.h>

static volatile bool transferDone;

static void pixy_edma_cb(LPI2C_Type *base,
                         lpi2c_master_edma_handle_t *handle,
                         status_t status,
                         void *userData)
{
    (void)base; (void)handle; (void)userData;
    if (status == kStatus_Success) {
        transferDone = true;
    }
}

void pixy_init(pixy_t *cam,
               LPI2C_Type *inst,
               uint8_t addr,
               edma_handle_t *rxHandle,
               edma_handle_t *txHandle)
{
    cam->instance = inst;
    cam->address  = addr;
    LPI2C_MasterCreateEDMAHandle(
        cam->instance,
        &cam->edmaHandle,
        rxHandle,
        txHandle,
        pixy_edma_cb,
        NULL);
}

static status_t pixy_send(pixy_t *cam, const uint8_t *cmd, size_t len)
{
    transferDone = false;
    lpi2c_master_transfer_t xfer = {
        .slaveAddress   = cam->address,
        .direction      = kLPI2C_Write,
        .subaddressSize = 0,
        .data           = (uint8_t *)cmd,
        .dataSize       = len,
        .flags          = kLPI2C_TransferDefaultFlag,
    };
    status_t s = LPI2C_MasterTransferEDMA(cam->instance, &cam->edmaHandle, &xfer);
    if (s != kStatus_Success) return s;
    while (!transferDone) {}
    return kStatus_Success;
}

static status_t pixy_recv(pixy_t *cam, uint8_t *buf, size_t len)
{
    transferDone = false;
    lpi2c_master_transfer_t xfer = {
        .slaveAddress   = cam->address,
        .direction      = kLPI2C_Read,
        .subaddressSize = 0,
        .data           = buf,
        .dataSize       = len,
        .flags          = kLPI2C_TransferDefaultFlag,
    };
    status_t s = LPI2C_MasterTransferEDMA(cam->instance, &cam->edmaHandle, &xfer);
    if (s != kStatus_Success) return s;
    while (!transferDone) {}
    return kStatus_Success;
}

status_t pixy_set_led(pixy_t *cam, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t cmd[7] = {0xAE, 0xC1, 20, 3, r, g, b};
    status_t s = pixy_send(cam, cmd, sizeof(cmd));
    if (s != kStatus_Success) return s;
    uint8_t resp[10];
    
    return pixy_recv(cam, resp, sizeof(resp));
}

status_t pixy_get_vectors(pixy_t *cam,
                          uint16_t *vectors,
                          size_t max_vectors,
                          size_t *num_vectors)
{
    uint8_t cmd[6] = {0xAE, 0xC1, 48, 2, 1, 1};
    status_t s = pixy_send(cam, cmd, sizeof(cmd));
    if (s != kStatus_Success) return s;

    uint8_t buf[100];
    s = pixy_recv(cam, buf, sizeof(buf));
    if (s != kStatus_Success) return s;

    uint16_t packetLen = buf[3] + 4;
    size_t   count     = 0;
    uint16_t idx       = 6;

    while (idx + 1 < packetLen && count < max_vectors) {
        uint8_t featLen = buf[idx + 1];
        uint8_t *data   = &buf[idx + 2];
        uint8_t nvec    = featLen / 6;

        for (uint8_t i = 0; i < nvec && count < max_vectors; i++) {
            size_t off = i * 6;
            vectors[4*count + 0] = data[off + 0];
            vectors[4*count + 1] = data[off + 1];
            vectors[4*count + 2] = data[off + 2];
            vectors[4*count + 3] = data[off + 3];
            count++;
        }
        idx += 2 + featLen;
    }

    *num_vectors = count;
    return kStatus_Success;
}
