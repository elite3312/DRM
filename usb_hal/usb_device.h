#ifndef __MSDISP_MS9132_H__
#define __MSDISP_MS9132_H__

#include "hal_adaptor.h"

#define MSDISP_913X_VENDOR 0x345f
#define MSDISP_9132_PRODUCT 0x9132
#define MSDISP_9133_PRODUCT 0x9133
#define MSDISP_9135_PRODUCT 0x9135

#define MSDISP_8268_VENDOR 0x1DE1
#define MSDISP_8268_PRODUCT 0xF201


extern const struct msdisp_hal_id ms9132_id;
extern const struct msdisp_hal_id ms9133_id;
extern const struct msdisp_hal_id ms9135_id;
extern const struct msdisp_hal_id am8268n_id;
extern struct msdisp_hal_dev ms9132_dev;
extern struct msdisp_hal_dev ms9133_dev;
extern struct msdisp_hal_dev ms9135_dev;
extern struct msdisp_hal_dev am8268n_dev;

#endif