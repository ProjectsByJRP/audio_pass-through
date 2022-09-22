
#include "i2c.h"


uint8_t CODEC_IO_Write(uint8_t Addr, uint16_t Reg, uint16_t Value) {
    uint16_t tmp = Value;

    Value = ((uint16_t) (tmp >> 8) & 0x00FF);

    Value |= ((uint16_t) (tmp << 8) & 0xFF00);


    // I2Cx_WriteMultiple(&hi2c4, Addr, Reg, I2C_MEMADD_SIZE_16BIT,(uint8_t*)&Value, 2);
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write(&hi2c4, Addr, (uint16_t) Reg, I2C_MEMADD_SIZE_16BIT, (uint8_t *) &Value, 2, 1000);

    /* Check the communication status */
    if (status != HAL_OK) {
        /* Re-Initiaize the I2C Bus */
        /* De-initialize the I2C communication bus */
        HAL_I2C_DeInit(&hi2c4);

        /* Re-Initialize the I2C communication bus */
        HAL_I2C_Init(&hi2c4);
        return 1;
    }

    return 0;
}


uint16_t CODEC_IO_Read(uint8_t Addr, uint16_t Reg) {
    uint16_t read_value = 0, tmp = 0;

//  I2Cx_ReadMultiple(&hi2c4, Addr, Reg, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&read_value, 2);
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c4, Addr, (uint16_t) Reg, I2C_MEMADD_SIZE_16BIT, (uint8_t *) &read_value, 2, 1000);

    /* Check the communication status */
    if (status != HAL_OK) {
        /* I2C error occured */
        //I2Cx_Error(&hi2c4, Addr);
        /* De-initialize the I2C communication bus */
        HAL_I2C_DeInit(&hi2c4);

        /* Re-Initialize the I2C communication bus */
        HAL_I2C_Init(&hi2c4);

    } else {

        tmp = ((uint16_t) (read_value >> 8) & 0x00FF);

        tmp |= ((uint16_t) (read_value << 8) & 0xFF00);

        read_value = tmp;
    }
    return read_value;
}


void CODEC_IO_Delay(uint32_t Delay) {
    HAL_Delay(Delay);
}
