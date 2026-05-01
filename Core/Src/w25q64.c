#include "w25q64.h"
#include "stdlib.h"
#include "spi.h"
#include "gpio.h"







//Read the chip ID
//// 0XEF16 indicates that the chip model is W25Q64
uint16_t W25Q64_readID(void)
{
    uint16_t  ID = 0;
    SPI_FLASH_CS(0);
    SPI1_ReadWriteByte(0x90);
    SPI1_ReadWriteByte(0x00);
    SPI1_ReadWriteByte(0x00);
    SPI1_ReadWriteByte(0x00);
    ID |= SPI1_ReadWriteByte(0xFF)<<8;
    ID |= SPI1_ReadWriteByte(0xFF);
    SPI_FLASH_CS(1);
    return ID;
}






void W25Q64_write_enable(void)
{
	SPI_FLASH_CS(0);
	SPI1_ReadWriteByte(0x06);
    SPI_FLASH_CS(1);
}



void W25Q64_wait_busy(void)
{
    unsigned char byte = 0;
    do
     {
    	SPI_FLASH_CS(0);
    	SPI1_ReadWriteByte(0x05);
        byte = SPI1_ReadWriteByte(0Xff);
        SPI_FLASH_CS(1);
     }while( ( byte & 0x01 ) == 1 );
}




void W25Q64_erase_sector(uint32_t addr)
{
        addr *= 4096;
        W25Q64_write_enable();
        W25Q64_wait_busy();
        SPI_FLASH_CS(0);
        SPI1_ReadWriteByte(0x20);
        SPI1_ReadWriteByte((uint8_t)((addr)>>16));
        SPI1_ReadWriteByte((uint8_t)((addr)>>8));
        SPI1_ReadWriteByte((uint8_t)addr);
        SPI_FLASH_CS(1);

        W25Q64_wait_busy();
}



void W25Q64_write(uint8_t* buffer, uint32_t addr, uint16_t numbyte)
{
    unsigned int i = 0;
    W25Q64_erase_sector(addr/4096);
    W25Q64_write_enable();
    W25Q64_wait_busy();
    SPI_FLASH_CS(0);
    SPI1_ReadWriteByte(0x02);
    SPI1_ReadWriteByte((uint8_t)((addr)>>16));
    SPI1_ReadWriteByte((uint8_t)((addr)>>8));
    SPI1_ReadWriteByte((uint8_t)addr);
    for(i=0;i<numbyte;i++)
    {
    	SPI1_ReadWriteByte(buffer[i]);
    }
    SPI_FLASH_CS(0);
    W25Q64_wait_busy();
}






void W25Q64_read(uint8_t* buffer,uint32_t read_addr,uint16_t read_length)
{
        uint16_t i;

        SPI_FLASH_CS(0);
        SPI1_ReadWriteByte(0x03);
        SPI1_ReadWriteByte((uint8_t)((read_addr)>>16));
        SPI1_ReadWriteByte((uint8_t)((read_addr)>>8));
        SPI1_ReadWriteByte((uint8_t)read_addr);
        for(i=0;i<read_length;i++)
        {
            buffer[i]= SPI1_ReadWriteByte(0XFF);
        }
        SPI_FLASH_CS(1);
}

