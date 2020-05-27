/** \file 
 * \brief
 * Header file for wiznet_drv.c
 * W5500 EtherCAT driver for raspberry pi
 * By Eric Chung, eric.chung[at]stratusauto.com
 */

/*****************************************************
*Wiznet driver for RPI SOEM - V1.0
*because Someone didn't open source his file
*so I make my own library
*Author: Eric  - pakchin.chung[at]gmail.com
******************************************************
*/

//gcc wiznet_drv.c -o test -lwiringPi


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <wiringPiSPI.h>
#include <wiringPi.h>
#include "wiznet_drv.h"

unsigned char Active_Port;

//temp test for UA part
const char mac_address[] = {0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
const char mac_address_red[] = {0x04, 0x04, 0x04, 0x04, 0x04, 0x04};
int initResult;
//end temp test for UA part

unsigned char wizchip_read(unsigned char block, unsigned short int address)
{
    unsigned char ret;
	
	unsigned char tData[5];
	
	tData[0] = (address & 0xFF00) >> 8;
	tData[1] = address & 0x00FF;
	block |= AccessModeRead;
	tData[2] = block;

	/*
    wizchip_cs_select();

    block |= AccessModeRead;

    wizchip_spi_write_byte((address & 0xFF00) >> 8);
    wizchip_spi_write_byte((address & 0x00FF) >> 0);
    wizchip_spi_write_byte(block);

    ret = wizchip_spi_read_byte();

    wizchip_cs_deselect();
	*/
	switch (Active_Port)
	{
		case 0:
			wiringPiSPIDataRW(0, &tData[0], 4);
			break;
		case 1:
			wiringPiSPIDataRW(1, &tData[0], 4);
			break;
	}
	
	ret = tData[3];
	
    return ret;
}

unsigned short int wizchip_read_word(unsigned char block, unsigned short int address)
{	
    return ((unsigned short int)wizchip_read(block, address) << 8) + wizchip_read(block, address + 1);
}

void wizchip_read_buf(unsigned char block, unsigned short int address, unsigned char* pBuf, unsigned short int len)
{
	/*
    unsigned short int i;

    wizchip_cs_select();

    block |= AccessModeRead;

    wizchip_spi_write_byte((address & 0xFF00) >> 8);
    wizchip_spi_write_byte((address & 0x00FF) >> 0);
    wizchip_spi_write_byte(block);
    for(i = 0; i < len; i++)
        pBuf[i] = wizchip_spi_read_byte();

    wizchip_cs_deselect();
	*/
		
	unsigned char tData[4096];			//not sure if this will cause any error or not as the array size is too big
	
	tData[0] = (address & 0xFF00) >> 8;
	tData[1] = address & 0x00FF;
	block |= AccessModeRead;
	tData[2] = block;
	
	
	switch (Active_Port)
	{
		case 0:
			wiringPiSPIDataRW(0, &tData[0], len + 3);
			break;
		case 1:
			wiringPiSPIDataRW(1, &tData[0], len + 3);
			break;
	}
	
	
	
	for(unsigned short int i = 0; i < len + 3; i++)
	{
		*pBuf = tData[i+3];
		pBuf++;
	}
	
}

void wizchip_write(unsigned char block, unsigned short int address, unsigned char wb)
{
	/*
    wizchip_cs_select();

    block |= AccessModeWrite;

    wizchip_spi_write_byte((address & 0xFF00) >> 8);
    wizchip_spi_write_byte((address & 0x00FF) >> 0);
    wizchip_spi_write_byte(block);
    wizchip_spi_write_byte(wb);

    wizchip_cs_deselect();
	*/
	
	unsigned char tData[4];
	
	tData[0] = (address & 0xFF00) >> 8;
	tData[1] = address & 0x00FF;
	block |= AccessModeWrite;
	tData[2] = block;
	tData[3] = wb;
	
	
	
	switch (Active_Port)
	{
		case 0:
			wiringPiSPIDataRW(0, &tData[0], 4);
			break;
		case 1:
			wiringPiSPIDataRW(1, &tData[0], 4);
			break;
	}
		
}

void wizchip_write_word(unsigned char block, unsigned short int address, unsigned short int word)
{
    wizchip_write(block, address,   (unsigned char)(word>>8));
    wizchip_write(block, address+1, (unsigned char) word);
}

void wizchip_write_buf(unsigned char block, unsigned short int address, const unsigned char* pBuf, unsigned short int len)
{
	/*
    unsigned short int i;

    wizchip_cs_select();

    block |= AccessModeWrite;

    wizchip_spi_write_byte((address & 0xFF00) >> 8);
    wizchip_spi_write_byte((address & 0x00FF) >> 0);
    wizchip_spi_write_byte(block);
    for(i = 0; i < len; i++)
        wizchip_spi_write_byte(pBuf[i]);

    wizchip_cs_deselect();
	*/
	
	unsigned char tData[4096];			//not sure if this will cause any error or not as the array size is too big
										//and hope 'len' is lesser than 4096
	
	tData[0] = (address & 0xFF00) >> 8;
	tData[1] = address & 0x00FF;
	block |= AccessModeWrite;
	tData[2] = block;
	
	for(unsigned short int i = 0; i < len + 3; i++)
	{
		tData[i + 3] = pBuf[i];
	}
	
	switch (Active_Port)
	{
		case 0:
			wiringPiSPIDataRW(0, &tData[0], len + 3);
			break;
		case 1:
			wiringPiSPIDataRW(1, &tData[0], len + 3);
			break;
	}
}

void setSn_CR(unsigned char cr) {
    // Write the command to the Command Register
    wizchip_write(BlockSelectSReg, Sn_CR, cr);

    // Now wait for the command to complete
    while( wizchip_read(BlockSelectSReg, Sn_CR) );
}

unsigned short int getSn_TX_FSR()
{
    unsigned short int val=0,val1=0;
    do
    {
        val1 = wizchip_read_word(BlockSelectSReg, Sn_TX_FSR);
        if (val1 != 0)
        {
            val = wizchip_read_word(BlockSelectSReg, Sn_TX_FSR);
        }
    } while (val != val1);
    return val;
}


unsigned short int getSn_RX_RSR()
{
    unsigned short int val=0,val1=0;
    do
    {
        val1 = wizchip_read_word(BlockSelectSReg, Sn_RX_RSR);
        if (val1 != 0)
        {
            val = wizchip_read_word(BlockSelectSReg, Sn_RX_RSR);
        }
    } while (val != val1);
    return val;
}

void wizchip_send_data(const unsigned char *wizdata, unsigned short int len)
{
    unsigned short int ptr = 0;

    if(len == 0) return;
    ptr = getSn_TX_WR();
    wizchip_write_buf(BlockSelectTxBuf, ptr, wizdata, len);

    ptr += len;

    setSn_TX_WR(ptr);
}

void wizchip_recv_data(unsigned char *wizdata, unsigned short int len)
{
    unsigned short int ptr;

    if(len == 0) return;
    ptr = getSn_RX_RD();
    wizchip_read_buf(BlockSelectRxBuf, ptr, wizdata, len);
    ptr += len;

    setSn_RX_RD(ptr);
}

void wizchip_recv_ignore(unsigned short int len)
{
    unsigned short int ptr;

    ptr = getSn_RX_RD();
    ptr += len;
    setSn_RX_RD(ptr);
}

void wizchip_sw_reset()
{
    setMR(MR_RST);
    getMR(); // for delay

    
	
	if (Active_Port == 1)
	{
		setSHAR(_mac_address_red);
	}
	else
	{
		setSHAR(_mac_address);
	}
}

int8_t wizphy_getphylink()
{
    int8_t tmp;
    if(getPHYCFGR() & PHYCFGR_LNK_ON)
        tmp = PHY_LINK_ON;
    else
        tmp = PHY_LINK_OFF;
    return tmp;
}

int8_t wizphy_getphypmode()
{
    int8_t tmp = 0;
    if(getPHYCFGR() & PHYCFGR_OPMDC_PDOWN)
        tmp = PHY_POWER_DOWN;
    else
        tmp = PHY_POWER_NORM;
    return tmp;
}

void wizphy_reset()
{
    unsigned char tmp = getPHYCFGR();
    tmp &= PHYCFGR_RST;
    setPHYCFGR(tmp);
    tmp = getPHYCFGR();
    tmp |= ~PHYCFGR_RST;
    setPHYCFGR(tmp);
}

int8_t wizphy_setphypmode(unsigned char pmode)
{
    unsigned char tmp = 0;
    tmp = getPHYCFGR();
    if((tmp & PHYCFGR_OPMD)== 0) return -1;
    tmp &= ~PHYCFGR_OPMDC_ALLA;
    if( pmode == PHY_POWER_DOWN)
        tmp |= PHYCFGR_OPMDC_PDOWN;
    else
        tmp |= PHYCFGR_OPMDC_ALLA;
    setPHYCFGR(tmp);
    wizphy_reset();
    tmp = getPHYCFGR();
    if( pmode == PHY_POWER_DOWN)
    {
        if(tmp & PHYCFGR_OPMDC_PDOWN) return 0;
    }
    else
    {
        if(tmp & PHYCFGR_OPMDC_ALLA) return 0;
    }
    return -1;
}

unsigned char begin(void)
{
    

	if (Active_Port == 1)
	{
		memcpy(_mac_address_red, mac_address, 6);
	}
	else
	{
		memcpy(_mac_address, mac_address, 6);
	}

    wizchip_sw_reset();
	
    // Use the full 16Kb of RAM for Socket 0
    setSn_RXBUF_SIZE(16);
    setSn_TXBUF_SIZE(16);
	
    // Set our local MAC address
	if (Active_Port == 1)
	{
		setSHAR(_mac_address_red);
	}
	else
	{
		setSHAR(_mac_address);
	}
	
    // Open Socket 0 in MACRaw mode
    setSn_MR(Sn_MR_MACRAW);
    setSn_CR(Sn_CR_OPEN);
	
    if (getSn_SR() != SOCK_MACRAW) {
        // Failed to put socket 0 into MACRaw mode
        return 0;
    }

    // Success
    return 1;
}

void end()
{
    setSn_CR(Sn_CR_CLOSE);

    // clear all interrupt of the socket
    setSn_IR(0xFF);

    // Wait for socket to change to closed
    while(getSn_SR() != SOCK_CLOSED);
}

int wiznet_macraw_recv(unsigned char channel, unsigned char * buffer, size_t bufsize)
{
    unsigned short int len = getSn_RX_RSR();
	
	//some user input error handling
	if (channel > 0)
	{
		Active_Port = 1;
	}
	else if (channel <= 0)
	{
		Active_Port = 0;
	}
	//end some user input error handling
    
    if (len > 0)
    {
        unsigned char head[2];
        unsigned short int data_len=0;

        wizchip_recv_data(head, 2);
        setSn_CR(Sn_CR_RECV);

        data_len = head[0];
        data_len = (data_len<<8) + head[1];
        data_len -= 2;

        if (data_len > bufsize)
        {
            // Packet is bigger than buffer - drop the packet
            wizchip_recv_ignore(data_len);
            setSn_CR(Sn_CR_RECV);
            return 0;
        }

        wizchip_recv_data(buffer, data_len);
        setSn_CR(Sn_CR_RECV);

        // Had problems with W5500 MAC address filtering (the Sn_MR_MFEN option)
        // Do it in software instead:
		
		if (Active_Port == 0)
		{
			if ((buffer[0] & 0x01) || memcmp(&buffer[0], _mac_address, 6) == 0)
			{
				// Addressed to an Ethernet multicast address or our unicast address
				return data_len;
			} 
			else 
			{
				return 0;
			}
		}
		else
		{
			if ((buffer[0] & 0x01) || memcmp(&buffer[0], _mac_address_red, 6) == 0)
			{
				// Addressed to an Ethernet multicast address or our unicast address
				return data_len;
			} 
			else 
			{
				return 0;
			}
		}
    }

    return 0;
}

int wiznet_macraw_send(unsigned char channel, void *buf, unsigned short int len)
{
	//some user input error handling
	if (channel > 0)
	{
		Active_Port = 1;
	}
	else if (channel <= 0)
	{
		Active_Port = 0;
	}
	//end some user input error handling
	
    // Wait for space in the transmit buffer
    while(1)
    {
        unsigned short int freesize = getSn_TX_FSR();
        if(getSn_SR() == SOCK_CLOSED) {
            return -1;
        }
        if (len <= freesize) break;
    };

    wizchip_send_data(buf, len);
    setSn_CR(Sn_CR_SEND);

    while(1)
    {
        unsigned char tmp = getSn_IR();
        if (tmp & Sn_IR_SENDOK)
        {
            setSn_IR(Sn_IR_SENDOK);
            // Packet sent ok
            break;
        }
        else if (tmp & Sn_IR_TIMEOUT)
        {
            setSn_IR(Sn_IR_TIMEOUT);
            // There was a timeout
            return -1;
        }
    }

    return len;
}

int wiznet_macraw_init(unsigned char Channel)
{
	//some user input error handling
	if (Channel > 0)
	{
		Active_Port = 1;
		wiringPiSPISetup(1, 62500000);
	}
	else if (Channel <= 0)
	{
		Active_Port = 0;
		wiringPiSPISetup(0, 62500000);
	}
	//end some user input error handling
	
	
	
	printf("\n\n******************************************************\n");
	printf("*Wiznet driver for RPI SOEM - V1.0\n");
	printf("*because someone didn't open source his file\n");
	printf("*so I make my own library\n");
	printf("*Author: Eric  - pakchin.chung[at]gmail.com\n");
	printf("*\n");
	printf("*Initializing channel %d\n", Active_Port);
	wiznet_hw_config(Channel, 1, 2000);
	printf("******************************************************\n\n\n");
	
	
	return 1;
}

int wiznet_hw_config(unsigned char Channel, int startup_rst, unsigned long link_wait_ms)
{
	int retCode = 0;
	//0 means no error
	//1 means link down
	
	//perform a hardware reset
	wiringPiSetup();	
	pinMode(8, OUTPUT);
	pinMode(9, OUTPUT);
	digitalWrite(8, HIGH);
	digitalWrite(9, HIGH);
	
	
	//some user input error handling
	if (link_wait_ms > 10000)
	{
		link_wait_ms = 10000;
	}
	else if ((link_wait_ms <= 2000) && (link_wait_ms > 0))
	{
		link_wait_ms = 2000;
	}
	else if (link_wait_ms < 0)
	{
		link_wait_ms = 0;
	}
	//end some user input error handling
	
	printf("*Link_wait_ms is : %d\n", link_wait_ms);
	
	if (startup_rst)
	{
		switch (Channel)
		{
			case 0:
				digitalWrite(8, LOW);
				delay(2);
				digitalWrite(8, HIGH);
				printf("*Hard reset channel 0 done.\n");
				break;
			case 1:
				digitalWrite(9, LOW);
				delay(2);
				digitalWrite(9, HIGH);
				printf("*Hard reset channel 1 done.\n");
				break;
		}
	}
	
	unsigned long TimerNow = millis();
	
	while ((millis() - TimerNow < link_wait_ms) && !wizphy_getphylink())
	{
	}
	
	if (!wizphy_getphylink() && (link_wait_ms != 0))
	{
		retCode = 1;
		printf("*Waiting LINK UP timeout!\n");
	}
	
	initResult = begin();
	if (initResult > 0)
	{
		printf("*MACRAW mode setup ok! \n", initResult);
	}
	else
	{
		printf("*MACRAW mode setup failed! \n", initResult);
	}
	
	printf("*Link status is : %d \n", wizphy_getphylink());
	printf("*Time taken to init channel %d is : %d \n", Active_Port, millis() - TimerNow);	
	
	return retCode;
}

void setMR(unsigned char mode) {
	wizchip_write(BlockSelectCReg, MR, mode);
}

unsigned char getMR() {
	return wizchip_read(BlockSelectCReg, MR);
}

void setSHAR(const unsigned char* macaddr) {
	wizchip_write_buf(BlockSelectCReg, SHAR, macaddr, 6);
}

void getSHAR(unsigned char* macaddr) {
	wizchip_read_buf(BlockSelectCReg, SHAR, macaddr, 6);
}

void setIR(unsigned char ir) {
	wizchip_write(BlockSelectCReg, IR, (ir & 0xF0));
}

unsigned char getIR() {
	return wizchip_read(BlockSelectCReg, IR) & 0xF0;
}

void setIMR(unsigned char imr) {
	wizchip_write(BlockSelectCReg, _IMR_, imr);
}

unsigned char getIMR() {
	return wizchip_read(BlockSelectCReg, _IMR_);
}

void setPHYCFGR(unsigned char phycfgr) {
	wizchip_write(BlockSelectCReg, PHYCFGR, phycfgr);
}

unsigned char getPHYCFGR() {
	return wizchip_read(BlockSelectCReg, PHYCFGR);
}

unsigned char getVERSIONR() {
	return wizchip_read(BlockSelectCReg, VERSIONR);
}

void setSn_MR(unsigned char mr) {
	wizchip_write(BlockSelectSReg, Sn_MR, mr);
}

unsigned char getSn_MR() {
	return wizchip_read(BlockSelectSReg, Sn_MR);
}

unsigned char getSn_CR() {
	return wizchip_read(BlockSelectSReg, Sn_CR);
}

void setSn_IR(unsigned char ir) {
	wizchip_write(BlockSelectSReg, Sn_IR, (ir & 0x1F));
}

unsigned char getSn_IR() {
	return (wizchip_read(BlockSelectSReg, Sn_IR) & 0x1F);
}

void setSn_IMR(unsigned char imr) {
	wizchip_write(BlockSelectSReg, Sn_IMR, (imr & 0x1F));
}

unsigned char getSn_IMR() {
	return (wizchip_read(BlockSelectSReg, Sn_IMR) & 0x1F);
}

unsigned char getSn_SR() {
	return wizchip_read(BlockSelectSReg, Sn_SR);
}

void setSn_RXBUF_SIZE(unsigned char rxbufsize) {
	wizchip_write(BlockSelectSReg, Sn_RXBUF_SIZE, rxbufsize);
}

unsigned char getSn_RXBUF_SIZE() {
	return wizchip_read(BlockSelectSReg, Sn_RXBUF_SIZE);
}

void setSn_TXBUF_SIZE(unsigned char txbufsize) {
	wizchip_write(BlockSelectSReg, Sn_TXBUF_SIZE, txbufsize);
}

unsigned char getSn_TXBUF_SIZE() {
	return wizchip_read(BlockSelectSReg, Sn_TXBUF_SIZE);
}

unsigned short int getSn_TX_RD() {
	return wizchip_read_word(BlockSelectSReg, Sn_TX_RD);
}

void setSn_TX_WR(unsigned short int txwr) {
	wizchip_write_word(BlockSelectSReg, Sn_TX_WR, txwr);
}

unsigned short int getSn_TX_WR() {
	return wizchip_read_word(BlockSelectSReg, Sn_TX_WR);
}

void setSn_RX_RD(unsigned short int rxrd) {
	wizchip_write_word(BlockSelectSReg, Sn_RX_RD, rxrd);
}

unsigned short int getSn_RX_RD() {
	return wizchip_read_word(BlockSelectSReg, Sn_RX_RD);
}

unsigned short int getSn_RX_WR() {
	return wizchip_read_word(BlockSelectSReg, Sn_RX_WR);
}