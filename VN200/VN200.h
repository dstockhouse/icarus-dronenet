/***************************************************************************\
 *
 * File:
 * 	VN200.h
 *
 * Description:
 *	Function and type declarations and constants for VN200.c
 * 
 * Author:
 * 	David Stockhouse
 *
 * Revision 0.1
 * 	Last edited 5/06/2019
 *
 ***************************************************************************/

#ifndef __VN200_H
#define __VN200_H

#include "../uart/uart.h"
#include "../buffer/buffer.h"
#include "../logger/logger.h"

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

#define VN200_DEVNAME "/dev/ttyUSB0"
#define VN200_BAUD 57600

typedef struct {
	int fd;
	BYTE_BUFFER inbuf;
	BYTE_BUFFER outbuf;
	LOG_FILE logFile;
	int baud; // Baud rate 115200, 128000, 230400, 460800, 921600
	int fs; // Sampling Frequency => 100
} VN200_DEV;

int VN200BaseInit(VN200_DEV *dev);

int VN200Poll(VN200_DEV *dev);

int VN200Consume(VN200_DEV *dev, int num);

int VN200FlushInput(VN200_DEV *dev);

int VN200Command(VN200_DEV *dev, char *buf, int num);

int VN200FlushOutput(VN200_DEV *dev);

int VN200Destroy(VN200_DEV *dev);

#endif
