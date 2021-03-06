/*
 * DioUsb1208hs.h
 *
 *     Author: Measurement Computing Corporation
 */

#ifndef USB_DIO_DIOUSB1208HS_H_
#define USB_DIO_DIOUSB1208HS_H_

#include "DioUsbBase.h"

namespace ul
{

class UL_LOCAL DioUsb1208hs: public DioUsbBase
{
public:
	DioUsb1208hs(const UsbDaqDevice& daqDevice);
	virtual ~DioUsb1208hs();

	virtual void initialize();

	virtual void dConfigPort(DigitalPortType portType, DigitalDirection direction);
	virtual void dConfigBit(DigitalPortType portType, int bitNum, DigitalDirection direction);

	virtual unsigned long long dIn(DigitalPortType portType);
	virtual void dOut(DigitalPortType portType, unsigned long long data);

	virtual bool dBitIn(DigitalPortType portType, int bitNum);
	virtual void dBitOut(DigitalPortType portType, int bitNum, bool bitValue);

protected:
	virtual unsigned long readPortDirMask(unsigned int portNum) const;

private:
	enum {CMD_DTRISTATE = 0x00, CMD_DPORT = 0x01, CMD_DLATCH = 0x02};
};

} /* namespace ul */

#endif /* USB_DIO_DIOUSB1208HS_H_ */
