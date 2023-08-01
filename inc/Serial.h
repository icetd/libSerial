#ifndef SERIAL_H
#define SERIAL_H

#include <string>
#include "MThread.h"

class Serial : public MThread
{
public:
	typedef struct{
		int speed;
		int dataBits;
		int stopbits;
		int	parity;
		int hardflow;
	} SerialOpt_t;

	Serial(const char *deviceName);
	Serial(const char *deviceName, int openFlags);
	virtual ~Serial();
	
	int Init();
	bool IsReady();
	int Setopt(SerialOpt_t *serialOpt);
	int Write(char *buf, int len);
	int Read(char *buf, int len);

    int Destroy();

    void SetSerialHandler(void (* xerc) (uint8_t*, uint16_t));
    void StartAutoRead();
    void StopAutoRead();
    virtual void run() override;


private:
	int m_fd;
    std::string m_deviceName;
	int m_openFlags;
	SerialOpt_t serialOpt;
    bool isAutoRead = false;
    void (* OnSerialEnd) (uint8_t *data, uint16_t len);
};

#endif
