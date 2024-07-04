#ifndef SERIAL_H
#define SERIAL_H

#include <bits/stdint-uintn.h>
#include <string>
#include <functional>
#include "MThread.h"


class Serial : public MThread
{
public:
	typedef struct{
		int	 speed;
		int dataBits;
		int stopbits;
		int parity;
		int flowControlMode;
		int vtime;
		int vmin;
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

	void SetSerialHandler(std::function<void (uint8_t*, uint16_t)> callback);
    void StartAutoRead();
    void StopAutoRead();
    virtual void run() override;


private:
	int m_fd;
    std::string m_deviceName;
	int m_openFlags;
	SerialOpt_t serialOpt;
    bool isAutoRead = false;
	std::function<void (uint8_t *, uint16_t )> onSerialCallback;
};

#endif
