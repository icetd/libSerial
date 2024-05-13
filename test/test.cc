#include <iostream>
#include <unistd.h>
#include "log.h"
#include "Serial.h"

int LogLevel;

void onSerialCmd(uint8_t *data, uint16_t len)
{
    LOG(NOTICE, "reveive message: %s, length: %d", data, len);
}


int main()
{
    initLogger(INFO);
    Serial *serial = new Serial("/dev/ttyUSB0");

    int re;
    re = serial->Init();
    if (re < 0) {
        LOG(ERROR, "Serial Init Failed.");
        exit(0);
    }

    Serial::SerialOpt_t serialOpt {
        .speed = 115200,
        .dataBits = 8,
        .stopbits = 1,
        .parity = 'n',
        .flowControlMode = 's',
    };

    re = serial->Setopt(&serialOpt);
    if (re < 0) {
        LOG(ERROR, "Serial Init Failed.");
        exit(0);
    } else {
        LOG(INFO, "Serial Successflly Set.");
    }

    serial->SetSerialHandler(onSerialCmd);
    serial->StartAutoRead();

    for (;;) {
	sleep(1);
        std::string buf = "hello\r\n";
        serial->Write((char *)buf.c_str(), 7);
    }

    serial->StopAutoRead();
	return 0;
}

