#pragma once
class ChannelDevice
{
public:
    virtual bool bind(const char*, uint32_t ) = 0;
    virtual bool open() = 0;
    virtual bool isOpen() = 0;
    virtual void close() = 0;
    virtual void flush() = 0;
    virtual int waitfordata(size_t data_count,uint32_t timeout = -1, size_t * returned_size = NULL) = 0;
    virtual int writedata(const uint8_t * data, size_t size) = 0;
    virtual int readdata(unsigned char * data, size_t size) = 0;
    virtual bool setDTR(bool level = true) {return true;}
    virtual int getByteTime() { return 0;}
};
