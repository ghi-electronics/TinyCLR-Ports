#include "GHIElectronics_TinyCLR_Devices.h"

#define TIMEOUT_COUNTER 10

#define I2C_STANDARD_SPEED_US 5

#define I2CDELAY(x) TinyCLR_Interop_Delay(x)

#define ClearSCL() MakePinOutput(softwareI2cConfig->scl)
#define ClearSDA() MakePinOutput(softwareI2cConfig->sda)

struct SoftwareI2CConfig {
    uint32_t sda;
    uint32_t scl;
    uint32_t clockSpeed; // currently ignored
    uint8_t address;	   // 7 bit address
    bool useSoftwarePullups;
} *softwareI2cConfig;

void MakePinOutput(uint32_t pin);
void MakePinInput(SoftwareI2CConfig* softwareI2cConfig, uint32_t pin);
bool ReadPinState(SoftwareI2CConfig* softwareI2cConfig, uint32_t pin);
bool ReadBit();
bool WriteBit(bool bit);
bool SendStartCondition();
bool SendStopCondition();
bool Transmit(bool sendStartCondition, bool sendStopCondition, uint8_t ByteToSend);
uint8_t Receive(bool sendAcknowledgeBit, bool sendStopCondition);
uint32_t WriteBytes(uint8_t *data, uint32_t length);
uint32_t ReadBytes(uint8_t *data, uint32_t length);
bool SoftwareI2C_Initialize(SoftwareI2CConfig *i2c);
bool SoftwareI2C_WriteRead(SoftwareI2CConfig *i2c, uint8_t *writeBuffer, uint32_t writeLength, uint8_t *readBuffer, uint32_t readLength, uint32_t *numWritten, uint32_t *numRead);
uint32_t WriteByte(uint8_t data, bool isFirstByte, bool isMiddleByte, bool isLastByte);
uint32_t ReadByte(uint8_t *data, bool isFirstByte, bool isMiddleByte, bool isLastByte);

bool softwareI2cStart;

static const TinyCLR_Gpio_Provider* provider = nullptr;

TinyCLR_Result Interop_GHIElectronics_TinyCLR_Devices_GHIElectronics_TinyCLR_Devices_I2c_I2cNativeSoftwareDeviceProvider::NativeWriteRead___STATIC___BOOLEAN__I4__I4__U1__BOOLEAN__SZARRAY_U1__I4__I4__SZARRAY_U1__I4__I4__BYREF_U4__BYREF_U4(const TinyCLR_Interop_MethodData md) {
    auto agr0 = TinyCLR_Interop_GetArguments(md, 0);
    auto agr1 = TinyCLR_Interop_GetArguments(md, 1);
    auto agr2 = TinyCLR_Interop_GetArguments(md, 2);
    auto agr3 = TinyCLR_Interop_GetArguments(md, 3);
    auto agr4 = TinyCLR_Interop_GetArguments(md, 4);
    auto agr5 = TinyCLR_Interop_GetArguments(md, 5);
    auto agr6 = TinyCLR_Interop_GetArguments(md, 6);
    auto agr7 = TinyCLR_Interop_GetArguments(md, 7);
    auto agr8 = TinyCLR_Interop_GetArguments(md, 8);
    auto agr9 = TinyCLR_Interop_GetArguments(md, 9);
    auto agr10 = TinyCLR_Interop_GetArguments(md, 10);
    auto agr11 = TinyCLR_Interop_GetArguments(md, 11);

    int32_t scl = agr0.Data.Numeric->I4;
    int32_t sda = agr1.Data.Numeric->I4;

    uint8_t address = agr2.Data.Numeric->U1;

    bool useSoftwarePullups = agr3.Data.Numeric->I1 != 0;

    uint8_t* writeBuffer = (uint8_t*)agr4.Data.SzArray.Data;

    int32_t writeOffset = agr5.Data.Numeric->I4;
    int32_t writeLength = agr6.Data.Numeric->I4;

    uint8_t* readBuffer = (uint8_t*)agr7.Data.SzArray.Data;
    int32_t readOffset = agr8.Data.Numeric->I4;
    int32_t readLength = agr9.Data.Numeric->I4;

    int32_t& writtenRef = agr10.Data.Numeric->I4;
    int32_t& readRef = agr11.Data.Numeric->I4;

    uint32_t written = 0;
    uint32_t read = 0;

    uint8_t* dataWrite = NULL;
    uint8_t* dataRead = NULL;

    SoftwareI2CConfig i2c;

    TinyCLR_Interop_ClrValue ret;

    auto prov = (const TinyCLR_Interop_Provider*)md.ApiProvider.FindDefault(&md.ApiProvider, TinyCLR_Api_Type::InteropProvider);

    if (prov == nullptr || (provider == nullptr && (provider = (const TinyCLR_Gpio_Provider*)apiProvider->FindDefault(apiProvider, TinyCLR_Api_Type::GpioProvider)) == nullptr))
        return TinyCLR_Result::ArgumentNull;

    if (writeBuffer != NULL)
        dataWrite = writeBuffer;

    if (readBuffer != NULL)
        dataRead = readBuffer;

    i2c.scl = scl;
    i2c.sda = sda;
    i2c.clockSpeed = I2C_STANDARD_SPEED_US;
    i2c.address = address;
    i2c.useSoftwarePullups = useSoftwarePullups;

    prov->GetReturn(prov, md.Stack, ret);

    ret.Data.Numeric->Boolean = SoftwareI2C_WriteRead(&i2c, dataWrite + writeOffset, writeLength, dataRead + readOffset, readLength, &written, &read);

    writtenRef = written;
    readRef = read;

    return TinyCLR_Result::Success;
}

bool SoftwareI2C_WriteRead(SoftwareI2CConfig *i2c, uint8_t *writeBuffer, uint32_t writeLength, uint8_t *readBuffer, uint32_t readLength, uint32_t *numWritten, uint32_t *numRead) {

    uint32_t i = 0;
    uint32_t write = 0;
    uint32_t read = 0;

    *numWritten = *numRead = 0;

    softwareI2cConfig = i2c;

    softwareI2cStart = false;

    MakePinInput(softwareI2cConfig, softwareI2cConfig->sda);
    MakePinInput(softwareI2cConfig, softwareI2cConfig->scl);

    // For write
    if (writeLength > 0) {
        if (!Transmit(true, false, (uint8_t)((softwareI2cConfig->address) << 1))) // Write address for write
        {
            for (i = 0; i < writeLength - 1; i++) {
                if (!Transmit(false, false, writeBuffer[i])) // Write data
                {
                    (write)++;
                }

            }
        }
        else {
            goto end_softwarei2c_readwrite;
        }
        //if (!Transmit(false, true, writeBuffer[i])) (write)++; // Write last byte
        //*numWritten = write;
        if (!Transmit(false, (readLength == 0), writeBuffer[i])) (write)++; // Write last byte
        *numWritten = write;

    }
    // For Read
    if (readLength > 0) {
        if (!Transmit(true, false, (uint8_t)(((softwareI2cConfig->address) << 1) | 1))) // Write address for Read
        {
            for (i = 0; i < readLength - 1; i++) {
                readBuffer[i] = Receive(true, false);
                (read)++;
            }
        }
        else {
            goto end_softwarei2c_readwrite;
        }
        readBuffer[i] = Receive(false, true); // Read  last byte
        (read)++;
        *numRead = read;
    }
end_softwarei2c_readwrite:
    return (write + read) == (writeLength + readLength);
}


void MakePinOutput(uint32_t pin) {
    provider->SetDriveMode(provider, (int32_t)pin, TinyCLR_Gpio_PinDriveMode::Output);
    provider->Write(provider, (int32_t)pin, TinyCLR_Gpio_PinValue::Low);
}

void MakePinInput(SoftwareI2CConfig* i2c, uint32_t pin) {
    provider->SetDriveMode(provider, (int32_t)pin, i2c->useSoftwarePullups ? TinyCLR_Gpio_PinDriveMode::InputPullUp : TinyCLR_Gpio_PinDriveMode::Input);
}

bool ReadPinState(SoftwareI2CConfig* i2c, uint32_t pin) {
    MakePinInput(i2c, pin);
    TinyCLR_Gpio_PinValue value;
    provider->Read(provider, (int32_t)pin, value);
    return value == TinyCLR_Gpio_PinValue::High;
}

bool ReadBit() {
    // "ReadSDA" makes SDA an input - processor lets go of pin and internal
    //  pull-up resistor makes it high.  Now slave can drive the pin.
    ReadPinState(softwareI2cConfig, softwareI2cConfig->sda);

    I2CDELAY(softwareI2cConfig->clockSpeed);

    // Clock stretching - Makes SCL an input and pull-up resistor makes
    //  the pin high.  Slave device can pull SCL low to extend clock cycle.
    long endStretch = 0;//Utility.GetMachineTime().Ticks + timeOutTicks;
    while (ReadPinState(softwareI2cConfig, softwareI2cConfig->scl) == 0 && endStretch < TIMEOUT_COUNTER) {
        // How long have we been stuck in the while loop?
        //if (Utility.GetMachineTime().Ticks >= endStretch)
        //    throw new TimeOutException();    // Too long, so bail out by throwing an exception.
        TinyCLR_Interop_Delay(1); // 1 microsecond;
        endStretch++;
    }
    // At this point, SCL is high and SDA is valid - so read the bit.
    uint8_t bit = ReadPinState(softwareI2cConfig, softwareI2cConfig->sda);

    I2CDELAY(softwareI2cConfig->clockSpeed);

    ClearSCL();     // Pull the serial clock line low ...

    return bit;     //  and return.
}

bool WriteBit(bool bit) {
    if (bit) {
        ReadPinState(softwareI2cConfig, softwareI2cConfig->sda);      // Make SDA an input ... so pin is pulled up.
    }
    else {
        ClearSDA();     // Make SDA an output ... so pin is pulled low.
    }
    I2CDELAY(softwareI2cConfig->clockSpeed);
    // Clock stretching - Makes SCL an input and pull-up resistor makes
    //  the pin high.  Slave device can pull SCL low to extend clock cycle.
    long endStretch = 0;//Utility.GetMachineTime().Ticks + TIMEOUT_COUNTER;
    while (!ReadPinState(softwareI2cConfig, softwareI2cConfig->scl) && endStretch < TIMEOUT_COUNTER) {
        // How long have we been stuck in the while loop?
        //if (Utility.GetMachineTime().Ticks >= endStretch)
        //   throw new TimeOutException();    // Too long, so bail out by throwing an exception.
        TinyCLR_Interop_Delay(1); // 1 microsecond;
        endStretch++;
    }
    // SCL is high and SDA is valid ...
    //  Check that nobody else is driving SDA
    if (bit && !ReadPinState(softwareI2cConfig, softwareI2cConfig->sda)) {
        return false;// Lost arbitration
    }

    I2CDELAY(softwareI2cConfig->clockSpeed);
    ClearSCL();

    return true;    // Success!
}

bool SendStartCondition() {
    if (softwareI2cStart) {
        // set SDA to 1
        ReadPinState(softwareI2cConfig, softwareI2cConfig->sda);
        I2CDELAY(softwareI2cConfig->clockSpeed);
        //
        // Clock stretching - Makes SCL an input and pull-up resistor makes
        //  the pin high.  Slave device can pull SCL low to extend clock cycle.
        long endStretch = 0;//Utility.GetMachineTime().Ticks + TIMEOUT_COUNTER;
        while (!ReadPinState(softwareI2cConfig, softwareI2cConfig->scl) && endStretch < TIMEOUT_COUNTER) {
            // How long have we been stuck in the while loop?
            //if (Utility.GetMachineTime().Ticks >= endStretch)
            //throw new TimeOutException();    // Too long, so bail out by throwing an exception.
            TinyCLR_Interop_Delay(1); // 1 microsecond;
            endStretch++;
        }
    }

    if (!ReadPinState(softwareI2cConfig, softwareI2cConfig->sda)) {
        return false;
    }

    // SCL is high, set SDA from 1 to 0
    ClearSDA();
    I2CDELAY(softwareI2cConfig->clockSpeed);
    ClearSCL();

    softwareI2cStart = true;

    return true;
}

bool SendStopCondition() {
    // set SDA to 0
    ClearSDA();
    I2CDELAY(softwareI2cConfig->clockSpeed);
    //
    // Clock stretching - Makes SCL an input and pull-up resistor makes
    //  the pin high.  Slave device can pull SCL low to extend clock cycle.
    long endStretch = 0;//Utility.GetMachineTime().Ticks + TIMEOUT_COUNTER;
    while (!ReadPinState(softwareI2cConfig, softwareI2cConfig->scl) && endStretch < TIMEOUT_COUNTER) {
        // How long have we been stuck in the while loop?
        //if (Utility.GetMachineTime().Ticks >= endStretch)
        //    throw new TimeOutException();    // Too long, so bail out by throwing an exception.
        TinyCLR_Interop_Delay(1); // 1 microsecond;
        endStretch++;

    }
    //
    // SCL is high, set SDA from 0 to 1
    if (!ReadPinState(softwareI2cConfig, softwareI2cConfig->sda)) {
        return false;
    }

    I2CDELAY(softwareI2cConfig->clockSpeed);
    softwareI2cStart = false;

    return true;
}

bool Transmit(bool sendStartCondition, bool sendStopCondition, uint8_t ByteToSend) {
    uint32_t bit;
    bool nack;
    if (sendStartCondition) {
        SendStartCondition();
    }

    for (bit = 0; bit < 8; bit++) {
        WriteBit((ByteToSend & 0x80) != 0);

        ByteToSend <<= 1;
    }

    nack = ReadBit();
    //
    if (sendStopCondition) {
        SendStopCondition();
    }
    // Return value is "true" for NAK
    //  "false" for ACK.
    return nack;
}

uint8_t Receive(bool sendAcknowledgeBit, bool sendStopCondition) {
    uint8_t d = 0;
    //bool b;
    uint32_t bit = 0;
    for (bit = 0; bit < 8; bit++) {
        d <<= 1;

        //b = ReadBit();
        if (ReadBit())
            d |= 1;
    }
    //
    WriteBit(!sendAcknowledgeBit);
    //
    if (sendStopCondition) {
        SendStopCondition();
    }
    //
    return d;
}
uint32_t WriteByte(uint8_t data, bool isFirstByte, bool isMiddleByte, bool isLastByte) {
    uint32_t numWrite = 0;
    uint32_t i = 0;
    if (isFirstByte) {
        if (!Transmit(true, false, (uint8_t)((softwareI2cConfig->address) << 1))) // start cond + address
        {
            if (!Transmit(false, false, data)) // raw data
            {
                numWrite++;
            }
        }
    }
    else if (isMiddleByte) {
        if (!Transmit(false, false, data)) // raw data
        {
            numWrite++;
        }
    }
    else if (isLastByte) {
        if (!Transmit(false, true, data)) // raw data + stop condition
        {
            numWrite++;
        }
    }
    return numWrite;


}
uint32_t WriteBytes(uint8_t *data, uint32_t length) {
    if (length == 0) return 0;
    uint32_t numWrite = 0;
    uint32_t i = 0;
    if (!Transmit(true, false, (uint8_t)((softwareI2cConfig->address) << 1))) {
        for (i = 0; i < length - 1; i++) {
            if (!Transmit(false, false, data[i])) {
                numWrite++;
            }

        }
    }
    if (!Transmit(false, true, data[i])) numWrite++;
    return numWrite;
}
uint32_t ReadByte(uint8_t *data, bool isFirstByte, bool isMiddleByte, bool isLastByte) {
    uint32_t numRead = 0;

    if (isFirstByte) {
        if (!Transmit(true, false, (uint8_t)(((softwareI2cConfig->address) << 1) | 1))) {
            *data = Receive(true, false);
            numRead++;
        }
    }
    else if (isMiddleByte) {
        *data = Receive(true, false);
        numRead++;
    }
    else if (isLastByte) {
        *data = Receive(false, true);
        numRead++;
    }

    return numRead;
}
uint32_t ReadBytes(uint8_t *data, uint32_t length) {
    if (length == 0) return 0;
    uint32_t numRead = 0;
    uint32_t i = 0;
    if (!Transmit(true, false, (uint8_t)(((softwareI2cConfig->address) << 1) | 1))) {
        for (i = 0; i < length - 1; i++) {
            data[i] = Receive(true, false);
            numRead++;
        }
    }

    data[i] = Receive(false, true);
    numRead++;

    return numRead;
}

