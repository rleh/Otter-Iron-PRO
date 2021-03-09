#ifndef STUSB4500_HPP
#define STUSUB4500_HPP

#include <modm/architecture/interface/i2c_device.hpp>

struct stusb4500
{
protected: 
    enum class
    Register : uint8_t
    {
        RdoRegStatus = 0x91,

        // PortStatus1 = 0x0E,
        // CcStatus = 0x11,

        // TxHeaderLow = 0x51,
        // PdCommandCtrl = 0x1A,

        DpmPdoNumb = 0x70,

        DpmSnkPdo1 = 0x85,
        DpmSnkPdo2 = 0x89,
        DpmSnkPdo3 = 0x8D,
    };
public:
    struct RdoRegStatusData {
        uint32_t MaxCurrent             ; //Bits 9..0
        uint32_t OperatingCurrent       ; //Bits 19..10
        // uint8_t reserved_22_20                  :       3;
        // uint8_t UnchunkedMess_sup               :       1;
        // uint8_t UsbSuspend                      :       1;
        // uint8_t UsbComCap                       :       1;
        // uint8_t CapaMismatch                    :       1;
        // uint8_t GiveBack                        :       1;
        uint8_t ObjectPos               ; //Bits 30..28 (3-bit)
    };
};

template<class I2cMaster>
class Stusb4500 : public stusb4500,  public modm::I2cDevice< I2cMaster, 4 >
{
public:
    inline Stusb4500(uint8_t address=0x28):
    modm::I2cDevice<I2cMaster,4>(address) {};

    inline modm::ResumableResult<RdoRegStatusData>
    getRdoRegStatus( ) {
        RF_BEGIN();
        RF_CALL(readRegister<4>(Register::RdoRegStatus, buffer2.data()));
        RF_WAIT_WHILE(this->isTransactionRunning());
        rdoRegStatusData.MaxCurrent = buffer2.at(0) | ((buffer2.at(1) & 0b11) << 8);
        rdoRegStatusData.MaxCurrent *= 10;
        rdoRegStatusData.OperatingCurrent = ((buffer2.at(1) & 0b1111'1100) >> 2) | ((buffer2.at(2) & 0b1111) << 6);
        rdoRegStatusData.OperatingCurrent *= 10;
        rdoRegStatusData.ObjectPos = ( buffer2.at(3) & 0b0110'0000 ) >> 5;

        // // FIXME: Endianess?!?
        // buffer3 = static_cast<uint32_t>(buffer2.at(0) + (buffer2.at(1)<<8) + (buffer2.at(2)<<16) + (buffer2.at(3)<<24));
        // rdoRegStatusData.MaxCurrent = buffer3 & 0b11'1111'1111;
        // rdoRegStatusData.OperatingCurrent = (buffer3 >> 10) & 0b11'1111'1111;
        // rdoRegStatusData.ObjectPos = (buffer3 >> 28) & 0b111;
        RF_END_RETURN(rdoRegStatusData);
    }

	inline modm::ResumableResult<void>
	configurePdo(uint8_t pdoNumber, uint32_t voltage, uint32_t current)
	{
        RF_BEGIN();
        RF_CALL(readRegister<4>(DpmSnkPdos.at(pdoNumber), buffer2.data()));
        RF_WAIT_WHILE(this->isTransactionRunning());
        
        current /= 10;
        voltage /= 50;
        buffer2.at(0) = (uint8_t) current; // first 8 bits of 10 bit current
        buffer2.at(1) = ((current >> 8) & 0b11); // other 2 bits of 10 bit current
        buffer2.at(1) = buffer2.at(1) | (( voltage & 0b11'1111 ) << 2 );      
        buffer2.at(2) = ( buffer2.at(2) & 0b1111'0000 ) | ( ( voltage >> 6 ) & 0b1111);
        // TODO: reduce number of buffers in use to 1 :-)
        // buffer3 = static_cast<uint32_t>(buffer2.at(0) | (buffer2.at(1)<<8) | (buffer2.at(2)<<16) | (buffer2.at(3)<<24));
        // buffer3 = (buffer3 & (~0b11'1111'1111ul)) | ((current / 10) & 0b11'1111'1111ul);
        // buffer3 = (buffer3 & ((~0b11'1111'1111ul) << 10)) | (((voltage / 50) & 0b11'1111'1111ul) << 10);
        RF_CALL(updateRegister<4>(DpmSnkPdos.at(pdoNumber), buffer2.data()));
        RF_WAIT_WHILE(this->isTransactionRunning());
        RF_END();
	}

	inline modm::ResumableResult<void>
	setValidPdo(uint8_t pdoNumber)
	{
        RF_BEGIN();
        RF_CALL(updateRegister<1>(Register::DpmPdoNumb, &pdoNumber));
        RF_WAIT_WHILE((this->isTransactionRunning()));
        RF_END();
	}

private:
    template<uint32_t length> 
    inline modm::ResumableResult<bool>
	readRegister(Register reg, uint8_t* output)
	{
		RF_BEGIN();
		buffer[0] = uint8_t(reg);
		this->transaction.configureWriteRead(buffer.data(), 1, output, length);
		RF_END_RETURN_CALL( this->runTransaction() );
	}

    template<uint32_t length> 
	inline modm::ResumableResult<bool>
	updateRegister(Register reg, uint8_t* data)
	{
		RF_BEGIN();
		buffer.at(0) = uint8_t(reg);
        // static_assert(length <= 4, "buffer too small, rleh is sad");
        std::memcpy(&buffer.at(1), data, 4);
		this->transaction.configureWriteRead(buffer.data(), length + 1, nullptr, 0);
		RF_END_RETURN_CALL( this->runTransaction() );
	}
    
private: 
    static constexpr std::array<Register, 3> DpmSnkPdos {Register::DpmSnkPdo1, Register::DpmSnkPdo2, Register::DpmSnkPdo3}; 
    std::array<uint8_t, 5> buffer;
    std::array<uint8_t, 4> buffer2;
    uint32_t buffer3;
    RdoRegStatusData rdoRegStatusData;

};

#endif // STUSUB4500_HPP