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

    struct RdoRegStatusData {
        uint32_t MaxCurrent                     :       10; //Bits 9..0
        uint32_t OperatingCurrent               :       10; //Bits 19..10
        // uint8_t reserved_22_20                  :       3;
        // uint8_t UnchunkedMess_sup               :       1;
        // uint8_t UsbSuspend                      :       1;
        // uint8_t UsbComCap                       :       1;
        // uint8_t CapaMismatch                    :       1;
        // uint8_t GiveBack                        :       1;
        uint8_t ObjectPos                       :       3; //Bits 30..28 (3-bit)
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
        readRegister<4>(Register::RdoRegStatus, reinterpret_cast<uint8_t*>(&buffer2));
        // FIXME: Endianess?!?
        rdoRegStatusData.MaxCurrent = buffer2 & 0b11'1111'1111;
        rdoRegStatusData.OperatingCurrent = (buffer2 >> 10) & 0b11'1111'1111;
        rdoRegStatusData.ObjectPos = (buffer2 >> 28) & 0b111;
        RF_END_RETURN(rdoRegStatusData);
    }

	inline modm::ResumableResult<void>
	configurePdo(uint8_t pdoNumber, uint16_t voltage, uint16_t current)
	{
        RF_BEGIN();
        readRegister<4>(DpmSnkPdos.at(pdoNumber), reinterpret_cast<uint8_t*>(&buffer2));
        // FIXME: Endianess?!?
        buffer2 = (buffer2 & ~0b11'1111'1111ul) | ((current / 10) & 0b11'1111'1111);
        buffer2 = (buffer2 & ((~0b11'1111'1111ul) << 10)) | (((voltage / 50) & 0b11'1111'1111) << 10);
        updateRegister<4>(DpmSnkPdos.at(pdoNumber), reinterpret_cast<uint8_t*>(&buffer2));
        RF_END();
	}

	inline modm::ResumableResult<bool>
	setValidPdo(uint8_t pdoNumber)
	{
        RF_BEGIN();
        updateRegister<1>(Register::DpmPdoNumb, &pdoNumber);
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
    uint32_t buffer2;
    RdoRegStatusData rdoRegStatusData;

};