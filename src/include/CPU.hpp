#include <cstdint>
#include <Bus.hpp>
#include <CPUOpcode.hpp>
#include <functional>
class CPU{
private:
    Bus* bus = nullptr;

    uint8_t accumulator = 0x00;
    uint8_t X = 0x00,Y = 0x00;
    uint8_t sp = 0x00; 
    uint16_t pc = 0x0000;
    uint8_t status = 0x00;

    uint16_t abs_addr = 0x00;
    uint16_t rel_addr = 0x00;
    uint32_t cycles = 0x00;

    uint8_t fetched = 0;
    uint8_t opcode = 0x00;

    const uint8_t stack_offset = 0x100; 
    enum StatusFlag{
        Carry = (1 << 0),
        Zero = (1<<1),
        Interrupt_disable = (1<<2),
        Decimal = (1<<3),
        Break = (1<<4),
        Unused = (1<<5),
        Overflow = (1<<6),
        Negative = (1<<7),
    };

    struct OpCode{
        std::string name;
        uint8_t(CPU::*addressing_mode)(void);
        uint8_t(CPU::*operate)(void);
        uint8_t cycles;
    }typedef OpCode;


    OpCode opCodeLookup[0xff] = 
    {
        {"BRK",&CPU::Imm,&CPU::BRK,7},{"ORA",&CPU::INDX,&CPU::ORA,6},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"ORA",&CPU::ZPG,&CPU::ORA,3},
        {"ASL",&CPU::ZPG,&CPU::ASL,5},{"???",nullptr,nullptr,0},{"PHP",&CPU::IMP,&CPU::PHP,3},{"ORA",&CPU::Imm,&CPU::ORA,2},{"BPL",&CPU::RLV,&CPU::BPL,2},{"ORA",&CPU::INDY,&CPU::ORA,5},
        {"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"ORA",&CPU::ZPGX,&CPU::ORA,4},{"ASL",&CPU::ZPGX,&CPU::ASL,6},{"CLC",&CPU::IMP,&CPU::CLC,2},{"???",nullptr,nullptr,0},
        {"ORA",&CPU::ABSY,&CPU::ORA,4},{"JSR",&CPU::ABS,&CPU::JSR,6},{"AND",&CPU::INDX,&CPU::AND,6},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"BIT",&CPU::ZPG,&CPU::BIT,3},{"AND",&CPU::ZPG,&CPU::AND,3},
        {"ROL",&CPU::ZPG,&CPU::ROL,5},{"???",nullptr,nullptr,0},{"PLP",&CPU::IMP,&CPU::PLP,4},{"AND",&CPU::Imm,&CPU::AND,2},{"BMI",&CPU::RLV,&CPU::BMI,2},{"???",nullptr,nullptr,0},{"AND",&CPU::INDY,&CPU::AND,5},
        {"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"AND",&CPU::ZPGX,&CPU::AND,4},{"ROL",&CPU::ZPGX,&CPU::ROL,6},{"???",nullptr,nullptr,0},{"SEC",&CPU::IMP,&CPU::SEC,2},{"AND",&CPU::ABSY,&CPU::AND,4},
        {"RTI",&CPU::IMP,&CPU::RTI,6},{"EOR",&CPU::INDX,&CPU::EOR,6},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"EOR",&CPU::ZPG,&CPU::EOR,3},{"LSR",&CPU::ZPG,&CPU::LSR,5},{"???",nullptr,nullptr,0},
        {"PHA",&CPU::IMP,&CPU::PHA,3},{"EOR",&CPU::Imm,&CPU::EOR,2},{"BVC",&CPU::RLV,&CPU::BVC,2},{"EOR",&CPU::INDY,&CPU::EOR,5},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"EOR",&CPU::ZPGX,&CPU::EOR,4},
        {"LSR",&CPU::ZPGX,&CPU::LSR,6},{"???",nullptr,nullptr,0},{"CLI",&CPU::IMP,&CPU::CLI,2},{"EOR",&CPU::ABSY,&CPU::EOR,4},{"RTS",&CPU::IMP,&CPU::RTS,6},
        {"ADC",&CPU::INDX,&CPU::ADC,6},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"ADC",&CPU::ZPG,&CPU::ADC,3},{"ROR",&CPU::ZPG,&CPU::ROR,5},{"???",nullptr,nullptr,0},{"PLA",&CPU::IMP,&CPU::PLA,4},
        {"ADC",&CPU::Imm,&CPU::ADC,2},{"BVS",&CPU::RLV,&CPU::BVS,2},{"ADC",&CPU::INDY,&CPU::ADC,5},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"ADC",&CPU::ZPGX,&CPU::ADC,4},{"ROR",&CPU::ZPGX,&CPU::ROR,6},
        {"???",nullptr,nullptr,0},{"SEI",&CPU::IMP,&CPU::SEI,2},{"ADC",&CPU::ABSY,&CPU::ADC,4},{"???",nullptr,nullptr,0},{"STA",&CPU::INDX,&CPU::STA,6},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"STY",&CPU::ZPG,&CPU::STY,3},
        {"STA",&CPU::ZPG,&CPU::STA,3},{"STX",&CPU::ZPG,&CPU::STX,3},{"???",nullptr,nullptr,0},{"DEY",&CPU::IMP,&CPU::DEY,2},{"???",nullptr,nullptr,0},{"BCC",&CPU::RLV,&CPU::BCC,2},{"STA",&CPU::INDY,&CPU::STA,6},{"???",nullptr,nullptr,0},
        {"???",nullptr,nullptr,0},{"STY",&CPU::ZPGX,&CPU::STY,4},{"STA",&CPU::ZPGX,&CPU::STA,4},{"STX",&CPU::ZPGY,&CPU::STX,4},{"???",nullptr,nullptr,0},{"TYA",&CPU::IMP,&CPU::TYA,2},{"STA",&CPU::ABSY,&CPU::STA,5},{"LDY",&CPU::Imm,&CPU::LDY,2},
        {"LDA",&CPU::INDX,&CPU::LDA,6},{"LDX",&CPU::Imm,&CPU::LDX,2},{"???",nullptr,nullptr,0},{"LDY",&CPU::ZPG,&CPU::LDY,3},{"LDA",&CPU::ZPG,&CPU::LDA,3},{"LDX",&CPU::ZPG,&CPU::LDX,3},{"???",nullptr,nullptr,0},{"TAY",&CPU::IMP,&CPU::TAY,2},
        {"LDA",&CPU::Imm,&CPU::LDA,2},{"TAX",&CPU::IMP,&CPU::TAX,2},{"???",nullptr,nullptr,0},{"LDY",&CPU::ABS,&CPU::LDY,4},{"LDA",&CPU::ABS,&CPU::LDA,4},{"LDX",&CPU::ABS,&CPU::LDX,4},{"???",nullptr,nullptr,0},
        {"BCS",&CPU::RLV,&CPU::BCS,2},{"LDA",&CPU::INDY,&CPU::LDA,5},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"LDY",&CPU::ZPGX,&CPU::LDY,4},{"LDA",&CPU::ZPGX,&CPU::LDA,4},{"LDX",&CPU::ZPGY,&CPU::LDX,4},{"???",nullptr,nullptr,0},{"CLV",&CPU::IMP,&CPU::CLV,2},
        {"LDA",&CPU::ABSY,&CPU::LDA,4},{"TSX",&CPU::IMP,&CPU::TSX,2},{"???",nullptr,nullptr,0},{"LDY",&CPU::ABSX,&CPU::LDY,4},{"LDX",&CPU::ABSY,&CPU::LDX,4},{"???",nullptr,nullptr,0},
        {"CPY",&CPU::Imm,&CPU::CPY,2},{"CMP",&CPU::INDX,&CPU::CMP,6},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"CPY",&CPU::ZPG,&CPU::CPY,3},{"CMP",&CPU::ZPG,&CPU::CMP,3},{"DEC",&CPU::ZPG,&CPU::DEC,5},{"???",nullptr,nullptr,0},{"INY",&CPU::IMP,&CPU::INY,2},
        {"CMP",&CPU::Imm,&CPU::CMP,2},{"DEX",&CPU::IMP,&CPU::DEX,2},{"???",nullptr,nullptr,0},{"CPY",&CPU::ABS,&CPU::CPY,4},{"CMP",&CPU::ABS,&CPU::CMP,4},{"DEC",&CPU::ABS,&CPU::DEC,6},{"???",nullptr,nullptr,0},
        {"BNE",&CPU::RLV,&CPU::BNE,2},{"CMP",&CPU::INDY,&CPU::CMP,5},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"CMP",&CPU::ZPGX,&CPU::CMP,4},{"DEC",&CPU::ZPGX,&CPU::DEC,6},{"???",nullptr,nullptr,0},{"CLD",&CPU::IMP,&CPU::CLD,2},{"CMP",&CPU::ABSY,&CPU::CMP,4},
        {"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"CMP",&CPU::ABSX,&CPU::CMP,4},{"DEC",&CPU::ABSX,&CPU::DEC,7},{"???",nullptr,nullptr,0},
        {"CPX",&CPU::Imm,&CPU::CPX,2},{"SBC",&CPU::INDX,&CPU::SBC,6},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"CPX",&CPU::ZPG,&CPU::CPX,3},{"SBC",&CPU::ZPG,&CPU::SBC,3},{"INC",&CPU::ZPG,&CPU::INC,5},{"???",nullptr,nullptr,0},{"INX",&CPU::IMP,&CPU::INX,2},{"SBC",&CPU::Imm,&CPU::SBC,2},
        {"NOP",&CPU::IMP,&CPU::NOP,2},{"???",nullptr,nullptr,0},{"CPX",&CPU::ABS,&CPU::CPX,4},{"SBC",&CPU::ABS,&CPU::SBC,4},{"INC",&CPU::ABS,&CPU::INC,6},{"???",nullptr,nullptr,0},
        {"BEQ",&CPU::RLV,&CPU::BEQ,2},{"SBC",&CPU::INDY,&CPU::SBC,5},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"SBC",&CPU::ZPGX,&CPU::SBC,4},{"INC",&CPU::ZPGX,&CPU::INC,6},{"???",nullptr,nullptr,0},{"SED",&CPU::IMP,&CPU::SED,2},{"SBC",&CPU::ABSY,&CPU::SBC,4},
        {"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"???",nullptr,nullptr,0},{"SBC",&CPU::ABSX,&CPU::SBC,4},{"INC",&CPU::ABSX,&CPU::INC,7},{"???",nullptr,nullptr,0}
    };



    CPU(Bus* bus);

    bool getStatusFlag(enum StatusFlag f);
    void setStatusFlag(enum StatusFlag f, bool value);
    
    bool checkSignBit(int8_t a, int8_t b );

    
    
    void irq();
    void nmi();

    void push(uint8_t value);
    void push16(uint16_t value);

    uint8_t pop();
    uint16_t pop16();    

    uint8_t fetch();
    void fetch_back();

    uint8_t read(Address address);
    void write(Address address, uint8_t value);

    uint8_t Imm();  uint8_t INDY();
    uint8_t ZPG();  uint8_t ABSX();
    uint8_t ZPGX(); uint8_t ABSY();
    uint8_t ZPGY(); uint8_t RLV();
    uint8_t ACC();  uint8_t IND();
    uint8_t IMP();  uint8_t INDX();
    uint8_t ABS();



    uint8_t ADC();  uint8_t CLC();  uint8_t JSR();  uint8_t SBC();
    uint8_t AND();  uint8_t CLD();  uint8_t LDA();  uint8_t SEC();
    uint8_t ASL();  uint8_t CLI();  uint8_t LDX();  uint8_t SED();
    uint8_t BCC();  uint8_t CLV();  uint8_t LDY();  uint8_t SEI();
    uint8_t BCS();  uint8_t CMP();  uint8_t LSR();  uint8_t STA();
    uint8_t BEQ();  uint8_t CPX();  uint8_t NOP();  uint8_t STX();
    uint8_t BIT();  uint8_t CPY();  uint8_t ORA();  uint8_t STY();
    uint8_t BIT();  uint8_t DEC();  uint8_t PHA();  uint8_t TAX();
    uint8_t BIT();  uint8_t DEX();  uint8_t PHP();  uint8_t TAY();
    uint8_t BMI();  uint8_t DEY();  uint8_t PLA();  uint8_t TSX();
    uint8_t BNE();  uint8_t EOR();  uint8_t PLP();  uint8_t TXA();
    uint8_t BPL();  uint8_t INC();  uint8_t ROL();  uint8_t TXS();
    uint8_t BRK();  uint8_t INX();  uint8_t ROR();  uint8_t TYA();
    uint8_t BVC();  uint8_t INY();  uint8_t RTI();
    uint8_t BVS();  uint8_t JMP();  uint8_t RTS();

public:
    void clock();
    void reset();
    const int clk = 2147727;
};