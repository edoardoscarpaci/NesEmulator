#include <cstdint>
#include <functional>

enum AddressingMode{
    Immediate,
    ZeroPageX,
    ZeroPageY,
    Accumulator,
    Implicit,
    ZeroPage,
    Absolute,
    Relative,
    Indirect,
    IndexedIndirect,
    IndirectIndexed,
    AbsoluteIndexedX,
    AbsoluteIndexedY
};


struct OpCode{
    std::string name;
    std::function<uint8_t()> addressing_mode;
    std::function<uint8_t()> operation;
    uint8_t cycles;
}typedef OpCode;


OpCode opCodeTable[256] = 
{
{}
};


