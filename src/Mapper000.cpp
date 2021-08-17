#include "include/Mapper000.hpp"


uint32_t Mapper000::decodePgrRomAddress(uint16_t address){
    if(n_pgr_rom > 1){
        return address;
    }

    if(address > 0xBFFF){
        return address - 0x2000;
    }
    return address;
}
uint32_t Mapper000::decodeChrRomAddress(uint16_t address){
    return address;
}
uint32_t Mapper000::decodePgrRamAddress(uint16_t address){
    return address;
}