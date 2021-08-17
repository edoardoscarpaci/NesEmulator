#include "Mapper.hpp"


class Mapper000 : public Mapper{

public:
    Mapper000(uint8_t n_pgr_rom,uint8_t n_chr_ram):Mapper(n_pgr_rom,n_chr_ram){
    }

    virtual uint32_t decodePgrRomAddress(uint16_t address);
    virtual uint32_t decodeChrRomAddress(uint16_t address);
    virtual uint32_t decodePgrRamAddress(uint16_t address);
};