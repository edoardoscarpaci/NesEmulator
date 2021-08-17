#include <cstdint>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include "Mapper.hpp"

struct NesHeader{ 
    uint8_t constant[4];
    uint8_t nPGRROM;
    uint8_t nCHRROM;
    union flag6{

        struct flagStruct{
            uint8_t mirroring : 1;
            uint8_t battery : 1;
            uint8_t trainer :1;
            uint8_t ignore_mirroring : 1;
            uint8_t lsn_mapper :4;
        }flagStruct;

        uint8_t flag;
    }flag6;
    
    union flag7{
        struct flagStruct{
            uint8_t VS_Unisystem: 1;
            uint8_t play_choiche:1;
            uint8_t nes2_format :2;
            uint8_t msn_mapper :4;
        }flagStruct;
        uint8_t flag;
    }flag7;
    uint8_t pgrRamSize;
    
    union flag9{
        struct flagStruct{
            uint8_t tv_system:1;
            uint8_t reserved :7;
        }flagStruct;
        uint8_t flag;
    }flag9;

    union flag10{
        struct flagStruct{
            uint8_t tvSystem:2;
            uint8_t unused :2;
            uint8_t pgr_ram_present : 1;
            uint8_t bus_conflict :1;
            uint8_t unused2:1;
        }flagStruct;
        uint8_t flag;
    }flag10;
    uint8_t padding[5];

};


class Cartidge{
private:
    NesHeader header;
    std::string path;

    std::vector<uint8_t> pgr_rom;
    std::vector<uint8_t> chr_rom;
    uint8_t* trainer;
    uint8_t* inst_ROM;
    uint8_t* prom;

    uint8_t mapperID;
    Mapper* mapper;

private:
    void inizializeMapper();


public:
    Cartidge(const std::string& path):path(path){
        loadCartidge(path);
        inizializeMapper();
    }


    void loadCartidge(const std::string& path);

    const std::string getPath() const;
    uint8_t readPGRROM(uint16_t address);
    void writePGRROM(uint16_t address,uint8_t value);
    uint8_t readCHRROM(uint16_t address);
    void writeCHRROM(uint16_t address,uint8_t value);


};