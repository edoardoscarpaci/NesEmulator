#include "include/Cartidge.hpp"
#include "include/Mapper000.hpp"

void Cartidge::loadCartidge(const std::string& path){
    std::ifstream file(path,std::ios::binary);

    if(!file.is_open()){
        perror("Cant'open the cartidge");
        return;
    }

    this->path = path;


    file.read((char*)&header,sizeof(NesHeader));
    if(header.flag6.flagStruct.trainer){
        trainer = new uint8_t[0x200];
        file.read((char*)trainer,0x200);
    }

    pgr_rom.resize(0x4000 * header.nPGRROM);
    file.read((char*)pgr_rom.data(),0x4000 * header.nPGRROM);

    if(header.nCHRROM > 0){
        chr_rom.resize(0x2000 * header.nCHRROM);
        file.read((char*)chr_rom.data(),0x2000 * header.nCHRROM);
    }

    if(header.flag7.flagStruct.play_choiche){
        inst_ROM = new uint8_t[0x2000];
        file.read((char*)inst_ROM,0x2000);
        prom = new uint8_t[0x20];
        file.read((char*)prom,0x20);
    }    
    
    mapperID = (header.flag7.flagStruct.msn_mapper << 4) | header.flag6.flagStruct.lsn_mapper;
}

void Cartidge::inizializeMapper(){
    switch (mapperID)
    {
    case 0:
        mapper = new Mapper000(header.nPGRROM,header.nPGRROM);
        break;
    
    default:
        break;
    }
}