#include <cstdint>
#include <memory>
#include <Cartidge.hpp>
class Address{
private:
    uint16_t address;

public:
    Address(uint16_t Address) :address(Address){

    }

    bool inRange(uint16_t min, uint16_t max){ 
        return (address >= min && address < max);
    }

    operator uint16_t() const{ 
        return address;
    }

};

class Bus{

private:
    uint8_t RAM[0x800]; 
    uint8_t NesPPURegister[0x008];
    uint8_t APUIORegister[0x018];
    uint8_t testFunctionality[0x008];   
    std::shared_ptr<Cartidge> cartidge;

public:

    Bus(){
        memset(RAM,0,0x800);
        memset(NesPPURegister,0,0x8);
        memset(APUIORegister,0,0x018);
        memset(testFunctionality,0,0x8);
    }

    void resetRAM(){
        memset(RAM,0,0x800);
    }

    void connectCartidge(Cartidge* cartidge);

    uint8_t read(Address address){
        if(address.inRange(0,0x2000)){
            return RAM[address % 0x800];
        }

        else if(address.inRange(0x2000,0x4000)){
            uint16_t index = (address - 0x2000) % 0x8;
            return NesPPURegister[index];
        }

        else if(address.inRange(0x4000,0x4018)){
            return NesPPURegister[address - 0x4000];
        }
        
        else if(address.inRange(0x4018,0x4020)){
            return NesPPURegister[address - 0x4018];
        }
        
        return cartidge->readPGRROM(address-0x4020);
    }

    void write(Address address,uint8_t value){
        if(address.inRange(0,0x2000)){
            RAM[address % 0x800] = value;
        }

        else if(address.inRange(0x2000,0x4000)){
            uint16_t index = (address - 0x2000) % 0x8;
            NesPPURegister[index] = value;
        }

        else if(address.inRange(0x4000,0x4018)){
            NesPPURegister[address - 0x4000] = value;
        }
        
        else if(address.inRange(0x4018,0x4020)){
            NesPPURegister[address - 0x4018] = value;
        }
        else{
            cartidge->writePGRROM(address-0x4020,value);
        }
    }
};