#include "include/CPU.hpp"

CPU::CPU(Bus* bus):bus(bus){
    reset();
}

void CPU::reset(){

    pc = 0x34;
    accumulator= 0;
    X =0;
    Y=0;
    write(4017,0);
    write(4015,0);
    for(int i=0;i<0xf;i++)
        write(4000+i,0);
    write(4010,0);
    write(4011,0);
    write(4012,0);
    write(4013,0);
    bus->resetRAM();
}

bool CPU::getStatusFlag(StatusFlag f){
    return this->status & f;
}

void CPU::setStatusFlag(StatusFlag f ,bool value){
    this->status &= ~f;
    if(value){
        this->status |= f;
    }
}

uint8_t CPU::read(Address address){
    return bus->read(address);
}
void CPU::write(Address address,uint8_t value){
    bus->write(address,value);
}

uint8_t CPU::pop(){
    uint8_t value = read(sp+stack_offset);
    sp++;
    return value;
}

uint16_t CPU::pop16(){
    uint16_t msb = read(sp+stack_offset);
    sp++;
    uint8_t lsb = read(sp+stack_offset);
    sp++;
    return (msb << 8) | lsb;
}


void CPU::push(uint8_t value){
    write(sp+stack_offset,value);
    sp--;
}

void CPU::push16(uint16_t value){
    uint8_t msb = (value & 0xff00) >> 8;
    uint8_t lsb = value & 0x00ff;

    write(sp+stack_offset,msb);
    sp--;
    write(sp+stack_offset,lsb);
    sp--;
}

void CPU::irq(){
    if(getStatusFlag(Interrupt_disable)){
        return;
    }

    push16(pc);
    setStatusFlag(Break,false);
    push(status);
    setStatusFlag(Interrupt_disable,true);
    uint8_t lsb = read(0xFFFE);
    uint16_t msb = read(0xFFFF);
    pc = (msb <<8 | lsb); 
}

void CPU::nmi(){
    push16(pc);
    setStatusFlag(Break,false);
    push(status);
    setStatusFlag(Interrupt_disable,true);
    uint8_t lsb = read(0xFFFA);
    uint16_t msb = read(0xFFFB);
    pc = (msb <<8 | lsb); 
}
uint8_t CPU::Imm(){
    abs_addr = pc++;
    return 0;
}

uint8_t CPU::ZPG(){
    abs_addr = read(pc++) & 0xFF;
    return 0;
}

uint8_t CPU::ZPGX(){
    abs_addr = (read(pc++) +X) & 0x0FF ;
    return 0;
}

uint8_t CPU::ZPGY(){
    abs_addr = (read(pc++) + Y) & 0x0FF;
    return 0;
}

uint8_t CPU::ABS(){
    uint16_t low =  read(pc++);
    uint16_t high = read(pc++);
    abs_addr = (high << 8) | low;
    return 0;
}

uint8_t CPU::ABSX(){
    uint16_t low =  read(pc++);
    uint16_t high = read(pc++);
    abs_addr = ((high << 8) | low)  + X;
    
    if(high << 8 != abs_addr & 0xff00){
        return 1;
    }

    return 0;
}
uint8_t CPU::ABSY(){
    uint16_t low =  read(pc++);
    uint16_t high = read(pc++);
    abs_addr = ((high << 8) | low)  + Y;
    
    if(high << 8 != abs_addr & 0xff00){
        return 1;
    }

    return 0;
}

uint8_t CPU::IND(){
    uint16_t ptr_low = read(pc++);
    uint16_t ptr_high = read(pc++);

    uint16_t ptr = ((ptr_high << 8) | ptr_low);


    if(ptr & 0x00ff == 0x00ff){
        abs_addr = ((read(ptr &0xff00) <<8) | read(ptr));
    }else{
        abs_addr =  ((read(ptr+1) << 8) | read(ptr));
    }
    return 0;
}

uint8_t CPU::INDX(){
    uint16_t ptr_low = read(pc++) & 0xff;
    uint16_t ptr_high = read(pc++) & 0xff;

    uint16_t ptr = ((ptr_high << 8) | ptr_low) + X;


    abs_addr =  (read(ptr+1) << 8) | read(ptr);
    return 0;
}

uint8_t CPU::INDY(){
    uint16_t ptr_low = read(pc++) & 0xff;
    uint16_t ptr_high = read(pc++) & 0xff;

    uint16_t ptr = (ptr_high << 8) | ptr_low;


    abs_addr =  (read(ptr+1) << 8) | read(ptr) + Y;

    if((abs_addr & 0xff00) != (read(ptr+1) << 8 ))
        return 1;
    
    return 0;
}

uint8_t CPU::RLV(){
    rel_addr = read(pc++);
    if(rel_addr & 0x80){
        rel_addr | 0xff00;
    }
    return 0;
}

uint8_t CPU::IMP(){
    pc++;
    fetched = accumulator;
    return 0;
}

uint8_t CPU::ACC(){
    return 0;
}

uint8_t CPU::fetch(){

    fetched =  read(abs_addr);
    return fetched;
}


bool CPU::checkSignBit(int8_t a,int8_t b){
    int8_t addition = a + b;
    bool aSign = a >=0;
    bool bSign = b >=0;

    if( a && b && addition < 0){
        return false;
    }

    if( !a && !b && addition >=0){
        return false;
    }

    return true;

}

void  CPU::fetch_back(){
    if(opCodeLookup[opcode].addressing_mode == &CPU::ACC){
        accumulator = fetched;
    }
    write(abs_addr,fetched);
}

void CPU::clock(){

    if(cycles ==0){
        opcode = read(pc++);
        
        OpCode code = opCodeLookup[opcode];
        cycles = code.cycles;

        uint8_t additional_cycles1 = (this->*code.addressing_mode)();
        uint8_t additional_cycles2 = (this->*code.operate)();

        cycles += (additional_cycles1 & additional_cycles2);


    }
    cycles--;
}

uint8_t CPU::AND(){
    fetch();

    accumulator &= fetched;

    setStatusFlag(StatusFlag::Zero,accumulator == 0x00);
    setStatusFlag(StatusFlag::Negative,accumulator & 0x80);
    return 1;
}

uint8_t CPU::BRK(){
    irq();
    return 0; 
}

uint8_t CPU::ASL(){
    fetch();
    setStatusFlag(Carry,fetched & 0x80);
    fetched =  fetched << 1;

    setStatusFlag(Zero,fetched == 0);
    setStatusFlag(Negative,fetched &0x80);

    fetch_back();

    return 0;
}




uint8_t CPU::BCC(){
    fetch();
    if(!getStatusFlag(Carry)){
        cycles ++;
        abs_addr = pc + rel_addr;

        if((pc &0xff00) != (abs_addr &0xff00))
            cycles++;
        pc= abs_addr;
    }

    return 1;
}
uint8_t CPU::BCS(){
     fetch();
    if(getStatusFlag(Carry)){
        cycles ++;
        abs_addr = pc + rel_addr;

        if((pc &0xff00) != (abs_addr &0xff00))
            cycles++;
        pc= abs_addr;
    }

    return 1;
}

uint8_t CPU::BEQ(){
     fetch();
    if(getStatusFlag(Zero)){
        cycles ++;
        abs_addr = pc + rel_addr;

        if((pc &0xff00) != (abs_addr &0xff00))
            cycles++;
        pc= abs_addr;
    }

    return 1;
}
uint8_t CPU::BMI(){
     fetch();
    if(getStatusFlag(Negative)){
        cycles ++;
        abs_addr = pc + rel_addr;

        if((pc &0xff00) != (abs_addr &0xff00))
            cycles++;
        pc= abs_addr;
    }

    return 1;
}

uint8_t CPU::BPL(){
     fetch();
    if(!getStatusFlag(Negative)){
        cycles ++;
        abs_addr = pc + rel_addr;

        if((pc &0xff00) != (abs_addr &0xff00))
            cycles++;
        pc= abs_addr;
    }

    return 1;
}


uint8_t CPU::BNE(){
     fetch();
    if(!getStatusFlag(Zero)){
        cycles ++;
        abs_addr = pc + rel_addr;

        if((pc &0xff00) != (abs_addr &0xff00))
            cycles++;
        pc= abs_addr;
    }

    return 1;
}

uint8_t CPU::BVC(){
     fetch();
    if(!getStatusFlag(Overflow)){
        cycles ++;
        abs_addr = pc + rel_addr;

        if((pc &0xff00) != (abs_addr &0xff00))
            cycles++;
        pc= abs_addr;
    }

    return 1;
}

uint8_t CPU::BVS(){
     fetch();
    if(getStatusFlag(Overflow)){
        cycles ++;
        abs_addr = pc + rel_addr;

        if((pc &0xff00) != (abs_addr &0xff00))
            cycles++;
        pc= abs_addr;
    }

    return 1;
}

uint8_t CPU::CLC(){
    setStatusFlag(Carry,false);
    return 0;
}
uint8_t CPU::CLD(){
    setStatusFlag(Decimal,false);
    return 0;
}

uint8_t CPU::CLI(){
    setStatusFlag(Interrupt_disable,false);
    return 0;
}

uint8_t CPU::CLV(){
    setStatusFlag(Overflow,false);
    return 0;
}

uint8_t CPU::SEC(){
    setStatusFlag(Carry,true);
    return 0;
}


uint8_t CPU::SED(){
    setStatusFlag(Decimal,true);
    return 0;
}

uint8_t CPU::SEI(){
    setStatusFlag(Interrupt_disable,true);
    return 0;
}

uint8_t CPU::BIT(){
    fetch();

    setStatusFlag(Zero,(accumulator & fetched) == 0 );
    setStatusFlag(Overflow,(accumulator & fetched) & 0x40 );
    setStatusFlag(Negative,(accumulator & fetched) & 0x80 );
    return 0;
}

uint8_t CPU::CMP(){
    fetch();
    char result = accumulator - fetched;
    setStatusFlag(Carry,result >= 0);
    setStatusFlag(Zero,result == 0);
    setStatusFlag(Negative,result < 0);

    return 1;
}

uint8_t CPU::CPX(){
    fetch();
    char result = X - fetched;
    setStatusFlag(Carry,result >= 0);
    setStatusFlag(Zero,result == 0);
    setStatusFlag(Negative,result < 0);

    return 1;
}


uint8_t CPU::CPY(){
    fetch();
    char result = Y - fetched;
    setStatusFlag(Carry,result >= 0);
    setStatusFlag(Zero,result == 0);
    setStatusFlag(Negative,result < 0);

    return 1;
}

uint8_t CPU::DEC(){
    fetch();
    fetched -=1;
    setStatusFlag(Zero,fetched == 0);
    setStatusFlag(Negative,fetched &0x80);
    fetch_back();

    return 0;
}

uint8_t CPU::DEX(){

    X -=1;
    setStatusFlag(Zero,X == 0);
    setStatusFlag(Negative,X &0x80);
    fetch_back();

    return 0;
}

uint8_t CPU::DEY(){
    Y -=1;
    setStatusFlag(Zero,Y == 0);
    setStatusFlag(Negative,Y &0x80);
    fetch_back();

    return 0;
}

uint8_t CPU::EOR(){
    fetch();
    accumulator ^=fetched;
    setStatusFlag(Zero,accumulator ==0);
    setStatusFlag(Negative,accumulator&0x80); 

    return 1;
}

uint8_t CPU::INC(){
    fetch();
    fetched +=1;
    setStatusFlag(Zero,fetched == 0);
    setStatusFlag(Negative,fetched &0x80);
    fetch_back();

    return 0;
}

uint8_t CPU::INX(){

    X +=1;
    setStatusFlag(Zero,X == 0);
    setStatusFlag(Negative,X &0x80);
    fetch_back();

    return 0;
}

uint8_t CPU::INY(){
    Y +=1;
    setStatusFlag(Zero,Y == 0);
    setStatusFlag(Negative,Y &0x80);
    fetch_back();

    return 0;
}

uint8_t CPU::JMP(){
    fetch();
    pc = fetched;
    return 0;
}




uint8_t CPU::JSR(){
    fetch();
    sp--;
    push16(pc);
    pc = fetched;
    return 0;
}

uint8_t CPU::LDA(){
    fetch();
    accumulator = fetched;
    
    setStatusFlag(Zero,accumulator==0);
    setStatusFlag(Negative,accumulator &0x80);
}
uint8_t CPU::LDX(){
    fetch();
    X = fetched;
    
    setStatusFlag(Zero,X==0);
    setStatusFlag(Negative,X &0x80);
}

uint8_t CPU::LDY(){
    fetch();
    Y = fetched;
    
    setStatusFlag(Zero,Y==0);
    setStatusFlag(Negative,Y &0x80);
}

uint8_t CPU::LSR(){
    fetch();
    setStatusFlag(Carry,fetched&0x1);
    fetched>>= 1;
    setStatusFlag(Zero,fetched==0);
    setStatusFlag(Negative,fetched &0x80);
    fetch_back();
    return 0;
}

uint8_t CPU::NOP(){
    return 0;
}

uint8_t CPU::ORA(){
    fetch();
    accumulator |= fetched;
    setStatusFlag(Zero,accumulator==0);
    setStatusFlag(Negative,accumulator &0x80);
    return 1;
}

uint8_t CPU::PHA(){
    push(accumulator);
    return 0;
}

uint8_t CPU::PHP(){
    push(status);
    return 0;
}
uint8_t CPU::PLA(){
    accumulator = pop();
    setStatusFlag(Zero,accumulator==0);
    setStatusFlag(Negative,accumulator&0x80);
    return 0;
}

uint8_t CPU::PLP(){
    status = pop();
    return 0;
}

uint8_t CPU::ROL(){
    fetch();
    setStatusFlag(Carry,fetched &0x80);
    fetched <<= 1;

    setStatusFlag(Zero,fetched==0);
    setStatusFlag(Negative,fetched &0x80);
    fetch_back();
    return 0;
}

uint8_t CPU::ROR(){
    fetch();
    setStatusFlag(Carry,fetched &0x1);
    fetched >>= 1;

    setStatusFlag(Zero,fetched==0);
    setStatusFlag(Negative,fetched &0x80);
    fetch_back();
    return 0;
}

uint8_t CPU::RTI(){
    status = pop();
    pc = pop16();
    return 0; 
}

uint8_t CPU::RTS(){
    pc = pop16();
    return 0; 
}
uint8_t CPU::STA(){
    fetch();

    write(fetched,accumulator);
    return 0;
}


uint8_t CPU::STX(){
    fetch();

    write(fetched,X);
    return 0;
}


uint8_t CPU::STY(){
    fetch();

    write(fetched,Y);
    return 0;
}

uint8_t CPU::TAX(){
    X = accumulator;
    setStatusFlag(Zero,X==0);
    setStatusFlag(Zero,X&0x80);
    return 0;
}
uint8_t CPU::TAY(){
    Y = accumulator;
    setStatusFlag(Zero,Y==0);
    setStatusFlag(Zero,Y&0x80);
    return 0;
}

uint8_t CPU::TSX(){
    X = sp;

    setStatusFlag(Zero,X==0);
    setStatusFlag(Zero,X&0x80);
    return 0;
}

uint8_t CPU::TXA(){
    accumulator = X;
    setStatusFlag(Zero,accumulator==0);
    setStatusFlag(Zero,accumulator&0x80);
    return 0;
}

uint8_t CPU::TXS(){
    sp = X;
    return 0;
}

uint8_t CPU::TYA(){
    accumulator= Y;

    setStatusFlag(Zero,accumulator==0);
    setStatusFlag(Zero,accumulator&0x80);

    return 0;
}

uint8_t CPU::ADC(){
    fetch();

    uint16_t value = (uint16_t)accumulator + (uint16_t)fetched + (uint16_t)getStatusFlag(Carry);
    setStatusFlag(Carry,value > 0xff);
    setStatusFlag(Zero,value==0);
    setStatusFlag(Negative,value &0x80);
    setStatusFlag(Overflow,checkSignBit(accumulator,fetched));

    accumulator = value &0x00ff;
    return 1; 
}


uint8_t CPU::SBC(){
    fetch();

    uint16_t temp  = (uint16_t)fetched ^ 0x00FF;

    uint16_t value = (uint16_t)accumulator + (uint16_t)temp + (uint16_t)getStatusFlag(Carry);
    setStatusFlag(Carry,value > 0xff);
    setStatusFlag(Zero,value==0);
    setStatusFlag(Negative,value &0x80);
    setStatusFlag(Overflow,checkSignBit(accumulator,temp));
    return 0;
}