`include "reg_file.v"
`include "mux_2b.v"
`include "mux_32b.v"
`include "data_memory.v"
`include "dcachec.v"
`include "alu.v"
`include "inst_cachec.v"
`include "inst_memory.v"

//-----------------------------------------------------------------------------------------------------------------------------------

// Computer Architecture (CO224) - Lab 05
// Design: Testbench of Integrated CPU of Simple Processor

`timescale  1ns/100ps

module cpu_tb;

    reg CLK, RESET;
    wire [31:0] PC;
    wire [31:0] INSTRUCTION;
    
    integer i;
    
    wire read,write, busywait;
    wire [7:0]address,writedata,readdata;


    wire read_mem,write_mem, busywait_mem;
    wire [5:0]address_mem;
    wire [31:0] writedata_mem,readdata_mem;
    wire busywait_inst;

    wire read_instruction;
    wire [5:0] address_instruction;
    wire [127:0] readinst;
    wire busywait_mem_instruction;

    /* 
    ------------------------
     SIMPLE INSTRUCTION MEM
    ------------------------ 
    */
    
    // TODO: Initialize an array of registers (8x1024) named 'instr_mem' to be used as instruction memory
    
    // TODO: Create combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
    //       (make sure you include the delay for instruction fetching here)
    reg [7:0] instr_mem[0:1023];      //1024 registers of 8 bits 
    initial
    begin
        // Initialize instruction memory with the set of instructions you need execute on CPU
        
    /*    
        // METHOD 1: manually loading instructions to instr_mem

        //loadi 0 9
        {instr_mem[10'd3], instr_mem[10'd2], instr_mem[10'd1], instr_mem[10'd0]} = 32'b00000111_00000000_00000000_00001001;  

        //loadi 1 1
        {instr_mem[10'd7], instr_mem[10'd6], instr_mem[10'd5], instr_mem[10'd4]} = 32'b00000111_00000001_00000000_00000001;  

        //swd 0 1
        {instr_mem[10'd11], instr_mem[10'd10], instr_mem[10'd9], instr_mem[10'd8]} = 32'b00001010_00000000_00000000_00000001;  

         //swi 1 0x00
        {instr_mem[10'd15], instr_mem[10'd14], instr_mem[10'd13], instr_mem[10'd12]} = 32'b00001011_00000000_00000001_00000000;

        //lwd 2 1
        {instr_mem[10'd19], instr_mem[10'd18], instr_mem[10'd17], instr_mem[10'd16]} = 32'b00001000_00000010_00000000_00000001; 

        //lwd 3 1        
        {instr_mem[10'd23], instr_mem[10'd22], instr_mem[10'd21], instr_mem[10'd20]} = 32'b00001000_00000011_00000000_00000001; 

        //sub 4 0 1 
        {instr_mem[10'd27], instr_mem[10'd26], instr_mem[10'd25], instr_mem[10'd24]} = 32'b00000001_00000100_00000000_00000001; 

        //swi 4 2
        {instr_mem[10'd31], instr_mem[10'd30], instr_mem[10'd29], instr_mem[10'd28]} = 32'b00001011_00000000_00000100_00000010; 

        //lwi 5 2
        {instr_mem[10'd35], instr_mem[10'd34], instr_mem[10'd33], instr_mem[10'd32]} = 32'b00001001_00000101_00000000_00000010; 

        //swi 4 0x20
        {instr_mem[10'd39], instr_mem[10'd38], instr_mem[10'd37], instr_mem[10'd36]} = 32'b00001011_00000000_00000100_00100000; 

         //lwi 6 0x20
        {instr_mem[10'd43], instr_mem[10'd42], instr_mem[10'd41], instr_mem[10'd40]} = 32'b00001001_00000110_00000000_00100000; 



*/



   //      //lwd 3 0x033
    //    {instr_mem[10'd47], instr_mem[10'd46], instr_mem[10'd45], instr_mem[10'd44]} = 32'b00001000_00000010_00000000_00000100; 

        
        // METHOD 2: loading instr_mem content from instr_mem.mem file
        //$readmemb("instr_mem.mem", instr_mem);
    end
    
    /* 
    -----
     CPU
    -----
    */
    cpu mycpu(PC, INSTRUCTION, CLK, RESET,read,write, address,writedata,readdata,busywait,busywait_inst);

   dcache c(
    busywait,read,write,writedata,readdata,address,
    busywait_mem,read_mem,write_mem,writedata_mem,readdata_mem,address_mem,
    CLK,
    RESET
);

data_memory mem(
	CLK,
    RESET,
    read_mem,
    write_mem,
    address_mem,
    writedata_mem,
    readdata_mem,
    busywait_mem
);
   
instruction_cache cache_in(CLK, PC, INSTRUCTION, busywait_inst,RESET,read_instruction,address_instruction,readinst,busywait_mem_instruction);

instruction_memory mm(CLK,read_instruction,address_instruction,readinst,busywait_mem_instruction);
   
    initial
    begin
    
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("cpu_wavedata.vcd");
		$dumpvars(0, cpu_tb);
        for(i=0;i<8;i=i+1) begin
        $dumpvars(0, mycpu.reg1.register[i]);
        $dumpvars(0, mycpu.c.data[i]);
        $dumpvars(0, mycpu.c.Ddirty[i]);
          $dumpvars(0, mycpu.c.valid[i]);
          $dumpvars(0, mycpu.c.Dtag[i]);
          $dumpvars(0, mycpu.mem.memory_array[i]);
          $dumpvars(0, mycpu.cache_in.inst_cache[i]);
          $dumpvars(0, mycpu.cache_in.tag_inst[i]);
          $dumpvars(0, mycpu.cache_in.valid_inst[i]);


        end

        for(i=0;i<32;i=i+1) 
         $dumpvars(0, mycpu.mem.memory_array[i]);

        CLK = 1'b0;                          //clock=0
        RESET = 1'b1;                        //reset=1
        
        // TODO: Reset the CPU (by giving a pulse to RESET signal) to start the program execution
        
        #5 RESET = 1'b0;                       //reset=0
        // finish simulation after some time
        #3000
        $finish;
        
    end
    
    // clock signal generation
    always
        #4 CLK = ~CLK;                      //to change the clock value after 4 time unit delay
        
  /*      
        always @(PC)
begin

//load instructions from instruction memory
#2;         //after 2 time units delay from the pc update 
INSTRUCTION={instr_mem[PC+3],instr_mem[PC+2],instr_mem[PC+1],instr_mem[PC]};     

end
*/

endmodule


//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

module cpu(PC, INSTRUCTION, CLK, RESET,read_mem,write_mem,address,writedata,readdata,busywait_mem,busywait_inst);


//declare inputs
input [31:0] INSTRUCTION;     //32 bit instruction
input CLK,RESET;              //1 bit 

//declare outputs 
output reg [31:0] PC;        // 32 bits        
  
output read_mem,write_mem;     //read and write signals for memory module
output reg [7:0] address,writedata; //address and data for write in memory

input busywait_inst,busywait_mem;
input [7:0] readdata;     //read data value from memory


wire[7:0] ALURESULT,REGOUT1,REGOUT2,OPERAND2,MUX_OUT;
reg [7:0]OPCODE;            //to take the opcode 
reg [7:0] COMLMNT;           //to take the compliment
wire WRITE,j,beq;           //to take WRITE ,j,beq signals
wire [2:0] ALUOP;
wire ZERO;                  //to get the ZERO output

wire SELECT1,SELECT2;    //inputs for mux select
reg SELECT3;

reg [31:0] pc_4;       //register for store pc+4 

reg  signed [31:0] OFFSET,jump_target;      //to store offset and and jump target address

wire [31:0] PC_next;    //next pc value (output from mux)

wire [7:0] WRITEDATA_REG;

//Initializing the reg_file module
reg_file reg1(WRITEDATA_REG, REGOUT1, REGOUT2, INSTRUCTION[18:16], INSTRUCTION[10:8], INSTRUCTION[2:0], WRITE, CLK, RESET);

//initializing the alu module
alu alu1(REGOUT1,OPERAND2,ALURESULT,ALUOP,ZERO);

//Initializing the mux module to select second operand + or -
mux2  mux1(REGOUT2,COMLMNT,SELECT1,MUX_OUT);

//Initiating the mux module to get the 2nd operand from the register file or the immediate value
mux2  mux2(INSTRUCTION[7:0],MUX_OUT,SELECT2,OPERAND2);

//Initiating the mux module to get readdata from the data memory or the aluresult
mux2  mux4(ALURESULT,readdata,SELECT4,WRITEDATA_REG);


//Initialize the mux32 module to get the pc+4 address or the address after j or beq
mux32  mux3(pc_4,jump_target,SELECT3,PC_next);

//Initiate the control unit module 
control_unit cntrl(INSTRUCTION,SELECT1,SELECT2,SELECT4,ALUOP,j,beq,WRITE,write_mem,read_mem,busywait_inst);

reg busywait;


always @(busywait_mem,busywait_inst)
    busywait = busywait_inst || busywait_mem;    





always @(posedge CLK,busywait_mem)                      //  positive egde of the clock
begin

    if(busywait==0) begin
   
        if(RESET)                             //if reset =1

            #1 PC = 0;                        //afer a time delay of 1 unit 
           
        else                                 //if reset is 0
       
           #1 PC = PC_next;                   //pc will get update

    end
end



always @(PC)
  pc_4= #1 PC + 4;        // to increase the pc value by 4 after a 1 unit delay


always @(REGOUT2) begin          
     
     COMLMNT = #1 (~REGOUT2 +1);   //taking the compliment of the REGOUT2
end

always @(INSTRUCTION) begin

//offset is 32 bits
OFFSET[1:0]=2'b00;   //to shift by 2   (to get 10 bits)
OFFSET[9:2]=INSTRUCTION[23:16];  //8 bit offset part from the instruction
OFFSET[31:10]={22{INSTRUCTION[23]}};   //sign extenstion----->32 bits

jump_target= #2 pc_4+OFFSET;          //to compute the  jump target addresss  
                                    //delay of 2 time units


end


//to get the select signal for the 32 bit mux
//select =1  --> address after j or beq 
//select=0  --> address of pc+4
always @(j,beq,ZERO) 
SELECT3=j|(beq&ZERO);


always @(ALURESULT)
address=ALURESULT;   //location in memory


always @(REGOUT1)
writedata=REGOUT1;   //data value to be stored in the memory

endmodule



module control_unit(INSTRUCTION,SELECT1,SELECT2,SELECT4,ALUOP,j,beq,WRITEN,write_mem,read_mem,busywait); /////////////////////////////

//Input definition
input [31:0] INSTRUCTION;
input busywait;

//output definition
output reg SELECT1,SELECT2,SELECT4;  //to values to mux1 ,mux2 and mux4
output reg [2:0] ALUOP; //to give values to ALUOP
output reg WRITEN;     //to give the WRITE signal
reg [7:0]OPCODE;      //to get the opcode 
output reg j,beq;   //to give j and beq signals
output reg write_mem,read_mem;
reg WRITE;
always @(INSTRUCTION)                    
begin

OPCODE=INSTRUCTION[31:24];          //selecting opcode bits



    case(OPCODE)
            8'b00000000      :  begin  #1;
                                ALUOP   = 3'b001 ;  //add
                                SELECT1 = 0 ;   
                                SELECT2 = 1 ;
                                SELECT4 = 0 ;
                                j=0;
                                beq=0;
                                WRITE   = 1 ;
                                read_mem = 0 ;
                                write_mem = 0 ;
                        end
                            
            8'b00000001      :   begin  #1;
                                ALUOP   = 3'b001 ;  //sub
                                SELECT1 = 1 ;       // to take the compliment
                                SELECT2 = 1 ;
                                SELECT4 = 0 ;
                                j=0;
                                beq=0;
                                WRITE   = 1 ;
                                read_mem = 0 ;
                                write_mem = 0 ;
                        end

            8'b00000010      :   begin #1;
                                ALUOP   = 3'b010 ;  //and
                                SELECT1 = 0 ;
                                SELECT2 = 1 ;
                                SELECT4 = 0 ;
                                j=0;
                                beq=0;
                                WRITE   = 1 ;
                                read_mem = 0 ;
                                write_mem = 0 ;
                        end

            8'b00000011      :   begin #1;
                                ALUOP   = 3'b011 ;  //or
                                SELECT1 = 0 ;
                                SELECT2 = 1 ;
                                SELECT4 = 0 ;
                                j=0;
                                beq=0;
                                WRITE   = 1 ;
                                read_mem = 0 ;
                                write_mem = 0 ;
                        end

            8'b00000100      :  begin #1; //j
                                j=1;
                                beq=0;
                                
                                WRITE   = 0 ;
                                read_mem = 0 ;
                                write_mem = 0 ;
                        end
                            
            8'b00000101      :  begin #1;    // beq
                                ALUOP   = 3'b001 ;  //ADD function
                                SELECT1 = 1 ;       //take the compliment  
                                SELECT2 = 1 ;
                                beq = 1;            //beq signal 1
                                j=0;
                                
                                WRITE   = 0 ;
                                read_mem = 0 ;
                                write_mem = 0 ;
                        end



            8'b00000110      :   begin #1;
                                ALUOP   = 3'b000 ;  //mov
                                SELECT1 = 0 ;
                                SELECT2 = 1 ;
                                SELECT4 = 0;
                                j=0;
                                beq=0;
                                WRITE   = 1 ;
                                read_mem = 0 ;
                                write_mem = 0 ;
                        end     

            8'b00000111      :   begin #1;
                                ALUOP   = 3'b000 ;  //loadi
                                SELECT1 = 0 ;
                                SELECT2 = 0 ;
                                SELECT4 = 0;
                                j=0;
                                beq=0;
                                WRITE   = 1 ;
                                read_mem = 0 ;
                                write_mem = 0 ;
                        end

            8'b00001000      :  begin  #1;
                                ALUOP   = 3'b000 ;  //lwd
                                SELECT1 = 0 ;   
                                SELECT2 = 1 ;
                                SELECT4 = 1 ;
                                j=0;
                                beq=0;
                                WRITE   = 1 ;
                                read_mem = 1 ;
                                write_mem = 0 ;
                        end

            8'b00001001      :  begin  #1;
                                ALUOP   = 3'b000 ;  //lwi
                                SELECT1 = 0 ;   
                                SELECT2 = 0 ;
                                SELECT4 = 1 ;
                                j=0;
                                beq=0;
                                WRITE   = 1 ;
                                read_mem = 1 ;
                                write_mem = 0 ;
                        end

            8'b00001010      :  begin  #1;
                                ALUOP   = 3'b000 ;  //swd
                                SELECT1 = 0 ;   
                                SELECT2 = 1 ;
                                SELECT4 = 1 ;
                                j=0;
                                beq=0;
                                WRITE   = 0 ;
                                read_mem = 0 ;
                                write_mem = 1 ;
                        end

            8'b00001011      :  begin  #1;
                                ALUOP   = 3'b000 ;  //swi
                                SELECT1 = 0 ;   
                                SELECT2 = 0 ;
                                SELECT4 = 1 ;
                                write_mem =1 ;
                                j=0;
                                beq=0;
                                WRITE   = 0 ;
                                read_mem = 0 ;
                                
                                
                        end

              
    endcase

end



always @(busywait) begin  
if(busywait == 0)
begin
read_mem=0;                 //read signal for data memory
write_mem=0;                 //write signal for data memory
end
end

always@(busywait,WRITE)
begin
WRITEN=WRITE & !busywait ;
end
endmodule