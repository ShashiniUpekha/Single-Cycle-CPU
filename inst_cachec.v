
`timescale  1ns/100ps

module instruction_cache(clock, PC, instruction, busywait,reset,read,address_mem,readinst,busywait_mem);

//Inputs and outputs with the CPU
input clock,reset;
input [31:0] PC; //32 bit pc


output reg busywait;

output reg read;



//to and from instruction memory
input [127:0] readinst;            //128 bit
input busywait_mem;          

output reg [31:0] instruction;   //32 bit instructions
output reg [5:0] address_mem;      //6 bit block address

//for insttruction cache storage

reg [127:0] inst_cache [0:7];      //8 data blocks---128 bits
reg [2:0] tag_inst [0:7];          // 8 tags of 3 bits
reg valid_inst [0:7];              //8 valid signals
 
reg [2:0] tag,index;    //3 bit tag and index
reg [1:0] offset;       //2 bit offset
reg valid;              //valid 
reg hit;             //for hit
reg write_memdata;
reg tag_ch;

integer i;       //variable


initial
begin

    read = 0;
    busywait = 0;

    //At the begining all cache entries are empty. 
    
    for(i=0;i<8;i=i+1)  
    begin
    valid_inst[i] = 0;  //to assign zero to valid bits 
    end

end

always@(*)
begin

//Split the address into tag,index,and offset
//address= pc[9:0]

    tag = PC [9:7];
    index = PC [6:4];
    offset = PC [3:2];
 #1;   
    tag_ch = tag_inst[index];
    valid = valid_inst[index];
end

//tag comparison and check whether it is a hit or a miss
always@(valid,tag_ch,index)
begin

    if((tag==tag_ch)&& valid)  //if tag is matched and valid bit=1
    #0.9 hit = 1;      //hit
    else
    #0.9 hit = 0;     //miss


end

always@(hit)
begin
    //busywait

    if(hit)
        busywait=0;

    else if(!hit & !busywait) 
    begin
        read = 1;        //memory read
        busywait = 1;
        address_mem = {tag,index};
        write_memdata = 1;
    end



end


always@(index,offset,inst_cache[index],hit)
begin
if(hit) begin
#1;
         case(offset)     //INSTRUCTION WORD SELECTION Based on the offset
                2'd0    : instruction = inst_cache[index][31:0];
                2'd1    : instruction = inst_cache[index][63:32];
                2'd2    : instruction = inst_cache[index][95:64];
                2'd3    : instruction = inst_cache[index][127:96];
            endcase
            
end
end

always @(posedge clock)
begin

//reset
        if(reset) begin
           
            for(i=0;i<8;i++)
            begin
            valid_inst[i] = 0;      //assign valid bits to zero
            end
            
end

 else if(write_memdata & !busywait_mem)
        begin
            read = 0;

            //latency of #1 time unit for writing operation(writing the fetched values to the cache memory)
        #1;
           
            inst_cache [index] = readinst;
            tag_inst [index] = tag;
            valid_inst [index] = 1;
            write_memdata = 0;
        end


end


endmodule