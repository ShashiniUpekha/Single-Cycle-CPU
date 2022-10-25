/*
Module  : Data Cache 
Author  : Isuru Nawinne, Kisaru Liyanage
Date    : 25/05/2020

Description	:

This file presents a skeleton implementation of the cache controller using a Finite State Machine model. Note that this code is not complete.
*/
`timescale  1ns/100ps
module dcache (BUSYWAIT_C,read_access,write_access,WRITEDATA_C,READDATA_C,ADDRESS_C,mem_busywait,mem_read,mem_write,mem_writedata,mem_readdata,mem_address,clock,reset);
    
    /*
    Combinational part for indexing, tag comparison for hit deciding, etc.
    ...
    ...
    */


//Inputs and outputs with the CPU
input read_access,write_access;         //to get read and write signals                    
input [7:0] WRITEDATA_C;   //Write data from the register file 8 bit
input [7:0] ADDRESS_C;     //8 bit Address from the alu

output reg BUSYWAIT_C;       //busywait signal to cpu
output reg [7:0] READDATA_C; //read data given to the register file

//Inputs and Outputs with the Data memory

input mem_busywait;            //Busywait signal to the memory     
input [31:0] mem_readdata;      //32 bit readdata

output reg mem_read,mem_write;   //read and write signals to the memory
output reg [31:0] mem_writedata; //32 bit write data to the memory
output reg [5:0] mem_address;    // 6 bit address


//For the other Signals 
input clock,reset;  //clock and reset

reg [2:0] Dtag [0:7];   //8 tag of 3 bit
reg valid [0:7];        //8 valid signals
reg Ddirty [0:7];       // 8 dirty signals
reg [31:0] data [0:7];  //32 bit data ---8 data blocks

reg cache_hit=0;   //to check if there is a hit
reg valid_check;   //to check valid bits 
reg tag_matched;   //to check whether tags are matched 
reg tag_checked;   //to check the tags

reg [2:0] block_no,offset; //to get the block number and offset
reg [2:0]index=0,tag;      //to get the index and tag
reg check;                 //to check tags

integer i;      //variable   

//Signals to enable 
reg write_back_enable,write_cpudata_en,write_memdata_en;
reg READ_C,WRITE_C;

reg dirty;   //for dirty bit

//set the values to zero at the start
initial begin

    //BUSYWAIT_C=0;
    mem_write=0;
    mem_read=0;
    
    for(i=0;i<8;i++)
    begin
    valid[i]=0;
    Ddirty[i]=0;
    end

end  




always @(*)
begin
if(READ_C == 1'b1 || WRITE_C == 1'b1)     //read=1 or write =1
               begin
        
        //To split the address into tag , index and offset
        #1;
            tag = ADDRESS_C[7:5];
            index = ADDRESS_C [4:2];
            offset = ADDRESS_C [1:0];
            dirty = Ddirty[index];
           end
 end 


//For tag comaparison
always @(*)
begin
    //(tag at Dtag[index] should match with address[7:5])
    if(Dtag[index]==tag)
    #0.9    tag_matched = 1;
    else
    #0.9    tag_matched = 0;

    

    //To determine whether the access is a hit or a miss
    cache_hit = tag_matched & valid[index];



end

//When read and write access values changes
always@(read_access,write_access) 
begin
    if(read_access || write_access)
    BUSYWAIT_C = 1;
  
    if(read_access && !write_access)
    READ_C = 1;

    if(!read_access && write_access) 
     WRITE_C = 1;

end
/*
always@(READ_C,WRITE_C) begin
if(READ_C || WRITE_C)
    BUSYWAIT_C = 1;
end
*/
always @(*)
begin

    if(READ_C & cache_hit)    //read=1 hit =1
    begin
     
               
                   #1;
                
 case(offset)     //Extract the dataword from the datablock according to the offset value  with #1 unit latency

                2'd0    : READDATA_C = data[index][7:0];
                2'd1    : READDATA_C = data[index][15:8];
                2'd2    : READDATA_C = data[index][23:16];
                2'd3    : READDATA_C = data[index][31:24];
            endcase
             
         
    end

end

/* Cache Controller FSM Start */

    parameter IDLE = 3'b000, MEM_READ = 3'b001, MEM_WRITE = 3'b010;
    reg [2:0] state, next_state;

 


    // combinational next state logic
    always @(*)
    begin
    
        case (state)

            IDLE:
                if ((READ_C || WRITE_C) && !dirty && !cache_hit)    //read or write=1    and   dirty=hit=0
                    next_state = MEM_READ;
                else if ((READ_C || WRITE_C) && dirty && !cache_hit)    //read or write=1    and   dirty=1 hit=0              
               
                    next_state = MEM_WRITE;   //if dirty=1 next state write mem
                else 
                    next_state = IDLE; 
            
            MEM_READ:
                if (!mem_busywait)     //busywait=0
                    next_state = IDLE;
                else    
                    next_state = MEM_READ;

            MEM_WRITE:
                if (!mem_busywait)          //busywait=0
                    next_state = MEM_READ;
                else    
                    next_state = MEM_WRITE;
            
        endcase
    end

    // combinational output logic
    always @(*)
    begin
        case(state)
            IDLE:
            begin
                mem_read = 0;
                mem_write = 0;
                mem_address = 8'dx;
                mem_writedata = 8'dx;
               
            end
         
            MEM_READ: 
            begin
                mem_read = 1;
                mem_write = 0;
                mem_address = {tag, index};
                mem_writedata = 32'dx;
                write_memdata_en = 1;
  

            end


            MEM_WRITE: 
            begin
                mem_read = 0;
                mem_write = 1;
                mem_address = {Dtag[index], index};
                mem_writedata = data[index];
                write_back_enable=1;
                if(!mem_busywait)
                begin
                    Ddirty[index]=0;
                    write_back_enable = 0;
                    
                end
    
            end
            
        endcase
    end


    // sequential logic for state transitioning 
    always @(posedge clock, reset)
    begin
        if(reset)
            state = IDLE;
        else
            state = next_state;
    end

   

always @(posedge clock)
begin
    if (reset)           //if reset
        begin
            for (i=0;i<8; i=i+1)
                data[i] =32'd0;            //set data block values to zero
                BUSYWAIT_C = 0;
        end

   
    else if(WRITE_C & cache_hit)           //write=1 and hit=1
    begin
    Ddirty[index] = 1;
        WRITE_C = 0;
        BUSYWAIT_C = 0;
    #1;
        case(offset)    //writing the fetched values to the cache memory
				        // latency of #1 time unit for writing operation

            2'd0    : data[index][7:0]    = WRITEDATA_C;
            2'd1    : data[index][15:8]   = WRITEDATA_C;
            2'd2    : data[index][23:16]  = WRITEDATA_C;
            2'd3    : data[index][31:24]  = WRITEDATA_C;
        endcase

        
    end

    else if(write_memdata_en & !mem_busywait)
    begin                                //writing the fetched values from memory to the cache memory
        #1;
        data[index] = mem_readdata;
        Dtag[index] = tag;
        Ddirty[index] = 0;
        valid[index] = 1;
        write_memdata_en = 0; 
        
    end
    else if(READ_C & cache_hit)    //read=1 hit =1
    begin
            READ_C = 0;
            BUSYWAIT_C = 0;
    end

end

 /* Cache Controller FSM End */
endmodule
