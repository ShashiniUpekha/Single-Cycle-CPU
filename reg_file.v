`timescale  1ns/100ps

module reg_file(IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);  //----------------------------module reg_file

input [7:0] IN;                                  //input and output ports
input [2:0] INADDRESS,OUT1ADDRESS,OUT2ADDRESS;
input WRITE,CLK,RESET;
output [7:0] OUT1,OUT2;


reg signed [7:0] register[0:7];                        //output - reg

integer i=0,j=0;


    
    assign #2 OUT1 =   register[OUT1ADDRESS];  //assign the stored value to output
     assign  OUT2 = register[OUT2ADDRESS]; 
    


always @(posedge CLK)                 //set for every positive edge
begin

    if(RESET)                         //if reset==1 ----> reset
    begin
    
           #1 for(i=0;i<8;i=i+1)              //loop for reset 
           begin
            register[i] <=  0;
            end

    end     
    

    else if(WRITE==1)                       //else if write==1 --> write
            begin
             register[INADDRESS] = #1 IN;
      
            end

end


initial begin
#5
$monitor($time,"   r0-->%d  r1-->%d  , r2-->%d , r3-->%d , r4-->%d , r5-->%d , r6-->%d , r7-->%d \n",register[0],register[1],register[2],register[3],register[4],register[5],register[6],register[7]);


end


endmodule