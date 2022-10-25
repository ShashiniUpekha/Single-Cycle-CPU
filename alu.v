`timescale  1ns/100ps

//----------------ALU funtn---------------------


module alu (DATA1, DATA2, RESULT, SELECT,ZERO); 

//initilaizing the inputs 
input [7:0] DATA1,DATA2;           
input [2:0] SELECT;
output reg ZERO;
//initilaizing the outout
output [7:0] RESULT;

wire [7:0] a[0:3];    

//instatiate the modules 

forward_op f1(DATA2,a[0]);                //forward
add_op     a1(DATA1,DATA2,a[1]);          //add
and_op     b1(DATA1,DATA2,a[2]);          //and
or_op      c1(DATA1,DATA2,a[3]);          //or

mux m1(a[0],a[1],a[2],a[3],SELECT,RESULT);  //mux


always @(RESULT) begin
ZERO= !(RESULT[0] | RESULT[1] | RESULT[2] | RESULT[3] | RESULT[4] | RESULT[5] | RESULT[6] | RESULT[7]  );



end
endmodule                          




module add_op(DATA1,DATA2,R);   //-----------------add funtn--------------

//initilaizing the inputs and outputs
input [7:0] DATA1 ,DATA2;
output [7:0] R;

assign #2 R = DATA1 + DATA2;   //add DATA ,DATA2 with time delay of 2 unit time 

endmodule



module forward_op(DATA2,R);    //-----------------forward funtn-------------------

//initilaizing the inputs and outputs
input [7:0] DATA2;
output [7:0] R;

assign #1 R =  DATA2;        //forward DATA2 with time delay of 1 unit time 

endmodule



module and_op(DATA1,DATA2,R); //------------------------and funtn-------------------

//initilaizing the inputs and outputs
input [7:0] DATA1 ,DATA2;
output [7:0] R;

 assign #1 R = DATA1 & DATA2;   //and opt for  DATA1,DATA2 with time delay of 1 unit time 

endmodule



module or_op(DATA1,DATA2,R);    //--------------------or funtn-------------------

//initilaizing the inputs and outputs
input [7:0] DATA1 ,DATA2;
output [7:0] R;

assign #1 R = DATA1 | DATA2;   //or opt for  DATA1,DATA2 with time delay of 1 unit time

endmodule



module mux(a,b,c,d,SELECT,RESULT);    //---------------------mux funtn---------------------

//initilaizing the inputs and outputs
input [7:0] a,b,c,d;
input [2:0] SELECT;
output reg signed [7:0] RESULT;

             
        
always @(a,b,c,d,SELECT)           //case statement
begin

    case(SELECT)
        3'b000      : RESULT = a ;  //forward
        3'b001      : RESULT = b ;  //add
        3'b010      : RESULT = c ;  //and
        3'b011      : RESULT = d ;  //or
        //default     : RESULT = 8'b0 ;  
    endcase
   // $display("aluresult %d",RESULT);
end
endmodule


