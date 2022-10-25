`timescale  1ns/100ps

module mux2(a,b,SELECT,c);

//input definitions 
input [7:0] a,b;
input SELECT;

//output definitions 
output reg[7:0] c;

always @(a,b,SELECT)           //case statement
begin
    case(SELECT)
            1'b0      : c = a ;  //a
            1'b1      : c = b ;  //b
          
    endcase
end

endmodule