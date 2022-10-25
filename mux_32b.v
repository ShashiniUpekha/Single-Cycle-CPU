`timescale  1ns/100ps

module mux32(a,b,SELECT,c);

input [31:0] a;
input [31:0] b;
input SELECT;

output reg[31:0] c;

always @(a,b,SELECT)           //case statement
begin
    case(SELECT)
            1'b0      : c = a ;  //a
            1'b1      : c = b ;  //b
        
    endcase
   
end

endmodule

