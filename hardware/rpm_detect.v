
`timescale 1ns / 1ps



module rpm_detector(clock, reset, sa_input , rpm_output);
    input clock;
    input reset;
    input sa_input;
    output reg[31:0] rpm_output;
    
    parameter [31:0] SAMPLING_FREQUENCY = 32'd100000000;
    reg       [31:0] SAMPLING_COUNT     = 32'd0; 		
	reg 	  [31:0] HIGH_COUNT         = 32'd0;

    wire      [31:0] pe;
    reg       sa_input_neg;
assign pe = sa_input & ~sa_input_neg;
    
    always@(posedge clock)
    begin
      if(~reset)
           begin
           SAMPLING_COUNT<=32'd0;
           HIGH_COUNT <= 32'd0;
           end
      else
           begin
           sa_input_neg <=  sa_input;
           if (SAMPLING_COUNT<=SAMPLING_FREQUENCY)
            begin
                if (pe)
                       begin
                       HIGH_COUNT<=HIGH_COUNT+1'd1;
                end
                else begin
                        HIGH_COUNT<=HIGH_COUNT;
                end
                SAMPLING_COUNT<=SAMPLING_COUNT + 1'b1;
                rpm_output <= rpm_output ;
            end
            else begin
                rpm_output <= HIGH_COUNT;
                HIGH_COUNT <= 32'b0;
                SAMPLING_COUNT <= 32'b0;
            end
           end
    end 
    
    
endmodule
