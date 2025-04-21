module IOBUF (input wire T, input wire I, inout wire IO, output wire O);
	assign	IO = (T) ? 1'bz : I;
	assign	O = (IO !== 1'b0);
endmodule
