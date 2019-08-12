all: test

llsddata_tb: llsddata_tb.v ../../rtl/llsddata.v
	iverilog llsddata_tb.v -y ../../rtl -o llsddata_tb

.PHONY: test
test: llsddata_tb
	./llsddata_tb

.PHONY: clean
clean:
	rm llsddata_tb *.vcd
