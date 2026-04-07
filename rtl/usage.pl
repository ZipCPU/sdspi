#!/bin/perl

## Configuration definitions
## {{{
my $sdspi  = "";
my $sdio_nodma  = " -chparam OPT_DMA 1\'b0 -chparam NUMIO 4 -chparam OPT_EMMC 1\'b0";
my $emmc_nodma  = " -chparam OPT_DMA 1\'b0 -chparam NUMIO 8 -chparam OPT_EMMC 1\'b1";
my $sdio_dma  = " -chparam OPT_DMA 1\'b1 -chparam OPT_EMMC 1\'b0 -chparam NUMIO 4 -chparam OPT_ISTREAM 1\'b1 -chparam OPT_OSTREAM 1\'b1 -chparam DMA_DW 64";
my $emmc_dma  = " -chparam OPT_DMA 1\'b1 -chparam OPT_EMMC 1\'b1 -chparam NUMIO 8 -chparam OPT_ISTREAM 1\'b1 -chparam OPT_OSTREAM 1\'b1 -chparam DMA_DW 64";
## }}}

## Files
## {{{
my @files = (
	"sdio/sdio.v", "sdio/sdwb.v", "sdio/sdckgen.v",
		"sdio/sdaxil.v", "sdskid.v",
		"sdio/sdcmd.v", "sdio/sdrxframe.v", "sdio/sdtxframe.v",
	"spi/sdspi.v", "spi/spicmd.v", "spi/spirxdata.v", "spi/spitxdata.v",
		"spi/llsdspi.v",
	"sdslave/sdsfsm.v", "sdslave/sdslave.v", "sdslave/sdsdma.v",
		"sdslave/sdscmd.v", "sdslave/sdtfrvalue.v",
		"sdslave/sdsrxframe.v","sdslave/sdstxframe.v",
	"sdio/sddma.v", "sdfifo.v", "afifo.v",
		"sddma_mm2s.v", "sddma_s2mm.v",
		"sdio/sdax_mm2s.v",  "sdio/sdax_s2mm.v",
		"sddma_rxgears.v", "sddma_txgears.v",
	"sdio/sdfrontend.v", "xsdddr.v", "xsdserdes8x.v" );
## }}}

my $logfile = "yosys.log";
my $scriptf = "usage.ys";
my $ice40synth = "synth_ice40";
my $xilinxsynth = "synth_xilinx";
my $asicsynth   = "synth";
my $asicpost    = "abc -g cmos2";

sub	calcusage($$$$$) {
	## {{{
	my($synth,$toplvl,$bus,$config,$postsynth)=@_;

	## Build the script
	## {{{
	unlink($scriptf);
	open(SCRIPT, "> $scriptf");
	if ($bus =~ "axil") {
		print SCRIPT "read -define SDIO_AXI\n";
	} foreach $key (@files) {
		print SCRIPT "read -sv $key\n";
	}

	print SCRIPT "hierarchy -top $toplvl $config\n";
	print SCRIPT "$synth -flatten -top $toplvl\n";
	if ($postsynth ne "") {
		print SCRIPT "$postsynth\n";
	}
	print SCRIPT "stat\n";
	close(SCRIPT);
	## }}}
	
	system("yosys -l $logfile -s $scriptf");

	## Read the log, looking for the usage statistics
	## {{{
	$usage = 0;
	open(LOG, "< $logfile");
	while($line = <LOG>) {
		if ($line =~ /Estimated number of LCs:\s*(\d+)\s*$/) {
			$usage = $1;
		} elsif ($line =~ /^\s*SB_LUT4\s*(\d+)\s*$/) {
			$usage = $1;
		} elsif ($line =~ /^\s*\$_NAND_\s*(\d+)\s*$/) {
			$usage = $1;
		}
	} close(LOG);
	## }}}

	$usage
}
## }}}

sub	topusage() {
	## {{{
	my $header = "";
	my $line = "";
	my $result = "";

	open(USAGE, "> tmp-usage.txt");

$header = "           iCE40  X7-s   RAW\n"
	. "Controller  4LUT  6LUT  NANDs\n"
	. "-----------------------------------\n";
	print USAGE $header;

	$line = sprintf("SDIO(AXIL):  %5d %5d %7d\n",
			## Synth target, top-level, bus, config, postsynth
		calcusage($ice40synth, "sdio", "axil", $sdio_nodma,""),
		calcusage($xilinxsynth,"sdio", "axil", $sdio_nodma,""),
		calcusage($asicsynth,  "sdio", "axil", $sdio_nodma,$asicpost));
	$result = $line;
	print USAGE $line;

	$line = sprintf("SDIO(WB):    %5d %5d %7d\n",
		calcusage($ice40synth, "sdio", "wb", $sdio_nodma,""),
		calcusage($xilinxsynth,"sdio", "wb", $sdio_nodma,""),
		calcusage($asicsynth,  "sdio", "wb", $sdio_nodma,$asicpost));
	$result = $result . $line;
	print USAGE $line;

	$line = sprintf("EMMC(AXIL):  %5d %5d %7d\n",
		calcusage($ice40synth, "sdio", "axil", $emmc_nodma,""),
		calcusage($xilinxsynth,"sdio", "axil", $emmc_nodma,""),
		calcusage($asicsynth,  "sdio", "axil", $emmc_nodma,$asicpost));
	$result = $result . $line;
	print USAGE $line;

	$line = sprintf("EMMC(WB):    %5d %5d %7d\n",
		calcusage($ice40synth, "sdio", "wb", $emmc_nodma,""),
		calcusage($xilinxsynth,"sdio", "wb", $emmc_nodma,""),
		calcusage($asicsynth,  "sdio", "wb", $emmc_nodma,$asicpost));
	$result = $result . $line;
	print USAGE $line;

	$line = sprintf("SDIO w/DMA:  %5d %5d %7d\n",
		calcusage($ice40synth, "sdio", "wb", $sdio_dma,""),
		calcusage($xilinxsynth,"sdio", "wb", $sdio_dma,""),
		calcusage($asicsynth,  "sdio", "wb", $sdio_dma,$asicpost));
	$result = $result . $line;
	print USAGE $line;

	$line = sprintf("EMMC w/DMA:  %5d %5d %7d\n",
		calcusage($ice40synth, "sdio", "wb", $emmc_dma,""),
		calcusage($xilinxsynth,"sdio", "wb", $emmc_dma,""),
		calcusage($asicsynth,  "sdio", "wb", $emmc_dma,$asicpost));
	$result = $result . $line;
	print USAGE $line;

	$line = sprintf("SDAXI w/DMA: %5d %5d %7d\n",
		calcusage($ice40synth, "sdio", "axil", $sdio_dma,""),
		calcusage($xilinxsynth,"sdio", "axil", $sdio_dma,""),
		calcusage($asicsynth,  "sdio", "axil", $sdio_dma,$asicpost));
	$result = $result . $line;
	print USAGE $line;

	$line = sprintf("EMAXI w/DMA: %5d %5d %7d\n",
		calcusage($ice40synth, "sdio", "axil", $emmc_dma,""),
		calcusage($xilinxsynth,"sdio", "axil", $emmc_dma,""),
		calcusage($asicsynth,  "sdio", "axil", $emmc_dma,$asicpost));
	$result = $result . $line;
	print USAGE $line;

	$line = sprintf("SDSPI:       %5d %5d %7d\n",
		calcusage($ice40synth, "sdspi", "wb", "",""),
		calcusage($xilinxsynth,"sdspi", "wb", "",""),
		calcusage($asicsynth,  "sdspi", "wb", "",$asicpost));
	$result = $result . $line;
	print USAGE $line;

	$line = sprintf("SDSLAVE(WB): %5d %5d %7d\n",
		calcusage($ice40synth, "sdslave", "wb", "",""),
		calcusage($xilinxsynth,"sdslave", "wb", "",""),
		calcusage($asicsynth,  "sdslave", "wb", "",$asicpost));
	$result = $result . $line;
	print USAGE $line;
	close(USAGE);

	$result
}
## }}}

$result = "";
$result = topusage();

## Add a header
$result = "           iCE40  X7-s   RAW\n"
	. "Controller  4LUT  6LUT  NANDs\n"
	. "-----------------------------------\n" . $result;
print $result;

open(USAGE, "> usage.txt");

print USAGE $result;

close(USAGE);
