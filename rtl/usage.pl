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
	"sdio.v", "sdwb.v", "sdckgen.v",
		"sdaxil.v", "sdskid.v",
		"sdcmd.v", "sdrxframe.v", "sdtxframe.v",
	"sdspi.v", "spicmd.v", "spirxdata.v", "spitxdata.v",
		"llsdspi.v",
	"sddma.v", "sdfifo.v",
		"sddma_mm2s.v", "sddma_s2mm.v",
		"sdax_mm2s.v",  "sdax_s2mm.v",
		"sddma_rxgears.v", "sddma_txgears.v",
	"sdfrontend.v", "xsdserdes8x.v" );
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
	my $result = "";

	$result = sprintf("SDIO(AXIL):  %5d %5d %7d\n",
			## Synth, top, bus, config, postsynth
		calcusage($ice40synth, "sdio", "axil", $sdio_nodma,""),
		calcusage($xilinxsynth,"sdio", "axil", $sdio_nodma,""),
		calcusage($asicsynth,  "sdio", "axil", $sdio_nodma,$asicpost));

	$result = $result . sprintf("SDIO(WB):    %5d %5d %7d\n",
		calcusage($ice40synth, "sdio", "wb", $sdio_nodma,""),
		calcusage($xilinxsynth,"sdio", "wb", $sdio_nodma,""),
		calcusage($asicsynth,  "sdio", "wb", $sdio_nodma,$asicpost));

	$result = $result . sprintf("EMMC(AXIL):  %5d %5d %7d\n",
		calcusage($ice40synth, "sdio", "axil", $emmc_nodma,""),
		calcusage($xilinxsynth,"sdio", "axil", $emmc_nodma,""),
		calcusage($asicsynth,  "sdio", "axil", $emmc_nodma,$asicpost));

	$result = $result . sprintf("EMMC(WB):    %5d %5d %7d\n",
		calcusage($ice40synth, "sdio", "wb", $emmc_nodma,""),
		calcusage($xilinxsynth,"sdio", "wb", $emmc_nodma,""),
		calcusage($asicsynth,  "sdio", "wb", $emmc_nodma,$asicpost));

	$result = $result . sprintf("SDIO w/DMA:  %5d %5d %7d\n",
		calcusage($ice40synth, "sdio", "wb", $sdio_dma,""),
		calcusage($xilinxsynth,"sdio", "wb", $sdio_dma,""),
		calcusage($asicsynth,  "sdio", "wb", $sdio_dma,$asicpost));

	$result = $result . sprintf("EMMC w/DMA:  %5d %5d %7d\n",
		calcusage($ice40synth, "sdio", "wb", $emmc_dma,""),
		calcusage($xilinxsynth,"sdio", "wb", $emmc_dma,""),
		calcusage($asicsynth,  "sdio", "wb", $emmc_dma,$asicpost));

	$result = $result . sprintf("SDAXI w/DMA: %5d %5d %7d\n",
		calcusage($ice40synth, "sdio", "axil", $sdio_dma,""),
		calcusage($xilinxsynth,"sdio", "axil", $sdio_dma,""),
		calcusage($asicsynth,  "sdio", "axil", $sdio_dma,$asicpost));

	$result = $result . sprintf("EMAXI w/DMA: %5d %5d %7d\n",
		calcusage($ice40synth, "sdio", "axil", $emmc_dma,""),
		calcusage($xilinxsynth,"sdio", "axil", $emmc_dma,""),
		calcusage($asicsynth,  "sdio", "axil", $emmc_dma,$asicpost));

	$result = $result . sprintf("SDSPI:       %5d %5d %7d\n",
		calcusage($ice40synth, "sdspi", "wb", "",""),
		calcusage($xilinxsynth,"sdspi", "wb", "",""),
		calcusage($asicsynth,  "sdspi", "wb", "",$asicpost));

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
