#!/bin/perl

## Configuration definitions
## {{{

my $sdspi  = "";
my $sdio   = "";
## }}}

## Files
## {{{
my @files = (
	"sdio.v", "sdwb.v", "sdckgen.v",
		"sdcmd.v", "sdrxframe.v", "sdtxframe.v",
	"sdspi.v", "spicmd.v", "spirxdata.v", "spitxdata.v",
		"llsdspi.v",
	"sdfrontend.v", "xsdserdes8x.v" );
## }}}

my $logfile = "yosys.log";
my $scriptf = "usage.ys";
my $ice40synth = "synth_ice40";
my $xilinxsynth = "synth_xilinx";
my $asicsynth   = "synth";
my $asicpost    = "abc -g cmos2";

sub	calcusage($$$$) {
	## {{{
	my($synth,$toplvl,$config,$postsynth)=@_;

	## Build the script
	## {{{
	unlink($scriptf);
	open(SCRIPT, "> $scriptf");
	foreach $key (@files) {
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

	$result = sprintf("SDIO:      %5d %5d %6d\n",
		calcusage($ice40synth, "sdio", "",""),
		calcusage($xilinxsynth,"sdio", "",""),
		calcusage($asicsynth,  "sdio", "",$asicpost));

	$result = $result . sprintf("SDSPI:     %5d %5d %6d\n",
		calcusage($ice40synth, "sdspi", "",""),
		calcusage($xilinxsynth,"sdspi", "",""),
		calcusage($asicsynth,  "sdspi", "",$asicpost));

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
