#!/bin/perl
################################################################################
##
## Filename: 	sim_sdio.pl
## {{{
## Project:	SDIO SD-Card controller
##
## Purpose:	
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
## }}}
## Copyright (C) 2023, Gisselquist Technology, LLC
## {{{
## This program is free software (firmware): you can redistribute it and/or
## modify it under the terms of the GNU General Public License as published
## by the Free Software Foundation, either version 3 of the License, or (at
## your option) any later version.
##
## This program is distributed in the hope that it will be useful, but WITHOUT
## ANY WARRANTY; without even the implied warranty of MERCHANTIBILITY or
## FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
## for more details.
##
## You should have received a copy of the GNU General Public License along
## with this program.  (It's in the $(ROOT)/doc directory.  Run make with no
## target there if the PDF file isn't present.)  If not, see
## <http://www.gnu.org/licenses/> for a copy.
## }}}
## License:	GPL, v3, as defined and found on www.gnu.org,
## {{{
##		http://www.gnu.org/licenses/gpl.html
##
##
################################################################################
##
## }}}
use Cwd;
$path_cnt = $ARGV;

$filelist = "sdio_files.txt";
$exefile = "sdiosim";

## Usage: perl sim_sdio.pl all
##   or
## 	perl sim_sdio.pl <testcasename>

## Process arguments
## {{{
## }}}

## timestamp
## {{{
sub timestamp {
	my $sc, $mn, $hr, $dy, $mo, $yr, $wday, $yday, $isdst;

	($sc,$mn,$hr,$dy,$mo,$yr,$wday,$yday,$isdst)=localtime(time);
	$yr=$yr+1900; $mo=$mo+1;
	$tstamp = sprintf("%04d/%02d/%02d %02d:%02d:%02d",
					$yr,$mo,$dy,$hr,$mn,$sc);
}
## }}}

## simline: Simulate, given a named test configuration
## {{{
sub simline($) {
	my $line = shift;

	my $tstname = "";
	my $args = "";

	my $vcddump=0;
	my $vcdfile="";
	my $toplevel="tb_sdio";

	while ($line =~ /^(.*)#.*/) {
		$line = $1;
	} if ($line =~ /^\s*(\S+)\s(.*)$/) {
		$tstname = $1;
		$args = $2;
	} elsif ($line =~ /^\s*(\S+)$/) {
		$tstname = $1;
		$args = "";
	}

	if ($tstname eq "") {
	} else {
		## {{{
		$tstamp = timestamp();

		## Set up the IVerilog command
		$cmd = "iverilog -g2012";

		$cmd = $cmd . " -DIVERILOG";
		$cmd = $cmd . " -DSCRIPT=\\\"../testscript/$tstname.v\\\"";
		
		while($args =~ /\s*(\S+)=(\S+)(.*)$/) {
			$p = $1;
			$v = $2;
			$args = $3;

			if ($v =~ /\"(.*)\"/) {
				$str = $1;
				$cmd = $cmd . " -P$toplevel.$p=\\\"$str\\\"";
			} else {
				$cmd = $cmd . " -P$toplevel.$p=$v";
			}
		}

		$cmd = $cmd . " -c " . $filelist;
		$cmd = $cmd . " -o " . $exefile;
		$sim_log = $tstname . ".txt";

		$cmd = $cmd . " |& tee $sim_log";
		system "echo \'$cmd\'";
		system "bash -c \'$cmd\'";
		$errB= $?;

		if ($errB == 0 and -x $exefile) {
			## Run the simulation
			## {{{
			$tstamp = timestamp();
			system "echo \"$tstamp -- Starting simulation\" | tee -a $sim_log";
			system "$exefile >> $sim_log";

			## Finish the log with another timestamp
			## {{{
			$tstamp = timestamp();
			system "echo \"$tstamp -- Simulation complete\" | tee -a $sim_log";
			$msg = sprintf("%s IVerilog  -- %s", $tstamp, $tstname);
			## }}}

			## Look through the log file(s) for any errors and
			## report them
			## {{{
			system "grep \'ERROR\' $sim_log | sort -u";
			system "grep -q \'ERROR\' $sim_log";
			$errE = $?;

			system "grep -iq \'assert.*fail\' $sim_log";
			$errA = $?;

			system "grep -iq \'timing violation\' $sim_log";
			$errT = $?;

			system "grep -iq \'fail\' $sim_log";
			$errF = $?;

			system "grep -iq \'TEST PASS\' $sim_log";
			$errS = $?;

			if ($errE == 0 or $errA == 0 or $errF == 0) {
				## ERRORs found, either assertion or other fail
				print     "ERRORS    $msg\n";
			} elsif ($errT == 0) {
				# Timing violations present
				print     "TIMING-ER $msg\n";
			} elsif ($errS != 0) {
				# No success (TEST_PASS) message present
				print     "FAIL      $msg\n";
			} else {
				print     "Pass      $msg\n";
			}
			## }}}
			## }}}
		} else {
			## Report that this failed to build
			$tstamp = timestamp();
			$msg = sprintf("%s IVerilog  -- %s", $tstamp, $tstname);
			print "BLD-FAIL  $msg\n";
		}
		## }}}
	}
}
## }}}

simline("sdiostart");
