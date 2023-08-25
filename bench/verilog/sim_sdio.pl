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
$path_cnt = @ARGV;

$filelist = "sdio_files.txt";
$testlist = "sdio_testcases.txt";
$exefile  = "sdiosim";
$linestr  = "----------------------------------------";
$report   = "sdio_report.txt";
$toplevel = "tb_sdio";
$testd    = "test/";

## Usage: perl sim_sdio.pl all
##   or
## 	perl sim_sdio.pl <testcasename>

## Process arguments
## {{{
$run_all = 0;
if ($ARGV[0] eq "") {
	print "No test cases given\n";
	exit(0);
} elsif ($ARGV[0] eq "all") {
	$run_all = 1;
	open(SUM,">> $report");
	print(SUM "\nRunning all tests:\n$linestr\n");
	close SUM;
} elsif((($ARGV[0] eq "icarus") or ($ARGV[0] eq "iverilog")) and $ARGV[1] eq "all") {
	$run_all = 1;
	open(SUM,">> $report");
	print(SUM "\nRunning all tests:\n$linestr\n");
	close SUM;
} elsif (($ARGV[0] eq "icarus") or ($ARGV[0] eq "iverilog")) {
	$run_all = 0;
	@array = @ARGV;
	# Remove the "Icarus" flag
	splice(@array, 0, 1);
} else {
	@array = @ARGV;
}
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

	while ($line =~ /^(.*)#.*/) {
		$line = $1;
	} if ($line =~ /^\s*(\S+)\s+(\S+)\s+(.*)$/) {
		$tstname = $1;
		$tstscript = $2;
		$args = $3;
	} elsif ($line =~ /^\s*(\S+)$/) {
		$tstname = $1;
		$tstscript = $2;
		$args = "";
	}

	if ($tstname eq "") {
	} else {
		## {{{
		## Remove any prior build products, so we can detect a failed
		## build.
		if (-e $exefile) {
			unlink $exefile;
		}

		$tstamp = timestamp();

		## Set up the IVerilog command
		$cmd = "iverilog -g2012";

		$cmd = $cmd . " -DIVERILOG";
		$cmd = $cmd . " -DSCRIPT=\\\"../testscript/$tstscript.v\\\"";
		
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
		$sim_log = $testd . $tstname . ".txt";

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

			open (SUM,">> $report");
			if ($errE == 0 or $errA == 0 or $errF == 0) {
				## ERRORs found, either assertion or other fail
				print SUM "ERRORS    $msg\n";
				print     "ERRORS    $msg\n";
				push @failed,$tstname;
			} elsif ($errT == 0) {
				# Timing violations present
				print SUM "TIMING-ER $msg\n";
				print     "TIMING-ER $msg\n";
				push @failed,$tstname;
			} elsif ($errS != 0) {
				# No success (TEST_PASS) message present
				print SUM "FAIL      $msg\n";
				print     "FAIL      $msg\n";
				push @failed,$tstname;
			} else {
				print SUM "Pass      $msg\n";
				print     "Pass      $msg\n";
				push @passed,$tstname;
			} close SUM;
			## }}}
			## }}}
		} else {
			## Report that this failed to build
			open (SUM,">> $report");
			$tstamp = timestamp();
			$msg = sprintf("%s IVerilog  -- %s", $tstamp, $tstname);
			print SUM "BLD-FAIL  $msg\n";
			print     "BLD-FAIL  $msg\n";
			push @failed,$tstname;
			close SUM;
		}
		## }}}
	}
}
## }}}

## gettest: Look up a test's configuration
## {{{
sub gettest($) {
	my ($key)=@_;
	my	$tstname;

	open(GTL, $testlist);
	while($line = <GTL>) {
		next if ($line =~ /^\s*#/);
		if ($line =~ /^\s*(\S+)\s/) {
			$tstname = $1;
			last if ($tstname eq $key);
		}
	} close GTL;
	if ($tstname eq $key) {
		$line;
	} else {
		"# FAIL";
	}
}
## }}}

## Run all tests
## {{{
if (!-d $testd) {
	mkdir $testd;
}

if ($run_all) {
	open(TL, $testlist);
	while($line = <TL>) {
		next if ($line =~ /^\s*#/);
		simline($line);
	}

	open(SUM,">> $report");
	print (SUM "$linestr\nTest run complete\n\n");
	close SUM;
} else {
	foreach $akey (@array) {
		$line = gettest($akey);
		next if ($line =~ /FAIL/);
		simline($line);
	}
}
## }}}

if (@failed) {
	print "\nFailed testcases:\n$linestr\n";
	foreach $akey (@failed) {
		print " $akey\n";
	}
}

if (@passed) {
	print "\nPassing testcases:\n$linestr\n";
	foreach $akey (@passed) {
		print " $akey\n";
	}
}

if (@failed) {
	1;
} 0;
