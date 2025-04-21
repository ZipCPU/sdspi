#!/bin/perl
################################################################################
##
## Filename:	bench/verilog/sim_run.pl
## {{{
## Project:	SD-Card controller
##
## Purpose:	Runs one or more of the test cases described in
##		dev_testcases.txt.
##
##	This module is now multi-tasked.  Multiple simulations may run
##	concurrently, up to (the internal value) $maxtasks.
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
## }}}
## Copyright (C) 2016-2025, Gisselquist Technology, LLC
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
################################################################################
##
## }}}
use Cwd;
$path_cnt = @ARGV;

$filelist = "dev_files.txt";
$cpu_files= "cpu_files.txt";
$testlist = "dev_testcases.txt";
$exefile  = "./devsim";
$linestr  = "----------------------------------------";
$report   = "report.txt";
$wbtoplvl = "tb_wb";
$axtoplvl = "tb_axi";
$cputop   = "tb_cpu";
$testd    = "test/";
$vivado   = 0;
$ntasks   = 0;
$maxtasks = 16;

## Usage: perl sim_sim.pl all
##   or
## 	perl sim_sim.pl <testcasename>

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
} elsif(($ARGV[0] eq "vivado") and $ARGV[1] eq "all") {
	$run_all = 1;
	$vivado  = 1;
	open(SUM,">> $report");
	print(SUM "\nRunning all tests:\n$linestr\n");
	close SUM;
} elsif ($ARGV[0] eq "vivado") {
	$run_all = 0;
	$vivado  = 1;
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

	# Vivado can't handle include `SCRIPT when running from the GUI, so we
	# instead create a DEFINE to be used anytime we are running from this
	# Perl-based regression running script (as defined by this file,
	# sim_run.pl).
	my $defs = " -DREGRESSION";
	my $parm = "";
	my $cpu_flag = 0;

	my $vcddump=0;
	my $vcdfile="";

	while ($line =~ /^(.*)#.*/) {
		$line = $1;
	} if ($line =~ /^\s*(\S+)\s+(\S+)\s+(\S+)\s+(.*)$/) {
		$tstname = $1;
		$tstcfg  = $2;
		$tstscript = $3;
		$args = $4;
	}

	if ($tstcfg =~ /WBCPU/i) {
		$toplevel = $wbtoplvl;
		$filelist = "dev_files.txt";
		$parm = $parm . " -P$toplevel.OPT_CPU=1";
		$defs = $defs . " -DSDIO_AXI";
		$defs = $defs . " -DSCRIPT=\\\"/dev/null\\\"";
		$cpu_flag = 1;
	} elsif ($tstcfg =~ /WB/i) {
		$toplevel = $wbtoplvl;
		$filelist = "dev_files.txt";
		$parm = $parm . " -P$toplevel.OPT_CPU=0 -P$toplevel.MEM_FILE=\\\"\\\"";
		$cpu_flag = 0;
	} elsif ($tstcfg =~ /AXICPU/i) {
		$toplevel = $axtoplvl;
		$filelist = "dev_files.txt";
		$parm = $parm . " -P$toplevel.OPT_CPU=1";
		$defs = $defs . " -DSCRIPT=\\\"/dev/null\\\"";
		$cpu_flag = 1;
	} elsif ($tstcfg =~ /AXI/i) {
		$toplevel = $axtoplvl;
		$filelist = "dev_files.txt";
		$parm = $parm . " -P$toplevel.OPT_CPU=0 -P$toplevel.MEM_FILE=\\\"\\\"";
		$defs = $defs . " -DSDIO_AXI";
		$cpu_flag = 0;
	} else {
		return();
	}

	if ($tstname eq "") {	## No test
		return();
	} elsif ($vivado > 0) {	## We have no Vivado test support
		return();
	} else {
		## {{{
		## Remove any prior build products, so we can detect a failed
		## build.
		$exefile = $testd . $tstname;
		if (-e $exefile) {
			unlink $exefile;
		}

		$tstamp = timestamp();

		## Set up the IVerilog command
		## {{{
		$cmd = "iverilog -g2012";
		$defs = $defs . " -DIVERILOG";

		## Process parameters
		## {{{
		while($args =~ /\s*(\S+)\s*(.*)$/) {
			my $eq = 0;
			if ($args =~ /\s*(\S+)\s*=\s*(\S+)(.*)$/) {
				$p = $1;
				$v = $2;
				$args = $3;
				$eq = 1;
			} elsif ($args =~ /\s*(\S+)\s*(.*)$/) {
				$p = $1;
				$v = "";
				$args = $2;
				$eq = 0;
			}

			if ($p =~ /^-D(\S+)/) { ## A macro definition
				## {{{
				$p = $1;
				if ($v =~ /\"(.*)\"/) {
					$str = $1;
					$defs = $defs . " -D$p=\\\"$str\\\"";
				} elsif ($eq) {
					$defs = $defs . " -D$p=$v";
				} else {
					$defs = $defs . " -D$p";
				}
				## }}}
			} elsif ($v =~ /\"(.*)\"/) { ## Parameter string
				## {{{
				$str = $1;
				$parm = $parm . " -P$toplevel.$p=\\\"$str\\\"";
				## }}}
			} else {
				$parm = $parm . " -P$toplevel.$p=$v";
			}
		}
		## }}}

		## Include the test script
		## {{{
		if ($cpu_flag) {
			if ($tstscript =~ /.hex$/) {
				$parm = $parm . " -P$toplevel.MEM_FILE=\\\"$tstscript\\\"";
			} else {
				$parm = $parm . " -P$toplevel.MEM_FILE=\\\"$tstscript.hex\\\"";
			}
		} else {
			$defs = $defs . " -DSCRIPT=\\\"../testscript/$tstscript.v\\\"";
		}
		## }}}

		$cmd = $cmd . " " . $defs . " " . $parm;

		$cmd = $cmd . " -c " . $filelist;
		if ($cpu_flag) {
			$cmd = $cmd . " -c " . $cpu_files;
		}
		$cmd = $cmd . " -o " . $exefile;
		$cmd = $cmd . " -s " . $toplevel;
		$sim_log = $testd . $tstname . ".txt";
		## }}}

		## Build the IVerilog simulation
		## {{{
		$cmd = $cmd . " |& tee $sim_log";
		system "echo \'$cmd\'";
		system "bash -c \'$cmd\'";
		$errB= $?;
		## }}}

		if ($errB == 0 and -x $exefile) {
			## Run the simulation
			## {{{
			$tstamp = timestamp();
			$pid = fork;
			if ($pid ne 0) {
				return;
			}

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
				$msg = sprintf("ERRORS    %s\n", $msg);
				push @failed,$tstname;
			} elsif ($errT == 0) {
				# Timing violations present
				$msg = sprintf("TIMING-ER %s\n", $msg);
				push @failed,$tstname;
			} elsif ($errS != 0) {
				# No success (TEST_PASS) message present
				$msg = sprintf("FAIL      %s\n", $msg);
				push @failed,$tstname;
			} else {
				$msg = sprintf("Pass      %s\n", $msg);
				push @passed,$tstname;
			}

			open (SUM,">> $report");
			print SUM $msg;
			close SUM;
			print     $msg;
			## }}}

			exit 0;
			## }}}
		} else {
			## Report that the simulation failed to build
			## {{{
			open (SUM,">> $report");
			$tstamp = timestamp();
			$msg = sprintf("%s IVerilog  -- %s", $tstamp, $tstname);
			print SUM "BLD-FAIL  $msg\n";
			print     "BLD-FAIL  $msg\n";
			push @failed,$tstname;
			close SUM;
			## }}}
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

print "Looking up $key\n";

	open(GTL, $testlist);
	while($line = <GTL>) {
		next if ($line =~ /^\s*#/);
		if ($line =~ /^\s*(\S+)\s+(\S+)\s/) {
			$tstname = $1;
			last if ($tstname eq $key);
		}
	} close GTL;
	if ($tstname eq $key) {
		$line;
	} else {
		print "ERR: Test not found: $key\n";
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

		if ($ntasks >= $maxtasks) {
			if (waitpid(-1,0) eq 0) {
				$ntasks = 0;
			} else {
				$ntasks = $ntasks - 1;
			}
		}

		simline($line);
		$ntasks = $ntasks + 1;
	}

	while(waitpid(-1,0) gt 0) {
		;
	}

	open(SUM,">> $report");
	print (SUM "$linestr\nTest run complete\n\n");
	close SUM;
} else {
	foreach $akey (@array) {
		$line = gettest($akey);
		next if ($line =~ /FAIL/);
		simline($line);

		while(waitpid(-1,0) gt 0) {
			;
		}
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
	die "Not all tests passed\n";
}
