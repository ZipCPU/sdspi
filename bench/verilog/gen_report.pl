#!/bin/perl
################################################################################
##
## Filename:	bench/verilog/gen_report.pl
## {{{
## Project:	SD-Card controller
##
## Purpose:	Generate an HTML report file, containing the status of the
##		last simulation run.
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
## }}}
## Copyright (C) 2025, Gisselquist Technology, LLC
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
$filelist = "dev_files.txt";
$rawreport= "report.txt";
$htmlfil= "report.html";
my $last_mtime = 0;
my $last_tstamp = "";

################################################################################
##
## Get the last file modified time
## {{{
open(FLIST, $filelist);
while($line = <FLIST>) {
	if ($line =~ /\s*(\S+)\s*$/) {
		$fname = $1;
	} else {
		next;
	}

	my ($dev, $ino, $mode, $nlink, $uid, $gid, $rdev, $size,
		$atime, $mtime, $ctime, $blksize, $blocks) = stat($fname);

	if ($mtime > $last_mtime) {
		$last_mtime = $mtime;
	}
}
close FLIST;

my $sc, $mn, $hr, $dy, $mo, $yr, $wd, $yd, $isdst;
($sc, $mn, $hr, $dy, $mo, $yr, $wd, $yd, $isdst) = localtime($last_mtime);
$yr = $yr+1900; $mo=$mo+1;
$last_tstamp = sprintf("%04d/%02d/%02d %02d:%02d:%02d",
			$yr,$mo,$dy,$hr,$mn,$sc);
## }}}
################################################################################
##
## Get the latest test result
## {{{
my $n=0;
my %STAT;

open(REPORT, $rawreport);
while($line = <REPORT>) {
	next if ($line =~ /^-/);
	if ($line =~ /(\S+)\s+(\d\d\d\d.\d\d.\d\d.\d\d:\d\d:\d\d)\s+(\S+)\s+..\s+(\S+)\s*$/) {
		$status = $1;
		$tstamp = $2;
		$tool = $3;
		$test = $4;

		if ("$tstamp" lt "$last_tstamp") {
			$STAT{$test} = "Out-of-date";
			$TOOL{$test} = $tool;
			$TSTAMP{$test} = $tstamp;
			next;
		}
		if (exists $STAT{$test}) {
			my $last_tool, $last_stat;

			$last_tool = $TOOL{$test};
			$last_stat = $STAT{$test};

			if ($last_tool =~ /$tool/) {
			} else {
				$TOOL{$test} = "$last_tool, $tool";
			}

			if ($last_stat =~ /fail/i) {
			} elsif ($last_stat =~ /error/i) {
			} elsif ($last_stat =~ /warn/i) {
				if ($stat =~ /fail/i) {
					$STAT{$test} = $status;
				} elsif ($stat =~ /error/i) {
					$STAT{$test} = $status;
				}
			} else {
				$STAT{$test} = $status;
			}
		} else {
			$STAT{$test} = $status;
			$TOOL{$test} = $tool;
		}

		$TSTAMP{$test} = $tstamp;
	}
}

printf("Last timestamp: $last_tstamp\n");

close REPORT;

foreach $key (sort (keys %STAT)) {
	printf("%-12s %12s $TSTAMP{$key} -- $TOOL{$key}\n", $key, $STAT{$key});
}

open(HTML, "> $htmlfil");
print HTML "<HTML><HEAD><TITLE>Simulation report</TITLE></HEAD><BODY>\n";
print HTML "<H1 align=center>SD Controller Simulation Report</H1>\n";
print HTML "<TABLE>\n";
print HTML "<TR><TH>Test</TH><TH>Status</TH><TH>Sim Timestamp</TH><TH>Tool</TH></TR>\n";
foreach $key (sort (keys %STAT)) {
	my $lin, $st;

	$st = $STAT{$key};
	$clr="white";
	if ($st =~ /fail/i) {
		$clr="#ffa4a";
	} elsif ($st =~ /err/i) {
		$clr="#ffa4a";
	} elsif ($st =~ /warn/i) {
		$clr="#ffffca";
	} elsif ($st =~ /out-of-date/i) {
		$clr="#ffffca";
	} elsif ($st =~ /pass/i) {
		$clr="#caeec8";
	}
	$lin = sprintf("<TR><TH>%s</TH><TD bgcolor=$clr>%s</TD><TD>$TSTAMP{$key}</TD><TD>$TOOL{$key}</TD></TR>\n", $key, $st);
	print HTML $lin;
}
print HTML "</TABLE></BODY></HTML>\n";
close HTML;
