#!/usr/bin/perl
# take clean html in ast.html and insert into Webpage.cpp IN-PLACE.
# N.B. All literal strings must use apostrophe, double quotes are retained by preceding with \
# N.B. max size of an F() string seems to be about 2KB

use strict;
use warnings;

# name of file with html, to edit and a tmp file
my $htmlfn = "ast.html";
my $editfn = "Webpage.cpp";
my $tmpfn = ".Webpage.cpp";

# we insist editfn is writable
my ($dev,$ino,$mode,$xxx) = stat ($editfn);
defined($mode) or die "Can not stat $editfn: $!\n";
($mode & 200) or die "$editfn must be writable\n";

# open edit file
open EF, "$editfn" or die "Can not open $editfn: $!\n";

# open html file
open HF, "$htmlfn" or die "Can not open $htmlfn: $!\n";

# create temp file
open TF, ">$tmpfn" or die "Can not create $tmpfn: $!\n";

# copy edit to tmp up through the first magic line, inclusive
while (<EF>) {
    print TF;
    last if (/111111111111111111111111111111/);
}

# now copy the modifed html into tmp
print TF "        client.print (F(\n";

my $nchars = 0;
while(<HF>) {
    chomp();
    $_ =~ s/\\/\\\\/g;		# retain all \ by turning into \\
    $_ =~ s/"/\\"/g;		# retain all " by turning into \"
    $_ =~ s/\t/        /g;	# expand tabs
    if (/XXXX/) {
	my $version = `date -u +'%Y%m%d%H'`;
	chomp($version);
	$_ =~ s/XXXX/$version/;
    }
    print TF "            \"$_ \\r\\n\"\n";
    $nchars += length();
    if ($nchars > 2000) {
	print TF "        ));\n";
	print TF "        client.print (F(\n";
	$nchars = 0;
    }
}

print TF "        ));\n";

# discard editfn down to second magic line, again inclusive
while (<EF>) {
    if (/999999999999999999999999999999/) {
	print TF;
	last;
    }
}

# copy remainder of editfn to tmp
while (<EF>) {
    print TF;
}

# close all files and replace edit with tmp
close HF;
close EF;
close TF;
unlink ($editfn);
rename ($tmpfn, $editfn);
