#!/usr/bin/perl -w
undef$/;$_=<>;
#open STDOUT,"|xli stdin" or die $!;
$x=shift || 1024;
$count=shift || 1;
$stride=shift || 0;
print "P6\n$x 256 255\n";
for $j (0..255) {
    for $i (0..$x-1) {
	my $out;
	for $n (0..$count-1) { $out.=pack"C",(substr($_,$i+$stride*$n,1)eq chr$j)?255:0 }
	print substr($out."\0"x3,0,3);
    }
}
