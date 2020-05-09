for i in rxtest/*.bin; do <$i DISPLAY=:0 perl plot.pl - 8192 1 0 | xli stdin;wait;done
