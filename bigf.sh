for i in rxtest/*.bin; do (echo 'P5\n1024 1024 255\n'; #head -c 508 </dev/zero
)|cat - $i |(echo $i && DISPLAY=:0 xli -gamma 2.2 stdin);done
