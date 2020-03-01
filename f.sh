for i in *bin; do (echo 'P5\n512 512 255\n'; head -c 508 </dev/zero)|cat - $i |DISPLAY=:0 xli -gamma 2.2 stdin;done
