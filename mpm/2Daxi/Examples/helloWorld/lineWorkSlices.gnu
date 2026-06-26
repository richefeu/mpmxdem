set bmargin 6
set lmargin 12
set rmargin 12
plot 'lineWork90Slices.txt' u 1:2 w l linewidth 2 title 'W_{n, tot}',\
'lineWork90Slices.txt' u 1:3 w l linewidth 2 title 'W_{t, tot}',\
'lineWork90Slices.txt' u 1:4 w l linewidth 2 title 'W_{int, tot}'
set xlabel font "Helvetica,20"
set ylabel font "Helvetica,20"
set title font "Helvetica,20"
set key font "Helvetica,15"
set xrange [* : *] noreverse nowriteback
set xlabel 't'
set ylabel 'E'
