set bmargin 6
set lmargin 12
set rmargin 12
plot 'lineWork10Slices.txt' u 1:4 w l linewidth 2 lc rgb 'orange' title 'W_{t, tot} 10 slices', \
'lineWork20Slices.txt' u 1:4 w l linewidth 2 lc rgb 'blue' title 'W_{t, tot} 20 slices', \
'lineWork40Slices.txt' u 1:4 w l linewidth 2 lc rgb 'green' title 'W_{t, tot} 40 slices'
set xlabel font "Helvetica,20"
set ylabel font "Helvetica,20"
set title font "Helvetica,20"
set key font "Helvetica,15"
set xrange [* : *] noreverse nowriteback
set xlabel 't'
set ylabel 'E'
