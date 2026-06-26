set bmargin 6
set lmargin 12
set rmargin 12

plot 'histWork40Slices.txt' u 1:3 w l linewidth 2 lc rgb 'green' title 'W_{t, tot} 40 slices', \
'histWork20Slices.txt' u 1:3 w l linewidth 2 lc rgb 'blue' title 'W_{t, tot} 20 slices', \
'histWork10Slices.txt' u 1:3 w l linewidth 2 lc rgb 'orange' title 'W_{t, tot} 10 slices'
set xlabel font "Helvetica,20"
set ylabel font "Helvetica,20"
set title font "Helvetica,20"
set key font "Helvetica,15"
set xrange [* : *] noreverse nowriteback
set xlabel 't'
set ylabel 'E'
