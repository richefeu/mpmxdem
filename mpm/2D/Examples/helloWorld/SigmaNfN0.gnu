set bmargin 6
set lmargin 12
set rmargin 12
#set key lm t
plot 'SigmaNfN0.txt' u 1:2 w l linewidth 2 lc rgb 'red' title 'σ_n/l' ,\
'SigmaNfN.txt' u 1:3 w l linewidth 2 title 'f_n'
set xlabel font "Helvetica,20"
set ylabel font "Helvetica,20"
set title font "Helvetica,20"
set key font "Helvetica,15"
set xrange [* : *] noreverse nowriteback
set xlabel 't'
set ylabel 'E'
